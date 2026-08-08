/* =====================================================================
 *  YdlidarX3.h — Driver skeleton cho YDLIDAR X3 (UART, 230400 baud)
 *
 *  Vai trò:
 *    1. Đọc scan 360° mỗi ~100ms (10 Hz) từ X3 qua Serial1.
 *    2. Lưu vào buffer `g_x3Scan[]` (mảng các điểm {angle, distance_mm, quality}).
 *    3. Cung cấp helper để:
 *       - Lấy khoảng cách min trong một cung góc (vd: trước xe, ±30°) → obstacle backup.
 *       - Lấy toàn bộ scan để gửi lên BE qua MQTT cho SLAM (Cartographer / Hector).
 *
 *  Giao thức YDLidar X3 (tham khảo datasheet X3):
 *    - Header 0xAA 0x55 (2 bytes)
 *    - Length (1 byte, ct ngay sau header)
 *    - Payload (Length bytes): mỗi point = 2 bytes distance + 1 byte angle_offset + 1 byte quality
 *    - Nếu bit cao length = 1 → "two-byte" distance mode (X3 mặc định)
 *
 *  Lưu ý:
 *    - Driver này viết skeleton; cần test với X3 thật để verify protocol byte order.
 *    - YDLIDAR-SDK chính thức (ydlidar_driver) cũng có thể dùng thay thế skeleton này.
 * =====================================================================*/
#ifndef YDLIDAR_X3_H
#define YDLIDAR_X3_H

#include <Arduino.h>
#include "Config.h"

#if USE_YDLIDAR_X3

/* -------------------- Cấu trúc dữ liệu scan ------------------------- */
struct LidarPoint {
  float angleRad;    // góc từ 0 đến 2π (0 = phía trước robot)
  uint16_t distanceMm; // mm (0 = invalid / out of range)
  uint8_t quality;     // 0..255 (càng cao càng tốt)
};

constexpr uint16_t X3_MAX_POINTS = YDLIDAR_MAX_POINTS;

/** Buffer scan mới nhất — được fill bởi task riêng, đọc bởi control loop. */
struct X3Scan {
  LidarPoint points[X3_MAX_POINTS];
  uint16_t count;          // số điểm hợp lệ trong scan hiện tại
  uint32_t scanSeq;        // sequence number (tăng mỗi scan mới)
  uint32_t lastScanMs;     // millis lần cuối có scan hoàn chỉnh
  bool scanReady;          // cờ báo có scan mới (set bởi parser, clear bởi reader)
};

extern X3Scan g_x3Scan;

/* -------------------- API init / loop ------------------------------- */

/**
 * Khởi tạo Serial1 cho X3, gửi lệnh start scan (X3 cần start command sequence).
 * X3 default protocol: 0xA5 0x60 (start scan) hoặc SCAN cmd tùy firmware version.
 */
inline void x3Init() {
#ifdef YDLIDAR_X3_M_CTR
  if (YDLIDAR_X3_M_CTR >= 0) {
    pinMode(YDLIDAR_X3_M_CTR, OUTPUT);
    digitalWrite(YDLIDAR_X3_M_CTR, HIGH); // Bật động cơ quay LiDAR (M_CTR = HIGH)
  }
#endif
  // === Khởi Serial1 ở baud X3 mặc định (115200) ===
  // NOTE: X3 firmware này KHÔNG hỗ trợ command đổi baud (0xA5 0x0B) → KHÔNG gửi lệnh này
  // nếu không ESP32 sẽ switch baud nhưng X3 vẫn ở 115200 → 0 pts.
  Serial1.begin(YDLIDAR_X3_BAUD, SERIAL_8N1, YDLIDAR_X3_RX, YDLIDAR_X3_TX);
  delay(100);

  g_x3Scan.count = 0;
  g_x3Scan.scanSeq = 0;
  g_x3Scan.lastScanMs = 0;
  g_x3Scan.scanReady = false;

  // === Tăng sample rate lên 4kHz (X3 firmware ≥ 1.4.0 hỗ trợ) ===
  // Command: 0xA5 0x09 [sample_rate_hz_lo] [sample_rate_hz_hi]
  // 0x0FA0 = 4000 Hz (max X3 hỗ trợ)
  uint8_t setSampleRateCmd[] = {0xA5, 0x09, 0xA0, 0x0F};
  Serial1.write(setSampleRateCmd, sizeof(setSampleRateCmd));
  delay(50);

  // === Start scan command ===
  uint8_t startCmd[] = {0xA5, 0x60};
  Serial1.write(startCmd, sizeof(startCmd));
  delay(50);

  Serial.printf("[X3] Init done — UART @ %d baud, sample rate target ~4kHz\n", YDLIDAR_X3_BAUD);
  Serial.printf("[X3] NOTE: X3 firmware này không hỗ trợ đổi baud. Pts sẽ ổn định ~349/scan.\n");
}

/**
 * Gọi mỗi ~50ms từ vòng control hoặc task riêng.
 * Đọc bytes sẵn có trên Serial1, parse frame X3, fill `g_x3Scan` nếu đủ 1 scan.
 *
 * X3 Frame format (cartographer mode):
 *   0xAA 0x55 | length(1) | freq(2) | payload(length-3 bytes) | checksum(1)
 *   - freq: 2 bytes little-endian, sampling frequency * 100
 *   - payload: cứ 5 bytes = 1 point:
 *       byte0..1: distance_mm (uint16_t little-endian, bit 15 = quality low bit)
 *       byte2    : quality high bits (top 7) + bit0 = angle inversion flag
 *       byte3    : angle_offset (độ * 64, relative to start_angle)
 *       (5th byte = end of point)
 *
 * Lưu ý: protocol cụ thể cần verify với X3 datasheet — đây là skeleton.
 */
inline void x3Poll() {
  // Buffer lớn 4096 byte (YDLIDAR_SCAN_BUFF_SIZE) để chứa nhiều packet liên tiếp
  // khi sample rate cao (4kHz). X3 gửi ~80 byte/packet, 4096 chứa ~50 packets.
  static uint8_t  s_buf[YDLIDAR_SCAN_BUFF_SIZE];
  static uint16_t s_bufLen = 0;
  static uint32_t s_dbgLastMs = 0;

  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();
    if (s_bufLen >= sizeof(s_buf)) {
      s_bufLen = 0;
    }
    s_buf[s_bufLen++] = b;

    // DEBUG: in 16 byte đầu + baud rate mỗi 5s để verify parser
    if (millis() - s_dbgLastMs > 5000) {
      s_dbgLastMs = millis();
      Serial.printf("[X3-DBG] baud=%d, avail=%d, bufLen=%d, first16=", YDLIDAR_X3_BAUD, Serial1.available(), s_bufLen);
      for (int i = 0; i < 16 && i < s_bufLen; i++) Serial.printf("%02X ", s_buf[i]);
      Serial.println();
    }

    if (s_bufLen >= 2) {
      if (s_buf[0] != 0xAA || s_buf[1] != 0x55) {
        s_buf[0] = s_buf[1];
        s_bufLen = 1;
        continue;
      }
    }

    if (s_bufLen < 10) continue;
    uint8_t sampleCount = s_buf[3];
    uint16_t packageLen = 10 + (sampleCount * 2);

    if (s_bufLen < packageLen) continue;

    uint16_t fsa = s_buf[4] | (s_buf[5] << 8);
    uint16_t lsa = s_buf[6] | (s_buf[7] << 8);
    float startAngle = (float)(fsa >> 1) / 64.0f;
    float endAngle   = (float)(lsa >> 1) / 64.0f;
    float diffAngle = endAngle - startAngle;
    if (diffAngle < 0) diffAngle += 360.0f;

    // Kiếm tra gói Zero Packet (Bit 0 của CT = 1 -> Bắt đầu vòng quay 360° mới)
    static LidarPoint s_accumPoints[X3_MAX_POINTS];
    static uint16_t   s_accumCount = 0;
    static uint32_t   s_lastPacketMs = 0;

    bool isZeroPacket = (s_buf[2] & 0x01) != 0;
    uint32_t nowMs = millis();
    // Timeout 80ms = ~12 Hz refresh (X3 ở baud 115200 gửi ~10 packets/s × 40 pts = 400 pts/s)
    // 80ms đủ chờ 1 packet liên tiếp, nếu quá 80ms không có packet → scan đã đứt quãng → flush
    bool timeout = (nowMs - s_lastPacketMs) > 80;

    // Flush scan khi:
    //   1. Timeout > 80ms + có >= 100 điểm (đảm bảo scan đủ dày, không rỗng)
    // KHÔNG dùng Zero packet (X3 firmware này gửi Zero packet không đều, gây dao động pts)
    if (timeout && s_accumCount >= 100) {
      // Copy Double-Buffer sang mảng hiển thị công khai (KHÔNG BAO GIỜ BỊ RESET VỀ 0 ĐỨNG HÌNH!)
      memcpy(g_x3Scan.points, s_accumPoints, s_accumCount * sizeof(LidarPoint));
      g_x3Scan.count = s_accumCount;
      g_x3Scan.scanSeq++;
      g_x3Scan.lastScanMs = nowMs;
      g_x3Scan.scanReady = true;

      // Serial log mỗi 2s để theo dõi
      static uint32_t lastLogMs = 0;
      if (nowMs - lastLogMs > 2000) {
        lastLogMs = nowMs;
        Serial.printf("[YDLIDAR X3] 360° Scan #%u | Points: %u\n", g_x3Scan.scanSeq, g_x3Scan.count);
      }
      s_accumCount = 0; // Reset bộ đệm tích lũy cho vòng quay tiếp theo
    }
    s_lastPacketMs = nowMs;

    if (sampleCount > 0 && sampleCount <= 200) {
      float angleStep = (sampleCount > 1) ? (diffAngle / (sampleCount - 1)) : 0.0f;
      for (uint8_t i = 0; i < sampleCount; i++) {
        uint16_t rawDist = s_buf[10 + i * 2] | (s_buf[11 + i * 2] << 8);
        uint16_t distMm = rawDist / 4;
        float angleDeg = startAngle + (angleStep * i);
        if (angleDeg >= 360.0f) angleDeg -= 360.0f;
        float angleRad = angleDeg * (float)M_PI / 180.0f;

        if (s_accumCount < X3_MAX_POINTS) {
          s_accumPoints[s_accumCount].angleRad = angleRad;
          s_accumPoints[s_accumCount].distanceMm = distMm;
          s_accumPoints[s_accumCount].quality = (rawDist > 0) ? 200 : 0;
          s_accumCount++;
        }
      }
    }

    // Trượt phần bộ nhớ dư còn lại trong buffer lên đầu (KHÔNG xóa s_bufLen về 0 để tránh vỡ sync UART)
    if (s_bufLen >= packageLen) {
      memmove(s_buf, s_buf + packageLen, s_bufLen - packageLen);
      s_bufLen -= packageLen;
    } else {
      s_bufLen = 0;
    }
  }
}

/**
 * Lấy khoảng cách min (mm) trong cung góc [centerDeg ± halfWidthDeg].
 * Dùng cho obstacle backup: lấy cung trước robot ±30° → set g_state.lidarFront.
 */
inline uint16_t x3MinInArc(float centerDeg, float halfWidthDeg, uint8_t minQuality = 10) {
  float cMin = centerDeg - halfWidthDeg;
  float cMax = centerDeg + halfWidthDeg;
  uint16_t minMm = 0xFFFF;

  for (uint16_t i = 0; i < g_x3Scan.count; i++) {
    const LidarPoint &p = g_x3Scan.points[i];
    if (p.quality < minQuality) continue;
    if (p.distanceMm == 0) continue;
    float deg = p.angleRad * 180.0f / (float)M_PI;
    if (deg < 0) deg += 360.0f;
    if (deg < cMin || deg > cMax) continue;
    if (p.distanceMm < minMm) minMm = p.distanceMm;
  }
  return minMm;
}

/** Stop scan (gửi lệnh stop trước khi tắt) */
inline void x3Stop() {
  uint8_t stopCmd[] = {0xA5, 0x65};
  Serial1.write(stopCmd, sizeof(stopCmd));
  delay(10);
  Serial1.end();
}

#else  // !USE_YDLIDAR_X3

/* Stubs khi X3 bị tắt — để code khác vẫn gọi được mà không cần #ifdef */
struct LidarPoint { float angleRad; uint16_t distanceMm; uint8_t quality; };
constexpr uint16_t X3_MAX_POINTS = 1;
struct X3Scan {
  LidarPoint points[X3_MAX_POINTS];
  uint16_t count;
  uint32_t scanSeq;
  uint32_t lastScanMs;
  bool scanReady;
};
extern X3Scan g_x3Scan;
inline void x3Init() {}
inline void x3Poll() {}
inline uint16_t x3MinInArc(float, float, uint8_t = 10) { return 0xFFFF; }
inline void x3Stop() {}

#endif // USE_YDLIDAR_X3

/** Definition biến global — dùng `inline` để tránh multiple-definition khi include ở nhiều .h */
inline X3Scan g_x3Scan = {};

/** Task wrapper: gọi x3Poll() trong vòng lặp vô hạn.
 *  Được `.ino` gọi qua xTaskCreatePinnedToCore(taskX3, ...).
 *  Luôn compile — x3Poll() sẽ là stub nếu USE_YDLIDAR_X3=0. */
inline void taskX3(void *) {
  for (;;) {
    x3Poll();
    delay(1);
  }
}

#endif // YDLIDAR_X3_H
