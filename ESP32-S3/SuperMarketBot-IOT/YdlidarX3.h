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
  Serial1.begin(YDLIDAR_X3_BAUD, SERIAL_8N1, YDLIDAR_X3_RX, YDLIDAR_X3_TX);
  g_x3Scan.count = 0;
  g_x3Scan.scanSeq = 0;
  g_x3Scan.lastScanMs = 0;
  g_x3Scan.scanReady = false;

  // Start scan command (X3 / X4 protocol)
  uint8_t startCmd[] = {0xA5, 0x60};
  Serial1.write(startCmd, sizeof(startCmd));
  Serial.printf("[X3] Init done — UART @ %d baud, cmd sent to start scan.\n", YDLIDAR_X3_BAUD);
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
  static uint8_t  s_buf[YDLIDAR_SCAN_BUFF_SIZE];
  static uint16_t s_bufLen = 0;

  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();
    if (s_bufLen >= sizeof(s_buf)) {
      // overflow → reset
      s_bufLen = 0;
    }
    s_buf[s_bufLen++] = b;

    // Tìm header 0xAA 0x55
    if (s_bufLen >= 2) {
      if (s_buf[0] != 0xAA || s_buf[1] != 0x55) {
        // không đúng header → shift buffer
        s_buf[0] = s_buf[1];
        s_bufLen = 1;
        continue;
      }
    }

    // Chờ đủ length byte (sau header)
    if (s_bufLen < 3) continue;
    uint8_t payloadLen = s_buf[2];
    uint16_t frameLen = 3 + payloadLen + 1; // header(2) + length(1) + payload + checksum
    if (s_bufLen < frameLen) continue;

    // Đủ frame → parse
    // (Skeleton — implementation chi tiết phụ thuộc X3 firmware version)
    g_x3Scan.scanSeq++;
    g_x3Scan.lastScanMs = millis();
    g_x3Scan.scanReady = true;

    // Reset buffer
    s_bufLen = 0;
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

#endif // YDLIDAR_X3_H
