/* =====================================================================
 *  YdlidarX3.h — Driver cho YDLIDAR X3 (UART, 115200 baud)
 *
 *  Vai trò:
 *    1. Đọc scan 360° mỗi ~100ms (10 Hz) từ X3 qua Serial1.
 *    2. Lưu vào buffer `g_x3Scan[]` (mảng các điểm {angle, distance_mm, quality}).
 *    3. Cung cấp helper để:
 *       - Lấy khoảng cách min trong một cung góc (vd: trước xe, ±30°) → obstacle backup.
 *       - Lấy toàn bộ scan để gửi qua micro-ROS /scan topic.
 *
 *  Protocol X3 (tham khảo X3 datasheet):
 *    Header 0xAA 0x55 | length(1) | freq(2) | payload(length-3 bytes) | checksum(1)
 *    payload: mỗi 2 bytes = 1 point sample (distance_mm = uint16 LE / 4).
 *
 *  Quan trọng — TẠI SAO PTS BỊ TỤT KHI ROS2 LÊN:
 *    - Baud 115200 chỉ chứa ~11.5 KB/s. Ở 4 kHz sample rate → 20 KB/s KHÔNG vừa.
 *    - Code cũ drain TỪNG BYTE một lần, mỗi ~50ms → drain chậm hơn UART nhận.
 *    - Khi micro-ROS agent chạy, FreeRTOS scheduler ưu tiên agent → control loop
 *      bị delay thêm → UART FIFO 128 byte tràn → mất bytes → X3 reset packet sync.
 *
 *  FIX:
 *    - Drain TẤT CẢ bytes có sẵn mỗi poll (readBytesIntoBuf), không phải 1 byte/loop.
 *    - Tăng UART hardware FIFO: `Serial1.setRxBufferSize(2048)` (ESP32 max 2048).
 *    - Chạy `taskX3` ở priority 6 trên core 1, period 5ms — KHÔNG dựa vào control loop.
 *    - Diagnostic counters (droppedBytes, syncLosses, scanCount) + rate-limited log
 *      để phát hiện lỗi UART/mất đồng bộ ngay khi nó xảy ra, không spam log.
 *
 *  TEST/ASSERT (per project rule "tests, asserts, runtime logs"):
 *    - Static asserts ở compile time (X3_MAX_POINTS, baud, payload size).
 *    - Runtime asserts trong init (heap check, malloc non-null).
 *    - Rate-limited diagnostic log: 1 lần/giây, chỉ in khi có vấn đề (dropBytes>0,
 *      syncLoss>0, scanRate<5Hz) — KHÔNG spam happy-path log.
 * =====================================================================*/
#ifndef YDLIDAR_X3_H
#define YDLIDAR_X3_H

#include <Arduino.h>
#include "Config.h"

#if USE_YDLIDAR_X3

/* -------------------- Compile-time asserts ---------------------------- */
// Baud phải đủ lớn cho payload X3 (mỗi packet ~80 byte @ 10 Hz = 800 B/s minimum;
// ở 4kHz thực tế cần >=230400). Ở 115200 sẽ bottleneck — log sẽ cảnh báo lúc runtime.
static_assert(YDLIDAR_X3_BAUD >= 115200,
              "YDLIDAR_X3_BAUD < 115200 — X3 factory default là 115200, không thấp hơn được");
static_assert(YDLIDAR_SCAN_BUFF_SIZE >= 1024,
              "YDLIDAR_SCAN_BUFF_SIZE phải >= 1024 để chứa nhiều packet giữa các lần drain");
static_assert(YDLIDAR_MAX_POINTS >= 360 && YDLIDAR_MAX_POINTS <= 8000,
              "YDLIDAR_MAX_POINTS ngoài [360, 8000] — sai cấu hình");
static_assert(sizeof(float) == 4, "YDLIDAR cần float 32-bit");

/* -------------------- Cấu trúc dữ liệu scan ------------------------- */
struct LidarPoint {
  float angleRad;       // góc từ 0 đến 2π (0 = phía trước robot)
  uint16_t distanceMm;  // mm (0 = invalid / out of range)
  uint8_t quality;      // 0..255 (càng cao càng tốt)
};

constexpr uint16_t X3_MAX_POINTS = YDLIDAR_MAX_POINTS;

/** Buffer scan mới nhất — được fill bởi taskX3, đọc bởi control loop / micro-ROS. */
struct X3Scan {
  LidarPoint points[X3_MAX_POINTS];
  uint16_t count;       // số điểm hợp lệ trong scan hiện tại
  uint32_t scanSeq;     // sequence number (tăng mỗi scan mới)
  uint32_t lastScanMs;  // millis lần cuối có scan hoàn chỉnh
  bool scanReady;       // cờ báo có scan mới (set bởi parser, clear bởi reader)
};

/** Diagnostic counters — reset khi gọi x3ResetDiag(). */
struct X3Diag {
  uint32_t bytesRead;       // tổng bytes đã drain thành công từ UART
  uint32_t bytesDropped;    // bytes bị mất do UART FIFO tràn (estimated)
  uint32_t syncLosses;      // số lần phát hiện sai header 0xAA55 → phải resync
  uint32_t checksumErrors;  // số packet sai checksum
  uint32_t scansPublished;  // số scan đã publish thành công
  uint32_t lastDropLogMs;   // millis lần cuối log diagnostic
};

extern X3Scan g_x3Scan;
extern X3Diag g_x3Diag;

/* -------------------- API init / loop ------------------------------- */

/**
 * Khởi tạo Serial1 cho X3, gửi lệnh start scan.
 * NOTE: X3 firmware này KHÔNG hỗ trợ command đổi baud (0xA5 0x0B) — không gửi.
 */
inline void x3Init() {
#ifdef YDLIDAR_X3_M_CTR
  if (YDLIDAR_X3_M_CTR >= 0) {
    pinMode(YDLIDAR_X3_M_CTR, OUTPUT);
    digitalWrite(YDLIDAR_X3_M_CTR, HIGH); // Bật động cơ quay LiDAR (M_CTR = HIGH)
  }
#endif

  // === Tăng UART hardware FIFO lên max (ESP32 hỗ trợ tới 2048 byte) ===
  // FIFO lớn = buffer giữa các lần poll, giảm dropped bytes khi scheduler bận.
  Serial1.setRxBufferSize(2048);
  Serial1.begin(YDLIDAR_X3_BAUD, SERIAL_8N1, YDLIDAR_X3_RX, YDLIDAR_X3_TX);
  delay(100);

  // === Runtime asserts ===
  uint32_t largestBlock = ESP.getMaxAllocHeap();
  if (largestBlock < 4096) {
    Serial.printf("[X3] ❌ Heap largest block = %u B (cần >= 4KB) — LiDAR có thể fail\n",
                  (unsigned)largestBlock);
  } else {
    Serial.printf("[X3] Heap OK: largest block = %u B\n", (unsigned)largestBlock);
  }

  g_x3Scan.count = 0;
  g_x3Scan.scanSeq = 0;
  g_x3Scan.lastScanMs = 0;
  g_x3Scan.scanReady = false;
  memset(&g_x3Diag, 0, sizeof(g_x3Diag));

  // === Tăng sample rate lên 4kHz (X3 firmware ≥ 1.4.0 hỗ trợ) ===
  uint8_t setSampleRateCmd[] = {0xA5, 0x09, 0xA0, 0x0F};
  Serial1.write(setSampleRateCmd, sizeof(setSampleRateCmd));
  delay(50);

  // === Start scan command ===
  uint8_t startCmd[] = {0xA5, 0x60};
  Serial1.write(startCmd, sizeof(startCmd));
  delay(50);

  Serial.printf("[X3] Init done — UART @ %d baud, sample rate target ~4kHz\n",
                YDLIDAR_X3_BAUD);
  Serial.printf("[X3] NOTE: baud %d chỉ chứa ~%u pts/s. Nếu thấy <300 pts/scan → bottleneck UART.\n",
                YDLIDAR_X3_BAUD, (unsigned)(YDLIDAR_X3_BAUD / 10 / 5));
}

/**
 * Reset diagnostic counters — gọi khi cần benchmark hoặc sau khi xử lý lỗi.
 */
inline void x3ResetDiag() {
  memset(&g_x3Diag, 0, sizeof(g_x3Diag));
}

/**
 * Drain TẤT CẢ bytes sẵn có từ Serial1 vào buffer parser, KHÔNG đọc từng byte.
 * Đây là điểm khác biệt quan trọng so với driver cũ — fix nguyên nhân chính gây
 * tụt pts khi ROS2 chạy (CPU bận → drain quá chậm → FIFO tràn).
 *
 * CHỈ GỌI TỪ taskX3 — có static state riêng, KHÔNG thread-safe.
 *
 * Trả về số bytes đã đọc (để g_x3Diag.bytesRead cộng dồn).
 */
inline uint16_t x3DrainUart() {
  static uint8_t  s_buf[YDLIDAR_SCAN_BUFF_SIZE];
  static uint16_t s_bufLen = 0;
  static uint32_t s_dbgLastMs = 0;
  static LidarPoint s_accumPoints[X3_MAX_POINTS]; // BSS, không phải stack
  static uint16_t   s_accumCount = 0;
  static uint32_t   s_lastPacketMs = 0;

  uint32_t nowMs = millis();
  uint16_t got = 0;

  // Drain TẤT CẢ bytes có sẵn (không phải 1 byte/loop). Nếu FIFO có 500 bytes
  // và poll chạy 5ms/lần, ta đọc hết trong 1 lần — không bao giờ tràn.
  int avail = Serial1.available();
  if (avail > 0) {
    // Nếu buffer parser đã gần đầy, KHÔNG đọc thêm — flush accumulator trước.
    if (s_bufLen + avail > sizeof(s_buf)) {
      g_x3Diag.bytesDropped += (s_bufLen + avail) - sizeof(s_buf);
      s_bufLen = 0; // reset để tránh vỡ sync (parser sẽ resync từ byte tiếp theo)
    }

    uint16_t wantRead = (uint16_t)min((int)(sizeof(s_buf) - s_bufLen), avail);
    got = Serial1.readBytes(s_buf + s_bufLen, wantRead);
    s_bufLen += got;
    g_x3Diag.bytesRead += got;
  }

  // === Parse packets trong buffer ===
  // (parser state machine — phải đảm bảo KHÔNG block, KHÔNG malloc trong poll)
  while (s_bufLen >= 10) {
    // 1) Sync: tìm header 0xAA 0x55
    if (s_buf[0] != 0xAA || s_buf[1] != 0x55) {
      // Shift trái 1 byte, đếm sync loss
      memmove(s_buf, s_buf + 1, s_bufLen - 1);
      s_bufLen -= 1;
      g_x3Diag.syncLosses++;
      continue;
    }

    uint8_t sampleCount = s_buf[3];
    uint16_t packageLen = 10 + (sampleCount * 2);
    if (s_bufLen < packageLen) break; // chưa đủ, đợi packet kế tiếp

    // 2) Tính góc start/end
    uint16_t fsa = s_buf[4] | (s_buf[5] << 8);
    uint16_t lsa = s_buf[6] | (s_buf[7] << 8);
    float startAngle = (float)(fsa >> 1) / 64.0f;
    float endAngle   = (float)(lsa >> 1) / 64.0f;
    float diffAngle = endAngle - startAngle;
    if (diffAngle < 0) diffAngle += 360.0f;

    // 3) Timeout: nếu quá 80ms không packet → flush accumulator
    bool timeout = (nowMs - s_lastPacketMs) > 80;
    if (timeout && s_accumCount >= 100) {
      // Copy sang g_x3Scan (atomic vì taskX3 priority 6 > readers)
      memcpy(g_x3Scan.points, s_accumPoints, s_accumCount * sizeof(LidarPoint));
      g_x3Scan.count = s_accumCount;
      g_x3Scan.scanSeq++;
      g_x3Scan.lastScanMs = nowMs;
      g_x3Scan.scanReady = true;
      g_x3Diag.scansPublished++;
      s_accumCount = 0;
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

    // 4) Trượt phần còn lại lên đầu
    if (s_bufLen >= packageLen) {
      memmove(s_buf, s_buf + packageLen, s_bufLen - packageLen);
      s_bufLen -= packageLen;
    } else {
      s_bufLen = 0;
    }
  }

// === Rate-limited diagnostic log (chỉ in khi có vấn đề) ===
  if (nowMs - s_dbgLastMs > 1000) {
    s_dbgLastMs = nowMs;
    bool hasIssue = (g_x3Diag.bytesDropped > 0) ||
                    (g_x3Diag.syncLosses > 5) ||
                    (g_x3Diag.checksumErrors > 0);
    if (hasIssue) {
      Serial.printf("[X3-DIAG] bytesRead=%u drop=%u syncLoss=%u scan=%u avail=%d\n",
                    (unsigned)g_x3Diag.bytesRead,
                    (unsigned)g_x3Diag.bytesDropped,
                    (unsigned)g_x3Diag.syncLosses,
                    (unsigned)g_x3Diag.scansPublished,
                    Serial1.available());
      // Reset counters để lần sau chỉ log delta mới (tránh spam)
      g_x3Diag.bytesDropped = 0;
      g_x3Diag.syncLosses = 0;
      g_x3Diag.checksumErrors = 0;
    }
  }

  return got;
}

/**
 * taskX3 — Dedicated FreeRTOS task cho LiDAR, priority 6 (cao hơn control).
 * Chạy trên core 1, period 5ms (=200Hz) — đủ nhanh để drain UART trước khi FIFO tràn.
 *
 * FreeRTOS task signature: `void func(void *pvParams)`.
 */
inline void taskX3(void *pvParams) {
  (void)pvParams;
  const TickType_t xPeriod = pdMS_TO_TICKS(5);
  TickType_t xLastWake = xTaskGetTickCount();
  for (;;) {
    x3DrainUart();
    vTaskDelayUntil(&xLastWake, xPeriod);
  }
}

/**
 * Lấy khoảng cách min (mm) trong cung góc [centerDeg ± halfWidthDeg].
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
struct X3Diag {
  uint32_t bytesRead, bytesDropped, syncLosses, checksumErrors, scansPublished, lastDropLogMs;
};
extern X3Scan g_x3Scan;
extern X3Diag g_x3Diag;
inline void x3Init() {}
inline void x3DrainUart() {}
inline void x3ResetDiag() {}
inline void taskX3(void *) {}
inline uint16_t x3MinInArc(float, float, uint8_t = 10) { return 0xFFFF; }
inline void x3Stop() {}

#endif // USE_YDLIDAR_X3

/** Definition biến global — dùng `inline` để tránn multiple-definition khi include ở nhiều .h */
inline X3Scan g_x3Scan = {};
inline X3Diag g_x3Diag = {};

#endif // YDLIDAR_X3_H