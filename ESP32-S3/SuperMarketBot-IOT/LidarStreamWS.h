/* =====================================================================
 *  LidarStreamWS.h — Stream dữ liệu LiDAR thô + Odometry sang Tablet
 *
 *  Mục tiêu: Cầu nối dữ liệu tốc độ cao ESP32 → Tablet Android.
 *  Tablet sẽ nhận luồng này và chạy thuật toán AMCL/Scan Matching
 *  để ước lượng tọa độ (x, y, heading) chính xác hơn Odometry bánh xe.
 *
 *  ⚠️  THIẾT KẾ: File này KHÔNG sửa bất kỳ logic nào của code cũ.
 *      Chỉ đọc dữ liệu từ g_state và g_pose, rồi gửi đi qua WS riêng.
 *
 *  WebSocket Server port: WS_LIDAR_PORT (mặc định: 82)
 *  Endpoint: ws://<ESP_IP>:82
 *
 *  Format dữ liệu gửi (JSON compact):
 *  {
 *    "pts": [[angle0, dist0_mm], [angle1, dist1_mm], ...],
 *    "ox": 1.23,   <- Odometry X (m) từ g_pose.x
 *    "oy": 0.45,   <- Odometry Y (m) từ g_pose.y
 *    "oh": -0.12,  <- Heading (rad) từ g_pose.headingRad
 *    "ts": 123456  <- millis()
 *  }
 *
 *  Tải trọng ước tính: ~5-10 KB/giây (không ảnh hưởng Core 1 điều khiển)
 *  Tần số gửi: LIDAR_STREAM_INTERVAL_MS (100ms = 10 Hz)
 * =====================================================================*/

#ifndef LIDAR_STREAM_WS_H
#define LIDAR_STREAM_WS_H

#include "Config.h"
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "Localization.h"   // g_pose (x, y, headingRad)
#include "ObstacleSensors.h" // obsFrontCm(), obsBackCm()

/* ─── Cấu hình ─────────────────────────────────────────────────── */
#ifndef WS_LIDAR_PORT
  #define WS_LIDAR_PORT 82
#endif

// Gửi frame mỗi 100ms (10 Hz) — đủ mượt cho SLAM trên Tablet
#define LIDAR_STREAM_INTERVAL_MS 100u

// Số điểm LiDAR mô phỏng mỗi vòng (TF-Luna chỉ có 2 trục F/B)
// Ở Phase 1: chỉ xuất 2 điểm thật (0° và 180°) kèm khoảng cách
// Phase 2: Nếu gắn thêm LiDAR quay (RPLIDAR/YDLidar), thay đổi phần này
#define LIDAR_STREAM_NUM_POINTS 2

/* ─── WebSocket Server riêng cho Lidar Stream ──────────────────── */
static WebSocketsServer g_wsLidarServer(WS_LIDAR_PORT);
static bool             g_lidarStreamActive = false;
static uint8_t          g_lidarStreamClientNum = 255;

/* ─── Event handler WebSocket Lidar ────────────────────────────── */
static void onLidarWsEvent(uint8_t num, WStype_t type,
                           uint8_t *payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = g_wsLidarServer.remoteIP(num);
      Serial.printf("[LidarStream] Tablet kết nối từ %d.%d.%d.%d\n",
                    ip[0], ip[1], ip[2], ip[3]);
      g_lidarStreamActive   = true;
      g_lidarStreamClientNum = num;
      break;
    }
    case WStype_DISCONNECTED:
      Serial.println(F("[LidarStream] Tablet ngắt kết nối."));
      g_lidarStreamActive   = false;
      g_lidarStreamClientNum = 255;
      break;
    case WStype_TEXT:
      // Tablet có thể gửi lệnh "ping" để kiểm tra
      if (len > 0 && strncmp((char*)payload, "ping", 4) == 0) {
        g_wsLidarServer.sendTXT(num, "{\"pong\":1}");
      }
      break;
    default:
      break;
  }
}

/* ─── Khởi tạo ─────────────────────────────────────────────────── */
inline void lidarStreamInit() {
  g_wsLidarServer.begin();
  g_wsLidarServer.onEvent(onLidarWsEvent);
  Serial.printf("[LidarStream] WebSocket server bắt đầu trên port %d\n",
                WS_LIDAR_PORT);
}

/* ─── Gọi từ loop() hoặc taskNetwork ──────────────────────────── */
/**
 * lidarStreamLoop() — Gọi trong vòng lặp chính (taskNetwork / loop())
 *
 * Thực hiện 2 việc:
 * 1. Poll WebSocket để xử lý kết nối/ngắt kết nối
 * 2. Gửi frame LiDAR + Pose mỗi LIDAR_STREAM_INTERVAL_MS (nếu có Tablet kết nối)
 *
 * Frame JSON ví dụ (Phase 1 với TF-Luna 2 trục):
 * {"pts":[[0,150],[180,320]],"ox":1.23,"oy":0.45,"oh":-0.12,"ts":123456}
 * → Angle 0° = phía trước, 180° = phía sau
 * → dist đơn vị mm
 */
inline void lidarStreamLoop() {
  // Poll WebSocket events
  g_wsLidarServer.loop();

  // Không có Tablet kết nối → bỏ qua
  if (!g_lidarStreamActive) return;

  static uint32_t lastSentMs = 0;
  uint32_t now = millis();
  if (now - lastSentMs < LIDAR_STREAM_INTERVAL_MS) return;
  lastSentMs = now;

  /* ── Lấy dữ liệu từ g_state (không gây race vì chỉ đọc) ─────── */
  // TF-Luna: obsFrontCm() và obsBackCm() trả về cm (int16_t)
  // Nhân 10 để ra mm — cần cast sang int32_t trước để tránh tràn int16_t
  int32_t frontMm = (int32_t)obsFrontCm() * 10;
  int32_t backMm  = (int32_t)obsBackCm()  * 10;

  // Clamp: TF-Luna đo tối đa 800cm = 8000mm
  if (frontMm < 0 || frontMm > 8000) frontMm = 8000;
  if (backMm  < 0 || backMm  > 8000) backMm  = 8000;

  /* ── Odometry từ g_pose (Localization.h) ──────────────────────── */
  // g_pose được cập nhật bởi taskControl (Core 1)
  // Đọc ở đây (Core 0/network task) là OK vì float read là atomic trên ESP32
  float ox = g_pose.x;
  float oy = g_pose.y;
  float oh = g_pose.headingRad;

  /* ── Đóng gói JSON compact ──────────────────────────────────────
   *
   * Phase 1: Chỉ có 2 điểm (trước 0° và sau 180°).
   * Phase 2: Khi gắn RPLIDAR/YDLidar (360 điểm), thay mảng pts[]
   *          bằng vòng for đọc dữ liệu từ LiDAR quay tại đây.
   *
   * Dùng thủ công sprintf thay vì ArduinoJson để cực nhanh và tiết kiệm RAM.
   * ──────────────────────────────────────────────────────────────── */
  char buf[200];
  int n = snprintf(buf, sizeof(buf),
    "{\"pts\":[[0,%ld],[180,%ld]],\"ox\":%.3f,\"oy\":%.3f,\"oh\":%.4f,\"ts\":%lu}",
    (long)frontMm, (long)backMm,
    ox, oy, oh,
    (unsigned long)now
  );

  if (n > 0 && n < (int)sizeof(buf)) {
    g_wsLidarServer.sendTXT(g_lidarStreamClientNum, buf);
  }
}

/* ─── Đăng ký HTTP /lidar_info vào WebServer hiện có ───────────────
 * ✘  KHÔNG dùng `extern WebServer g_httpServer` vì nó được khai báo là `static`
 *    trong WebUI.h, khai báo extern sẽ gây linker error.
 * ✔  Thay bằng cách truyền tham chiếu WebServer vào hàm này.
 */
inline void lidarStreamRegisterHttpEndpoint(WebServer &httpServer) {
  httpServer.on("/lidar_info", HTTP_GET, [&httpServer]() {
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    char resp[128];
    snprintf(resp, sizeof(resp),
      "{\"wsPort\":%d,\"intervalMs\":%d,\"numPoints\":%d}",
      WS_LIDAR_PORT, LIDAR_STREAM_INTERVAL_MS, LIDAR_STREAM_NUM_POINTS);
    httpServer.send(200, "application/json", resp);
  });

  Serial.println(F("[LidarStream] HTTP /lidar_info đăng ký thành công."));
}

#endif // LIDAR_STREAM_WS_H
