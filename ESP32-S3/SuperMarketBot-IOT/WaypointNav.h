/* =====================================================================
 *  WaypointNav.h — Điều hướng theo danh sách waypoint (x, y) [mét]
 *
 *  Thuật toán: Pure Pursuit simplified
 *    1. Tìm waypoint mục tiêu hiện tại
 *    2. Tính góc α = atan2(dy, dx) − heading
 *    3. cmdX ≈ K·α → steer robot; cmdY = tiến
 *    4. Khi dist < ARRIVE_THRESH_M → chuyển sang waypoint kế tiếp
 *    5. Hết danh sách → DONE → back về MODE_AUTO
 *
 *  Tích hợp FSM tự hành:
 *    - MODE_WAYPOINT: WaypointNav active, LiDAR obstacle FSM vẫn chạy
 *    - Khi gặp chướng ngại vật: tạm suspend WP, chạy obstacle FSM,
 *      sau khi clear → resume WP
 *
 *  API (gọi từ taskControl / .ino):
 *    wpNavSetRoute(waypoints, count)  — Set danh sách waypoint mới
 *    wpNavStart()                     — Bắt đầu (set MODE_WAYPOINT)
 *    wpNavStop()                      — Dừng, về MODE_MANUAL
 *    wpNavTick()                      — Gọi mỗi SAFE_LOOP_MS trong taskControl
 *    wpNavIsActive()                  — true khi đang dẫn đường
 *    wpNavIsDone()                    — true khi hoàn thành route
 *    g_wpStatus                       — Chuỗi trạng thái ("idle"/"navigating"/"done"/"blocked")
 * =====================================================================*/
#ifndef WAYPOINT_NAV_H
#define WAYPOINT_NAV_H

#include "Config.h"
#include "Localization.h"
#include "PidController.h"
#include "Motors.h"
#include <math.h>

/* Forward declaration — tránh circular include với MqttClient.h */
extern volatile bool g_mqttStatusPending;
extern volatile char g_mqttPendingStatus[32];

/* ==================== Tham số điều hướng =========================== */
/** Khoảng cách ngưỡng "đến nơi" mỗi waypoint (m). */
#define WP_ARRIVE_THRESH_M    0.12f
/** Hệ số steer (cmdX = K·α·scale) — chỉnh thực địa. */
#define WP_STEER_K            55.f
/** Tốc độ tiến khi điều hướng (% PWM_MAX) */
#define WP_CRUISE_SPEED_PCT   45
/** Tốc độ khi gần waypoint (m) — giảm tốc */
#define WP_SLOW_RADIUS_M      0.4f
#define WP_SLOW_SPEED_PCT     25
/** Giới hạn góc lệch để tiến (radian) — quá lệch thì quay tại chỗ trước */
#define WP_MAX_STEER_RAD      1.2f   // ~70°
/** Timeout mỗi waypoint (ms) — nếu quá lâu không tới → skip hoặc stop */
#define WP_TIMEOUT_MS         20000u
/** Số waypoint tối đa trong 1 route */
#define WP_MAX_WAYPOINTS      32

/* ==================== Cấu trúc ===================================== */
struct Waypoint {
  float x;   // mét
  float y;   // mét
  int   nodeId; // node ID backend (−1 = không rõ)
};

enum WpFsmState : uint8_t {
  WP_IDLE = 0,
  WP_NAVIGATING,
  WP_OBSTACLE_HOLD,  // Tạm dừng vì LiDAR chặn
  WP_DONE,
  WP_ABORTED
};

/* ==================== Biến nội bộ ================================== */
static Waypoint   s_wpRoute[WP_MAX_WAYPOINTS];
static uint8_t    s_wpCount = 0;
uint8_t           s_wpIndex = 0;   // expose để MqttClient đọc wpIndex
static WpFsmState s_wpFsm   = WP_IDLE;
static uint32_t   s_wpT0    = 0;   // Millis khi bắt đầu waypoint hiện tại

/* Obstacle-hold timer */
static uint32_t   s_wpObstHoldMs = 0;
#define WP_OBST_HOLD_MS  3000u   // Chờ tối đa 3s, rồi skip WP nếu vẫn bị chặn

/* Status string cho MQTT */
char g_wpStatus[32] = "idle";

/* ==================== Helpers ===================================== */
static float wpAngleDiff(float target, float current) {
  float d = target - current;
  while (d >  (float)M_PI) d -= 2.f * (float)M_PI;
  while (d < -(float)M_PI) d += 2.f * (float)M_PI;
  return d;
}

/* ==================== API ========================================= */
inline void wpNavSetRoute(const Waypoint *pts, uint8_t count) {
  if (count > WP_MAX_WAYPOINTS) count = WP_MAX_WAYPOINTS;
  for (uint8_t i = 0; i < count; i++) s_wpRoute[i] = pts[i];
  s_wpCount = count;
  s_wpIndex = 0;
  s_wpFsm   = WP_IDLE;
  strncpy(g_wpStatus, "route_set", sizeof(g_wpStatus) - 1);
  Serial.printf("[WP] Route set: %d waypoints\n", (int)count);
}

inline void wpNavStart() {
  if (s_wpCount == 0) {
    Serial.println(F("[WP] No waypoints — abort"));
    return;
  }
  s_wpIndex = 0;
  s_wpFsm   = WP_NAVIGATING;
  s_wpT0    = millis();
  g_state.mode = MODE_WAYPOINT;
  pidSpeedReset();
  pidYawReset();
  strncpy(g_wpStatus, "navigating", sizeof(g_wpStatus) - 1);
  Serial.printf("[WP] Start → WP[0] (%.3f, %.3f)\n",
                s_wpRoute[0].x, s_wpRoute[0].y);
}

inline void wpNavStop() {
  s_wpFsm = WP_ABORTED;
  g_state.mode = MODE_MANUAL;
  botStop();
  strncpy(g_wpStatus, "aborted", sizeof(g_wpStatus) - 1);
}

inline bool wpNavIsActive() {
  return (s_wpFsm == WP_NAVIGATING || s_wpFsm == WP_OBSTACLE_HOLD);
}

inline bool wpNavIsDone() {
  return (s_wpFsm == WP_DONE);
}

/* ==================== TICK (gọi từ taskControl mỗi SAFE_LOOP_MS) == */
inline void wpNavTick() {
  if (s_wpFsm == WP_IDLE || s_wpFsm == WP_DONE || s_wpFsm == WP_ABORTED) return;

  const uint32_t now = millis();
  const int16_t  f   = g_state.lidarFront;
  const int16_t  b   = g_state.lidarBack;

  /* ── Obstacle detection (LiDAR trước/sau) ─────────────────────── */
  const bool obstFront = (f < AUTO_LIDAR_BLOCK_CM) && (f > 3);
  const bool obstRear  = (AUTO_LIDAR_BLOCK_USE_REAR != 0) && (b < AUTO_LIDAR_BLOCK_CM) && (b > 3);

  if (s_wpFsm == WP_NAVIGATING && (obstFront || obstRear)) {
    botStop();
    s_wpFsm = WP_OBSTACLE_HOLD;
    s_wpObstHoldMs = now;
    strncpy(g_wpStatus, "blocked", sizeof(g_wpStatus) - 1);
    Serial.println(F("[WP] Obstacle! Hold."));
    return;
  }

  if (s_wpFsm == WP_OBSTACLE_HOLD) {
    if (!obstFront && !obstRear) {
      /* Clear — resume */
      s_wpFsm = WP_NAVIGATING;
      s_wpT0  = now; // reset timeout
      pidSpeedReset();
      strncpy(g_wpStatus, "navigating", sizeof(g_wpStatus) - 1);
      Serial.println(F("[WP] Clear — resume."));
    } else if (now - s_wpObstHoldMs >= WP_OBST_HOLD_MS) {
      /* Timeout hold → skip waypoint */
      s_wpIndex++;
      Serial.printf("[WP] Obstacle timeout — skip to WP[%d]\n", (int)s_wpIndex);
      if (s_wpIndex >= s_wpCount) {
        s_wpFsm = WP_DONE;
        g_state.mode = MODE_AUTO;
        botStop();
        strncpy(g_wpStatus, "done", sizeof(g_wpStatus) - 1);
        Serial.println(F("[WP] Route done (last WP skipped due to obstacle)."));
        return;
      }
      s_wpFsm = WP_NAVIGATING;
      s_wpT0  = now;
    }
    return;
  }

  /* ── Navigate ─────────────────────────────────────────────────── */
  if (s_wpIndex >= s_wpCount) {
    s_wpFsm = WP_DONE;
    g_state.mode = MODE_AUTO;
    botStop();
    strncpy(g_wpStatus, "done", sizeof(g_wpStatus) - 1);
    Serial.println(F("[WP] All waypoints reached!"));
    /* Thông báo MQTT qua flag */
    strncpy((char *)g_mqttPendingStatus, "wp_done", sizeof(g_mqttPendingStatus) - 1);
    g_mqttStatusPending = true;
    return;
  }

  /* Timeout waypoint hiện tại */
  if (now - s_wpT0 >= WP_TIMEOUT_MS) {
    Serial.printf("[WP] Timeout WP[%d] — skip\n", (int)s_wpIndex);
    s_wpIndex++;
    s_wpT0 = now;
    return;
  }

  float tx = s_wpRoute[s_wpIndex].x;
  float ty = s_wpRoute[s_wpIndex].y;
  float dx = tx - g_pose.x;
  float dy = ty - g_pose.y;
  float dist = sqrtf(dx * dx + dy * dy);

  /* Đến nơi */
  if (dist < WP_ARRIVE_THRESH_M) {
    Serial.printf("[WP] WP[%d] reached! (%.3f, %.3f) dist=%.3fm\n",
                  (int)s_wpIndex, tx, ty, dist);
    s_wpIndex++;
    s_wpT0 = now;
    pidSpeedReset();
    if (s_wpIndex >= s_wpCount) {
      s_wpFsm = WP_DONE;
      g_state.mode = MODE_AUTO;
      botStop();
      strncpy(g_wpStatus, "done", sizeof(g_wpStatus) - 1);
      strncpy((char *)g_mqttPendingStatus, "wp_done", sizeof(g_mqttPendingStatus) - 1);
      g_mqttStatusPending = true;
      Serial.println(F("[WP] Route complete!"));
    }
    return;
  }

  /* Tính góc đến target */
  float targetHeading = atan2f(dy, dx);
  float alpha = wpAngleDiff(targetHeading, g_pose.headingRad);

  /* Nếu quá lệch — xoay tại chỗ trước */
  if (fabsf(alpha) > WP_MAX_STEER_RAD) {
    int dir = (alpha > 0) ? 1 : -1;
    uint16_t turnPwm = (uint16_t)((uint32_t)PWM_MAX * 30u / 100u);
    if (dir > 0) { botRotateCW(turnPwm); }
    else         { botRotateCCW(turnPwm); }
    return;
  }

  /* Steer: cmdX ∝ α */
  int16_t steer = (int16_t)(WP_STEER_K * alpha);
  if (steer >  100) steer =  100;
  if (steer < -100) steer = -100;

  /* Speed: giảm khi gần */
  uint16_t spd;
  if (dist < WP_SLOW_RADIUS_M) {
    spd = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)WP_SLOW_SPEED_PCT / 100u);
  } else {
    spd = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)WP_CRUISE_SPEED_PCT / 100u);
  }

  /* PID speed */
  float dt_s = (float)SAFE_LOOP_MS * 0.001f;
  float targetMps = pwmToEstMps(spd);
  float actualMps = robotActualSpeedMps();
  float pidOut = pidSpeedCompute(targetMps, actualMps, dt_s);
  int32_t runPwm = (int32_t)spd + (int32_t)pidOut;
  if (runPwm < 0) runPwm = 0;
  if (runPwm > (int32_t)PWM_MAX) runPwm = (int32_t)PWM_MAX;

  /* Drive */
  botDrive(steer, 80, (uint16_t)runPwm);
}

/* ==================== Parse MQTT navigate payload ================= */
/**
 * Parse JSON payload từ command "navigate":
 *   {"waypoints":["nodeId1","nodeId2",...]}
 *   hoặc {"waypoints":[{"x":1.2,"y":0.5,"nodeId":3},...]}
 *
 * Phase 3 đơn giản: backend gửi theo format {"waypoints":[id1,id2,...]}
 * + node coord sẽ được lookup từ bản đồ node trên robot (Phase 3.5 sau).
 * Hiện tại dùng fake coord (0, index * 0.5m) để test navigation flow.
 */
inline bool wpNavParseAndStart(const char *jsonPayload) {
  /* Dùng ArduinoJson để parse */
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, jsonPayload);
  if (err) {
    Serial.printf("[WP] JSON parse error: %s\n", err.c_str());
    return false;
  }

  JsonArray arr = doc["waypoints"].as<JsonArray>();
  if (!arr) {
    Serial.println(F("[WP] No 'waypoints' array in payload"));
    return false;
  }

  uint8_t count = 0;
  static Waypoint pts[WP_MAX_WAYPOINTS];

  for (JsonVariant v : arr) {
    if (count >= WP_MAX_WAYPOINTS) break;
    if (v.is<JsonObject>()) {
      /* {"x":1.2,"y":0.5,"nodeId":3} */
      pts[count].x      = v["x"] | 0.0f;
      pts[count].y      = v["y"] | 0.0f;
      pts[count].nodeId = v["nodeId"] | -1;
    } else {
      /* Chỉ có nodeId — dùng fake coord theo thứ tự */
      pts[count].x      = 0.f;
      pts[count].y      = count * 0.5f; // 0.5m giữa các node (placeholder)
      pts[count].nodeId = v.as<int>();
    }
    count++;
  }

  if (count == 0) {
    Serial.println(F("[WP] Empty waypoints — abort"));
    return false;
  }

  wpNavSetRoute(pts, count);
  wpNavStart();
  return true;
}

#endif // WAYPOINT_NAV_H
