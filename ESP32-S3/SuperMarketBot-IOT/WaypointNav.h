/* =====================================================================
 *  WaypointNav.h — Điều hướng waypoint + Local Obstacle Avoidance
 *
 *  Thuật toán chính: Pure Pursuit simplified
 *    1. Tính góc α = atan2(dy, dx) − heading → steer
 *    2. Khi dist < ARRIVE_THRESH_M → waypoint kế tiếp
 *    3. Hết route → DONE → dừng, về MODE_MANUAL (không tự chạy Auto)
 *
 *  Phase 3.5: LocalObstacleAvoid.h (cùng logic bước 2 Auto)
 *    Quét 2 bên → cần ≥ PATH_CLEAR_MIN_CM (1m) mới lách → vượt → Pure Pursuit
 *    Thất bại → WP_OBSTACLE_HOLD → reroute_needed (MQTT)
 *
 *  Safety hardstop: LiDAR < AUTO_LIDAR_BLOCK_CM → dừng ngay trong MỌI state
 *
 *  API:
 *    wpNavSetRoute(pts, count)  — Đặt danh sách waypoint mới (chỉ lưu, chưa chạy)
 *    wpNavStart()               — Bắt đầu chạy (gọi SAU wpNavSetRoute)
 *    wpNavStop()                — Dừng, về MODE_MANUAL
 *    wpNavCancel()              — Hủy route (dùng cho auto-dock)
 *    wpNavTick()                — Gọi mỗi SAFE_LOOP_MS từ taskControl
 *    wpNavIsActive()            — true khi đang dẫn đường
 *    wpNavIsDone()              — true khi hoàn thành
 *    g_wpStatus                 — Chuỗi trạng thái (MQTT telemetry)
 * =====================================================================*/
#ifndef WAYPOINT_NAV_H
#define WAYPOINT_NAV_H

#include "Config.h"
#include "Localization.h"
#include "LocalObstacleAvoid.h"
#include "ObstacleSensors.h"
#include "PidController.h"
#include "Motors.h"
#include <math.h>

/* Forward declaration — tránh circular include với MqttClient.h */
extern volatile bool g_mqttStatusPending;
extern volatile char g_mqttPendingStatus[32];

/* ==================== Tham số điều hướng =========================== */
#define WP_ARRIVE_THRESH_M    0.12f   // Ngưỡng "đến nơi" (m)
#define WP_STEER_K            55.f    // Hệ số steer Pure Pursuit
#define WP_CRUISE_SPEED_PCT   (ROBOT_HEAVY_LOAD ? 28 : 45)
#define WP_SLOW_RADIUS_M      0.4f    // Bán kính giảm tốc (m)
#define WP_SLOW_SPEED_PCT     25      // Tốc độ khi gần waypoint
#define WP_MAX_STEER_RAD      1.2f    // ~70° — quá lệch thì xoay tại chỗ trước
#define WP_TIMEOUT_MS         20000u  // Timeout mỗi waypoint (ms)
#define WP_MAX_WAYPOINTS      32      // Số waypoint tối đa trong 1 route

/* ==================== Cấu trúc ===================================== */
struct Waypoint {
  float x;      // mét
  float y;      // mét
  int   nodeId; // node ID backend (−1 = không rõ)
};

enum WpFsmState : uint8_t {
  WP_IDLE = 0,
  WP_ROUTE_SET,            // Vào mode Tự hành rồi nhưng chưa có lộ trình — chờ backend gửi navigate
  WP_NAVIGATING,           // Pure Pursuit đang chạy
  WP_OBSTACLE_HOLD,       // Không lách được → chờ / reroute MQTT
  WP_DONE,
  WP_ABORTED
};

/* ==================== Biến nội bộ ================================== */
static Waypoint   s_wpRoute[WP_MAX_WAYPOINTS];
static uint8_t    s_wpCount = 0;
uint8_t           s_wpIndex = 0;          // expose để MqttClient đọc
static WpFsmState s_wpFsm   = WP_IDLE;
static uint32_t   s_wpT0    = 0;          // Millis khi bắt đầu waypoint hiện tại

/* Obstacle-hold fallback */
static uint32_t   s_wpObstHoldStart = 0;
static OaContext  s_wpOa;

/* Status string cho MQTT telemetry */
char g_wpStatus[32] = "idle";

/* ==================== Helpers ===================================== */
static inline float wpNormalizeAngle(float a) {
  while (a >  (float)M_PI) a -= 2.f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.f * (float)M_PI;
  return a;
}

static inline float wpAngleDiff(float target, float current) {
  return wpNormalizeAngle(target - current);
}

/** Đặt trạng thái FSM và cập nhật g_wpStatus */
static inline void wpSetState(WpFsmState st, uint32_t now, const char *statusStr) {
  s_wpFsm = st;
  s_wpT0  = now;
  strncpy(g_wpStatus, statusStr, sizeof(g_wpStatus) - 1);
  g_wpStatus[sizeof(g_wpStatus) - 1] = '\0';
}

/** Tính khoảng cách đến waypoint hiện tại */
static inline float wpDistToTarget() {
  if (s_wpIndex >= s_wpCount) return 0.f;
  float dx = s_wpRoute[s_wpIndex].x - g_pose.x;
  float dy = s_wpRoute[s_wpIndex].y - g_pose.y;
  return sqrtf(dx * dx + dy * dy);
}

/** PWM từ phần trăm */
static inline uint16_t wpPct2Pwm(uint8_t pct) {
  return (uint16_t)((uint32_t)PWM_MAX * (uint32_t)pct / 100u);
}

/* ==================== API ========================================= */
inline void wpNavSetRoute(const Waypoint *pts, uint8_t count) {
  if (count > WP_MAX_WAYPOINTS) count = WP_MAX_WAYPOINTS;
  for (uint8_t i = 0; i < count; i++) s_wpRoute[i] = pts[i];
  s_wpCount  = count;
  s_wpIndex  = 0;
  oaReset(s_wpOa);
  /* KHÔNG đổi FSM — chỉ lưu lộ trình, chờ wpNavStart() */
  strncpy(g_wpStatus, "route_set", sizeof(g_wpStatus) - 1);
  Serial.printf("[WP] Route set: %d waypoints\n", (int)count);
}

/**
 * Bắt đầu điều hướng — gọi sau khi backend gửi lệnh navigate.
 * Chỉ chạy khi đang ở WP_ROUTE_SET (tức đã bật mode Tự hành).
 */
inline void wpNavStart() {
  if (s_wpCount == 0) {
    Serial.println(F("[WP] No waypoints — ignore start."));
    return;
  }
  if (s_wpFsm != WP_ROUTE_SET) {
    Serial.println(F("[WP] Not waiting for route (WP_ROUTE_SET)."));
    return;
  }
  s_wpIndex    = 0;
  oaReset(s_wpOa);
  s_wpFsm      = WP_NAVIGATING;
  s_wpT0       = millis();
  pidSpeedReset();
  pidYawReset();
  pidHoldReset();
  strncpy(g_wpStatus, "navigating", sizeof(g_wpStatus) - 1);
  Serial.printf("[WP] Started → WP[0] (%.3f, %.3f)\n",
                s_wpRoute[0].x, s_wpRoute[0].y);
}

inline void wpNavStop() {
  s_wpFsm      = WP_ABORTED;
  g_state.mode = MODE_MANUAL;
  botStop();
  strncpy(g_wpStatus, "aborted", sizeof(g_wpStatus) - 1);
}

/** Hủy route và về IDLE — dùng cho auto-dock hoặc lệnh cancel */
inline void wpNavCancel() {
  s_wpFsm      = WP_IDLE;
  s_wpCount    = 0;
  s_wpIndex    = 0;
  oaReset(s_wpOa);
  g_state.mode = MODE_MANUAL;
  botStop();
  pidSpeedReset();
  strncpy(g_wpStatus, "cancelled", sizeof(g_wpStatus) - 1);
  Serial.println(F("[WP] Route cancelled."));
}

inline bool wpNavIsActive() {
  return (s_wpFsm == WP_NAVIGATING || s_wpFsm == WP_OBSTACLE_HOLD);
}

inline bool wpNavIsDone() { return (s_wpFsm == WP_DONE); }

/** Đang quét/lách — cần đọc LiDAR dày hơn trong taskControl */
inline bool wpNavOaActive() {
  return (s_wpFsm == WP_NAVIGATING && s_wpOa.state != OA_IDLE);
}

/* ==================== TICK ======================================== */
inline void wpNavTick() {
  if (s_wpFsm == WP_IDLE || s_wpFsm == WP_DONE || s_wpFsm == WP_ABORTED) return;

  /* WP_ROUTE_SET: chờ lộ trình từ backend — chỉ dừng motor, không chạy Pure Pursuit */
  if (s_wpFsm == WP_ROUTE_SET) {
    botStop();
    return;
  }

  const uint32_t now   = millis();
  const int16_t  fCm   = obsFrontCm();
  const int16_t  bCm   = obsBackCm();

  /* ── Safety hardstop — ưu tiên tuyệt đối (mọi state) ───────── */
  const bool hardFront = obsFrontBlocked();
  const bool hardRear  = obsRearBlocked();

  if (hardFront || hardRear) {
    /* Không can thiệp FSM — chỉ dừng motor, FSM tự xử lý ở state tương ứng */
    botStop();
    /* Trong state OA đang xoay → hardstop nhưng không reset FSM,
       tick tiếp theo sẽ detect dist và chuyển state đúng */
  }

  /* ── Local OA (dùng chung Auto bước 2) — chạy trước switch ─────── */
  if (s_wpFsm == WP_NAVIGATING && s_wpOa.state != OA_IDLE) {
    strncpy(g_wpStatus, "oa_active", sizeof(g_wpStatus) - 1);
    OaTickResult r = oaTick(s_wpOa, fCm, now);
    if (r == OA_RES_DONE) {
      s_wpT0 = now;
      strncpy(g_wpStatus, "navigating", sizeof(g_wpStatus) - 1);
      Serial.println(F("[WP] OA done — resume Pure Pursuit."));
    } else if (r == OA_RES_BLOCKED) {
      wpSetState(WP_OBSTACLE_HOLD, now, "blocked");
      s_wpObstHoldStart = now;
      Serial.println(F("[WP] OA blocked — hold / reroute."));
    }
    return;
  }

  switch (s_wpFsm) {

  /* ═══════════════════════════════════════════════════════════════
   *  WP_NAVIGATING — Pure Pursuit: steer + drive về waypoint
   * ══════════════════════════════════════════════════════════════ */
  case WP_NAVIGATING: {
    if (s_wpIndex >= s_wpCount) {
      /* Hết route → DONE */
      s_wpFsm      = WP_DONE;
      g_state.mode = MODE_MANUAL;
      botStop();
      strncpy(g_wpStatus, "done", sizeof(g_wpStatus) - 1);
      strncpy((char *)g_mqttPendingStatus, "wp_done", sizeof(g_mqttPendingStatus) - 1);
      g_mqttStatusPending = true;
      Serial.println(F("[WP] Route complete — ve Manual."));
      return;
    }

    /* Timeout waypoint */
    if (now - s_wpT0 >= WP_TIMEOUT_MS) {
      Serial.printf("[WP] Timeout WP[%d] — skip\n", (int)s_wpIndex);
      s_wpIndex++;
      oaReset(s_wpOa);
      s_wpT0 = now;
      return;
    }

    float tx   = s_wpRoute[s_wpIndex].x;
    float ty   = s_wpRoute[s_wpIndex].y;
    float dx   = tx - g_pose.x;
    float dy   = ty - g_pose.y;
    float dist = sqrtf(dx * dx + dy * dy);

    /* Đến nơi */
    if (dist < WP_ARRIVE_THRESH_M) {
      Serial.printf("[WP] WP[%d] reached (%.2f,%.2f) dist=%.3fm\n",
                    (int)s_wpIndex, tx, ty, dist);
      s_wpIndex++;
      oaReset(s_wpOa);
      s_wpT0 = now;
      pidSpeedReset();
      if (s_wpIndex >= s_wpCount) {
        botStop();
        s_wpFsm      = WP_DONE;
        g_state.mode = MODE_MANUAL;
        botStop();
        strncpy(g_wpStatus, "done", sizeof(g_wpStatus) - 1);
        strncpy((char *)g_mqttPendingStatus, "wp_done", sizeof(g_mqttPendingStatus) - 1);
        g_mqttStatusPending = true;
        Serial.println(F("[WP] All waypoints reached — ve Manual."));
      }
      return;
    }

    /* Gặp vật → quét/lách (≥1m mới coi bên trống) */
    if (obsOaTriggered(fCm)) {
      if (oaBegin(s_wpOa, fCm, now)) {
        if (s_wpOa.state == OA_BLOCKED) {
          wpSetState(WP_OBSTACLE_HOLD, now, "blocked");
          s_wpObstHoldStart = now;
        }
        return;
      }
    }

    /* Chưa đủ 1m phía trước → dừng, không lao vào vật (LiDAR cao hay đọc thấp) */
    if (!obsPathClear(fCm)) {
      botStop();
      return;
    }

    /* ── Pure Pursuit steering + heading hold (encoder "dò line ảo") ── */
    float targetH = atan2f(dy, dx);
    float alpha    = wpAngleDiff(targetH, g_pose.headingRad);

    /* Quá lệch → xoay tại chỗ */
    if (fabsf(alpha) > WP_MAX_STEER_RAD) {
      uint16_t turnPwm = wpPct2Pwm(30);
      if (alpha > 0) { botRotateCW(turnPwm); }
      else           { botRotateCCW(turnPwm); }
      return;
    }

    /* Pure Pursuit steer (góc lệch từ hướng robot tới waypoint) */
    int16_t ppSteer = (int16_t)(WP_STEER_K * alpha);
    if (ppSteer >  100) ppSteer =  100;
    if (ppSteer < -100) ppSteer = -100;

    /* Heading hold: bù drift encoder để đi thẳng chính xác hơn */
    float dt_s     = (float)SAFE_LOOP_MS * 0.001f;
    float holdCorr = pidHoldCompute(targetH, g_pose.headingRad, dt_s);

    /* Speed: giảm khi gần waypoint */
    uint16_t spd = (dist < WP_SLOW_RADIUS_M)
                 ? wpPct2Pwm(WP_SLOW_SPEED_PCT)
                 : wpPct2Pwm(WP_CRUISE_SPEED_PCT);

    /* Speed PID */
    float pidOut  = pidSpeedCompute(pwmToEstMps(spd), robotActualSpeedMps(), dt_s);
    int32_t runPwm = (int32_t)spd + (int32_t)pidOut;
    if (runPwm < 0) runPwm = 0;
    if (runPwm > (int32_t)PWM_MAX) runPwm = (int32_t)PWM_MAX;

    /* Tổng steer: Pure Pursuit + heading hold */
    int16_t turn = ppSteer + (int16_t)holdCorr;
    if (turn >  100) turn =  100;
    if (turn < -100) turn = -100;

    /* Mecanum: strafe=0, fwd=80%, turn=correction */
    botDriveMecanum(0, 80, turn, (uint16_t)runPwm);
    break;
  }

  /* ═══════════════════════════════════════════════════════════════
   *  WP_OBSTACLE_HOLD — Fallback: chờ OA_FALLBACK_WAIT_MS
   *                     rồi publish "reroute_needed" → Backend gửi route mới
   * ══════════════════════════════════════════════════════════════ */
  case WP_OBSTACLE_HOLD: {
    botStop();

    /* Chỉ resume khi phía trước ≥ 1m (PATH_CLEAR), không phải ~70cm */
    const bool pathClear = obsPathClear(fCm);
    if (pathClear) {
      pidSpeedReset();
      oaReset(s_wpOa);
      wpSetState(WP_NAVIGATING, now, "navigating");
      Serial.println(F("[WP] Duong truoc du xa — resuming."));
      return;
    }

    if (now - s_wpObstHoldStart >= OA_FALLBACK_WAIT_MS) {
      /* Timeout → yêu cầu reroute từ backend */
      strncpy(g_wpStatus, "reroute_needed", sizeof(g_wpStatus) - 1);
      strncpy((char *)g_mqttPendingStatus, "reroute_needed",
              sizeof(g_mqttPendingStatus) - 1);
      g_mqttStatusPending = true;
      /* Reset hold timer — tránh spam publish */
      s_wpObstHoldStart = now;
      Serial.printf("[WP] Reroute requested (WP[%d] blocked)\n", (int)s_wpIndex);
    }
    break;
  }

  default:
    break;
  }
}

/* ==================== Parse MQTT navigate payload ================= */
/**
 * Parse JSON waypoints từ Backend — CHỈ lưu lộ trình, KHÔNG bắt đầu chạy.
 * Gọi wpNavStart() riêng sau khi xác nhận đang ở WP_ROUTE_SET.
 *
 * Format JSON từ Backend:
 *   {"waypoints":[{"x":1.2,"y":0.5,"nodeId":3}, ...]}
 *   hoặc {"waypoints":[nodeId1, nodeId2, ...]}  (dùng fake coord — test only)
 */
inline bool wpNavParseAndStart(const char *jsonPayload) {
  StaticJsonDocument<1024> doc;
  if (deserializeJson(doc, jsonPayload) != DeserializationError::Ok) {
    Serial.println(F("[WP] JSON parse error"));
    return false;
  }

  JsonArray arr = doc["waypoints"].as<JsonArray>();
  if (!arr || arr.size() == 0) {
    Serial.println(F("[WP] No waypoints in payload"));
    return false;
  }

  uint8_t count = 0;
  static Waypoint pts[WP_MAX_WAYPOINTS];

  for (JsonVariant v : arr) {
    if (count >= WP_MAX_WAYPOINTS) break;
    if (v.is<JsonObject>()) {
      pts[count].x      = v["x"]      | 0.0f;
      pts[count].y      = v["y"]      | 0.0f;
      pts[count].nodeId = v["nodeId"] | -1;
    } else {
      /* Chỉ có nodeId — fake coord (test) */
      pts[count].x      = 0.f;
      pts[count].y      = count * 0.5f;
      pts[count].nodeId = v.as<int>();
    }
    count++;
  }

  if (count == 0) return false;

  wpNavSetRoute(pts, count);
  return true;
}

#endif // WAYPOINT_NAV_H
