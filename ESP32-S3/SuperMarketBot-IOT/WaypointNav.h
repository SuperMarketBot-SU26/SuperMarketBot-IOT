/* =====================================================================
 *  WaypointNav.h — Điều hướng waypoint + Local Obstacle Avoidance
 *
 *  Thuật toán chính: Pure Pursuit simplified
 *    1. Tính góc α = atan2(dy, dx) − heading → steer
 *    2. Khi dist < ARRIVE_THRESH_M → waypoint kế tiếp
 *    3. Hết route → DONE → back về MODE_AUTO
 *
 *  Phase 3.5: Local Obstacle Avoidance
 *    Khi phát hiện vật cản < OA_DETECT_CM trong WP_NAVIGATING:
 *      1. WP_OBS_SCAN_CW  → xoay CW ±50° quét bên phải (đọc LiDAR trước)
 *      2. WP_OBS_SCAN_CCW → xoay CCW ±50° quét bên trái
 *      3. Chọn bên trống (> OA_CLEAR_MIN_CM):
 *         → WP_OBS_SWERVE: lái chéo 40cm sang bên trống
 *         → WP_OBS_PASS:   đi thẳng 50cm (song song route gốc)
 *         → WP_NAVIGATING: Pure Pursuit tự kéo về route ✅
 *      4. Nếu cả 2 bên chật hoặc thử OA_MAX_ATTEMPTS lần thất bại:
 *         → WP_OBSTACLE_HOLD: chờ OA_FALLBACK_WAIT_MS
 *         → publish "reroute_needed" qua MQTT
 *
 *  Safety hardstop: LiDAR < AUTO_LIDAR_BLOCK_CM → dừng ngay trong MỌI state
 *
 *  API:
 *    wpNavSetRoute(pts, count)  — Đặt danh sách waypoint mới
 *    wpNavStart()               — Bắt đầu (MODE_WAYPOINT)
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
#include "PidController.h"
#include "Motors.h"
#include <math.h>

/* Forward declaration — tránh circular include với MqttClient.h */
extern volatile bool g_mqttStatusPending;
extern volatile char g_mqttPendingStatus[32];

/* ==================== Tham số điều hướng =========================== */
#define WP_ARRIVE_THRESH_M    0.12f   // Ngưỡng "đến nơi" (m)
#define WP_STEER_K            55.f    // Hệ số steer Pure Pursuit
#define WP_CRUISE_SPEED_PCT   45      // Tốc độ cruising (% PWM_MAX)
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
  WP_NAVIGATING,            // Pure Pursuit tiến về waypoint
  // ─── Local Obstacle Avoidance (Phase 3.5) ───────────────────────
  WP_OBS_SCAN_CW,           // Xoay CW ≤50° đọc LiDAR → đo bên phải
  WP_OBS_SCAN_CCW,          // Xoay CCW ≤50° (qua heading gốc) đo bên trái
  WP_OBS_SWERVE,            // Xoay tới heading chéo rồi lái chéo sang bên trống
  WP_OBS_PASS,              // Xoay về heading gốc rồi đi thẳng vượt qua vật cản
  // ─── Fallback / kết thúc ─────────────────────────────────────────
  WP_OBSTACLE_HOLD,         // Cả 2 bên chật / hết lần thử → chờ reroute
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

/* ── Phase 3.5: Obstacle Avoidance state vars ──────────────────── */
static float  s_oaScanMaxRight   = 0.f;  // LiDAR max đo được bên phải (cm)
static float  s_oaScanMaxLeft    = 0.f;  // LiDAR max đo được bên trái  (cm)
static float  s_oaHeadingBefore  = 0.f;  // Heading trước khi bắt đầu scan (rad)
static float  s_oaSwerveTarget   = 0.f;  // Heading mục tiêu khi lái chéo (rad)
static float  s_oaPoseXBefore    = 0.f;  // Toạ độ X trước khi SWERVE/PASS
static float  s_oaPoseYBefore    = 0.f;  // Toạ độ Y trước khi SWERVE/PASS
static uint8_t s_oaAttempts      = 0;    // Số lần thử OA trên waypoint hiện tại
static int8_t  s_oaSwerveDir     = 0;    // +1=tránh phải, -1=tránh trái

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

/** Khoảng cách di chuyển kể từ điểm lưu */
static inline float wpDistMoved() {
  float dx = g_pose.x - s_oaPoseXBefore;
  float dy = g_pose.y - s_oaPoseYBefore;
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
  s_oaAttempts = 0;
  s_wpFsm    = WP_IDLE;
  strncpy(g_wpStatus, "route_set", sizeof(g_wpStatus) - 1);
  Serial.printf("[WP] Route set: %d waypoints\n", (int)count);
}

inline void wpNavStart() {
  if (s_wpCount == 0) {
    Serial.println(F("[WP] No waypoints — abort"));
    return;
  }
  s_wpIndex    = 0;
  s_oaAttempts = 0;
  s_wpFsm      = WP_NAVIGATING;
  s_wpT0       = millis();
  g_state.mode = MODE_WAYPOINT;
  pidSpeedReset();
  pidYawReset();
  strncpy(g_wpStatus, "navigating", sizeof(g_wpStatus) - 1);
  Serial.printf("[WP] Start → WP[0] (%.3f, %.3f)\n",
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
  s_oaAttempts = 0;
  g_state.mode = MODE_MANUAL;
  botStop();
  pidSpeedReset();
  strncpy(g_wpStatus, "cancelled", sizeof(g_wpStatus) - 1);
  Serial.println(F("[WP] Route cancelled."));
}

inline bool wpNavIsActive() {
  return (s_wpFsm == WP_NAVIGATING
       || s_wpFsm == WP_OBS_SCAN_CW
       || s_wpFsm == WP_OBS_SCAN_CCW
       || s_wpFsm == WP_OBS_SWERVE
       || s_wpFsm == WP_OBS_PASS
       || s_wpFsm == WP_OBSTACLE_HOLD);
}

inline bool wpNavIsDone() { return (s_wpFsm == WP_DONE); }

/* ==================== TICK ======================================== */
inline void wpNavTick() {
  if (s_wpFsm == WP_IDLE || s_wpFsm == WP_DONE || s_wpFsm == WP_ABORTED) return;

  const uint32_t now   = millis();
  const int16_t  fCm   = g_state.lidarFront;  // LiDAR trước (cm)
  const int16_t  bCm   = g_state.lidarBack;   // LiDAR sau  (cm)

  /* ── Safety hardstop — ưu tiên tuyệt đối (mọi state) ───────── */
  const bool hardFront = (fCm > 3 && fCm < (int16_t)AUTO_LIDAR_BLOCK_CM);
  const bool hardRear  = (AUTO_LIDAR_BLOCK_USE_REAR != 0)
                       && (bCm > 3 && bCm < (int16_t)AUTO_LIDAR_BLOCK_CM);

  if (hardFront || hardRear) {
    /* Không can thiệp FSM — chỉ dừng motor, FSM tự xử lý ở state tương ứng */
    botStop();
    /* Trong state OA đang xoay → hardstop nhưng không reset FSM,
       tick tiếp theo sẽ detect dist và chuyển state đúng */
  }

  switch (s_wpFsm) {

  /* ═══════════════════════════════════════════════════════════════
   *  WP_NAVIGATING — Pure Pursuit: steer + drive về waypoint
   * ══════════════════════════════════════════════════════════════ */
  case WP_NAVIGATING: {
    if (s_wpIndex >= s_wpCount) {
      /* Hết route → DONE */
      botStop();
      s_wpFsm      = WP_DONE;
      g_state.mode = MODE_AUTO;
      strncpy(g_wpStatus, "done", sizeof(g_wpStatus) - 1);
      strncpy((char *)g_mqttPendingStatus, "wp_done", sizeof(g_mqttPendingStatus) - 1);
      g_mqttStatusPending = true;
      Serial.println(F("[WP] Route complete!"));
      return;
    }

    /* Timeout waypoint */
    if (now - s_wpT0 >= WP_TIMEOUT_MS) {
      Serial.printf("[WP] Timeout WP[%d] — skip\n", (int)s_wpIndex);
      s_wpIndex++;
      s_oaAttempts = 0;
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
      s_oaAttempts = 0;
      s_wpT0 = now;
      pidSpeedReset();
      if (s_wpIndex >= s_wpCount) {
        botStop();
        s_wpFsm      = WP_DONE;
        g_state.mode = MODE_AUTO;
        strncpy(g_wpStatus, "done", sizeof(g_wpStatus) - 1);
        strncpy((char *)g_mqttPendingStatus, "wp_done", sizeof(g_mqttPendingStatus) - 1);
        g_mqttStatusPending = true;
        Serial.println(F("[WP] All waypoints reached!"));
      }
      return;
    }

    /* ── Phase 3.5: phát hiện vật cản sớm hơn ─────────────────── */
    if (fCm > 3 && fCm < (int16_t)OA_DETECT_CM) {
      botStop();
      pidSpeedReset();

      if (s_oaAttempts < OA_MAX_ATTEMPTS) {
        s_oaAttempts++;
        s_oaHeadingBefore = g_pose.headingRad;
        s_oaScanMaxRight  = 0.f;
        s_oaScanMaxLeft   = 0.f;
        wpSetState(WP_OBS_SCAN_CW, now, "oa_scan_cw");
        Serial.printf("[WP-OA] Obstacle at %dcm. Scan attempt %d/%d...\n",
                      (int)fCm, (int)s_oaAttempts, (int)OA_MAX_ATTEMPTS);
      } else {
        /* Hết lần thử → fallback chờ */
        wpSetState(WP_OBSTACLE_HOLD, now, "blocked");
        s_wpObstHoldStart = now;
        s_oaAttempts = 0;
        Serial.println(F("[WP-OA] Max OA attempts. Waiting for reroute..."));
      }
      return;
    }

    /* Reset attempts khi đường thông suốt */
    if (fCm <= 3 || fCm >= (int16_t)OA_DETECT_CM) {
      s_oaAttempts = 0;
    }

    /* ── Pure Pursuit steering ─────────────────────────────────── */
    float targetH = atan2f(dy, dx);
    float alpha   = wpAngleDiff(targetH, g_pose.headingRad);

    /* Quá lệch → xoay tại chỗ */
    if (fabsf(alpha) > WP_MAX_STEER_RAD) {
      uint16_t turnPwm = wpPct2Pwm(30);
      if (alpha > 0) { botRotateCW(turnPwm); }
      else           { botRotateCCW(turnPwm); }
      return;
    }

    int16_t steer = (int16_t)(WP_STEER_K * alpha);
    if (steer >  100) steer =  100;
    if (steer < -100) steer = -100;

    uint16_t spd = (dist < WP_SLOW_RADIUS_M)
                 ? wpPct2Pwm(WP_SLOW_SPEED_PCT)
                 : wpPct2Pwm(WP_CRUISE_SPEED_PCT);

    float dt_s    = (float)SAFE_LOOP_MS * 0.001f;
    float pidOut  = pidSpeedCompute(pwmToEstMps(spd), robotActualSpeedMps(), dt_s);
    int32_t runPwm = (int32_t)spd + (int32_t)pidOut;
    if (runPwm < 0) runPwm = 0;
    if (runPwm > (int32_t)PWM_MAX) runPwm = (int32_t)PWM_MAX;

    botDrive(steer, 80, (uint16_t)runPwm);
    break;
  }

  /* ═══════════════════════════════════════════════════════════════
   *  WP_OBS_SCAN_CW — Xoay CW ≤OA_SCAN_ANGLE_DEG°, đọc LiDAR → đo bên phải
   * ══════════════════════════════════════════════════════════════ */
  case WP_OBS_SCAN_CW: {
    /* Safety: nếu LiDAR sau chặn khi đang xoay → cũng dừng motor (hardstop đã xử lý)
       Tiếp tục logic FSM vì đang xoay tại chỗ, sau chặn không nguy hiểm */

    /* Ghi nhận khoảng cách lớn nhất khi quét bên phải */
    if (fCm > 3) {
      float fFloat = (float)fCm;
      if (fFloat > s_oaScanMaxRight) s_oaScanMaxRight = fFloat;
    }

    /* Tính góc đã xoay CW (dương) kể từ heading gốc */
    float rotated = fabsf(wpNormalizeAngle(g_pose.headingRad - s_oaHeadingBefore));
    float maxRad  = (float)OA_SCAN_ANGLE_DEG * (float)M_PI / 180.f;

    if (rotated >= maxRad) {
      /* Đã quét xong bên phải → bắt đầu quét bên trái */
      botStop();
      wpSetState(WP_OBS_SCAN_CCW, now, "oa_scan_ccw");
      Serial.printf("[WP-OA] CW scan done. RightMax=%.0fcm. Scanning left...\n",
                    s_oaScanMaxRight);
      return;
    }

    if (!hardFront && !hardRear) {
      botRotateCW(wpPct2Pwm(OA_SCAN_SPEED_PCT));
    }
    break;
  }

  /* ═══════════════════════════════════════════════════════════════
   *  WP_OBS_SCAN_CCW — Xoay CCW qua heading gốc rồi sang ≤50° bên trái
   *  Quét tổng cộng ≤2×OA_SCAN_ANGLE_DEG° từ vị trí sau CW scan
   * ══════════════════════════════════════════════════════════════ */
  case WP_OBS_SCAN_CCW: {
    /* Ghi nhận bên trái: robot đã xoay về phía trái của heading gốc */
    if (fCm > 3) {
      float fFloat = (float)fCm;
      if (fFloat > s_oaScanMaxLeft) s_oaScanMaxLeft = fFloat;
    }

    /* Góc CCW so với heading gốc (tổng = từ đỉnh CW qua 0 đến đỉnh CCW = 2×maxRad) */
    float maxRad      = (float)OA_SCAN_ANGLE_DEG * (float)M_PI / 180.f;
    float fromOrigCCW = wpNormalizeAngle(s_oaHeadingBefore - g_pose.headingRad);
    /* fromOrigCCW tăng từ 0 đến maxRad khi xoay sang trái đủ */

    if (fromOrigCCW >= maxRad) {
      botStop();
      Serial.printf("[WP-OA] Scan done. Right=%.0fcm  Left=%.0fcm\n",
                    s_oaScanMaxRight, s_oaScanMaxLeft);

      bool rightOk = (s_oaScanMaxRight >= (float)OA_CLEAR_MIN_CM);
      bool leftOk  = (s_oaScanMaxLeft  >= (float)OA_CLEAR_MIN_CM);

      if (rightOk || leftOk) {
        /* Chọn bên trống — ưu tiên bên có khoảng trống lớn hơn */
        if (rightOk && leftOk) {
          s_oaSwerveDir = (s_oaScanMaxRight >= s_oaScanMaxLeft) ? 1 : -1;
        } else {
          s_oaSwerveDir = rightOk ? 1 : -1;
        }

        /* Heading chéo = heading gốc ± OA_SWERVE_ANGLE_DEG */
        float swerveRad   = (float)OA_SWERVE_ANGLE_DEG * (float)M_PI / 180.f;
        s_oaSwerveTarget  = wpNormalizeAngle(s_oaHeadingBefore + s_oaSwerveDir * swerveRad);
        s_oaPoseXBefore   = g_pose.x;
        s_oaPoseYBefore   = g_pose.y;

        wpSetState(WP_OBS_SWERVE, now, "oa_swerve");
        Serial.printf("[WP-OA] Swerve %s (headingTarget=%.1f°)\n",
                      s_oaSwerveDir > 0 ? "RIGHT" : "LEFT",
                      s_oaSwerveTarget * 180.f / (float)M_PI);
      } else {
        /* Cả 2 bên đều chật → fallback */
        wpSetState(WP_OBSTACLE_HOLD, now, "blocked");
        s_wpObstHoldStart = now;
        s_oaAttempts = OA_MAX_ATTEMPTS; // Ngăn thử lại ngay
        Serial.println(F("[WP-OA] Both sides blocked. Fallback wait."));
      }
      return;
    }

    if (!hardFront && !hardRear) {
      botRotateCCW(wpPct2Pwm(OA_SCAN_SPEED_PCT));
    }
    break;
  }

  /* ═══════════════════════════════════════════════════════════════
   *  WP_OBS_SWERVE — Bước 1: xoay tới heading chéo
   *                  Bước 2: tiến OA_SWERVE_DIST_M sang bên trống
   * ══════════════════════════════════════════════════════════════ */
  case WP_OBS_SWERVE: {
    /* Safety: LiDAR trước chặn khi đang lái chéo → dừng hẳn, fallback */
    if (hardFront) {
      botStop();
      pidSpeedReset();
      wpSetState(WP_OBSTACLE_HOLD, now, "blocked");
      s_wpObstHoldStart = now;
      Serial.println(F("[WP-OA] Obstacle during swerve — fallback."));
      return;
    }

    float headingErr = wpAngleDiff(s_oaSwerveTarget, g_pose.headingRad);

    /* Bước 1: Xoay về heading chéo */
    if (fabsf(headingErr) > 0.12f) {
      uint16_t rotPwm = wpPct2Pwm(OA_SCAN_SPEED_PCT);
      if (headingErr > 0) { botRotateCW(rotPwm); }
      else               { botRotateCCW(rotPwm); }
      return;
    }

    /* Bước 2: Tiến chéo */
    float distMoved = wpDistMoved();
    if (distMoved >= OA_SWERVE_DIST_M) {
      botStop();
      /* Chuẩn bị PASS: heading về lại route gốc */
      s_oaSwerveTarget = s_oaHeadingBefore;
      s_oaPoseXBefore  = g_pose.x;
      s_oaPoseYBefore  = g_pose.y;
      wpSetState(WP_OBS_PASS, now, "oa_pass");
      Serial.println(F("[WP-OA] Swerve done. Passing obstacle..."));
      return;
    }

    uint16_t swervePwm = wpPct2Pwm(OA_SWERVE_SPEED_PCT);
    float dt_s   = (float)SAFE_LOOP_MS * 0.001f;
    float pidOut = pidSpeedCompute(pwmToEstMps(swervePwm), robotActualSpeedMps(), dt_s);
    int32_t runPwm = (int32_t)swervePwm + (int32_t)pidOut;
    if (runPwm < 0) runPwm = 0;
    if (runPwm > (int32_t)PWM_MAX) runPwm = (int32_t)PWM_MAX;
    botForward((uint16_t)runPwm);
    break;
  }

  /* ═══════════════════════════════════════════════════════════════
   *  WP_OBS_PASS — Bước 1: xoay về heading gốc (song song route)
   *               Bước 2: tiến OA_PASS_DIST_M để vượt qua vật cản
   *               Bước 3: về WP_NAVIGATING (Pure Pursuit tự kéo về route)
   * ══════════════════════════════════════════════════════════════ */
  case WP_OBS_PASS: {
    if (hardFront) {
      botStop();
      pidSpeedReset();
      wpSetState(WP_OBSTACLE_HOLD, now, "blocked");
      s_wpObstHoldStart = now;
      Serial.println(F("[WP-OA] Obstacle during pass — fallback."));
      return;
    }

    float headingErr = wpAngleDiff(s_oaSwerveTarget, g_pose.headingRad);

    /* Bước 1: Xoay về heading gốc */
    if (fabsf(headingErr) > 0.12f) {
      uint16_t rotPwm = wpPct2Pwm(OA_SCAN_SPEED_PCT);
      if (headingErr > 0) { botRotateCW(rotPwm); }
      else               { botRotateCCW(rotPwm); }
      return;
    }

    /* Bước 2: Tiến thẳng */
    float distMoved = wpDistMoved();
    if (distMoved >= OA_PASS_DIST_M) {
      botStop();
      pidSpeedReset();
      /* Thành công! Pure Pursuit sẽ tự kéo về route */
      wpSetState(WP_NAVIGATING, now, "navigating");
      Serial.println(F("[WP-OA] ★ Obstacle passed! Resuming navigation."));
      /* Reset timeout waypoint */
      s_wpT0 = now;
      return;
    }

    uint16_t passPwm = wpPct2Pwm(OA_SWERVE_SPEED_PCT);
    float dt_s   = (float)SAFE_LOOP_MS * 0.001f;
    float pidOut = pidSpeedCompute(pwmToEstMps(passPwm), robotActualSpeedMps(), dt_s);
    int32_t runPwm = (int32_t)passPwm + (int32_t)pidOut;
    if (runPwm < 0) runPwm = 0;
    if (runPwm > (int32_t)PWM_MAX) runPwm = (int32_t)PWM_MAX;
    botForward((uint16_t)runPwm);
    break;
  }

  /* ═══════════════════════════════════════════════════════════════
   *  WP_OBSTACLE_HOLD — Fallback: chờ OA_FALLBACK_WAIT_MS
   *                     rồi publish "reroute_needed" → Backend gửi route mới
   * ══════════════════════════════════════════════════════════════ */
  case WP_OBSTACLE_HOLD: {
    botStop();

    /* Nếu đường đột nhiên thông trước khi timeout → resume */
    const bool stillBlocked = (fCm > 3 && fCm < (int16_t)OA_DETECT_CM);
    if (!stillBlocked) {
      pidSpeedReset();
      wpSetState(WP_NAVIGATING, now, "navigating");
      s_oaAttempts = 0;
      Serial.println(F("[WP] Path cleared — resuming."));
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
 * Format JSON từ Backend (Phase 3 NavigationCommandService):
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
  wpNavStart();
  return true;
}

#endif // WAYPOINT_NAV_H
