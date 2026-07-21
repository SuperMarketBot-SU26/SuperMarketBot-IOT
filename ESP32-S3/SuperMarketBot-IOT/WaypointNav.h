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
#include "Odometry.h"   // P0-1 FIX: cần cho odomResetDistance() trong wpNavStart()
#include "LocalObstacleAvoid.h"
#include "ObstacleSensors.h"
#include "PidController.h"
#include "Motors.h"
#include <math.h>

/* Forward declaration — tránh circular include với MqttClient.h */
extern volatile bool g_mqttStatusPending;
extern volatile char g_mqttPendingStatus[32];

/* Forward declaration — Sensors.h (tránh phụ thuộc thứ tự include) */
extern void usFilterReset();

/* ==================== Tham số điều hướng =========================== */
#define WP_ARRIVE_THRESH_M    0.12f   // Ngưỡng "đến nơi" (m)
#define WP_STEER_K            55.f    // Hệ số steer Pure Pursuit
#define WP_CRUISE_SPEED_PCT   (ROBOT_HEAVY_LOAD ? 50 : 40)


#define WP_SLOW_RADIUS_M      0.4f    // Bán kính giảm tốc (m)
#define WP_SLOW_SPEED_PCT     (ROBOT_HEAVY_LOAD ? 35 : 28)
#define WP_MAX_STEER_RAD      1.2f    // ~70° — quá lệch thì xoay tại chỗ trước
#define WP_TIMEOUT_MS         20000u  // Timeout mỗi waypoint (ms)
#define WP_MAX_WAYPOINTS      32      // Số waypoint tối đa trong 1 route

/* ==================== NV3 — Accel limiter + smooth blend =============
 *
 * Giúp robot không "giật ga" khi chuyển waypoint hoặc khi PID output dao động.
 *
 * - WP_ACCEL_STEP_PWM_PER_TICK: tối đa PWM thay đổi mỗi SAFE_LOOP_MS (50ms).
 *   Ví dụ 30 → từ 0 → 100% mất ~1.7s, tránh sụt áp nguồn (brownout).
 * - WP_STEER_BLEND_ALPHA: low-pass filter cho steer cmd. 1.0 = no filter,
 *   0.3 = mượt nhiều. Khuyến nghị 0.5-0.7.
 * - WP_STRAFE_BLEND_ALPHA: tương tự cho strafe cmd.
 * - WP_HOLD_AFTER_ARRIVE_MS: sau khi đến waypoint, giữ tốc độ 0 trong 200ms
 *   để localization ổn định trước khi tính segment mới.
 * -------------------------------------------------------------------- */
#define WP_ACCEL_STEP_PWM_PER_TICK   30u
#define WP_STEER_BLEND_ALPHA         0.40f  // Giảm từ 0.6 → steer MƯỢT hơn (ít nhảy)
#define WP_STRAFE_BLEND_ALPHA        0.45f  // Giảm từ 0.7 → strafe MƯỢT hơn (ít nhảy)
#define WP_HOLD_AFTER_ARRIVE_MS      200u

/* ==================== Cấu trúc ===================================== */
struct Waypoint {
  float x;      // mét
  float y;      // mét
  int   nodeId; // node ID backend (−1 = không rõ)
};

enum WpFsmState : uint8_t {
  WP_IDLE = 0,
  WP_NAVIGATING,            // Pure Pursuit (+ LocalObstacleAvoid.h khi gặp vật)
  WP_OBSTACLE_HOLD,         // Không lách được → chờ / reroute MQTT
  WP_DONE,
  WP_ABORTED
};

/* ==================== Biến nội bộ ================================== */
extern Waypoint            s_wpRoute[WP_MAX_WAYPOINTS];
extern volatile uint8_t    s_wpCount;
extern volatile uint8_t    s_wpIndex;          // expose để MqttClient đọc
extern volatile WpFsmState s_wpFsm;
extern volatile uint32_t   s_wpT0;          // Millis khi bắt đầu waypoint hiện tại

/* Obstacle-hold fallback */
extern volatile uint32_t   s_wpObstHoldStart;
extern OaContext           s_wpOa;

/* Settle delay sau khi OA xong — tránh lao ngay vào hướng mới */
extern volatile uint32_t   s_wpSettleUntilMs;

/* Status string cho MQTT telemetry */
char g_wpStatus[32] = "idle";

/* Mecanum segment path-following variables */
static float s_wpStartColX = 0.f;
static float s_wpStartColY = 0.f;
static float s_wpSegmentHeading = 0.f;
static bool  s_wpAligned = false;

/* Spin-once state (file-scope để tránh static-in-scope bug) */
static int8_t   s_wpSpinDir   = 0;   // +1=CCW, -1=CW, 0=not spinning
static uint32_t s_wpSpinStart = 0;

/* NV3 — Accel limiter + low-pass blend (initialized to 0 = "not yet computed") */
static int16_t s_wpLastSteer = 0;
static int16_t s_wpLastStrafe = 0;
static uint16_t s_wpLastPwm = 0;
static uint32_t s_wpHoldUntilMs = 0;

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
  s_wpFsm    = WP_IDLE;
  strncpy(g_wpStatus, "route_set", sizeof(g_wpStatus) - 1);
  Serial.printf("[WP] Route set: %d waypoints\n", (int)count);
}

inline void wpNavStart() {
  if (s_wpCount == 0) {
    Serial.println(F("[WP] No waypoints — abort"));
    return;
  }

  // [P0-1 FIX] Reset toàn bộ odom (về pose 0,0) TRƯỚC khi snap pose mới
  // để encoder delta của tick tiếp theo được tính từ 0 → không bị "trôi" pose
  odomResetDistance();

  // Tự động căn chỉnh toạ độ Robot với Waypoint đầu tiên của lộ trình
  // (đặt SAU odomResetDistance để không bị reset về 0,0)
  g_pose.x = s_wpRoute[0].x;
  g_pose.y = s_wpRoute[0].y;

  // Căn chỉnh góc quay hướng tới Waypoint thứ 2 (nếu có) — đặt trước
  // khi IMU đọc lại để tránh bị override ngay tick sau.
  if (s_wpCount > 1) {
    float dx0 = s_wpRoute[1].x - s_wpRoute[0].x;
    float dy0 = s_wpRoute[1].y - s_wpRoute[0].y;
    g_pose.headingRad = atan2f(dy0, dx0);
    while (g_pose.headingRad < 0.f)                g_pose.headingRad += 2.f * (float)M_PI;
    while (g_pose.headingRad >= 2.f * (float)M_PI) g_pose.headingRad -= 2.f * (float)M_PI;
  }

  s_wpIndex    = 0;
  oaReset(s_wpOa);
  s_wpFsm      = WP_NAVIGATING;
  s_wpT0       = millis();
  g_state.mode = MODE_WAYPOINT;
  pidSpeedReset();
  pidYawReset();
  
  s_wpStartColX = g_pose.x;
  s_wpStartColY = g_pose.y;
  s_wpAligned = false;
  s_wpSpinDir = 0;
  s_wpSpinStart = 0;
  // NV3 — Reset accel/blend state
  s_wpLastSteer = 0;
  s_wpLastStrafe = 0;
  s_wpLastPwm = 0;
  s_wpHoldUntilMs = 0;
  if (s_wpCount > 0) {
    float dx = s_wpRoute[0].x - g_pose.x;
    float dy = s_wpRoute[0].y - g_pose.y;
    s_wpSegmentHeading = atan2f(dy, dx);
  }
  
  strncpy(g_wpStatus, "navigating", sizeof(g_wpStatus) - 1);
  Serial.printf("[WP] Start → WP[0] (%.3f, %.3f) | Pose aligned to map!\n",
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

  const uint32_t now   = millis();
  const int16_t  fCm   = obsFrontCm();
  const int16_t  bCm   = obsBackCm();

  /* ── Safety hardstop — chỉ dừng khi không né được (OA_BLOCKED) ── */
  if (s_wpOa.state == OA_BLOCKED) {
    botStop();
  }

  /* ── Local OA — chạy trước switch ─────── */
  if (s_wpFsm == WP_NAVIGATING) {
    // Chạy OA tick nếu đang active (không phải IDLE)
    if (s_wpOa.state != OA_IDLE) {
      strncpy(g_wpStatus, "oa_active", sizeof(g_wpStatus) - 1);
      OaTickResult r = oaTick(s_wpOa, fCm, now);
      if (r == OA_RES_DONE) {
        s_wpT0 = now;
        s_wpSettleUntilMs = now + 450;  // Pause 450ms để filter sensor ổn định
        usFilterReset();
        pidSpeedReset();
        pidYawReset();
        // [FIX] Reset alignment: sau OA robot đã xoay sang hướng khác,
        // cần tính lại segmentHeading từ pose hiện tại đến waypoint đích
        // và re-spin nếu lệch > 60°
        s_wpAligned = false;            // Buộc re-check hướng
        s_wpSpinDir = 0;
        s_wpSpinStart = 0;
        s_wpStartColX = g_pose.x;       // Cập nhật điểm bắt đầu segment
        s_wpStartColY = g_pose.y;
        if (s_wpIndex < s_wpCount) {
          float dx2 = s_wpRoute[s_wpIndex].x - g_pose.x;
          float dy2 = s_wpRoute[s_wpIndex].y - g_pose.y;
          s_wpSegmentHeading = atan2f(dy2, dx2);  // Tính lại hướng đích
        }
        strncpy(g_wpStatus, "navigating", sizeof(g_wpStatus) - 1);
        Serial.printf("[WP] OA done. Re-align to segHead=%.2f rad. settle 450ms.\n", s_wpSegmentHeading);
      } else if (r == OA_RES_BLOCKED) {
        wpSetState(WP_OBSTACLE_HOLD, now, "blocked");
        s_wpObstHoldStart = now;
        Serial.println(F("[WP] OA blocked — hold / reroute."));
      }
      return;
    }
    
    // Kiểm tra vật cản phía trước - bắt đầu OA nếu cần
    if (obsOaTriggered(fCm)) {
      if (oaBegin(s_wpOa, fCm, now)) {
        // OA đã bắt đầu, s_wpOa.state sẽ != OA_IDLE
        // Tick tiếp theo sẽ xử lý OA
        return;
      }
    }
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

    /* Timeout waypoint - Tạm tắt theo yêu cầu để tránh bỏ qua waypoint khi kẹt */
    /*
    if (now - s_wpT0 >= WP_TIMEOUT_MS) {
      Serial.printf("[WP] Timeout WP[%d] — skip\n", (int)s_wpIndex);
      s_wpIndex++;
      oaReset(s_wpOa);
      s_wpT0 = now;
      return;
    }
    */

    float tx   = s_wpRoute[s_wpIndex].x;
    float ty   = s_wpRoute[s_wpIndex].y;
    float dx   = tx - g_pose.x;
    float dy   = ty - g_pose.y;
    float dist = sqrtf(dx * dx + dy * dy);

    // [NV3 FIX] Hold-after-arrive: giữ botStop trong WP_HOLD_AFTER_ARRIVE_MS (200ms)
    // để localization ổn định trước khi segment mới được tính (tránh heading nhảy giật).
    if (s_wpHoldUntilMs > 0) {
      if (now < s_wpHoldUntilMs) {
        botStop();
        return;
      } else {
        s_wpHoldUntilMs = 0;
      }
    }

    static uint32_t lastWpLog = 0;
    if (now - lastWpLog > 200u) {
      lastWpLog = now;
      Serial.printf("[WP DEBUG] idx=%d, target=(%.3f,%.3f), pose=(%.3f,%.3f), heading=%.3f, dist=%.3f\n",
                    (int)s_wpIndex, tx, ty, g_pose.x, g_pose.y, g_pose.headingRad, dist);
      // [NV-DBG] In kèm giá trị cảm biến để biết tại sao bị stop nếu có
      Serial.printf("[WP DEBUG] sensors: front=%d cm (clear=%d, stop=%d), back=%d cm, usPathClearCm=%u, usStopCm=%u\n",
                    (int)fCm, (int)obsPathClear(fCm), (int)obsFrontBlocked(),
                    (int)bCm, (unsigned)g_state.usPathClearCm, (unsigned)g_state.usStopCm);
    }

    /* Đến nơi */
    if (dist < WP_ARRIVE_THRESH_M) {
      Serial.printf("[WP] WP[%d] reached (%.2f,%.2f) dist=%.3fm\n",
                    (int)s_wpIndex, tx, ty, dist);
      s_wpIndex++;
      oaReset(s_wpOa);
      s_wpT0 = now;
      pidSpeedReset();
      pidYawReset();
      // [NV3 FIX] Reset accel/blend state + hold 200ms để localization ổn định.
      s_wpLastSteer = 0;
      s_wpLastStrafe = 0;
      s_wpLastPwm = 0;
      s_wpHoldUntilMs = now + WP_HOLD_AFTER_ARRIVE_MS;
      s_wpStartColX = g_pose.x;
      s_wpStartColY = g_pose.y;
      s_wpAligned = false;
      s_wpSpinDir = 0;
      s_wpSpinStart = 0;
      if (s_wpIndex < s_wpCount) {
        float dx = s_wpRoute[s_wpIndex].x - g_pose.x;
        float dy = s_wpRoute[s_wpIndex].y - g_pose.y;
        s_wpSegmentHeading = atan2f(dy, dx);
      }
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

    /* ── Settle delay sau OA: dừng chờ filter sensor ổn định ──── */
    if (s_wpSettleUntilMs > 0) {
      if (now < s_wpSettleUntilMs) {
        botStop();
        pidSpeedReset();
        return;
      } else {
        s_wpSettleUntilMs = 0;
        Serial.println(F("[WP] Settle done — tiep tuc Pure Pursuit."));
      }
    }

    /* ── Differential Drive: Lái hướng về Waypoint ────────────────
     *
     *  CHIẾN LƯỢC 4WD BÁNH THƯỜNG:
     *  Bước 1 (s_wpAligned==false): Nếu lệch > 90° thì xoay tại chỗ 1 LẦN
     *                                cho tới khi alpha < 60°. Sau đó set
     *                                s_wpAligned=true và KHÔNG BAO GIỜ spin lại.
     *  Bước 2 (s_wpAligned==true) : Tiến thẳng và PID yaw bẻ lái liên tục.
     *                               KHÔNG trigger spin dù heading trôi.
     *
     * ──────────────────────────────────────────────────────────── */
    float alpha = wpAngleDiff(s_wpSegmentHeading, g_pose.headingRad);

    if (!s_wpAligned) {
      if (fabsf(alpha) > 0.349f) {   // > 20° → cần xoay trước khi tiến
        // ──── Khóa chiều xoay ĐÚNG ngay từ đầu, KHÔNG bao giờ đổi lại ────
        // Đã xác nhận từ log: botRotateCWImmediate → heading TĂNG
        //                     botRotateCCWImmediate → heading GIẢM
        // alpha = segmentHeading - headingRad (normalized -π..π)
        //   alpha > 0 → cần TĂNG heading → CW  (dir = -1)
        //   alpha < 0 → cần GIẢM heading → CCW (dir = +1)
        if (s_wpSpinStart == 0) {
          s_wpSpinStart = now;
          s_wpSpinDir = (alpha > 0.f) ? -1 : +1;  // ĐÚNG: không flip sau khi set
          Serial.printf("[WP SPIN START] alpha=%.2f rad → dir=%s (shortest path)\n",
                        alpha, s_wpSpinDir < 0 ? "CW(+hdg)" : "CCW(-hdg)");
        }
        // KHÔNG update s_wpSpinDir nữa — khóa hoàn toàn cho đến khi alpha < 20°

        // PWM tỉ lệ 200..250, mượt không vọt lố
        uint16_t spinPwm = 200u + (uint16_t)(fabsf(alpha) * 13.0f);
        if (spinPwm > 250u) spinPwm = 250u;

        static uint32_t lastSpinLog = 0;
        if (now - lastSpinLog > 500u) {
          lastSpinLog = now;
          Serial.printf("[WP SPIN] alpha=%.2f rad (%.0f deg) dir=%s pwm=%u t=%ums\n",
                        alpha, alpha*180.f/(float)M_PI,
                        s_wpSpinDir < 0 ? "CW" : "CCW", spinPwm, now - s_wpSpinStart);
        }
        if (s_wpSpinDir < 0) botRotateCWImmediate(spinPwm);
        else                 botRotateCCWImmediate(spinPwm);

        // Timeout 6s → ép heading để thoát spin (phòng IMU drift / motor stall)
        if (now - s_wpSpinStart > 6000u) {
          Serial.printf("[WP] Spin timeout 6s → force heading %.3f rad\n", s_wpSegmentHeading);
          g_pose.headingRad = s_wpSegmentHeading;
          while (g_pose.headingRad < 0.f)              g_pose.headingRad += 2.f*(float)M_PI;
          while (g_pose.headingRad >= 2.f*(float)M_PI) g_pose.headingRad -= 2.f*(float)M_PI;
          s_wpSpinStart = 0; s_wpSpinDir = 0;
          s_wpAligned = true;
          pidSpeedReset(); pidYawReset();
        }
        return;  // Chưa aligned → không tiến

      } else {
        // alpha <= 20°: đủ thẳng hướng → aligned, bắt đầu tiến thẳng
        s_wpAligned = true;
        s_wpSpinStart = 0; s_wpSpinDir = 0;
        pidSpeedReset(); pidYawReset();
        Serial.printf("[WP] Aligned at alpha=%.2f rad, starting cruise.\n", alpha);
      }
    }
    // s_wpAligned == true: tiến thẳng, PID yaw bẻ lái. KHÔNG spin nữa.

    // Tính toán sai số lệch ngang (Cross-Track Error) so với đường thẳng đoạn thẳng đang đi
    float dx_p = g_pose.x - s_wpStartColX;
    float dy_p = g_pose.y - s_wpStartColY;
    float e_lat = -dx_p * sinf(s_wpSegmentHeading) + dy_p * cosf(s_wpSegmentHeading);
    
    int16_t strafeCmd = 0;
    int16_t steer = 0;
    float dt_s = (float)SAFE_LOOP_MS * 0.001f;

    // Bánh thường (4WD vi sai): Giữ heading dọc theo đoạn thẳng hiện tại (s_wpSegmentHeading)
    // Khi gần đích (dist < WP_SLOW_RADIUS_M) mới bám trực tiếp vào waypoint (go-to-pose).
    // Lý do: trong khi đi thẳng dài, target = atan2(ty-y, tx-x) bám theo pose → alpha không giảm về 0 → xoay mãi không align.
    float targetHeading = s_wpSegmentHeading;
    if (dist < WP_SLOW_RADIUS_M) {
      targetHeading = atan2f(ty - g_pose.y, tx - g_pose.x);
    }
    float yawOut = pidYawCompute(targetHeading, g_pose.headingRad, dt_s);
    steer = (int16_t)constrain(yawOut, -100, 100);
    strafeCmd = 0;

    uint16_t cruiseSpd = g_state.autoBaseSpeed;
    if (cruiseSpd == 0) cruiseSpd = g_state.baseSpeed;
    if (cruiseSpd == 0) cruiseSpd = wpPct2Pwm(WP_CRUISE_SPEED_PCT);

    // Tỉ lệ giảm tốc khi gần đích
    uint16_t slowSpd = (uint16_t)((uint32_t)cruiseSpd * (uint32_t)WP_SLOW_SPEED_PCT / (uint32_t)WP_CRUISE_SPEED_PCT);
    if (slowSpd < wpPct2Pwm(ROBOT_HEAVY_LOAD ? 30 : 20)) {
      slowSpd = wpPct2Pwm(ROBOT_HEAVY_LOAD ? 30 : 20);
    }

    uint16_t spd = (dist < WP_SLOW_RADIUS_M) ? slowSpd : cruiseSpd;

    float pidOut  = pidSpeedCompute(pwmToEstMps(spd), robotActualSpeedMps(), dt_s);
    int32_t runPwm = (int32_t)spd + (int32_t)pidOut;
    if (runPwm < 0) runPwm = 0;
    if (runPwm > (int32_t)PWM_MAX) runPwm = (int32_t)PWM_MAX;

    // [NV3 FIX] Accel limiter: tránh tăng/giảm PWM đột ngột gây sụt áp nguồn (brownout).
    // Đặc biệt quan trọng khi vừa rời WP_ARRIVE (PWM=0) → cruise (PWM lớn).
    int32_t pwmDiff = runPwm - (int32_t)s_wpLastPwm;
    if (pwmDiff >  (int32_t)WP_ACCEL_STEP_PWM_PER_TICK) runPwm = (int32_t)s_wpLastPwm + WP_ACCEL_STEP_PWM_PER_TICK;
    if (pwmDiff < -(int32_t)WP_ACCEL_STEP_PWM_PER_TICK) runPwm = (int32_t)s_wpLastPwm - WP_ACCEL_STEP_PWM_PER_TICK;
    s_wpLastPwm = (uint16_t)runPwm;

    // [NV3 FIX] Low-pass blend cho steer & strafe để tránh giật khi PID output dao động.
    // steerRaw và strafeRaw được tính ở trên; áp blend = alpha*new + (1-alpha)*old
    int16_t steerBlended = (int16_t)(WP_STEER_BLEND_ALPHA * (float)steer
                                  + (1.f - WP_STEER_BLEND_ALPHA) * (float)s_wpLastSteer);
    int16_t strafeBlended = (int16_t)(WP_STRAFE_BLEND_ALPHA * (float)strafeCmd
                                   + (1.f - WP_STRAFE_BLEND_ALPHA) * (float)s_wpLastStrafe);

    // [NV-SMOOTH] Accel limiter cho steer + strafe: tránh bước nhảy PWM đột ngột
    // gây giật bánh (đặc biệt khi vừa rời alignment → cruise).
    // Steer: ±100, giới hạn thay đổi tối đa 15/50ms = 300%/s
    const int16_t STEER_ACCEL_MAX = 15;
    int16_t sd = steerBlended - s_wpLastSteer;
    if (sd >  STEER_ACCEL_MAX) steerBlended = s_wpLastSteer + STEER_ACCEL_MAX;
    if (sd < -STEER_ACCEL_MAX) steerBlended = s_wpLastSteer - STEER_ACCEL_MAX;

    // Strafe: ±60, giới hạn thay đổi tối đa 10/50ms = 200%/s
    const int16_t STRAFE_ACCEL_MAX = 10;
    int16_t fd = strafeBlended - s_wpLastStrafe;
    if (fd >  STRAFE_ACCEL_MAX) strafeBlended = s_wpLastStrafe + STRAFE_ACCEL_MAX;
    if (fd < -STRAFE_ACCEL_MAX) strafeBlended = s_wpLastStrafe - STRAFE_ACCEL_MAX;

    s_wpLastSteer = steerBlended;
    s_wpLastStrafe = strafeBlended;

    // Tiến thẳng và bẻ lái IMU giữ đầu thẳng (bánh thường: bỏ strafe)
    botDrive(steerBlended, 100, (uint16_t)runPwm);
    break;
  }

  /* ═══════════════════════════════════════════════════════════════
   *  WP_OBSTACLE_HOLD — Fallback: chờ OA_FALLBACK_WAIT_MS
   *                     rồi publish "reroute_needed" → Backend gửi route mới
   * ══════════════════════════════════════════════════════════════ */
  case WP_OBSTACLE_HOLD: {
    botStop(); // Dừng tại chỗ chờ hoặc yêu cầu định tuyến lại, không lùi xe
    pidSpeedReset();

    /* Chỉ resume khi phía trước ≥ 1m (PATH_CLEAR) */
    const bool pathClear = obsPathClear(fCm);
    if (pathClear) {
      oaReset(s_wpOa);
      usFilterReset();
      s_wpSettleUntilMs = now + 450;  // Settle trước khi đi tiếp
      wpSetState(WP_NAVIGATING, now, "navigating");
      Serial.println(F("[WP] Duong truoc du xa — settle 450ms roi resume."));
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
