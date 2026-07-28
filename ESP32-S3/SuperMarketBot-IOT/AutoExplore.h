/* =====================================================================
 *  AutoExplore.h — Mode Tự đi quét Map (Frontier Exploration + LIDAR + US fusion)
 *
 *  Mục tiêu:
 *    - Robot tự khám phá toàn bộ không gian (không cần waypoint)
 *    - Hợp nhất cảm biến:
 *        + LIDAR YDLIDAR X3: 360°, dùng để tìm hướng trống xa nhất (frontier)
 *        + US 4 sensor (LF/LR/RF/RR): phản ứng nhanh với vật gần (proactive brake)
 *    - Tự động kết thúc khi "coverage ≥ 95%" (ước lượng diện tích đã quét)
 *    - Publish MQTT: {t:"scan_progress", pct, dist, durMs, state}
 *    - Publish MQTT: {t:"scan_complete", sessionId, totalDist, durMs, pathLen}
 *
 *  4 trạng thái (FSM):
 *    1. CRUISE      Đi thẳng theo hướng target (default: heading hiện tại)
 *    2. SPIN_DETECT Xoay tại chỗ quét LIDAR tìm hướng trống xa nhất
 *    3. AVOID_US    US phát hiện vật quá gần → lùi + xoay
 *    4. DONE        Đã quét đủ → báo cáo + dừng
 *
 *  Coverage estimate (đơn giản, không lưu grid trên ESP32):
 *    - Theo dõi quỹ đạo path (x, y) đã đi
 *    - Tổng quãng đường / perimeter ước lượng → %
 *    - Khi PID bám tường đi hết 1 vòng → coi như > 80%
 *    - Khi số cung LIDAR trống giảm dần < 10% thì coi như 95%
 *
 *  Đặc tả Config:
 *    - AUTO_EXPLORE_TARGET_COVERAGE_PCT : 95 (mặc định)
 *    - AUTO_EXPLORE_MAX_DURATION_MS     : 10 phút
 *    - AUTO_EXPLORE_MIN_WALL_DIST_MM    : 350 (bám tường cách 35cm)
 *    - AUTO_EXPLORE_US_BRAKE_MM         : 250 (US brake nếu < 25cm)
 *    - AUTO_EXPLORE_PROGRESS_INTERVAL_MS: 5000 (publish progress mỗi 5s)
 * =====================================================================*/
#ifndef AUTO_EXPLORE_H
#define AUTO_EXPLORE_H

#include <math.h>
#include "Config.h"
#include "Localization.h"
#include "Motors.h"
#include "MotorControlPro.h"  // [Bước 3 - 2026-07-27] botDriveSmoothNormal()
#include "ObstacleSensors.h"
#include "YdlidarX3.h"
#include "PidController.h"
#include "WaypointNav.h"  // for wpNormalizeAngle()

#ifndef AUTO_EXPLORE_TARGET_COVERAGE_PCT
#define AUTO_EXPLORE_TARGET_COVERAGE_PCT  95.0f
#endif
#ifndef AUTO_EXPLORE_MAX_DURATION_MS
#define AUTO_EXPLORE_MAX_DURATION_MS      (10UL * 60UL * 1000UL)
#endif
#ifndef AUTO_EXPLORE_MIN_WALL_DIST_MM
#define AUTO_EXPLORE_MIN_WALL_DIST_MM     350
#endif
#ifndef AUTO_EXPLORE_US_BRAKE_MM
#define AUTO_EXPLORE_US_BRAKE_MM          250
#endif
#ifndef AUTO_EXPLORE_PROGRESS_INTERVAL_MS
#define AUTO_EXPLORE_PROGRESS_INTERVAL_MS 5000UL
#endif
#ifndef AUTO_EXPLORE_PATH_MAX_POINTS
#define AUTO_EXPLORE_PATH_MAX_POINTS      256
#endif

namespace autoExplore {

enum FsmState : uint8_t {
  ST_CRUISE      = 0,
  ST_SPIN_DETECT = 1,
  ST_AVOID_US    = 2,
  ST_DONE        = 3
};

struct State {
  FsmState   fsm;
  uint32_t   startMs;
  uint32_t   lastProgressMs;
  uint32_t   scanSessionId;
  uint32_t   totalScanSeq;        // số scan LIDAR đã nhận
  float      totalDistanceM;      // tổng quãng đường đã đi
  float      prevX, prevY;
  float      wallTargetHeading;   // heading target khi bám tường
  bool       hasWallTarget;

  // Path tracking (để estimate coverage)
  float      pathX[AUTO_EXPLORE_PATH_MAX_POINTS];
  float      pathY[AUTO_EXPLORE_PATH_MAX_POINTS];
  uint16_t   pathCount;
  float      pathBoundingAreaM2;  // diện tích bounding box của path

  // Tracking scan completeness
  uint8_t    emptyArcCount;       // số cung LIDAR "trống" (max range > 4m)
  float      coveragePct;         // 0..100

  // FSM transition timers
  uint32_t   stateEnterMs;
  uint32_t   lastCoverageMs;       // lần cuối estimate coverage (để throttle)
  float      spinStartHeading;
  float      spinTargetHeading;
  bool       hasSpinTarget;

  // Session info
  bool       active;
};

inline State& getState() {
  static State s = {};
  return s;
}

/* ===== Helpers ========================================================= */

/**
 * Tìm hướng trống xa nhất bằng cách chia LIDAR thành 36 cung (10°/cung).
 * Trả về heading (deg) và khoảng cách lớn nhất trong cung đó (mm).
 */
inline void findOpenHeading(float &outHeadingDeg, uint16_t &outMaxMm) {
  constexpr uint8_t  NUM_ARCS = 36;        // 36 × 10° = 360°
  constexpr uint8_t  MIN_QUALITY = 10;
  float    arcMaxMm[NUM_ARCS] = {0};
  uint16_t arcCount[NUM_ARCS] = {0};

  for (uint16_t i = 0; i < g_x3Scan.count; i++) {
    const LidarPoint &p = g_x3Scan.points[i];
    if (p.quality < MIN_QUALITY || p.distanceMm == 0) continue;
    if (p.distanceMm > 8000) continue;     // bỏ điểm nhiễu > 8m

    float deg = p.angleRad * 180.0f / (float)M_PI;
    if (deg < 0) deg += 360.0f;
    uint8_t arc = (uint8_t)(deg / 10.0f);
    if (arc >= NUM_ARCS) arc = NUM_ARCS - 1;
    if (p.distanceMm > arcMaxMm[arc]) arcMaxMm[arc] = p.distanceMm;
    arcCount[arc]++;
  }

  // Tìm cung có max range lớn nhất (ưu tiên cung có nhiều điểm hợp lệ)
  float bestScore = -1;
  uint8_t bestArc = 0;
  for (uint8_t a = 0; a < NUM_ARCS; a++) {
    if (arcCount[a] < 3) continue;     // bỏ cung quá rỗng (có thể che khuất)
    float score = arcMaxMm[a];
    // Score = maxRange + bonus cho cung ở phía trước (tránh xoay quá xa)
    if (score > 2000) score += 200;     // bonus cho cung xa
    if (score > bestScore) {
      bestScore = score;
      bestArc = a;
    }
  }

  outHeadingDeg = (float)bestArc * 10.0f + 5.0f;   // giữa cung
  outMaxMm = (uint16_t)arcMaxMm[bestArc];
}

/**
 * Tính coverage estimate (số cung LIDAR "trống" = max range > 4m).
 * Khi số cung trống < 10% tổng số cung → coi như đã quét hết.
 */
inline float estimateCoverageFromLidar() {
  constexpr uint8_t  NUM_ARCS = 36;
  constexpr uint8_t  MIN_QUALITY = 10;
  float    arcMaxMm[NUM_ARCS] = {0};
  uint16_t arcCount[NUM_ARCS] = {0};

  for (uint16_t i = 0; i < g_x3Scan.count; i++) {
    const LidarPoint &p = g_x3Scan.points[i];
    if (p.quality < MIN_QUALITY || p.distanceMm == 0) continue;
    if (p.distanceMm > 8000) continue;
    float deg = p.angleRad * 180.0f / (float)M_PI;
    if (deg < 0) deg += 360.0f;
    uint8_t arc = (uint8_t)(deg / 10.0f);
    if (arc >= NUM_ARCS) arc = NUM_ARCS - 1;
    if (p.distanceMm > arcMaxMm[arc]) arcMaxMm[arc] = p.distanceMm;
    arcCount[arc]++;
  }

  uint8_t emptyArcs = 0;
  uint8_t validArcs = 0;
  for (uint8_t a = 0; a < NUM_ARCS; a++) {
    if (arcCount[a] >= 3) {
      validArcs++;
      if (arcMaxMm[a] >= 4000) emptyArcs++;
    }
  }

  if (validArcs == 0) return 0;
  // Coverage = 1 - (emptyArcs / validArcs) * 100
  float pct = 100.0f - ((float)emptyArcs / (float)validArcs) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

/** Path tracking: lưu điểm mỗi ~30cm, update bounding box */
inline void trackPath(float x, float y) {
  State& s = getState();
  if (s.pathCount == 0) {
    s.pathX[0] = x;
    s.pathY[0] = y;
    s.pathCount = 1;
    return;
  }
  const float &lx = s.pathX[s.pathCount - 1];
  const float &ly = s.pathY[s.pathCount - 1];
  float dx = x - lx, dy = y - ly;
  if (dx*dx + dy*dy < 0.09f) return;     // 0.3m^2 = chỉ lưu khi đi xa > 0.3m
  if (s.pathCount >= AUTO_EXPLORE_PATH_MAX_POINTS) return;
  s.pathX[s.pathCount] = x;
  s.pathY[s.pathCount] = y;
  s.pathCount++;
}

/* ===== Init / lifecycle ================================================ */

inline void start() {
  State& s = getState();
  s.fsm          = ST_CRUISE;
  s.startMs      = millis();
  s.lastProgressMs = 0;
  s.scanSessionId = (uint32_t)(micros() & 0xFFFFFFFF);
  s.totalScanSeq = 0;
  s.totalDistanceM = 0;
  s.prevX = g_pose.x;
  s.prevY = g_pose.y;
  s.wallTargetHeading = g_pose.headingRad;
  s.hasWallTarget = true;
  s.pathCount = 0;
  s.pathBoundingAreaM2 = 0;
  s.emptyArcCount = 0;
  s.coveragePct = 0;
  s.stateEnterMs = millis();
  s.spinStartHeading = g_pose.headingRad;
  s.spinTargetHeading = g_pose.headingRad;
  s.hasSpinTarget = false;
  s.active = true;
  pidYawReset();
  Serial.printf("[AUTO-EXPLORE] ▶ BẮT ĐẦU QUÉT MAP — session #%u\n", s.scanSessionId);
}

inline void stop() {
  State& s = getState();
  s.fsm = ST_DONE;
  s.active = false;
  botStop();
  Serial.printf("[AUTO-EXPLORE] ⏹ DỪNG THỦ CÔNG — pct=%.1f%%, dist=%.2fm, dur=%lus\n",
                s.coveragePct, s.totalDistanceM, (millis() - s.startMs) / 1000UL);
}

inline bool isActive() {
  return getState().active;
}

/* ===== Main tick (gọi mỗi SAFE_LOOP_MS từ taskControl) =============== */

inline void tick() {
  State& s = getState();
  if (!s.active) return;

  const uint32_t now = millis();
  const uint32_t durMs = now - s.startMs;

  /* --- Update path tracking --- */
  float dx = g_pose.x - s.prevX;
  float dy = g_pose.y - s.prevY;
  s.totalDistanceM += sqrtf(dx*dx + dy*dy);
  s.prevX = g_pose.x;
  s.prevY = g_pose.y;
  trackPath(g_pose.x, g_pose.y);

  /* --- Estimate coverage từ LIDAR mỗi 500ms --- */
  static uint32_t s_lastCoverageMs = 0;
  if (now - s_lastCoverageMs >= 500) {
    s.coveragePct = estimateCoverageFromLidar();
    s.lastCoverageMs = now;
  }

  /* --- Kiểm tra điều kiện kết thúc --- */
  // Chỉ cho phép hoàn thành do coverage sau ít nhất 30 giây chạy thực tế (tránh nảy 100% ở phòng hẹp ngay giây đầu)
  if (durMs >= 30000u && s.coveragePct >= AUTO_EXPLORE_TARGET_COVERAGE_PCT) {
    Serial.println(F("[AUTO-EXPLORE] ✅ Đã quét đạt target coverage — hoàn thành"));
    s.fsm = ST_DONE;
  }
  if (durMs >= AUTO_EXPLORE_MAX_DURATION_MS) {
    Serial.println(F("[AUTO-EXPLORE] ⚠ Timeout max duration — dừng"));
    s.fsm = ST_DONE;
  }

  /* --- FSM --- */
  switch (s.fsm) {
    case ST_CRUISE: {
      // 1) US brake: nếu US trước quá gần → AVOID_US
      int16_t frontCm = obsFrontCm();
      if (obsCmValid(frontCm) && frontCm < (int16_t)(AUTO_EXPLORE_US_BRAKE_MM / 10)) {
        botStop();
        pidYawReset();
        s.fsm = ST_AVOID_US;
        s.stateEnterMs = now;
        Serial.println(F("[AUTO-EXPLORE] Front US < 25cm → AVOID_US"));
        break;
      }

      // 2) Bám tường phải: dùng US.PHẢI (khi có) hoặc LIDAR cung 60-120°
      // Nếu mất tường phải (> 80cm) → SPIN_DETECT
      uint16_t rightMm = x3MinInArc(90.0f, 30.0f);    // cung 60-120° (phía phải)
      if (rightMm < 100 || rightMm > 1500) {
        // Mất tường phải → xoay tìm hướng mới
        botStop();
        pidYawReset();
        s.fsm = ST_SPIN_DETECT;
        s.stateEnterMs = now;
        s.spinStartHeading = g_pose.headingRad;
        s.hasSpinTarget = false;
        Serial.println(F("[AUTO-EXPLORE] Mất tường phải → SPIN_DETECT"));
        break;
      }

      // 3) Bám tường: nếu quá gần → rẽ phải nhẹ, nếu quá xa → rẽ trái nhẹ
      float wallErr = (float)rightMm - (float)AUTO_EXPLORE_MIN_WALL_DIST_MM;
      // wallErr > 0: quá xa (rẽ trái → steer âm với wall ở bên phải)
      // wallErr < 0: quá gần (rẽ phải → steer dương)
      float steer = -wallErr / 100.0f;        // scale: 100mm = 1.0 steer
      steer = constrain(steer, -40.0f, 40.0f);

      uint16_t spd = g_state.waypointBaseSpeed;
      if (spd == 0) spd = g_state.autoBaseSpeed;
      if (spd == 0) spd = g_state.baseSpeed;
      if (spd == 0) spd = (uint16_t)((uint32_t)PWM_MAX * 55 / 100);

      // Đi thẳng + bẻ lái MƯỢT (giống manual) — qua botDriveSmoothNormal.
      botDriveSmoothNormal((int16_t)steer, 100, spd);
      break;
    }

    case ST_SPIN_DETECT: {
      uint16_t spd = (g_state.rotateBaseSpeed > 0) ? g_state.rotateBaseSpeed : g_state.baseSpeed;
      if (spd == 0) spd = (uint16_t)((uint32_t)PWM_MAX * 55 / 100);

      // Xoay 360° thu thập LIDAR, tìm hướng trống xa nhất
      if (!s.hasSpinTarget) {
        // Phase 1: xoay liên tục, đợi đủ 1 vòng LIDAR (~500ms)
        botDriveSmoothNormal(40, 0, spd);
        if (now - s.stateEnterMs >= 700) {
          float hDeg; uint16_t maxMm;
          findOpenHeading(hDeg, maxMm);
          s.spinTargetHeading = hDeg * (float)M_PI / 180.0f;
          s.hasSpinTarget = true;
          s.stateEnterMs = now;
          Serial.printf("[AUTO-EXPLORE] Tìm thấy hướng trống: %.0f° (max=%umm)\n", hDeg, maxMm);
        }
      } else {
        // Phase 2: xoay về target heading
        float dh = wpNormalizeAngle(s.spinTargetHeading - g_pose.headingRad);
        if (fabsf(dh) < 0.05f) {
          // Đã đến nơi → CRUISE
          botStop();
          pidYawReset();
          s.fsm = ST_CRUISE;
          s.stateEnterMs = now;
          s.wallTargetHeading = g_pose.headingRad;
          s.hasWallTarget = true;
          Serial.println(F("[AUTO-EXPLORE] → CRUISE (đã xoay đến hướng trống)"));
        } else {
          int sign = (dh > 0) ? 1 : -1;
          botDriveSmoothNormal((int16_t)(sign * 50), 0, spd);
        }
        // Timeout phase 2: 5s
        if (now - s.stateEnterMs >= 5000) {
          botStop();
          s.fsm = ST_CRUISE;
          s.stateEnterMs = now;
          Serial.println(F("[AUTO-EXPLORE] Timeout spin → CRUISE"));
        }
      }
      break;
    }

    case ST_AVOID_US: {
      // Lùi thẳng giữ heading 500ms (MƯỢT), sau đó SPIN_DETECT
      static float s_backHeading = 0.f;
      static bool  s_backHave = false;
      if (!s_backHave) {
        s_backHeading = g_pose.headingRad;
        s_backHave = true;
        pidYawReset();
      }
      float dt_s = (float)SAFE_LOOP_MS / 1000.f;
      float steer = pidYawCompute(s_backHeading, g_pose.headingRad, dt_s);
      steer = constrain(steer, -70.0f, 70.0f);
      uint16_t spd = g_state.swerveBaseSpeed;
      if (spd == 0) spd = (uint16_t)((uint32_t)PWM_MAX * 40 / 100);

      // Lùi mượt — giữ heading, không giật.
      botDriveSmoothNormal((int16_t)steer, -100, spd);

      if (now - s.stateEnterMs >= 600) {
        s_backHave = false;
        botStop();
        s.fsm = ST_SPIN_DETECT;
        s.stateEnterMs = now;
        s.hasSpinTarget = false;
        s.spinStartHeading = g_pose.headingRad;
        Serial.println(F("[AUTO-EXPLORE] Backup done → SPIN_DETECT"));
      }
      break;
    }

    case ST_DONE: {
      // Kết thúc: stop robot, mark inactive, callback sẽ publish scan_complete
      botStop();
      s.active = false;
      Serial.printf("[AUTO-EXPLORE] ✅ HOÀN THÀNH — pct=%.1f%%, dist=%.2fm, dur=%lus\n",
                    s.coveragePct, s.totalDistanceM, durMs / 1000UL);
      break;
    }
  }

  /* --- Publish progress mỗi 5s --- */
  if (now - s.lastProgressMs >= AUTO_EXPLORE_PROGRESS_INTERVAL_MS) {
    s.lastProgressMs = now;
    // Callback sẽ publish MQTT scan_progress (xem AutoExplore binding)
    extern void autoExplorePublishProgress(float pct, float dist, uint32_t durMs, uint8_t state);
    autoExplorePublishProgress(s.coveragePct, s.totalDistanceM, durMs, (uint8_t)s.fsm);
  }
}

}  // namespace autoExplore

#endif // AUTO_EXPLORE_H