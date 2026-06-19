/* =====================================================================
 *  LocalObstacleAvoid.h — Né vật: SR04 4 góc (chọn trái/phải) hoặc quét LiDAR
 *  Dùng chung MODE_AUTO và MODE_WAYPOINT.
 * =====================================================================*/
#ifndef LOCAL_OBSTACLE_AVOID_H
#define LOCAL_OBSTACLE_AVOID_H

#include "Config.h"
#include "ObstacleSensors.h"
#include "Localization.h"
#include "PidController.h"
#include "Motors.h"
#include <math.h>

enum OaFsmState : uint8_t {
  OA_IDLE = 0,
  OA_SCAN_CW,
  OA_SCAN_CCW,
  OA_SWERVE,
  OA_PASS,
  OA_BLOCKED
};

enum OaTickResult : uint8_t {
  OA_RES_RUNNING = 0,
  OA_RES_DONE,
  OA_RES_BLOCKED,
  OA_RES_IDLE
};

struct OaContext {
  OaFsmState state         = OA_IDLE;
  uint32_t   stateT0       = 0;
  float      headingBefore = 0.f;
  float      cruiseHeading = 0.f;
  float      scanMaxRight  = 0.f;
  float      scanMaxLeft   = 0.f;
  float      swerveTarget  = 0.f;
  float      poseXBefore   = 0.f;
  float      poseYBefore   = 0.f;
  uint8_t    attempts      = 0;
  int8_t     swerveDir     = 0;
  uint8_t    pathClearStreak = 0;
};

static inline float oaNorm(float a) {
  while (a >  (float)M_PI) a -= 2.f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.f * (float)M_PI;
  return a;
}

static inline float oaAngleDiff(float target, float current) {
  return oaNorm(target - current);
}

static inline uint16_t oaPct2Pwm(uint8_t pct) {
  return (uint16_t)((uint32_t)PWM_MAX * (uint32_t)pct / 100u);
}

static inline float oaDistMoved(const OaContext &ctx) {
  float dx = g_pose.x - ctx.poseXBefore;
  float dy = g_pose.y - ctx.poseYBefore;
  return sqrtf(dx * dx + dy * dy);
}

inline void oaReset(OaContext &ctx) {
  ctx.state = OA_IDLE;
  ctx.attempts = 0;
  ctx.pathClearStreak = 0;
}

#if USE_HC_SR04_HARDWARE
/** SR04: đọc ngay trái/phải (4 góc) — không cần xoay quét. */
static inline bool oaPickSideAndSwerve(OaContext &ctx, uint32_t now) {
  ctx.scanMaxLeft  = (float)obsLeftCm();
  ctx.scanMaxRight = (float)obsRightCm();
  const float need = (float)US_PATH_CLEAR_CM;

  bool leftOk  = (ctx.scanMaxLeft  >= need);
  bool rightOk = (ctx.scanMaxRight >= need);

  Serial.printf("[OA-US] L=%.0f R=%.0f cm (can>=%d)\n",
                ctx.scanMaxLeft, ctx.scanMaxRight, (int)US_PATH_CLEAR_CM);

  if (!leftOk && !rightOk) {
    ctx.state = OA_BLOCKED;
    ctx.stateT0 = now;
    ctx.attempts = OA_MAX_ATTEMPTS;
    Serial.println(F("[OA-US] Hai ben chat — blocked."));
    return false;
  }

  if (leftOk && rightOk) {
    ctx.swerveDir = (ctx.scanMaxRight >= ctx.scanMaxLeft) ? 1 : -1;
  } else {
    ctx.swerveDir = rightOk ? 1 : -1;
  }

  float swerveRad = (float)OA_SWERVE_ANGLE_DEG * (float)M_PI / 180.f;
  ctx.swerveTarget = oaNorm(ctx.headingBefore + ctx.swerveDir * swerveRad);
  ctx.poseXBefore = g_pose.x;
  ctx.poseYBefore = g_pose.y;
  ctx.state = OA_SWERVE;
  ctx.stateT0 = now;
  Serial.printf("[OA-US] Lach %s\n", ctx.swerveDir > 0 ? "PHAI" : "TRAI");
  return true;
}
#endif

inline bool oaBegin(OaContext &ctx, int16_t frontCm, uint32_t now) {
  if (!obsOaTriggered(frontCm)) {
    return false;
  }
  botStop();
  pidSpeedReset();

  if (ctx.attempts >= OA_MAX_ATTEMPTS) {
    ctx.state = OA_BLOCKED;
    ctx.stateT0 = now;
    Serial.println(F("[OA] Max attempts — blocked."));
    return true;
  }

  ctx.attempts++;
  ctx.headingBefore = g_pose.headingRad;
  ctx.cruiseHeading = g_pose.headingRad;
  ctx.scanMaxRight = 0.f;
  ctx.scanMaxLeft  = 0.f;

#if USE_HC_SR04_HARDWARE
  (void)oaPickSideAndSwerve(ctx, now);
  Serial.printf("[OA-US] Truoc %dcm — lach (try %d/%d)\n",
                (int)frontCm, (int)ctx.attempts, (int)OA_MAX_ATTEMPTS);
#else
  ctx.state = OA_SCAN_CW;
  ctx.stateT0 = now;
  Serial.printf("[OA] Obstacle %dcm — scan CW (try %d/%d)\n",
                (int)frontCm, (int)ctx.attempts, (int)OA_MAX_ATTEMPTS);
#endif
  return true;
}

inline OaTickResult oaTick(OaContext &ctx, int16_t frontCm, uint32_t now) {
  const bool hardFront = obsFrontBlocked();
  const bool hardRear  = obsRearBlocked();

  switch (ctx.state) {
  case OA_IDLE:
    return OA_RES_IDLE;

#if !USE_HC_SR04_HARDWARE
  case OA_SCAN_CW: {
    if (obsCmValid(frontCm) && (float)frontCm > ctx.scanMaxRight) {
      ctx.scanMaxRight = (float)frontCm;
    }
    float rotated = fabsf(oaNorm(g_pose.headingRad - ctx.headingBefore));
    float maxRad = (float)OA_SCAN_ANGLE_DEG * (float)M_PI / 180.f;
    if (rotated >= maxRad) {
      botStop();
      ctx.state = OA_SCAN_CCW;
      ctx.stateT0 = now;
      return OA_RES_RUNNING;
    }
    if (!hardFront && !hardRear) {
      botRotateCW(oaPct2Pwm(OA_SCAN_SPEED_PCT));
    }
    return OA_RES_RUNNING;
  }

  case OA_SCAN_CCW: {
    if (obsCmValid(frontCm) && (float)frontCm > ctx.scanMaxLeft) {
      ctx.scanMaxLeft = (float)frontCm;
    }
    float maxRad = (float)OA_SCAN_ANGLE_DEG * (float)M_PI / 180.f;
    float fromOrigCCW = oaNorm(ctx.headingBefore - g_pose.headingRad);
    if (fromOrigCCW >= maxRad) {
      botStop();
      bool rightOk = (ctx.scanMaxRight >= (float)PATH_CLEAR_MIN_CM);
      bool leftOk  = (ctx.scanMaxLeft  >= (float)PATH_CLEAR_MIN_CM);
      if (rightOk || leftOk) {
        if (rightOk && leftOk) {
          ctx.swerveDir = (ctx.scanMaxRight >= ctx.scanMaxLeft) ? 1 : -1;
        } else {
          ctx.swerveDir = rightOk ? 1 : -1;
        }
        float swerveRad = (float)OA_SWERVE_ANGLE_DEG * (float)M_PI / 180.f;
        ctx.swerveTarget = oaNorm(ctx.headingBefore + ctx.swerveDir * swerveRad);
        ctx.poseXBefore = g_pose.x;
        ctx.poseYBefore = g_pose.y;
        ctx.state = OA_SWERVE;
        ctx.stateT0 = now;
      } else {
        ctx.state = OA_BLOCKED;
        ctx.stateT0 = now;
        ctx.attempts = OA_MAX_ATTEMPTS;
        return OA_RES_BLOCKED;
      }
      return OA_RES_RUNNING;
    }
    if (!hardFront && !hardRear) {
      botRotateCCW(oaPct2Pwm(OA_SCAN_SPEED_PCT));
    }
    return OA_RES_RUNNING;
  }
#endif

  case OA_SWERVE: {
    if (hardFront || obsAnyCornerBlocked()) {
      botStop();
      pidSpeedReset();
      ctx.state = OA_BLOCKED;
      ctx.stateT0 = now;
      return OA_RES_BLOCKED;
    }
    float headingErr = oaAngleDiff(ctx.swerveTarget, g_pose.headingRad);
    if (fabsf(headingErr) > 0.12f) {
      uint16_t rotPwm = oaPct2Pwm(OA_SCAN_SPEED_PCT);
      if (headingErr > 0) botRotateCW(rotPwm);
      else botRotateCCW(rotPwm);
      return OA_RES_RUNNING;
    }
    if (oaDistMoved(ctx) >= OA_SWERVE_DIST_M) {
      botStop();
      ctx.swerveTarget = ctx.headingBefore;
      ctx.poseXBefore = g_pose.x;
      ctx.poseYBefore = g_pose.y;
      ctx.state = OA_PASS;
      ctx.stateT0 = now;
      return OA_RES_RUNNING;
    }
    uint16_t pwm = oaPct2Pwm(OA_SWERVE_SPEED_PCT);
    float dt_s = (float)SAFE_LOOP_MS * 0.001f;
    float pidOut = pidSpeedCompute(pwmToEstMps(pwm), robotActualSpeedMps(), dt_s);
    int32_t run = (int32_t)pwm + (int32_t)pidOut;
    if (run < 0) run = 0;
    if (run > (int32_t)PWM_MAX) run = (int32_t)PWM_MAX;
    botForward((uint16_t)run);
    return OA_RES_RUNNING;
  }

  case OA_PASS: {
    if (hardFront || obsAnyCornerBlocked()) {
      botStop();
      pidSpeedReset();
      ctx.state = OA_BLOCKED;
      ctx.stateT0 = now;
      return OA_RES_BLOCKED;
    }
    float headingErr = oaAngleDiff(ctx.swerveTarget, g_pose.headingRad);
    if (fabsf(headingErr) > 0.12f) {
      uint16_t rotPwm = oaPct2Pwm(OA_SCAN_SPEED_PCT);
      if (headingErr > 0) botRotateCW(rotPwm);
      else botRotateCCW(rotPwm);
      return OA_RES_RUNNING;
    }
    if (oaDistMoved(ctx) >= OA_PASS_DIST_M) {
      botStop();
      pidSpeedReset();
      ctx.cruiseHeading = ctx.headingBefore;
      ctx.pathClearStreak = 0;
      ctx.state = OA_IDLE;
      ctx.attempts = 0;
      Serial.println(F("[OA] Passed — resume."));
      return OA_RES_DONE;
    }
    uint16_t pwm = oaPct2Pwm(OA_SWERVE_SPEED_PCT);
    float dt_s = (float)SAFE_LOOP_MS * 0.001f;
    float pidOut = pidSpeedCompute(pwmToEstMps(pwm), robotActualSpeedMps(), dt_s);
    int32_t run = (int32_t)pwm + (int32_t)pidOut;
    if (run < 0) run = 0;
    if (run > (int32_t)PWM_MAX) run = (int32_t)PWM_MAX;
    botForward((uint16_t)run);
    return OA_RES_RUNNING;
  }

  case OA_BLOCKED:
    botStop();
    if (obsPathClear(frontCm)) {
      ctx.attempts = 0;
      ctx.state = OA_IDLE;
      return OA_RES_IDLE;
    }
    return OA_RES_BLOCKED;

  default:
    ctx.state = OA_IDLE;
    return OA_RES_IDLE;
  }
}

inline bool oaCruiseForward(OaContext &ctx, int16_t frontCm, uint16_t cruisePwm) {
  if (!obsPathClear(frontCm)) {
    ctx.pathClearStreak = 0;
    botStop();
    return false;
  }
  if (ctx.pathClearStreak < 255) ctx.pathClearStreak++;
  if (ctx.pathClearStreak < OA_PATH_CLEAR_STREAK) {
    botStop();
    return false;
  }

  /* Heading-hold PID: giữ hướng bằng encoder — "dò line ảo" */
  float dt_s = (float)SAFE_LOOP_MS * 0.001f;
  float holdCorrection = pidHoldCompute(ctx.cruiseHeading, g_pose.headingRad, dt_s);

  float err = oaAngleDiff(ctx.cruiseHeading, g_pose.headingRad);
  if (fabsf(err) > 0.18f) {
    uint16_t rotPwm = oaPct2Pwm(OA_SCAN_SPEED_PCT);
    if (err > 0) botRotateCW(rotPwm);
    else botRotateCCW(rotPwm);
    return true;
  }

  /* Speed PID */
  float pidOut = pidSpeedCompute(pwmToEstMps(cruisePwm), robotActualSpeedMps(), dt_s);
  int32_t run = (int32_t)cruisePwm + (int32_t)pidOut;
  if (run < 0) run = 0;
  if (run > (int32_t)PWM_MAX) run = (int32_t)PWM_MAX;

  /* Side steering từ SR04 (nếu có) */
#if USE_HC_SR04_HARDWARE
  int16_t steer = 0;
  if (obsCmValid(obsLeftCm()) && obsLeftCm() < (int16_t)SAFE_SIDE_AVOID_CM) {
    steer += 35;
  }
  if (obsCmValid(obsRightCm()) && obsRightCm() < (int16_t)SAFE_SIDE_AVOID_CM) {
    steer -= 35;
  }
  if (steer >  80) steer =  80;
  if (steer < -80) steer = -80;

  if (steer != 0) {
    /* SR04 phát hiện sát tường → bẻ lái mạnh, cộng thêm heading hold */
    int16_t turn = steer + (int16_t)holdCorrection;
    if (turn >  100) turn =  100;
    if (turn < -100) turn = -100;
    botDriveMecanum(0, 80, turn, (uint16_t)run);
  } else {
    botDriveMecanum(0, 80, (int16_t)holdCorrection, (uint16_t)run);
  }
#else
  /* LiDAR-only: chỉ heading hold, đi thẳng "như dò line ảo" */
  int16_t turn = (int16_t)holdCorrection;
  if (turn >  100) turn =  100;
  if (turn < -100) turn = -100;
  botDriveMecanum(0, 80, turn, (uint16_t)run);
#endif
  return true;
}

#endif // LOCAL_OBSTACLE_AVOID_H
