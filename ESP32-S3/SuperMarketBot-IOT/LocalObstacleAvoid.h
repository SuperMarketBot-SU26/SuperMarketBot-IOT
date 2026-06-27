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

extern uint16_t autoSpeedPwm();

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
/** SR04: Tính điểm 2 bên sườn và lách tránh về hướng rộng hơn. */
static inline bool oaPickSideAndSwerve(OaContext &ctx, uint32_t now) {
  int16_t lf = g_state.usLF;
  int16_t rf = g_state.usRF;
  int16_t lr = g_state.usLR;
  int16_t rr = g_state.usRR;

  // Tính điểm 2 bên sườn (chỉ dùng cảm biến trước-bên để tránh nhiễu/vật cản từ bức tường phía sau)
  int16_t leftScore  = lf;
  int16_t rightScore = rf;

  // Ngưỡng an toàn tối thiểu để có thể lách tránh (stop distance + 3cm dự phòng)
  const int16_t minSwerveDist = (int16_t)(US_STOP_CM + 3);

  bool leftOk  = (leftScore >= minSwerveDist);
  bool rightOk = (rightScore >= minSwerveDist);

  // Nếu cả 2 bên đều quá chật hẹp -> Chuyển sang Lùi xe
  if (!leftOk && !rightOk) {
    ctx.state = OA_BLOCKED;
    ctx.stateT0 = now;
    ctx.attempts = OA_MAX_ATTEMPTS;
    Serial.printf("[OA-US] Ca 2 ben deu ket cung (<%dcm) -> Blocked (lui xe). Trai:%d Phai:%d\n", 
                  (int)minSwerveDist, (int)leftScore, (int)rightScore);
    return false;
  }

  // So sánh điểm số để chọn hướng an toàn nhất (hướng rộng hơn)
  if (rightScore >= leftScore) {
    ctx.swerveDir = 1;  // Lách sang Phải
  } else {
    ctx.swerveDir = -1; // Lách sang Trái
  }

  // Giữ nguyên heading khi strafe — không xoay xe
  ctx.swerveTarget = ctx.headingBefore;
  ctx.poseXBefore = g_pose.x;
  ctx.poseYBefore = g_pose.y;
  ctx.state = OA_SWERVE;
  ctx.stateT0 = now;
  Serial.printf("[OA-US] Quyet dinh trut ngang sang: %s (Diem Trai: %dcm, Diem Phai: %dcm)\n", 
                ctx.swerveDir > 0 ? "PHAI" : "TRAI", (int)leftScore, (int)rightScore);
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
#if !USE_HC_SR04_HARDWARE
  const bool hardRear  = obsRearBlocked();
#endif

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
    // Chỉ kiểm tra các góc bị chặn trong hướng di chuyển (trượt ngang)
    bool sideBlocked = false;
    if (ctx.swerveDir > 0) { // Đang trượt sang phải -> kiểm tra RF
      sideBlocked = (obsCmValid(g_state.usRF) && g_state.usRF < (int16_t)US_STOP_CM);
    } else if (ctx.swerveDir < 0) { // Đang trượt sang trái -> kiểm tra LF
      sideBlocked = (obsCmValid(g_state.usLF) && g_state.usLF < (int16_t)US_STOP_CM);
    }
    if (sideBlocked) {
      botStop();
      ctx.state = OA_BLOCKED;
      ctx.stateT0 = now;
      return OA_RES_BLOCKED;
    }
    // Nếu đường phía trước đã thông thoáng
    if (obsPathClear(frontCm)) {
      ctx.state = OA_PASS; // trượt thêm 1 chút rồi tiếp tục thẳng
      ctx.stateT0 = now;
      return OA_RES_RUNNING;
    }
    // Tiến chéo thông minh: dạt ngang tối đa lực, đi tiến nhẹ nếu phía trước còn trống tương đối
    int16_t strafeCmd = (ctx.swerveDir > 0) ? 100 : -100;
    int16_t fwdCmd = 0;
    if (obsCmValid(frontCm) && frontCm >= 20) {
      fwdCmd = 45; // Tiến chéo mượt mà
    }
    uint16_t spd = g_state.swerveBaseSpeed;
    if (spd == 0) spd = oaPct2Pwm(45); // Mặc định 45% nếu chưa chỉnh
    botDriveMecanum(strafeCmd, fwdCmd, 0, spd);
    return OA_RES_RUNNING;
  }

  case OA_PASS: {
    // Chỉ kiểm tra các góc bị chặn trong hướng di chuyển (trượt ngang)
    bool sideBlocked = false;
    if (ctx.swerveDir > 0) { // Đang trượt sang phải -> kiểm tra RF
      sideBlocked = (obsCmValid(g_state.usRF) && g_state.usRF < (int16_t)US_STOP_CM);
    } else if (ctx.swerveDir < 0) { // Đang trượt sang trái -> kiểm tra LF
      sideBlocked = (obsCmValid(g_state.usLF) && g_state.usLF < (int16_t)US_STOP_CM);
    }
    if (sideBlocked) {
      botStop();
      ctx.state = OA_BLOCKED;
      ctx.stateT0 = now;
      return OA_RES_BLOCKED;
    }
    // Vật cản lại xuất hiện phía trước → quay lại strafe
    if (!obsPathClear(frontCm)) {
      ctx.state = OA_SWERVE;
      ctx.stateT0 = now;
      return OA_RES_RUNNING;
    }
    // Trượt thêm 600ms để vượt bề rộng vật cản trước khi tiến thẳng
    if (now - ctx.stateT0 >= 600u) {
      botStop();
      ctx.state = OA_IDLE;
      ctx.attempts = 0;
      Serial.println(F("[OA] Vuot vat can bang strafe hoan tat."));
      return OA_RES_DONE;
    }
    int16_t strafeCmd = (ctx.swerveDir > 0) ? 100 : -100;
    int16_t fwdCmd = 45; // Vượt chướng ngại vật mượt mà
    uint16_t spd = g_state.swerveBaseSpeed;
    if (spd == 0) spd = oaPct2Pwm(45); // Mặc định 45% nếu chưa chỉnh
    botDriveMecanum(strafeCmd, fwdCmd, 0, spd);
    return OA_RES_RUNNING;
  }

  case OA_BLOCKED:
    // Bị chặn cứng -> lập tức kích hoạt lùi xe (backup) để thoát kẹt ngay
    oaReset(ctx);
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

#if USE_HC_SR04_HARDWARE
  // Sử dụng trượt ngang (strafe) thay vì xoay khi né cạnh tường để đi thẳng mượt mà
  int16_t strafe = 0;
  if (obsCmValid(obsLeftCm()) && obsLeftCm() < (int16_t)SAFE_SIDE_AVOID_CM) {
    strafe += 40; // Trượt sang phải nếu bên trái gần tường
  }
  if (obsCmValid(obsRightCm()) && obsRightCm() < (int16_t)SAFE_SIDE_AVOID_CM) {
    strafe -= 40; // Trượt sang trái nếu bên phải gần tường
  }
  if (strafe > 80) strafe = 80;
  if (strafe < -80) strafe = -80;

  float dt_s = (float)SAFE_LOOP_MS * 0.001f;
  float pidOut = pidSpeedCompute(pwmToEstMps(cruisePwm), robotActualSpeedMps(), dt_s);
  int32_t run = (int32_t)cruisePwm + (int32_t)pidOut;
  if (run < 0) run = 0;
  if (run > (int32_t)PWM_MAX) run = (int32_t)PWM_MAX;

  // Closed-loop yaw control using IMU (if enabled) to keep robot driving straight
  int16_t steer = 0;
#if USE_IMU_MPU6050
  extern bool g_imuEnabled;
  if (g_imuEnabled) {
    float yawOut = pidYawCompute(ctx.cruiseHeading, g_pose.headingRad, dt_s);
    steer = (int16_t)constrain(yawOut, -100, 100);
  }
#endif

  if (strafe != 0) {
    // Tiến thẳng đồng thời trượt ngang né tường và bù góc xoay
    botDriveMecanum(strafe, 100, steer, (uint16_t)run);
  } else {
    // Tiến thẳng đồng thời bù góc xoay để giữ hướng đi hoàn toàn thẳng
    botDriveMecanum(0, 100, steer, (uint16_t)run);
  }
  return true;
#else
  float dt_s = (float)SAFE_LOOP_MS * 0.001f;
  float pidOut = pidSpeedCompute(pwmToEstMps(cruisePwm), robotActualSpeedMps(), dt_s);
  int32_t run = (int32_t)cruisePwm + (int32_t)pidOut;
  if (run < 0) run = 0;
  if (run > (int32_t)PWM_MAX) run = (int32_t)PWM_MAX;

  // Closed-loop yaw control using IMU (if enabled) to keep robot driving straight
  int16_t steer = 0;
#if USE_IMU_MPU6050
  extern bool g_imuEnabled;
  if (g_imuEnabled) {
    float yawOut = pidYawCompute(ctx.cruiseHeading, g_pose.headingRad, dt_s);
    steer = (int16_t)constrain(yawOut, -100, 100);
  }
#endif

  botDriveMecanum(0, 100, steer, (uint16_t)run);
  return true;
#endif
}

#endif // LOCAL_OBSTACLE_AVOID_H
