/* =====================================================================
 *  PidController.h — PID tốc độ cho AN_CRUISE (tự hành)
 *
 *  Dùng 1 PID scalar cho tốc độ trung bình (speed PID):
 *    setpoint = vTargetMps (m/s mong muốn)
 *    measured = vActualMps (trung bình 4 bánh, tính từ RPM)
 *    output   = điều chỉnh PWM thêm/bớt
 *
 *  Dùng 1 PID scalar cho heading (heading/yaw PID) — Phase 3 bổ sung:
 *    setpoint = headingTargetRad
 *    measured = g_pose.headingRad
 *    output   = điều chỉnh steer (cmdX)
 *
 *  API:
 *    pidSpeedReset()  / pidSpeedCompute(setpoint, measured, dt_s)
 *    pidYawReset()    / pidYawCompute(setpoint, measured, dt_s)
 * =====================================================================*/
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Config.h"

/* ==================== Hằng số mặc định ============================= */
/** Speed PID — điều chỉnh PWM để đạt v mong muốn trong AUTO CRUISE */
#define PID_SPEED_KP    0.f
#define PID_SPEED_KI    0.f
#define PID_SPEED_KD    0.f
#define PID_SPEED_I_MAX 0.8f  // anti-windup (giới hạn tích phân, đơn vị mét)
#define PID_SPEED_OUT_MAX ((float)PWM_MAX)

/** Yaw/Heading PID — P-only để tránh oscillation từ gyro derivative noise */
#define PID_YAW_KP      15.f   // Giảm từ 20 → P-only nhẹ, mượt
#define PID_YAW_KI      0.f
#define PID_YAW_KD      0.f    // Tắt D — gyro derivative noise gây oscillation
#define PID_YAW_I_MAX   0.f    // Không dùng I
#define PID_YAW_OUT_MAX 100.f  // cmdX range -100..100

/* ==================== Struct PID =================================== */
struct PidState {
  float kp, ki, kd;
  float iMax, outMax;
  float integral;
  float prevError;
  bool  firstRun;
};

static PidState s_pidSpeed = {
  PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD,
  PID_SPEED_I_MAX, PID_SPEED_OUT_MAX,
  0.f, 0.f, true
};

static PidState s_pidYaw = {
  PID_YAW_KP, PID_YAW_KI, PID_YAW_KD,
  PID_YAW_I_MAX, PID_YAW_OUT_MAX,
  0.f, 0.f, true
};

/* ==================== Core PID compute ============================= */
static inline float pidCompute(PidState &pid, float setpoint, float measured, float dt_s) {
  if (dt_s <= 0.f) return 0.f;

  float err = setpoint - measured;

  /* Derivative — bỏ qua vòng đầu tránh spike */
  float derivative = 0.f;
  if (!pid.firstRun) {
    derivative = (err - pid.prevError) / dt_s;
  }
  pid.firstRun = false;
  pid.prevError = err;

  /* Integral với anti-windup (clamping) */
  pid.integral += err * dt_s;
  if (pid.integral >  pid.iMax) pid.integral =  pid.iMax;
  if (pid.integral < -pid.iMax) pid.integral = -pid.iMax;

  float out = pid.kp * err + pid.ki * pid.integral + pid.kd * derivative;

  /* Clamp output */
  if (out >  pid.outMax) out =  pid.outMax;
  if (out < -pid.outMax) out = -pid.outMax;

  return out;
}

/* ==================== Speed PID ================================== */
inline void pidSpeedReset() {
  s_pidSpeed.integral = 0.f;
  s_pidSpeed.prevError = 0.f;
  s_pidSpeed.firstRun = true;
}

/**
 * @param targetMps  Tốc độ mong muốn (m/s)
 * @param actualMps  Tốc độ thực (tính từ RPM trung bình)
 * @param dt_s       Chu kỳ điều khiển (giây)
 * @return PWM delta — cộng vào baseSpeed
 */
inline float pidSpeedCompute(float targetMps, float actualMps, float dt_s) {
  return pidCompute(s_pidSpeed, targetMps, actualMps, dt_s);
}

/** Chuyển đổi RPM → m/s */
inline float rpmToMps(float rpm) {
  return (rpm / 60.0f) * WHEEL_CIRC_M;
}

/** Tốc độ thực tế (trung bình 4 bánh, m/s) */
inline float robotActualSpeedMps() {
  float rpm = (g_state.rpmFL + g_state.rpmFR + g_state.rpmRL + g_state.rpmRR) * 0.25f;
  return rpmToMps(rpm);
}

/** Chuyển đổi PWM → m/s ước lượng (dùng làm setpoint khi chưa có encoder chuẩn) */
inline float pwmToEstMps(uint16_t pwm) {
  /* Ước tính tuyến tính: PWM_MAX tương ứng ~0.8 m/s (chỉnh thực địa) */
  return (float)pwm * 0.8f / (float)PWM_MAX;
}

/* ==================== Yaw PID (Phase 3) =========================== */
inline void pidYawReset() {
  s_pidYaw.integral = 0.f;
  s_pidYaw.prevError = 0.f;
  s_pidYaw.firstRun = true;
}

/**
 * @param targetRad  Heading mong muốn (rad)
 * @param actualRad  Heading thực từ g_pose.headingRad
 * @param dt_s       Chu kỳ điều khiển
 * @return cmdX delta [-100, 100]
 */
inline float pidYawCompute(float targetRad, float actualRad, float dt_s) {
  /* Normalise error vào (-π, π] */
  float err = targetRad - actualRad;
  while (err >  (float)M_PI) err -= 2.f * (float)M_PI;
  while (err < -(float)M_PI) err += 2.f * (float)M_PI;
  s_pidYaw.prevError = err;  // override để tương thích chính xác trạng thái trước đây
  return pidCompute(s_pidYaw, actualRad + err, actualRad, dt_s); // pass err thông qua setpoint trick
}

#endif // PID_CONTROLLER_H
