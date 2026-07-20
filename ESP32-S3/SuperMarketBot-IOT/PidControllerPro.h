/* =====================================================================
 *  PidControllerPro.h — Cải tiến PID Controller cho di chuyển MƯỢT MÀ
 *  
 *  Cải tiến so với PidController.h gốc:
 *    1. Adaptive PID gains theo tốc độ
 *    2. Derivative filtering (chống noise)
 *    3. Bang-bang pre-action trước khi vào PID
 *    4. Better anti-windup
 *    5. Feedforward term
 *    6. Dead-zone cho error nhỏ
 *
 *  API tương thích ngược với PidController.h
 * =====================================================================*/
#ifndef PID_CONTROLLER_PRO_H
#define PID_CONTROLLER_PRO_H

#include "Config.h"

/* ==================== IMPROVED CONSTANTS ========================= */
/** Speed PID — Cải tiến với feedforward */
#define PID_SPEED_KP      8.0f    // Tăng nhẹ từ 0 → P-only responsive
#define PID_SPEED_KI      0.5f    // Thêm I term cho steady-state accuracy
#define PID_SPEED_KD      1.5f    // Thêm D term cho damping
#define PID_SPEED_I_MAX   0.5f    // Giảm từ 0.8 → ít overshoot hơn
#define PID_SPEED_OUT_MAX  ((float)PWM_MAX)

/** Yaw/Heading PID — Cải tiến */
#define PID_YAW_KP        25.0f   // Tăng từ 15 → responsive hơn
#define PID_YAW_KI        0.5f    // Thêm I term
#define PID_YAW_KD        3.0f    // Tăng D term cho damping
#define PID_YAW_I_MAX     10.0f   // Giới hạn I term
#define PID_YAW_OUT_MAX   100.0f  // cmdX range -100..100

/** Derivative filter — lọc noise từ gyro */
#define PID_DERIV_FILTER_ALPHA  0.3f  // Low-pass filter cho derivative (0-1)

/** Dead-zone cho small errors — tránh hunting */
#define PID_SPEED_DEADZONE  0.01f   // m/s — dưới ngưỡng này = 0
#define PID_YAW_DEADZONE    0.02f   // rad — dưới ngưỡng này = 0

/** Pre-act bang-bang threshold */
#define PID_PREACT_THRESH   0.15f   // rad — lệch lớn dùng bang-bang trước

/* ==================== PID STATE ================================ */
struct PidState {
    float kp, ki, kd;
    float iMax, outMax;
    float integral;
    float prevError;
    float prevDerivative;  // Cho derivative filter
    bool  firstRun;
    
    // Cải tiến: adaptive gain
    float kpMin, kpMax;   // Adaptive gain range
    float derivativeFilter; // Filtered derivative
};

static PidState s_pidSpeed = {
    PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD,
    PID_SPEED_I_MAX, PID_SPEED_OUT_MAX,
    0.f, 0.f, 0.f, true,
    PID_SPEED_KP * 0.8f, PID_SPEED_KP * 1.2f,  // kpMin, kpMax
    0.f
};

static PidState s_pidYaw = {
    PID_YAW_KP, PID_YAW_KI, PID_YAW_KD,
    PID_YAW_I_MAX, PID_YAW_OUT_MAX,
    0.f, 0.f, 0.f, true,
    PID_YAW_KP * 0.7f, PID_YAW_KP * 1.3f,  // kpMin, kpMax
    0.f
};

/* ==================== IMPROVED PID COMPUTE ======================= */
/**
 * Cải tiến PID với:
 * - Derivative filtering
 * - Dead-zone
 * - Anti-windup cải tiến
 * - Adaptive gains
 */
static inline float pidComputePro(
    PidState &pid, 
    float setpoint, 
    float measured, 
    float dt_s,
    float deadzone = 0.0f
) {
    if (dt_s <= 0.f) return 0.f;
    
    float err = setpoint - measured;
    
    // Dead-zone: nếu error quá nhỏ, return 0
    if (abs(err) < deadzone) {
        return 0.f;
    }
    
    // Adaptive gain: điều chỉnh Kp theo magnitude của error
    float errMag = abs(err);
    float kp = pid.kp;
    if (errMag < 0.1f) {
        kp = pid.kpMin;  // Nhạy hơn khi gần đích
    } else if (errMag > 0.5f) {
        kp = pid.kpMax;  // Mạnh hơn khi lệch xa
    }
    
    /* Derivative với low-pass filter — giảm gyro noise */
    float derivative = 0.f;
    if (!pid.firstRun) {
        derivative = (err - pid.prevError) / dt_s;
        // Low-pass filter derivative
        pid.derivativeFilter = PID_DERIV_FILTER_ALPHA * derivative 
                              + (1.0f - PID_DERIV_FILTER_ALPHA) * pid.derivativeFilter;
    }
    pid.firstRun = false;
    pid.prevError = err;
    
    /* Integral với anti-windup cải tiến (back-calculation) */
    pid.integral += err * dt_s;
    
    // Clamping anti-windup
    float kpErr = kp * err;
    float kiInt = pid.ki * pid.integral;
    
    // Nếu output bị clamp, giảm integral
    if (kiInt > pid.iMax) {
        pid.integral -= (kiInt - pid.iMax) / pid.ki;
    } else if (kiInt < -pid.iMax) {
        pid.integral -= (kiInt + pid.iMax) / pid.ki;
    }
    
    // Final clamping
    if (pid.integral >  pid.iMax) pid.integral =  pid.iMax;
    if (pid.integral < -pid.iMax) pid.integral = -pid.iMax;
    
    /* Output = P + I + D */
    float out = kp * err + pid.ki * pid.integral + pid.kd * pid.derivativeFilter;
    
    /* Clamp output */
    if (out >  pid.outMax) out =  pid.outMax;
    if (out < -pid.outMax) out = -pid.outMax;
    
    return out;
}

/* ==================== SPEED PID ================================ */
inline void pidSpeedReset() {
    s_pidSpeed.integral = 0.f;
    s_pidSpeed.prevError = 0.f;
    s_pidSpeed.derivativeFilter = 0.f;
    s_pidSpeed.firstRun = true;
}

/**
 * Speed PID với deadzone
 */
inline float pidSpeedCompute(float targetMps, float actualMps, float dt_s) {
    return pidComputePro(s_pidSpeed, targetMps, actualMps, dt_s, PID_SPEED_DEADZONE);
}

/* ==================== YAW PID ================================ */
inline void pidYawReset() {
    s_pidYaw.integral = 0.f;
    s_pidYaw.prevError = 0.f;
    s_pidYaw.derivativeFilter = 0.f;
    s_pidYaw.firstRun = true;
}

/**
 * Yaw PID với deadzone và pre-action
 * @return cmdX delta [-100, 100]
 */
inline float pidYawCompute(float targetRad, float actualRad, float dt_s) {
    // Normalize error
    float err = targetRad - actualRad;
    while (err >  (float)M_PI) err -= 2.f * (float)M_PI;
    while (err < -(float)M_PI) err += 2.f * (float)M_PI;
    
    // Pre-action: bang-bang khi lệch lớn → nhanh vào PID range
    if (abs(err) > PID_PREACT_THRESH) {
        // Sử dụng sign của error làm output tạm
        float preact = (err > 0) ? PID_YAW_OUT_MAX * 0.8f : -PID_YAW_OUT_MAX * 0.8f;
        // Reset integral để tránh windup
        s_pidYaw.integral *= 0.5f;
        return preact;
    }
    
    return pidComputePro(s_pidYaw, targetRad, actualRad, dt_s, PID_YAW_DEADZONE);
}

/* ==================== FEEDFORWARD TERM ========================= */
/**
 * Feedforward term cho speed control
 * Bù trước PWM cần thiết dựa trên target speed
 */
inline float pidSpeedFeedforward(float targetMps) {
    // Linear model: PWM = k * speed + offset
    // Ước tính: PWM_MAX ≈ 0.8 m/s → k ≈ 1023/0.8 ≈ 1279
    constexpr float FF_K = 1279.0f;
    constexpr float FF_OFFSET = (float)MOTOR_DEADBAND_MIN; // Deadband compensation
    
    return FF_K * targetMps + FF_OFFSET;
}

/**
 * Feedforward cho heading (dựa trên velocity)
 */
inline float pidYawFeedforward(float targetOmega, float speed) {
    // Centripetal compensation: cần steer nhiều hơn khi quay nhanh + đi nhanh
    // Đơn giản: steer = k * omega * speed
    constexpr float FF_K = 10.0f;
    return FF_K * targetOmega * speed;
}

/* ==================== TUNING HELPERS ========================== */
inline void pidSpeedTune(float kp, float ki, float kd) {
    s_pidSpeed.kp = kp;
    s_pidSpeed.ki = ki;
    s_pidSpeed.kd = kd;
    s_pidSpeed.kpMin = kp * 0.8f;
    s_pidSpeed.kpMax = kp * 1.2f;
}

inline void pidYawTune(float kp, float ki, float kd) {
    s_pidYaw.kp = kp;
    s_pidYaw.ki = ki;
    s_pidYaw.kd = kd;
    s_pidYaw.kpMin = kp * 0.7f;
    s_pidYaw.kpMax = kp * 1.3f;
}

/* ==================== STATUS ================================ */
struct PidDiagnostics {
    float error;
    float pTerm;
    float iTerm;
    float dTerm;
    float output;
    float derivativeFiltered;
};

inline PidDiagnostics pidSpeedGetDiagnostics() {
    PidDiagnostics d;
    d.error = s_pidSpeed.prevError;
    d.pTerm = s_pidSpeed.kp * s_pidSpeed.prevError;
    d.iTerm = s_pidSpeed.ki * s_pidSpeed.integral;
    d.dTerm = s_pidSpeed.kd * s_pidSpeed.derivativeFilter;
    d.output = d.pTerm + d.iTerm + d.dTerm;
    d.derivativeFiltered = s_pidSpeed.derivativeFilter;
    return d;
}

inline PidDiagnostics pidYawGetDiagnostics() {
    PidDiagnostics d;
    d.error = s_pidYaw.prevError;
    d.pTerm = s_pidYaw.kp * s_pidYaw.prevError;
    d.iTerm = s_pidYaw.ki * s_pidYaw.integral;
    d.dTerm = s_pidYaw.kd * s_pidYaw.derivativeFilter;
    d.output = d.pTerm + d.iTerm + d.dTerm;
    d.derivativeFiltered = s_pidYaw.derivativeFilter;
    return d;
}

#endif // PID_CONTROLLER_PRO_H
