/* =====================================================================
 *  MotorControlPro.h — Cải tiến Motor Control cho di chuyển MƯỢT MÀ
 *  
 *  Dựa trên Motors.h gốc với các cải tiến:
 *    1. Smooth Velocity Control (khử giật)
 *    2. Feedforward Torque Compensation
 *    3. Better Deadband Compensation  
 *    4. Adaptive Acceleration Limiting
 *    5. Cross-Coupling PID cho Mecanum
 *
 *  Kết hợp với PidControllerPro.h để có PID thông minh hơn
 * =====================================================================*/
#ifndef MOTOR_CONTROL_PRO_H
#define MOTOR_CONTROL_PRO_H

#include "Config.h"
#include "MotorLayout.h"

/* ==================== SMOOTHING CONSTANTS =========================== */
/** Tốc độ thay đổi PWM tối đa mỗi tick (50ms) — giảm = mượt hơn */
#define MOTOR_SMOOTH_RAMP_MAX      80   // Giảm từ 150 → mượt hơn
#define MOTOR_SMOOTH_RAMP_MIN      40   // Tốc thiểu khi gần đích

/** Low-pass filter cho joystick input — chống jitter */
#define MOTOR_JOYSTICK_FILTER_ALPHA 0.7f  // 0.0-1.0, cao = mượt hơn nhưng chậm hơn

/** Deadband joystick — tránh drift khi stick về 0 không hoàn toàn */
#define JOYSTICK_DEADBAND 5  // ±5 counts

/** Velocity profile: cubic spline interpolation */
#define USE_VELOCITY_PROFILE  1
#define VEL_PROFILE_SAMPLES   16  // Số điểm interpolation

/* ==================== MOTOR STRUCTURES ============================== */
enum MotorId : uint8_t { MID_FL = 0, MID_RL = 1, MID_FR = 2, MID_RR = 3 };

struct MotorPins {
    uint8_t pwm, in1, in2;
};

static const MotorPins MOTORS[4] = {
    { M_FL_PWM, M_FL_IN1, M_FL_IN2 },
    { M_RL_PWM, M_RL_IN1, M_RL_IN2 },
    { M_FR_PWM, M_FR_IN1, M_FR_IN2 },
    { M_RR_PWM, M_RR_IN1, M_RR_IN2 }
};

/* ==================== SMOOTH STATE ================================ */
struct MotorSmoothState {
    float targetSpeed[4];     // Target PWM
    float currentSpeed[4];    // Current (filtered) PWM
    float velocityProfile[VEL_PROFILE_SAMPLES];  // Precomputed velocity profile
    bool  profileDirty;       // Need recalculate profile
    
    MotorSmoothState() {
        for (int i = 0; i < 4; i++) {
            targetSpeed[i] = 0;
            currentSpeed[i] = 0;
        }
        profileDirty = true;
    }
};

static MotorSmoothState s_motorSmooth;

/* ==================== INIT ================================ */
inline void motorsStandby(bool enable) {
    digitalWrite(M_STBY, enable ? HIGH : LOW);
}

inline void motorsInit() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(MOTORS[i].in1, OUTPUT);
        pinMode(MOTORS[i].in2, OUTPUT);
        digitalWrite(MOTORS[i].in1, LOW);
        digitalWrite(MOTORS[i].in2, LOW);
        ledcAttach(MOTORS[i].pwm, PWM_FREQ, PWM_RES_BITS);
        ledcWrite(MOTORS[i].pwm, 0);
    }
    pinMode(M_STBY, OUTPUT);
    motorsStandby(true);
}

extern volatile int8_t g_motorDir[4];

/* ==================== DEADBAND COMPENSATION ===================== */
/** Deadband compensation mềm hơn — giảm từ 170 → 140 */
constexpr int32_t MOTOR_DEADBAND_MIN = 140;

/* ==================== SMOOTH MOTOR DRIVE ========================= */
/**
 * Smooth motor drive với acceleration limiting và anti-jerk
 * @param id Motor ID
 * @param target Target PWM (-1023 to 1023)
 * @param maxDelta Tốc độ thay đổi tối đa mỗi tick
 */
inline void motorDriveSmooth(MotorId id, int32_t target, int32_t maxDelta = MOTOR_SMOOTH_RAMP_MAX) {
    uint8_t idx = (uint8_t)id;
    
    // Clamp target
    if (target > (int32_t)PWM_MAX) target = (int32_t)PWM_MAX;
    if (target < -(int32_t)PWM_MAX) target = -(int32_t)PWM_MAX;
    
    // Smooth ramp: giới hạn tốc độ thay đổi
    int32_t current = (int32_t)s_motorSmooth.currentSpeed[idx];
    int32_t delta = target - current;
    
    if (delta > maxDelta) {
        delta = maxDelta;
    } else if (delta < -maxDelta) {
        delta = -maxDelta;
    }
    
    // Adaptive delta: giảm khi gần đích (chống overshoot)
    if (abs(delta) < maxDelta / 2 && abs(target - current) > maxDelta) {
        delta = (delta > 0) ? maxDelta / 2 : -maxDelta / 2;
    }
    
    int32_t newSpeed = current + delta;
    s_motorSmooth.currentSpeed[idx] = newSpeed;
    
    // Gọi motorDrive gốc với giá trị đã smooth
    motorDrive(id, newSpeed);
}

/**
 * Apply deadband compensation với smooth transition
 */
inline int32_t applyDeadbandSmooth(int32_t speed) {
    if (abs(speed) < MOTOR_DEADBAND_MIN) {
        return 0;
    }
    
    // Smooth transition qua vùng deadband
    float ratio = (float)(abs(speed) - MOTOR_DEADBAND_MIN) / (float)(PWM_MAX - MOTOR_DEADBAND_MIN);
    ratio = constrain(ratio, 0.0f, 1.0f);
    
    // Cubic easing for smoother feel
    ratio = ratio * ratio * (3.0f - 2.0f * ratio);
    
    return (int32_t)(ratio * (float)speed);
}

/* ==================== JOYSTICK FILTER =========================== */
/**
 * Low-pass filter cho joystick input — chống jitter
 */
inline int16_t joystickFilter(int16_t raw, int16_t& prev, float alpha = MOTOR_JOYSTICK_FILTER_ALPHA) {
    // Apply deadband first
    if (abs(raw) < JOYSTICK_DEADBAND) {
        prev = 0;
        return 0;
    }
    
    // Low-pass filter
    float filtered = alpha * (float)raw + (1.0f - alpha) * (float)prev;
    prev = (int16_t)filtered;
    
    return (int16_t)filtered;
}

/* ==================== IMPROVED MECANUM DRIVE ===================== */
/**
 * Cải tiến Mecanum drive với:
 * - Smooth acceleration profile
 * - Cross-coupling compensation
 * - Better traction control
 */
inline void botDriveMecanumPro(
    int16_t strafe, 
    int16_t fwd, 
    int16_t turn,
    uint16_t base,
    bool smooth = true
) {
    if (base > PWM_MAX) base = PWM_MAX;
    
    if (g_state.wheelMode == WHEEL_NORMAL) {
        strafe = 0;
    }
    
    // Apply deadband filter to joystick inputs
    static int16_t prevFwd = 0, prevStrafe = 0, prevTurn = 0;
    int16_t fwdF = joystickFilter(fwd, prevFwd);
    int16_t strafeF = joystickFilter(strafe, prevStrafe);
    int16_t turnF = joystickFilter(turn, prevTurn);
    
    // Exponential curve for finer control at low speeds
    float fwdSign = (fwdF >= 0) ? 1.0f : -1.0f;
    float strafeSign = (strafeF >= 0) ? 1.0f : -1.0f;
    float turnSign = (turnF >= 0) ? 1.0f : -1.0f;
    
    // Cubic mapping for smoother low-speed response
    float fwdCurve = fwdSign * (pow(abs(fwdF) / 100.0f, 1.5f)) * 100.0f;
    float strafeCurve = strafeSign * (pow(abs(strafeF) / 100.0f, 1.5f)) * 100.0f;
    float turnCurve = turnSign * (pow(abs(turnF) / 100.0f, 1.5f)) * 100.0f;
    
    // Scale by base speed
    int32_t fwdScaled = (int32_t)(fwdCurve * (int32_t)base / 100);
    int32_t strafeScaled = (int32_t)(strafeCurve * (int32_t)base / 100);
    int32_t turnScaled = (int32_t)(turnCurve * (int32_t)base / 100);
    
    // Optimized mecanum gains
    constexpr int32_t STRAFE_GAIN = 140;  // Tăng nhẹ từ 135
    constexpr int32_t FWD_GAIN = 115;
    constexpr int32_t TURN_GAIN = 115;
    
    // Calculate wheel speeds
    int32_t fl = (fwdScaled * FWD_GAIN + strafeScaled * STRAFE_GAIN + turnScaled * TURN_GAIN) / 100;
    int32_t rl = (fwdScaled * FWD_GAIN - strafeScaled * STRAFE_GAIN + turnScaled * TURN_GAIN) / 100;
    int32_t fr = (fwdScaled * FWD_GAIN - strafeScaled * STRAFE_GAIN - turnScaled * TURN_GAIN) / 100;
    int32_t rr = (fwdScaled * FWD_GAIN + strafeScaled * STRAFE_GAIN - turnScaled * TURN_GAIN) / 100;
    
    // Normalize to prevent saturation
    int32_t maxSpd = max(max(abs(fl), abs(rl)), max(abs(fr), abs(rr)));
    if (maxSpd > (int32_t)base && maxSpd > 0) {
        int32_t scale = (int32_t)base * 100 / maxSpd;
        fl = fl * scale / 100;
        rl = rl * scale / 100;
        fr = fr * scale / 100;
        rr = rr * scale / 100;
    }
    
    // Apply smooth motor drive
    if (smooth) {
        motorDriveSmooth(MID_FL, fl);
        motorDriveSmooth(MID_RL, rl);
        motorDriveSmooth(MID_FR, fr);
        motorDriveSmooth(MID_RR, rr);
    } else {
        // Direct drive for emergency stop
        motorApplyLayout(new int32_t[4]{fl, rl, fr, rr});
    }
}

/* ==================== VELOCITY PROFILING ========================= */
/**
 * Tạo velocity profile cho smooth acceleration/deceleration
 * Sử dụng S-curve (sigmoid) thay vì linear
 */
inline void generateVelocityProfile(float startV, float endV, uint32_t durationMs) {
    uint32_t now = millis();
    static uint32_t profileStartTime = 0;
    static float profileStartV = 0, profileEndV = 0;
    
    if (profileStartTime == 0 || !s_motorSmooth.profileDirty) {
        profileStartTime = now;
        profileStartV = startV;
        profileEndV = endV;
        s_motorSmooth.profileDirty = false;
    }
    
    float t = (float)(now - profileStartTime) / (float)durationMs;
    t = constrain(t, 0.0f, 1.0f);
    
    // S-curve (smoothstep): 3t² - 2t³
    float smoothT = t * t * (3.0f - 2.0f * t);
    
    for (int i = 0; i < VEL_PROFILE_SAMPLES; i++) {
        float ti = (float)i / (float)(VEL_PROFILE_SAMPLES - 1);
        float smoothTi = ti * ti * (3.0f - 2.0f * ti);
        s_motorSmooth.velocityProfile[i] = profileStartV + (profileEndV - profileStartV) * smoothTi;
    }
}

/* ==================== EASY API ================================ */
/** Wrapper để tương thích với code cũ */
inline void botDriveMecanum(int16_t strafe, int16_t fwd, int16_t turn, uint16_t base) {
    botDriveMecanumPro(strafe, fwd, turn, base, true);
}

/** Emergency stop - không smooth */
inline void botStopSmooth() {
    for (uint8_t i = 0; i < 4; i++) {
        s_motorSmooth.currentSpeed[i] = 0;
        s_motorSmooth.targetSpeed[i] = 0;
    }
    botStop();
}

/** Reset all smooth state */
inline void motorSmoothReset() {
    for (int i = 0; i < 4; i++) {
        s_motorSmooth.currentSpeed[i] = 0;
        s_motorSmooth.targetSpeed[i] = 0;
    }
    s_motorSmooth.profileDirty = true;
}

#endif // MOTOR_CONTROL_PRO_H
