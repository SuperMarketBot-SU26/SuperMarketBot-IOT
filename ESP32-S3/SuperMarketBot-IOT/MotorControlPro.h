/* =====================================================================
 *  MotorControlPro.h — Cải tiến Motor Control cho di chuyển MƯỢT MÀ
 *
 *  File này CHỈ chứa các hàm BỔ SUNG cho Motors.h — KHÔNG định nghĩa lại
 *  MotorId/MotorPins/MOTORS[]/motorsStandby/motorsInit (đã có trong Motors.h).
 *
 *  Bao gồm:
 *    1) botDriveMecanumPro(strafe, fwd, turn, base)  — mecanum với smooth ramp
 *    2) botDriveMecanum(...)                           — wrapper backward-compat
 *    3) botStopSmooth()                                — soft stop
 *    4) motorSmoothReset()                             — reset state smooth
 *    5) motorDriveSmooth(MotorId, target)              — 1 motor với accel ramp
 *    6) applyDeadbandSmooth(speed)                     — deadband easing
 *    7) joystickFilter(raw, prev, alpha)               — low-pass filter
 *    8) botDriveSmoothNormal(turn, fwd, base)          — entry-point cho Mode 1 & 2
 *
 *  Kết hợp với PidControllerPro.h để có PID thông minh hơn
 * =====================================================================*/
#ifndef MOTOR_CONTROL_PRO_H
#define MOTOR_CONTROL_PRO_H

#include "Config.h"
#include "MotorLayout.h"
#include "Motors.h"   // MotorId, MotorPins, MOTORS[], motorDrive(), motorApplyLayout()

/* ==================== SMOOTHING CONSTANTS =========================== */
/** Tốc độ thay đổi PWM tối đa mỗi tick (50ms) — giảm = mượt hơn */
#define MOTOR_SMOOTH_RAMP_MAX      80   // Giảm từ 150 → mượt hơn
#define MOTOR_SMOOTH_RAMP_MIN      40   // Tối thiểu khi gần đích

/** Low-pass filter cho joystick input — chống jitter */
#define MOTOR_JOYSTICK_FILTER_ALPHA 0.7f  // 0.0-1.0, cao = mượt hơn nhưng chậm hơn

/** Deadband joystick — tránh drift khi stick về 0 không hoàn toàn */
#define JOYSTICK_DEADBAND 5  // ±5 counts

/** Velocity profile: cubic spline interpolation */
#define USE_VELOCITY_PROFILE  1
#define VEL_PROFILE_SAMPLES   16  // Số điểm interpolation

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

    // Gọi motorDrive gốc với giá trị đã smooth (motorDrive nằm trong Motors.h)
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

    // [NOTE] Project này dùng bánh thường (WHEEL_NORMAL) → ép strafe = 0.
    // Không khai báo WHEEL_NORMAL enum (đã comment out trong Config.h) — bỏ check wheelMode.
    // Nếu sau này cần mecanum thật, thêm enum WHEEL_NORMAL/WHEEL_MECANUM vào Config.h.

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

    // Scale by base speed & rotate speed (Dùng rotateBaseSpeed riêng cho lực xoay góc)
    uint16_t rotBase = (g_state.rotateBaseSpeed > 0) ? g_state.rotateBaseSpeed : base;
    int32_t fwdScaled = (int32_t)(fwdCurve * (int32_t)base / 100);
    int32_t strafeScaled = (int32_t)(strafeCurve * (int32_t)base / 100);
    int32_t turnScaled = (int32_t)(turnCurve * (int32_t)rotBase / 100);

    // Optimized mecanum gains
    constexpr int32_t STRAFE_GAIN = 140;  // Tăng nhẹ từ 135
    constexpr int32_t FWD_GAIN = 115;
    constexpr int32_t TURN_GAIN = 135;   // Tăng gain xoay góc giúp xoay bốc cho robot nặng

    // Calculate wheel speeds
    int32_t fl = (fwdScaled * FWD_GAIN + strafeScaled * STRAFE_GAIN + turnScaled * TURN_GAIN) / 100;
    int32_t rl = (fwdScaled * FWD_GAIN - strafeScaled * STRAFE_GAIN + turnScaled * TURN_GAIN) / 100;
    int32_t fr = (fwdScaled * FWD_GAIN - strafeScaled * STRAFE_GAIN - turnScaled * TURN_GAIN) / 100;
    int32_t rr = (fwdScaled * FWD_GAIN + strafeScaled * STRAFE_GAIN - turnScaled * TURN_GAIN) / 100;

    // Normalize to prevent saturation (Giới hạn công suất theo max(base, rotBase))
    int32_t maxAllowedSpd = max((int32_t)base, (int32_t)rotBase);
    int32_t maxSpd = max(max(abs(fl), abs(rl)), max(abs(fr), abs(rr)));
    if (maxSpd > maxAllowedSpd && maxSpd > 0) {
        int32_t scale = maxAllowedSpd * 100 / maxSpd;
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
        const int32_t sp[4] = {fl, rl, fr, rr};
        motorApplyLayout(sp);
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

    if (profileStartTime == 0 || s_motorSmooth.profileDirty) {
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

/**
 * Smooth drive cho WHEEL_NORMAL (bánh thường, differential drive).
 *
 * Đây là entry-point DUY NHẤT mà Mode 1 (AUTO_EXPLORE) và Mode 2 (WAYPOINT)
 * phải gọi để di chuyển — đảm bảo MƯỢT giống Mode MANUAL:
 *   1) Joystick filter (low-pass α=0.7)
 *   2) Cubic curve (mượn cảm giác tay lái)
 *   3) Deadband easing (khử giật khi tốc độ thấp)
 *   4) Accel limiter (PWM ramp 80/tick)
 *   5) Differential drive formula (fl=rl=left, fr=rr=right)
 *
 * Forward declaration ở Motors.h:25 — definition ở đây.
 */
inline void botDriveSmoothNormal(int16_t turn, int16_t fwd, uint16_t base, bool smooth = true) {
    if (base > PWM_MAX) base = PWM_MAX;

    // 1) Wheel mode WHEEL_NORMAL → ép strafe = 0 (không dùng mecanum).
    //    (WHEEL_NORMAL enum đã được comment-out trong Config.h, project dùng fixed differential.)

    // 2) Apply joystick filter (low-pass α=0.7) — chống jitter.
    static int16_t prevFwd = 0, prevTurn = 0;
    const int16_t fwdF   = joystickFilter(fwd,  prevFwd);
    const int16_t turnF  = joystickFilter(turn, prevTurn);

    // 3) Cubic mapping (mượn cảm giác tay lái).
    float fwdSign   = (fwdF  >= 0) ? 1.0f : -1.0f;
    float turnSign  = (turnF >= 0) ? 1.0f : -1.0f;
    float fwdCurve  = fwdSign  * (pow(abs(fwdF)  / 100.0f, 1.5f)) * 100.0f;
    float turnCurve = turnSign * (pow(abs(turnF) / 100.0f, 1.5f)) * 100.0f;

    // 4) Rotate base riêng (cho xoay góc bốc).
    uint16_t rotBase = (g_state.rotateBaseSpeed > 0) ? g_state.rotateBaseSpeed : base;
    int32_t fwdScaled  = (int32_t)(fwdCurve  * (int32_t)base   / 100);
    int32_t turnScaled = (int32_t)(turnCurve * (int32_t)rotBase / 100);

    // 5) Differential drive (không strafe):
    //    left  = fwd + turn
    //    right = fwd - turn
    //    fl=rl=left, fr=rr=right
    constexpr int32_t FWD_GAIN  = 115;
    constexpr int32_t TURN_GAIN = 135;
    int32_t leftS  = (fwdScaled  * FWD_GAIN  + turnScaled * TURN_GAIN) / 100;
    int32_t rightS = (fwdScaled  * FWD_GAIN  - turnScaled * TURN_GAIN) / 100;
    int32_t fl = leftS, rl = leftS;
    int32_t fr = rightS, rr = rightS;

    // 6) Normalize để không saturate motor.
    int32_t maxAllowedSpd = max((int32_t)base, (int32_t)rotBase);
    int32_t maxSpd = max(max(abs(fl), abs(rl)), max(abs(fr), abs(rr)));
    if (maxSpd > maxAllowedSpd && maxSpd > 0) {
        int32_t scale = maxAllowedSpd * 100 / maxSpd;
        fl = fl * scale / 100;
        rl = rl * scale / 100;
        fr = fr * scale / 100;
        rr = rr * scale / 100;
        leftS  = leftS  * scale / 100;
        rightS = rightS * scale / 100;
    }

    // 7) Báo Localization (dùng % so với base).
    if (base > 0) {
        locSetDriveCmd((int16_t)((leftS  * 100) / (int32_t)base),
                       (int16_t)((rightS * 100) / (int32_t)base));
    } else {
        locSetDriveCmd(0, 0);
    }

    // 8) Apply to motors (smooth or immediate).
    if (smooth) {
        motorDriveSmooth(MID_FL, fl);
        motorDriveSmooth(MID_RL, rl);
        motorDriveSmooth(MID_FR, fr);
        motorDriveSmooth(MID_RR, rr);
    } else {
        const int32_t sp[4] = {fl, rl, fr, rr};
        motorApplyLayout(sp);
    }
}

#endif // MOTOR_CONTROL_PRO_H