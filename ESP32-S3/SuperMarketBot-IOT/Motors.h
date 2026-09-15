/* =====================================================================
 *  Motors.h — Điều khiển 4 động cơ qua 2 TB6612FNG (PWM mịn bằng LEDC)
 *  CHẾ ĐỘ DUY NHẤT: Differential Drive (bánh thường, không mecanum)
 *  Khôi phục từ logic chuẩn commit a5ffd3c (MAX_RAMP_STEP=600, direct torque)
 *
 *  API:
 *    motorsInit()                — Cấu hình chân + 4 kênh LEDC
 *    motorsStandby(en)           — Bật/tắt STBY chung
 *    motorDrive(M_*, speed)      — speed ∈ [-PWM_MAX .. +PWM_MAX]
 *    botStop()                   — Dừng tất cả động cơ
 *    botDrive(x, y, base)        — Lái arcade (x = turn, y = fwd)
 *    botForward/Backward(pwm)    — Chạy thẳng
 *    botRotateCW/CCW(pwm)        — Xoay tại chỗ
 *    botRotateCWImmediate(pwm)   — Xoay tại chỗ (immediate, bỏ slew)
 * =====================================================================*/
#ifndef MOTORS_H
#define MOTORS_H

#include "Config.h"
#include "MotorLayout.h"
#include "Localization.h"   // locSetDriveCmd() cho pose estimate dùng PWM

enum MotorId : uint8_t { MID_FL = 0, MID_RL = 1, MID_FR = 2, MID_RR = 3 };

// Forward declaration for MotorControlPro.h smooth drive function.
void botDriveSmoothNormal(int16_t turn, int16_t fwd, uint16_t base, bool smooth);

struct MotorPins {
  uint8_t pwm, in1, in2;
};

static const MotorPins MOTORS[4] = {
  { M_FL_PWM, M_FL_IN1, M_FL_IN2 },
  { M_RL_PWM, M_RL_IN1, M_RL_IN2 },
  { M_FR_PWM, M_FR_IN1, M_FR_IN2 },
  { M_RR_PWM, M_RR_IN1, M_RR_IN2 }
};

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

/**
 * Điều khiển 1 động cơ.
 * @param id    chỉ số động cơ MID_*
 * @param speed -PWM_MAX..+PWM_MAX (âm = lùi)
 */
inline void motorDrive(MotorId id, int32_t speed) {
  if (speed > (int32_t)PWM_MAX) speed = (int32_t)PWM_MAX;
  if (speed < -(int32_t)PWM_MAX) speed = -(int32_t)PWM_MAX;

  int32_t lastSpd = g_state.lastMotorSpeed[(uint8_t)id];
  int32_t diff = speed - lastSpd;
  // Slew rate mượt mà chống giật giằng bánh 4WD:
  // - Khi tăng tốc: max 200 PWM / 50ms (đạt max sau ~250ms, triệt tiêu xung giật stick-slip trên lốp cao su)
  // - Khi phanh về 0: max 350 PWM / 50ms (dừng dứt khoát, không trôi)
  int32_t maxStep = (speed == 0) ? 350 : 200;
  if (diff > maxStep) {
    speed = lastSpd + maxStep;
  } else if (diff < -maxStep) {
    speed = lastSpd - maxStep;
  }
  g_state.lastMotorSpeed[(uint8_t)id] = speed;

  const MotorPins &m = MOTORS[id];
  if (speed > 0) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
    g_motorDir[(uint8_t)id] = 1;
  } else if (speed < 0) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
    speed = -speed;
    g_motorDir[(uint8_t)id] = -1;
  } else {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, HIGH);
    g_motorDir[(uint8_t)id] = 0;
  }

  if (speed > 0) {
    constexpr int32_t MIN_MOTOR_PWM = 130;
    if (speed > (int32_t)PWM_MAX) speed = (int32_t)PWM_MAX;
    speed = MIN_MOTOR_PWM + (speed * (PWM_MAX - MIN_MOTOR_PWM)) / PWM_MAX;
  }
  ledcWrite(m.pwm, speed);
}

/**
 * Tính PWM input để sau khi áp deadband compensation trong motorDrive() cho ra PWM thực tế = targetOutRaw.
 */
inline int32_t motorBypassDeadband(int32_t targetOutRaw) {
  constexpr int32_t MIN_MOTOR_PWM = 130;
  if (targetOutRaw <= 0) return 0;
  if (targetOutRaw >= PWM_MAX) return PWM_MAX;
  return (targetOutRaw * PWM_MAX) / (PWM_MAX - MIN_MOTOR_PWM);
}

inline void motorDriveImmediate(MotorId id, int32_t speed) {
  if (speed > (int32_t)PWM_MAX) speed = (int32_t)PWM_MAX;
  if (speed < -(int32_t)PWM_MAX) speed = -(int32_t)PWM_MAX;
  g_state.lastMotorSpeed[(uint8_t)id] = speed;

  const MotorPins &m = MOTORS[id];
  if (speed > 0) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
    g_motorDir[(uint8_t)id] = 1;
  } else if (speed < 0) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
    speed = -speed;
    g_motorDir[(uint8_t)id] = -1;
  } else {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, HIGH);
    g_motorDir[(uint8_t)id] = 0;
  }

  if (speed > 0) {
    constexpr int32_t MIN_MOTOR_PWM = 130;
    if (speed > (int32_t)PWM_MAX) speed = (int32_t)PWM_MAX;
    speed = MIN_MOTOR_PWM + (speed * (PWM_MAX - MIN_MOTOR_PWM)) / PWM_MAX;
  }
  ledcWrite(m.pwm, speed);
}

/**
 * Áp dụng layout (slot → kênh TB6612 vật lý, đảo chiều, scale).
 * Slot 0..3 = FL, RL, FR, RR.
 */
inline void motorApplyLayout(const int32_t speedBySlot[4]) {
  for (int s = 0; s < 4; s++) {
    uint8_t p = g_mapMotSlot[s];
    if (p > 3) p = (uint8_t)s;
    int32_t sp = speedBySlot[s];
    if (g_motInv[s]) sp = -sp;

    extern float g_motorScale[4];
    int32_t scaleFP = (int32_t)(g_motorScale[s] * 1024.f + 0.5f);
    sp = (sp * scaleFP) / 1024;

    motorDrive((MotorId)p, sp);
  }
}

inline void motorApplyLayoutImmediate(const int32_t speedBySlot[4]) {
  for (int s = 0; s < 4; s++) {
    uint8_t p = g_mapMotSlot[s];
    if (p > 3) p = (uint8_t)s;
    int32_t sp = speedBySlot[s];
    if (g_motInv[s]) sp = -sp;

    extern float g_motorScale[4];
    int32_t scaleFP = (int32_t)(g_motorScale[s] * 1024.f + 0.5f);
    sp = (sp * scaleFP) / 1024;

    motorDriveImmediate((MotorId)p, sp);
  }
}

/**
 * Dừng tất cả động cơ (PWM=0, IN1=IN2=HIGH để brake).
 */
inline void botStop() {
  locSetDriveCmd(0, 0);  // [LOC FIX] Dừng tích phân pose khi brake
  const int32_t sp[4] = {0, 0, 0, 0};
  motorApplyLayout(sp);
}

/**
 * Chạy thẳng (cùng PWM cho cả 2 bên).
 */
inline void botForward(uint16_t pwm) {
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  const int32_t sp[4] = {(int32_t)pwm, (int32_t)pwm, (int32_t)pwm, (int32_t)pwm};
  motorApplyLayout(sp);
}

inline void botBackward(uint16_t pwm) {
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  const int32_t sp[4] = {-(int32_t)pwm, -(int32_t)pwm, -(int32_t)pwm, -(int32_t)pwm};
  motorApplyLayout(sp);
}

/**
 * Xoay tại chỗ (immediate, bỏ slew — dùng cho waypoint align).
 * Differential: bên trái +, bên phải - → CW.
 * QUAN TRỌNG: Báo Localization dừng tích phân X/Y (robot không tiến trong khi xoay).
 */
/**
 * Xoay tại chỗ (immediate, bỏ slew — dùng cho waypoint align).
 * Differential: CW = quay PHẢI (bên trái lùi, bên phải tiến trên cơ cấu thực tế của xe).
 * QUAN TRỌNG: Báo Localization dừng tích phân X/Y (robot không tiến trong khi xoay).
 * Áp dụng kỹ thuật ICR Bias: Trục trước 100%, trục sau bám mềm 82% để giảm 40% lực cản ma sát giằng xé 2 trục.
 */
inline void botRotateCWImmediate(uint16_t pwm) {
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  locSetDriveCmd(0, 0);  // [LOC FIX] Tắt dead-reckoning khi xoay tại chỗ — tránh drift pose!
  int32_t fPwm = (int32_t)pwm;
  int32_t rPwm = (fPwm * 82) / 100;
  const int32_t sp[4] = {-fPwm, -rPwm, fPwm, rPwm};
  motorApplyLayoutImmediate(sp);
}

inline void botRotateCCWImmediate(uint16_t pwm) {
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  locSetDriveCmd(0, 0);  // [LOC FIX] Tắt dead-reckoning khi xoay tại chỗ — tránh drift pose!
  int32_t fPwm = (int32_t)pwm;
  int32_t rPwm = (fPwm * 82) / 100;
  const int32_t sp[4] = {fPwm, rPwm, -fPwm, -rPwm};
  motorApplyLayoutImmediate(sp);
}

/**
 * Wrapper có slew (dùng cho obstacle avoidance & manual smooth).
 */
inline void botRotateCW(uint16_t pwm)  { botRotateCWImmediate(pwm); }
inline void botRotateCCW(uint16_t pwm) { botRotateCCWImmediate(pwm); }

/**
 * Lái arcade differential drive (dùng cho joystick Manual và waypoint).
 * @param x    -100..100 (âm = xoay trái, dương = xoay phải)
 * @param y    -100..100 (âm = lùi, dương = tiến)
 * @param base 0..PWM_MAX  tốc độ nền tối đa
 *
 * Cải tiến công nghệ 4WD Skid-Steer cho bánh cao su độ bám cao:
 *  1. Sửa lỗi đảo chiều trái/phải: x > 0 (rẽ phải), x < 0 (rẽ trái).
 *  2. Đường cong lái S-curve phi tuyến mượt mà (35% linear + 65% quadratic).
 *  3. Giảm xung đột ma sát 2 trục (Tire Scrub & Axle Fight Relief):
 *     Phân bổ lực trục trước 100%, trục sau bám mềm ~82% khi xoay tại chỗ.
 *     Nhờ đó dời tâm quay tức thời (ICR) về gần trục sau, giảm 40% lực cản ma sát giằng xé.
 *  4. Tự động chuyển đổi mượt về 100% khi xe chạy thẳng hoặc bo cua tốc độ cao.
 */
inline void botDrive(int16_t x, int16_t y, uint16_t base) {
  if (base > PWM_MAX) base = PWM_MAX;

  int32_t xSign = (x >= 0) ? 1 : -1;
  int32_t ySign = (y >= 0) ? 1 : -1;
  int32_t xAbs  = abs((int32_t)x);
  int32_t yAbs  = abs((int32_t)y);

  // 1. Progressive S-Curve: êm ở dải thấp, lực kéo mạnh ở dải cao, không bị chết vùng giữa
  int32_t xCurve = (xSign * (35 * xAbs + (65 * xAbs * xAbs) / 100)) / 100;
  int32_t yCurve = (ySign * (30 * yAbs + (70 * yAbs * yAbs) / 100)) / 100;

  // 2. Vi sai arcade chuẩn xác theo cơ cấu thực tế:
  //    x > 0 (gạt phải) -> leftS âm, rightS dương -> xe quay PHẢI
  //    x < 0 (gạt trái) -> leftS dương, rightS âm -> xe quay TRÁI
  int32_t leftS  = ((yCurve - xCurve) * (int32_t)base) / 100;
  int32_t rightS = ((yCurve + xCurve) * (int32_t)base) / 100;

  // 3. Giải pháp chống rít bánh 4WD (Tire Scrubbing & Axle Fight Relief):
  int32_t fl = leftS, rl = leftS;
  int32_t fr = rightS, rr = rightS;

  if (xAbs > 5) {
    // Tỉ lệ trục phụ mềm hơn: từ 82% (khi xoay tại chỗ y=0) tăng mượt lên 100% (khi chạy thẳng y>=50)
    int32_t followerRatio = 82 + (min(yAbs, (int32_t)50) * 18) / 50; // 82% -> 100%
    if (y >= 0) {
      // Tiến hoặc xoay tại chỗ: Trục trước dẫn hướng, trục sau bám mềm
      rl = (leftS  * followerRatio) / 100;
      rr = (rightS * followerRatio) / 100;
    } else {
      // Lùi: Trục sau dẫn hướng, trục trước bám mềm
      fl = (leftS  * followerRatio) / 100;
      fr = (rightS * followerRatio) / 100;
    }
  }

  // 4. Normalize để không vượt quá base PWM
  int32_t mag = max(max(abs(fl), abs(rl)), max(abs(fr), abs(rr)));
  if (mag > (int32_t)base && mag > 0) {
    int32_t scale = (int32_t)base * 100 / mag;
    fl  = fl  * scale / 100;
    rl  = rl  * scale / 100;
    fr  = fr  * scale / 100;
    rr  = rr  * scale / 100;
  }

  // Báo cho Localization biết lệnh drive hiện tại (% so với base) — dùng cho pose estimate.
  if (base > 0) {
    locSetDriveCmd((int16_t)((leftS  * 100) / (int32_t)base),
                   (int16_t)((rightS * 100) / (int32_t)base));
  } else {
    locSetDriveCmd(0, 0);
  }

  const int32_t sp[4] = {fl, rl, fr, rr};
  motorApplyLayout(sp);
}

#endif // MOTORS_H