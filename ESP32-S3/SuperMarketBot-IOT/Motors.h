/* =====================================================================
 *  Motors.h — Điều khiển 4 động cơ qua 2 TB6612FNG (PWM mịn bằng LEDC)
 *  CHẾ ĐỘ DUY NHẤT: Differential Drive (bánh thường, không mecanum)
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
// Default argument (smooth=true) chỉ đặt ở definition trong MotorControlPro.h.
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
  constexpr int32_t MAX_RAMP_STEP = 600;
  if (diff > MAX_RAMP_STEP) {
    speed = lastSpd + MAX_RAMP_STEP;
  } else if (diff < -MAX_RAMP_STEP) {
    speed = lastSpd - MAX_RAMP_STEP;
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
inline void botRotateCWImmediate(uint16_t pwm) {
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  locSetDriveCmd(0, 0);  // [LOC FIX] Tắt dead-reckoning khi xoay tại chỗ — tránh drift pose!
  static uint32_t s_cwStartMs = 0;
  static bool s_cwActive = false;
  uint32_t now = millis();
  if (!s_cwActive) { s_cwStartMs = now; s_cwActive = true; }
  uint16_t effPwm = pwm;
  if (now - s_cwStartMs < 120 && effPwm < 720) effPwm = 720;
  const int32_t sp[4] = {(int32_t)effPwm, (int32_t)effPwm, -(int32_t)effPwm, -(int32_t)effPwm};
  motorApplyLayoutImmediate(sp);
}

inline void botRotateCCWImmediate(uint16_t pwm) {
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  locSetDriveCmd(0, 0);  // [LOC FIX] Tắt dead-reckoning khi xoay tại chỗ — tránh drift pose!
  static uint32_t s_ccwStartMs = 0;
  static bool s_ccwActive = false;
  uint32_t now = millis();
  if (!s_ccwActive) { s_ccwStartMs = now; s_ccwActive = true; }
  uint16_t effPwm = pwm;
  if (now - s_ccwStartMs < 120 && effPwm < 720) effPwm = 720;
  const int32_t sp[4] = {-(int32_t)effPwm, -(int32_t)effPwm, (int32_t)effPwm, (int32_t)effPwm};
  motorApplyLayoutImmediate(sp);
}

/**
 * Wrapper có slew (dùng cho obstacle avoidance & manual smooth).
 */
inline void botRotateCW(uint16_t pwm)  { botRotateCWImmediate(pwm); }
inline void botRotateCCW(uint16_t pwm) { botRotateCCWImmediate(pwm); }

/**
 * Lái arcade differential drive (dùng cho joystick Manual và waypoint).
 * @param x    -100..100 (âm = xoay trái/CCW, dương = xoay phải/CW)
 * @param y    -100..100 (âm = lùi, dương = tiến)
 * @param base 0..PWM_MAX  tốc độ nền tối đa
 *
 * Công thức (differential, không strafe):
 *   left  = (y + x) * base / 100     (sau curve phi tuyến + kickstart/gyro boost)
 *   right = (y - x) * base / 100
 *   fl = rl = left
 *   fr = rr = right
 */
inline void botDrive(int16_t x, int16_t y, uint16_t base) {
  if (base > PWM_MAX) base = PWM_MAX;

  int32_t xSign = (x >= 0) ? 1 : -1;
  int32_t ySign = (y >= 0) ? 1 : -1;
  int32_t xCurve = ((int32_t)x * (int32_t)x * xSign) / 100;
  int32_t yCurve = ((int32_t)y * (int32_t)y * ySign) / 100;

  // Lực xoay tại chỗ (y == 0, x != 0): Trên khung 4WD bánh cao su, ma sát trượt ngang rất lớn.
  // Blend thêm thành phần tuyến tính để tránh sụt PWM vào deadzone khi xoay nhẹ.
  const bool isPureRot = (y == 0 && abs(x) > 10);
  if (isPureRot) {
    int32_t xAbs = abs(x);
    xCurve = ((xAbs * xAbs / 100 + xAbs) / 2) * xSign;
  }

  int32_t leftS  = ((yCurve + xCurve) * (int32_t)base) / 100;
  int32_t rightS = ((yCurve - xCurve) * (int32_t)base) / 100;

  // Thuật toán: Xung bứt phá ma sát tĩnh (Kickstart) + Closed-loop Gyro Assist
  static uint32_t s_manRotStartMs = 0;
  static bool s_manWasRotating = false;
  static int32_t s_manGyroBoost = 0;
  static uint32_t s_lastManBoostMs = 0;
  const uint32_t nowMs = millis();

  if (isPureRot) {
    if (!s_manWasRotating) {
      s_manRotStartMs = nowMs;
      s_manWasRotating = true;
      s_manGyroBoost = 0;
      s_lastManBoostMs = nowMs;
    }

    // Sau 140ms, nếu IMU báo vận tốc góc thực tế vẫn quá nhỏ (< 0.06 rad/s), xe đang bị kẹt sàn:
    if (nowMs - s_manRotStartMs > 140 && (nowMs - s_lastManBoostMs >= 40)) {
      s_lastManBoostMs = nowMs;
      float actualOmega = fabsf(g_state.currentGyroZ);
      if (actualOmega < 0.06f) {
        if (s_manGyroBoost < 250) s_manGyroBoost += 25;
      } else if (actualOmega >= 0.15f) {
        if (s_manGyroBoost > 0) s_manGyroBoost -= 10;
      }
    }

    // Áp dụng Kickstart 120ms đầu hoặc Gyro Boost
    if (nowMs - s_manRotStartMs < 120) {
      constexpr int32_t KICKSTART_MIN_PWM = 700;
      if (abs(leftS) < KICKSTART_MIN_PWM) {
        leftS = (leftS >= 0) ? KICKSTART_MIN_PWM : -KICKSTART_MIN_PWM;
      }
      if (abs(rightS) < KICKSTART_MIN_PWM) {
        rightS = (rightS >= 0) ? KICKSTART_MIN_PWM : -KICKSTART_MIN_PWM;
      }
    } else if (s_manGyroBoost > 0) {
      if (leftS > 0) leftS += s_manGyroBoost; else if (leftS < 0) leftS -= s_manGyroBoost;
      if (rightS > 0) rightS += s_manGyroBoost; else if (rightS < 0) rightS -= s_manGyroBoost;
    }
  } else {
    s_manWasRotating = false;
    s_manRotStartMs = 0;
    s_manGyroBoost = 0;
  }

  leftS = constrain(leftS, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);
  rightS = constrain(rightS, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);

  int32_t fl = leftS, rl = leftS;
  int32_t fr = rightS, rr = rightS;

  int32_t magLimit = isPureRot ? (int32_t)PWM_MAX : (int32_t)base;
  int32_t mag = max(max(abs(fl), abs(rl)), max(abs(fr), abs(rr)));
  if (mag > magLimit && mag > 0) {
    int32_t scale = magLimit * 100 / mag;
    fl  = fl  * scale / 100;
    rl  = rl  * scale / 100;
    fr  = fr  * scale / 100;
    rr  = rr  * scale / 100;
  }

  // Lưu pre-layout speed để debug
  int32_t fl_pre = fl, rl_pre = rl, fr_pre = fr, rr_pre = rr;

  // Báo cho Localization biết lệnh drive hiện tại (% so với base) — dùng cho pose estimate.
  // leftS/rightS đã qua curve + clamp, chia base ra % (-100..+100).
  if (base > 0) {
    locSetDriveCmd((int16_t)((leftS  * 100) / (int32_t)base),
                   (int16_t)((rightS * 100) / (int32_t)base));
  } else {
    locSetDriveCmd(0, 0);
  }

  // Tắt in log debug định kỳ khi lái để tránh xung đột chân UART0 TX (GPIO 43) với encoder
  /*
  static uint32_t lastDbgDrv = 0;
  if (millis() - lastDbgDrv > 500u) {
    lastDbgDrv = millis();
    extern float g_motorScale[4];
    extern uint8_t g_motInv[4];
    Serial.printf("[Drive] x=%d y=%d base=%u → L=%ld R=%ld (fl=%ld rl=%ld fr=%ld rr=%ld) [scFL=%.2f scRL=%.2f scFR=%.2f scRR=%.2f invFL=%d invRL=%d invFR=%d invRR=%d]\n",
                  x, y, (unsigned)base, (long)leftS, (long)rightS,
                  (long)fl_pre, (long)rl_pre, (long)fr_pre, (long)rr_pre,
                  g_motorScale[0], g_motorScale[1], g_motorScale[2], g_motorScale[3],
                  (int)g_motInv[0], (int)g_motInv[1], (int)g_motInv[2], (int)g_motInv[3]);
  }
  */

  const int32_t sp[4] = {fl, rl, fr, rr};
  motorApplyLayout(sp);
}

#endif // MOTORS_H