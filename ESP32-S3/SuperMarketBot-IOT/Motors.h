/* =====================================================================
 *  Motors.h — Điều khiển 4 động cơ qua 2 TB6612FNG (PWM mịn bằng LEDC)
 *  API:
 *    motorsInit()            — Cấu hình chân + 4 kênh LEDC
 *    motorsStandby(en)       — Bật/tắt STBY chung
 *    motorDrive(M_*, speed)  — speed ∈ [-PWM_MAX .. +PWM_MAX]
 *    botForward / Backward / RotateCW / RotateCCW / Stop
 *    botDriveMecanum(s,fwd,t,base) — Mecanum X-config kinematics
 *    botDrive(x, y, base)           — arcade mixing (backward-compatible)
 * =====================================================================*/
#ifndef MOTORS_H
#define MOTORS_H

#include "Config.h"
#include "MotorLayout.h"

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

inline void motorsStandby(bool enable) {
  digitalWrite(M_STBY, enable ? HIGH : LOW);
}

inline void motorsInit() {
  // Chân hướng
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MOTORS[i].in1, OUTPUT);
    pinMode(MOTORS[i].in2, OUTPUT);
    digitalWrite(MOTORS[i].in1, LOW);
    digitalWrite(MOTORS[i].in2, LOW);
    // Arduino-ESP32 core 3.x: ledcAttach(pin, freq, resBits)
    ledcAttach(MOTORS[i].pwm, PWM_FREQ, PWM_RES_BITS);
    ledcWrite(MOTORS[i].pwm, 0);
  }
  // STBY chung
  pinMode(M_STBY, OUTPUT);
  motorsStandby(true);
}

/**
 * Điều khiển 1 động cơ.
 * @param id    chỉ số động cơ MID_*
 * @param speed -PWM_MAX..+PWM_MAX (âm = lùi)
 */
inline void motorDrive(MotorId id, int32_t speed) {
  const MotorPins &m = MOTORS[id];
  if (speed > 0) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else if (speed < 0) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
    speed = -speed;
  } else {
    // Phanh ngắn (short brake) để dừng dứt khoát
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, HIGH);
  }
  if (speed > PWM_MAX) speed = PWM_MAX;
  ledcWrite(m.pwm, speed);
}

/**
 * Ánh xạ lệnh theo 4 góc xe (slot 0..3 = LF,LR,RF,RR) → kênh TB6612 vật lý.
 * Mảng speedBySlot[] cùng thứ tự với g_mapMotSlot / g_motInv.
 */
inline void motorApplyLayout(const int32_t speedBySlot[4]) {
  for (int s = 0; s < 4; s++) {
    uint8_t p = g_mapMotSlot[s];
    if (p > 3) p = (uint8_t)s;
    int32_t sp = speedBySlot[s];
    if (g_motInv[s]) sp = -sp;
    motorDrive((MotorId)p, sp);
  }
}

inline void botStop() {
  for (uint8_t i = 0; i < 4; i++) motorDrive((MotorId)i, 0);
}

inline void botForward(uint16_t s) {
  int32_t v = (int32_t)s;
  const int32_t sp[4] = {v, v, v, v};
  motorApplyLayout(sp);
}

inline void botBackward(uint16_t s) {
  int32_t v = -(int32_t)s;
  const int32_t sp[4] = {v, v, v, v};
  motorApplyLayout(sp);
}

// Xoay tại chỗ sang phải (tank turn CW)
inline void botRotateCW(uint16_t s) {
  int32_t v = (int32_t)s;
  const int32_t sp[4] = {v, v, -v, -v};
  motorApplyLayout(sp);
}

// Xoay tại chỗ sang trái (tank turn CCW)
inline void botRotateCCW(uint16_t s) {
  int32_t v = (int32_t)s;
  const int32_t sp[4] = {-v, -v, v, v};
  motorApplyLayout(sp);
}

/**
 * Điều khiển chuyển động đa hướng bằng bánh xe Mecanum (chuẩn chữ X).
 *
 * Công thức động học Mecanum (X-configuration):
 *   fl = fwd + strafe + turn
 *   rl = fwd - strafe + turn
 *   fr = fwd - strafe - turn
 *   rr = fwd + strafe - turn
 *
 * @param strafe  -100..100  tịnh tiến ngang (âm=trái, dương=phải)
 * @param fwd     -100..100  tiến/lùi  (âm=lùi,  dương=tiến)
 * @param turn    -100..100  xoay tại chỗ (âm=CCW, dương=CW)
 * @param base    0..PWM_MAX tốc độ nền tối đa
 */
inline void botDriveMecanum(int16_t strafe, int16_t fwd, int16_t turn,
                             uint16_t base) {
  if (base > PWM_MAX) base = PWM_MAX;

  // Tính tốc độ 4 bánh theo công thức X-config
  int32_t fl = (int32_t)fwd + (int32_t)strafe + (int32_t)turn;
  int32_t rl = (int32_t)fwd - (int32_t)strafe + (int32_t)turn;
  int32_t fr = (int32_t)fwd - (int32_t)strafe - (int32_t)turn;
  int32_t rr = (int32_t)fwd + (int32_t)strafe - (int32_t)turn;

  // Chuẩn hóa (normalize) để không bánh nào vượt quá 'base'
  int32_t mag = max(max(abs(fl), abs(rl)), max(abs(fr), abs(rr)));
  if (mag > (int32_t)base && mag > 0) {
    int32_t scale = (int32_t)base * 100 / mag;
    fl  = fl  * scale / 100;
    rl  = rl  * scale / 100;
    fr  = fr  * scale / 100;
    rr  = rr  * scale / 100;
  }

  const int32_t sp[4] = {fl, rl, fr, rr};
  motorApplyLayout(sp);
}

/**
 * Lái kiểu joystick (Arcade drive mixing) — tương thích ngược.
 * Chuyển tiếp sang botDriveMecanum: tay lái cũ x=xoay, y=tiến/lùi,
 * không có tịnh tiến ngang → strafe=0.
 *
 * @param x    -100..100 (âm = xoay trái)
 * @param y    -100..100 (âm = lùi)
 * @param base 0..PWM_MAX — mức tốc độ nền tối đa
 */
inline void botDrive(int16_t x, int16_t y, uint16_t base) {
  botDriveMecanum(0, y, x, base);
}

#endif // MOTORS_H
