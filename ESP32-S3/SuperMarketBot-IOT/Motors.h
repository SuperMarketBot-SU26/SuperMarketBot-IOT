/* =====================================================================
 *  Motors.h — Điều khiển 4 động cơ qua 2 TB6612FNG (PWM mịn bằng LEDC)
 *  API:
 *    motorsInit()            — Cấu hình chân + 4 kênh LEDC
 *    motorsStandby(en)       — Bật/tắt STBY chung
 *    motorDrive(M_*, speed)  — speed ∈ [-PWM_MAX .. +PWM_MAX]
 *    botForward / Backward / RotateCW / RotateCCW / Stop
 *    botDrive(x, y, base)    — Lái kiểu joystick (arcade mixing)
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

extern volatile int8_t g_motorDir[4];

/**
 * Điều khiển 1 động cơ.
 * @param id    chỉ số động cơ MID_*
 * @param speed -PWM_MAX..+PWM_MAX (âm = lùi)
 */
inline void motorDrive(MotorId id, int32_t speed) {
  // 1. Giới hạn dải đầu vào
  if (speed > (int32_t)PWM_MAX) speed = (int32_t)PWM_MAX;
  if (speed < -(int32_t)PWM_MAX) speed = -(int32_t)PWM_MAX;

  // 2. Bộ lọc dốc (Slew rate limiter) giảm dòng khởi động đột ngột (inrush current) chống sụt áp nguồn (brownout)
  int32_t lastSpd = g_state.lastMotorSpeed[(uint8_t)id];
  int32_t diff = speed - lastSpd;
  constexpr int32_t MAX_RAMP_STEP = 150; // Giới hạn thay đổi PWM tối đa mỗi chu kỳ 10ms
  if (diff > MAX_RAMP_STEP) {
    speed = lastSpd + MAX_RAMP_STEP;
  } else if (diff < -MAX_RAMP_STEP) {
    speed = lastSpd - MAX_RAMP_STEP;
  }
  g_state.lastMotorSpeed[(uint8_t)id] = speed;

  const MotorPins &m = MOTORS[id];
  // 3. Thiết lập hướng quay động cơ
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

  // 4. Bù vùng chết (Deadband compensation) cho động cơ vàng DC (quy đổi theo hệ 10-bit PWM: 0..1023)
  if (speed > 0) {
    constexpr int32_t MIN_MOTOR_PWM = 170; // Hạ từ 280 xuống 170 để khởi hành mượt mà, bò chậm chính xác, tránh giật cục
    if (speed > (int32_t)PWM_MAX) speed = (int32_t)PWM_MAX;
    speed = MIN_MOTOR_PWM + (speed * (PWM_MAX - MIN_MOTOR_PWM)) / PWM_MAX;
  }
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
 * Lái Mecanum X-config (cho WebSocket joystick 3 trục).
 *   fl = fwd + strafe + turn
 *   rl = fwd - strafe + turn
 *   fr = fwd - strafe - turn
 *   rr = fwd + strafe - turn
 * @param strafe  -100..100  tịnh tiến ngang (âm=trái, dương=phải)
 * @param fwd     -100..100  tiến/lùi
 * @param turn    -100..100  xoay tại chỗ (âm=CCW, dương=CW)
 * @param base    0..PWM_MAX tốc độ nền tối đa
 */
inline void botDriveMecanum(int16_t strafe, int16_t fwd, int16_t turn,
                             uint16_t base) {
  if (base > PWM_MAX) base = PWM_MAX;

  if (g_state.wheelMode == WHEEL_NORMAL) {
    strafe = 0;
  }

  // Áp dụng đường cong phi tuyến Quadratic (Exponential Curve) giúp điều khiển cực kỳ mịn ở tốc độ thấp
  int32_t fwdSign = (fwd >= 0) ? 1 : -1;
  int32_t strafeSign = (strafe >= 0) ? 1 : -1;
  int32_t turnSign = (turn >= 0) ? 1 : -1;

  int32_t fwdCurve = ((int32_t)fwd * (int32_t)fwd * fwdSign) / 100;
  int32_t strafeCurve = ((int32_t)strafe * (int32_t)strafe * strafeSign) / 100;
  int32_t turnCurve = ((int32_t)turn * (int32_t)turn * turnSign) / 100;

  // Scale các đầu vào joystick phi tuyến theo tốc độ nền base
  int32_t fwdScaled = fwdCurve * (int32_t)base / 100;
  int32_t strafeScaled = strafeCurve * (int32_t)base / 100;
  int32_t turnScaled = turnCurve * (int32_t)base / 100;

  // Mecanum con lăn tạo ma sát cao → nhân thêm để bù
  // strafe yếu nhất → gain 1.35; fwd/turn gain 1.15
  constexpr int32_t STRAFE_GAIN = 135;  // ×1.35
  constexpr int32_t FWD_GAIN   = 115;  // ×1.15
  constexpr int32_t TURN_GAIN  = 115;  // ×1.15

  int32_t fl = (fwdScaled * FWD_GAIN + strafeScaled * STRAFE_GAIN + turnScaled * TURN_GAIN) / 100;
  int32_t rl = (fwdScaled * FWD_GAIN - strafeScaled * STRAFE_GAIN + turnScaled * TURN_GAIN) / 100;
  int32_t fr = (fwdScaled * FWD_GAIN - strafeScaled * STRAFE_GAIN - turnScaled * TURN_GAIN) / 100;
  int32_t rr = (fwdScaled * FWD_GAIN + strafeScaled * STRAFE_GAIN - turnScaled * TURN_GAIN) / 100;

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
 * Lái kiểu joystick (Arcade drive mixing) — tương thích ngược bánh thường.
 * @param x    -100..100 (âm = xoay trái)
 * @param y    -100..100 (âm = lùi)
 * @param base 0..PWM_MAX — mức tốc độ nền tối đa
 */
inline void botDrive(int16_t x, int16_t y, uint16_t base) {
  botDriveMecanum(0, y, x, base);
}

#endif // MOTORS_H
