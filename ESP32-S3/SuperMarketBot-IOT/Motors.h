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
  // Với 2WD + Caster: tốc độ tăng tốc 160 PWM/20ms (tương đương 0-100% trong 120ms) — êm ái, không giật xe
  constexpr int32_t MAX_RAMP_STEP = 160;
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
    // 2WD + Caster có ma sát lăn rất nhẹ, ngưỡng deadband tự nhiên 130 PWM là đủ để bắt đầu quay mượt mà
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
 * Slot 0..1 (FL, RL) = Kênh Trái; Slot 2..3 (FR, RR) = Kênh Phải.
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

// Forward declaration để botStop reset cờ xoay
inline void botRotationReset();

/**
 * Dừng tất cả động cơ (PWM=0, IN1=IN2=HIGH để brake).
 */
inline void botStop() {
  locSetDriveCmd(0, 0);
  botRotationReset();
  const int32_t sp[4] = {0, 0, 0, 0};
  motorApplyLayout(sp);
}

/**
 * Chạy thẳng (cùng PWM cho cả 2 bên).
 */
inline void botForward(uint16_t pwm) {
  if (pwm > PWM_MAX) pwm = PWM_MAX;
#if REVERSE_CHASSIS_ORIENTATION
  const int32_t sp[4] = {-(int32_t)pwm, 0, -(int32_t)pwm, 0};
#else
  const int32_t sp[4] = {(int32_t)pwm, 0, (int32_t)pwm, 0};
#endif
  motorApplyLayout(sp);
}

inline void botBackward(uint16_t pwm) {
  if (pwm > PWM_MAX) pwm = PWM_MAX;
#if REVERSE_CHASSIS_ORIENTATION
  const int32_t sp[4] = {(int32_t)pwm, 0, (int32_t)pwm, 0};
#else
  const int32_t sp[4] = {-(int32_t)pwm, 0, -(int32_t)pwm, 0};
#endif
  motorApplyLayout(sp);
}

// Biến quản lý trạng thái xoay mượt mà
static uint32_t s_rotStartMs = 0;
static bool     s_rotActive = false;
static int32_t  s_rotGyroAssist = 0;
static uint32_t s_lastGyroAssistMs = 0;

volatile bool g_isRotating = false;

inline void botRotationReset() {
  g_isRotating = false;
  s_rotActive = false;
  s_rotStartMs = 0;
  s_rotGyroAssist = 0;
  s_lastGyroAssistMs = 0;
}

/**
 * Tính toán xung PWM xoay tại chỗ mượt mà cho hệ 2WD + Caster.
 * @param targetPwm Tốc độ xoay mong muốn (từ slider xoay hướng)
 */
inline uint16_t botComputeSmoothRotatePwm(uint16_t targetPwm) {
  if (targetPwm > PWM_MAX) targetPwm = PWM_MAX;
  uint32_t now = millis();
  if (!s_rotActive) {
    s_rotStartMs = now;
    s_rotActive = true;
    s_rotGyroAssist = 0;
    s_lastGyroAssistMs = now;
  }

  uint16_t eff = targetPwm;

  // 1. Soft-Kickstart nhẹ nhàng (100ms đầu) cho 2WD + Caster:
  // Vì bánh caster tự xoay theo hướng lực đẩy, chỉ cần mồi nhẹ min 260 PWM để thắng quán tính tĩnh của hộp số
  if (now - s_rotStartMs < 100) {
    uint16_t softKick = (uint16_t)min((uint32_t)PWM_MAX, (uint32_t)targetPwm * 115u / 100u);
    constexpr uint16_t KICK_MIN_2WD = 260;
    if (softKick < KICK_MIN_2WD) softKick = KICK_MIN_2WD;
    if (softKick > (uint16_t)PWM_MAX) softKick = (uint16_t)PWM_MAX;
    if (softKick > targetPwm) eff = softKick;
  } else {
    // 2. Closed-loop Gyro Assist ổn định (sau 100ms):
    // Giúp robot giữ tốc độ quay đều đặn
    if (now - s_lastGyroAssistMs >= 60) {
      s_lastGyroAssistMs = now;
      float actualOmega = fabsf(g_state.currentGyroZ);
      if (actualOmega < 0.12f) {
        if (s_rotGyroAssist < 120) s_rotGyroAssist += 8;
      } else if (actualOmega >= 0.35f) {
        if (s_rotGyroAssist > 0) s_rotGyroAssist -= 10;
      }
    }
    eff = (uint16_t)min((uint32_t)PWM_MAX, (uint32_t)eff + (uint32_t)s_rotGyroAssist);
  }

  return eff;
}

/**
 * Xoay tại chỗ (dùng cho waypoint align & phím xoay).
 * 2WD Differential: bánh trái FL tiến, bánh phải FR lùi (hoặc ngược lại). 2 bánh caster tự lựa xoay theo.
 */
inline void botRotateCWImmediate(uint16_t pwm) {
  g_isRotating = true;
  locSetDriveCmd(0, 0);
  uint16_t effPwm = botComputeSmoothRotatePwm(pwm);
  const int32_t sp[4] = {(int32_t)effPwm, 0, -(int32_t)effPwm, 0};
  motorApplyLayout(sp);
}

inline void botRotateCCWImmediate(uint16_t pwm) {
  g_isRotating = true;
  locSetDriveCmd(0, 0);
  uint16_t effPwm = botComputeSmoothRotatePwm(pwm);
  const int32_t sp[4] = {-(int32_t)effPwm, 0, (int32_t)effPwm, 0};
  motorApplyLayout(sp);
}

inline void botRotateCW(uint16_t pwm)  { botRotateCWImmediate(pwm); }
inline void botRotateCCW(uint16_t pwm) { botRotateCCWImmediate(pwm); }

/**
 * Lái Arcade Differential Drive chuẩn cho 2WD + 2 Caster.
 * @param x    -100..100 (âm = rẽ trái, dương = rẽ phải)
 * @param y    -100..100 (âm = lùi, dương = tiến)
 * @param base 0..PWM_MAX  tốc độ nền tối đa (từ slider Lái tay)
 */
inline void botDrive(int16_t x, int16_t y, uint16_t base) {
  if (base > PWM_MAX) base = PWM_MAX;

  // Nhận diện xoay tại chỗ thuần túy (|x| >= 10, |y| <= 15)
  const bool isPureRot = (abs(x) >= 10 && abs(y) <= 15);

  int32_t leftS, rightS;

  if (isPureRot) {
    g_isRotating = true;
    uint16_t rotSpd = (g_state.rotateBaseSpeed > 0) ? g_state.rotateBaseSpeed : base;
    // Tỉ lệ công suất theo góc gạt joystick
    uint16_t baseMag = (uint16_t)((uint32_t)rotSpd * (uint32_t)abs(x) / 100u);
    if (baseMag > PWM_MAX) baseMag = PWM_MAX;
    uint16_t smoothPwm = botComputeSmoothRotatePwm(baseMag);

    // Quay tại chỗ: 2 bên quay ngược chiều nhau
    leftS  = (x > 0) ? (int32_t)smoothPwm : -(int32_t)smoothPwm;
    rightS = (x > 0) ? -(int32_t)smoothPwm : (int32_t)smoothPwm;
  } else {
    botRotationReset();
    // Vi sai 2WD + Caster chuẩn:
    // fwdSpeed và turnSpeed đều tỷ lệ tuyến tính với 'base' (thanh trượt Tốc độ Lái tay)
    int32_t fwdSpeed  = ((int32_t)y * (int32_t)base) / 100;
    int32_t turnSpeed = ((int32_t)x * (int32_t)base) / 100;

    leftS  = fwdSpeed + turnSpeed;
    rightS = fwdSpeed - turnSpeed;

    // Giới hạn công suất tối đa theo 'base' để thanh trượt WebUI kiểm soát 100% tốc độ thực tế
    int32_t maxMag = max(abs(leftS), abs(rightS));
    if (maxMag > (int32_t)base && maxMag > 0) {
      leftS  = (leftS  * (int32_t)base) / maxMag;
      rightS = (rightS * (int32_t)base) / maxMag;
    }
  }

  leftS = constrain(leftS, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);
  rightS = constrain(rightS, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);

  // Phân bổ tín hiệu cho 2WD:
  // FL (slot 0) = leftS (Động cơ Trái)
  // FR (slot 2) = rightS (Động cơ Phải)
  // RL (slot 1) & RR (slot 3) = 0 (Bánh Caster)
#if REVERSE_CHASSIS_ORIENTATION
  // Khi đảo hướng xe (Caster thành trước, Motor thành sau):
  // Bánh bên TRÁI mới là motor FR cũ, bánh bên PHẢI mới là motor FL cũ.
  // Chiều tiến mới tương ứng với PWM âm của motor vật lý.
  int32_t fl = -rightS;
  int32_t fr = -leftS;
#else
  int32_t fl = leftS;
  int32_t fr = rightS;
#endif

  // Báo cho Localization biết lệnh drive hiện tại (% so với base)
  if (base > 0) {
    locSetDriveCmd((int16_t)((leftS  * 100) / (int32_t)base),
                   (int16_t)((rightS * 100) / (int32_t)base));
  } else {
    locSetDriveCmd(0, 0);
  }

  const int32_t sp[4] = {fl, 0, fr, 0};
  motorApplyLayout(sp);
}

#endif // MOTORS_H