/* =====================================================================
 *  Odometry.h — Đọc Encoder thật 2 bánh trước (ISR GPIO) + IMU heading fusion
 *
 *  v2.0 — 2026-07-27 (Root-Cause fix)
 *    - Kích hoạt đọc 2 encoder bánh trước (ENC_L=GPIO35, ENC_R=GPIO36) qua
 *      ngắt ngoài (ISR) — KHÔNG còn mô phỏng PWM ảo.
 *    - Tính RPM + distance thật từ số xung tích lũy + ENC_PPR + WHEEL_CIRC_M.
 *    - Pose thật: ds = (sL + sR)/2 × dt ; dθ = (sR - sL)/WHEEL_BASE_M × dt.
 *    - Localization.h giờ nhận được odom thật (delta x, y, heading) thay vì
 *      LOC_PWM_TO_MPS tích lũy sai số.
 *    - Pose drift heading vẫn được EKF (ImuFusion.h) bù bằng gyro MPU6050.
 *
 *  Đặc tả phần cứng:
 *    - 2 encoder Hall/bột thuỷ tinh, 1 kênh (D0), VCC=3.3V, GND chung.
 *    - Mỗi encoder có đĩa 20 khe chữ U → ENC_PPR = 20.
 *    - 2 bánh trước (FL+FR) đọc xung; 2 bánh sau (RL+RR) dùng chung xung
 *      với bánh trước cùng bên (giả định robot bám đường tốt, 2 bánh cùng
 *      bên không slip độc lập).
 *
 *  API:
 *    odomInit()            — Attach ISR + reset counters.
 *    odomUpdate()          — Tính RPM/distance + locUpdate() mỗi ODOM_PERIOD_MS.
 *    odomResetDistance()   — Reset counters + pose.
 *    odomGetTicksL/R()     — Đọc số xung tích lũy (cho telemetry/auto-trim).
 * =====================================================================*/
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Config.h"
#include "SensorLayout.h"
#include "Motors.h"           // v2.4: cần MotorId (MID_FL/FR) + g_motorDir[] cho encoder sign
#include "Localization.h"
#include "ImuFusion.h"        // v2.5: cần g_dThetaEnc cho EKF updateWheel

#define ODOM_PERIOD_MS 20   // Upgraded to 20 ms (50 Hz): High-frequency real-time odometry updates over USB Serial!

/** Tổng quãng đường (m) cho từng bánh (slot logic LF/LR/RF/RR). */
float g_distFL = 0, g_distRL = 0, g_distFR = 0, g_distRR = 0;

/** Encoder-derived dTheta (rad/s) — consumed by imuFusion::step() every SAFE_LOOP_MS. */
float g_dThetaEncRate = 0.f;

/* ============== Encoder thật (ISR-safe) ===============================
 *
 *  Số xung tích lũy được cập nhật bởi ISR attachInterrupt() trong odomInit().
 *  Đọc bằng noInterrupts()/interrupts() để tránh race với ISR.
 *  PortMUX_TYPE dùng cho ESP32 (critical section thay vì noInterrupts thuần).
 * ---------------------------------------------------------------------*/
#if USE_ENCODER_HARDWARE
static volatile int32_t s_encTicksL = 0;   // Encoder bên trái (cộng xung lên/xuống theo chiều quay)
static volatile int32_t s_encTicksR = 0;   // Encoder bên phải
// g_encPhyLastPulseMs[] đã được extern trong Config.h:555 — không khai báo lại ở đây.
static portMUX_TYPE s_encMux = portMUX_INITIALIZER_UNLOCKED;

// ISR — gọi trong ngắt ngoài, cần IRAM_ATTR + portENTER_CRITICAL_ISR.
static inline void IRAM_ATTR isr_enc_left() {
  portENTER_CRITICAL_ISR(&s_encMux);
  s_encTicksL++;
  g_encPhyLastPulseMs[0] = (uint32_t)millis();  // mark "left encoder pulse seen at this ms" cho RobotTelemetry.jEnOn[]
  portEXIT_CRITICAL_ISR(&s_encMux);
}

static inline void IRAM_ATTR isr_enc_right() {
  portENTER_CRITICAL_ISR(&s_encMux);
  s_encTicksR++;
  g_encPhyLastPulseMs[1] = (uint32_t)millis();  // mark "right encoder pulse seen at this ms"
  portEXIT_CRITICAL_ISR(&s_encMux);
}

// API đọc encoder thread-safe — dùng từ taskControl.
static inline int32_t odomGetTicksL() {
  portENTER_CRITICAL(&s_encMux);
  int32_t v = s_encTicksL;
  portEXIT_CRITICAL(&s_encMux);
  return v;
}

static inline int32_t odomGetTicksR() {
  portENTER_CRITICAL(&s_encMux);
  int32_t v = s_encTicksR;
  portEXIT_CRITICAL(&s_encMux);
  return v;
}

// Reset counters (giữ vị trí encoder tuyệt đối nếu cần — dùng cho calibration).
static inline void odomResetTicks() {
  portENTER_CRITICAL(&s_encMux);
  s_encTicksL = 0;
  s_encTicksR = 0;
  portEXIT_CRITICAL(&s_encMux);
}
#else
static inline int32_t odomGetTicksL() { return 0; }
static inline int32_t odomGetTicksR() { return 0; }
static inline void   odomResetTicks() {}
#endif // USE_ENCODER_HARDWARE

/** Khởi tạo: attach ISR cho 2 chân encoder + reset counters. */
inline void odomInit() {
  g_distFL = g_distRL = g_distFR = g_distRR = 0;

#if USE_ENCODER_HARDWARE
  // Đảm bảo chân input + pullup nội (encoder open-collector thường cần pullup).
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  // Attach ngắt cạnh RISING — phù hợp với đĩa 20 khe chữ U.
  attachInterrupt(digitalPinToInterrupt(ENC_L), isr_enc_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isr_enc_right, RISING);

  odomResetTicks();

  Serial.printf("[Odom] Encoder hardware ENABLED — L=GPIO%d, R=GPIO%d, PPR=%u, WheelD=%.0fmm, Base=%.0fmm\n",
                (int)ENC_L, (int)ENC_R, (unsigned)ENC_PPR,
                WHEEL_DIAM_M * 1000.0f, WHEEL_BASE_M * 1000.0f);
#else
  Serial.println(F("[Odom] Encoder DISABLED — falling back to PWM simulation."));
#endif
}

/** Cập nhật mỗi ODOM_PERIOD_MS (100ms). */
inline void odomUpdate() {
#if USE_ENCODER_HARDWARE
  // ---- 1) Đọc delta xung trong 100ms qua ----
  static int32_t s_lastTicksL = 0;
  static int32_t s_lastTicksR = 0;
  static uint32_t s_lastUpdateMs = 0;

  const uint32_t nowMs = millis();
  const int32_t ticksL_now = odomGetTicksL();
  const int32_t ticksR_now = odomGetTicksR();
  const int32_t dTicksL = ticksL_now - s_lastTicksL;
  const int32_t dTicksR = ticksR_now - s_lastTicksR;
  s_lastTicksL = ticksL_now;
  s_lastTicksR = ticksR_now;

  // dt = 100ms (mặc định). Nếu taskControl bị delay (rare), clamp.
  const float dt = (float)ODOM_PERIOD_MS / 1000.0f;

  // ---- 2) Quãng đường mỗi bên (m) ----
  // v2.4 (2026-07-28): FIX direction-blind encoder.
  //
  // Hardware chỉ có 1 kênh encoder mỗi bánh (không có quadrature) → chỉ đếm
  // xung, không phân biệt chiều. Encoder ISR chỉ ++, không bao giờ --.
  //
  // Bug: khi xoay tại chỗ (botRotateCW: trái +pwm, phải -pwm), cả 2 bánh
  // đều quay → cả 2 encoder đều ++. Công thức cũ:
  //     ds = (dsL + dsR) / 2  (cộng magnitude)
  // → nghĩ robot đang TIẾN với vận tốc 2×1 bánh, gây teleport pose 5-15 m
  // khi chỉ xoay 6s lệnh 0.4 rad/s (user-observed 2026-07-28).
  //
  // v2.5 (2026-09-04): FIX ghost encoder pulses when motor power is OFF.
  //
  // GPIO 43/44 (encoder) picks up EMI from WiFi, I2C (MPU6050), and PWM
  // switching even when motors are mechanically stopped (12V power off).
  // If g_motorDir != 0 (direction set from last cmd) but PWM is below the
  // dead-zone threshold, any ticks counted are electrical noise. We gate them
  // out using the actual PWM value stored in g_state.lastMotorSpeed.
  //
  constexpr int32_t ENC_PWM_DEADZONE = 50; // ticks gated out below this PWM (motor can't spin)
  extern uint8_t g_mapMotSlot[4];
  extern uint8_t g_motInv[4];
  uint8_t pL = g_mapMotSlot[0] > 3 ? 0 : g_mapMotSlot[0];
  uint8_t pR = g_mapMotSlot[2] > 3 ? 2 : g_mapMotSlot[2];
  float rawDirL = (float)g_motorDir[pL];
  float rawDirR = (float)g_motorDir[pR];
  const float dirL = g_motInv[0] ? -rawDirL : rawDirL;
  const float dirR = g_motInv[2] ? -rawDirR : rawDirR;

  // Gate: if PWM is in dead-zone, motor isn't spinning → ticks = EMI noise
  const bool motorLActive = abs(g_state.lastMotorSpeed[pL]) > ENC_PWM_DEADZONE;
  const bool motorRActive = abs(g_state.lastMotorSpeed[pR]) > ENC_PWM_DEADZONE;

  const float mPerTick = WHEEL_CIRC_M / ENC_PPR;
  const float dsL = (dirL == 0.f || !motorLActive) ? 0.f : dirL * ((float)dTicksL * mPerTick);
  const float dsR = (dirR == 0.f || !motorRActive) ? 0.f : dirR * ((float)dTicksR * mPerTick);
  const float ds  = ((dirL * dirR) < 0.f) ? 0.f : ((dsL + dsR) * 0.5f);

  // ---- 3) RPM thật ----
  const float rpmL = dirL * ((float)dTicksL / dt) * 60.0f / ENC_PPR;
  const float rpmR = dirR * ((float)dTicksR / dt) * 60.0f / ENC_PPR;

  const float rpmPhy[4] = { rpmL, rpmL, rpmR, rpmR };
  const float distPhy[4] = { dsL, dsL, dsR, dsR };

  // ---- 4) Cộng dồn distance theo slot LOGIC (LF/LR/RF/RR) ----
  g_distFL += distPhy[g_mapEncSlot[SLOT_LF]];
  g_distRL += distPhy[g_mapEncSlot[SLOT_LR]];
  g_distFR += distPhy[g_mapEncSlot[SLOT_RF]];
  g_distRR += distPhy[g_mapEncSlot[SLOT_RR]];

  // ---- 5) Ghi ra g_state ----
  g_state.rpmFL  = rpmPhy[g_mapEncSlot[SLOT_LF]];
  g_state.rpmRL  = rpmPhy[g_mapEncSlot[SLOT_LR]];
  g_state.rpmFR  = rpmPhy[g_mapEncSlot[SLOT_RF]];
  g_state.rpmRR  = rpmPhy[g_mapEncSlot[SLOT_RR]];
  g_state.distFL = g_distFL;
  g_state.distRL = g_distRL;
  g_state.distFR = g_distFR;
  g_state.distRR = g_distRR;

  // ---- 6) g_dThetaEncRate cho EKF heading fusion ----
  g_dThetaEncRate = ((dsR - dsL) / WHEEL_BASE_M) / (dt > 0.001f ? dt : 0.1f);

  // ---- 7) Localization ----
  locUpdate(dsL, dsR, dt);
  (void)s_lastUpdateMs;
#else
  // Fallback: PWM simulation
  constexpr float MAX_TICKS_PER_TICK = (200.0f * (float)ODOM_PERIOD_MS / 60000.0f) * 20.0f;
  static float s_accTicks[4] = {0, 0, 0, 0};

  int32_t pwm[4] = {
    abs(g_state.lastMotorSpeed[0]),
    abs(g_state.lastMotorSpeed[1]),
    abs(g_state.lastMotorSpeed[2]),
    abs(g_state.lastMotorSpeed[3])
  };

  float rpmPhy[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) {
    s_accTicks[i] += ((float)pwm[i] / (float)PWM_MAX) * MAX_TICKS_PER_TICK;
    uint32_t intTicks = (uint32_t)s_accTicks[i];
    s_accTicks[i] -= (float)intTicks;
    rpmPhy[i] = ((float)intTicks / (float)ODOM_PERIOD_MS) * (60000.0f / 20.0f);
  }

  constexpr float MAX_DIST_PER_TICK = (WHEEL_CIRC_M * 200.0f * (float)ODOM_PERIOD_MS) / 60000.0f;
  float distPhy[4];
  for (int i = 0; i < 4; i++) {
    distPhy[i] = ((float)pwm[i] / (float)PWM_MAX) * MAX_DIST_PER_TICK;
  }

  g_distFL += distPhy[g_mapEncSlot[SLOT_LF]];
  g_distRL += distPhy[g_mapEncSlot[SLOT_LR]];
  g_distFR += distPhy[g_mapEncSlot[SLOT_RF]];
  g_distRR += distPhy[g_mapEncSlot[SLOT_RR]];

  g_state.rpmFL  = rpmPhy[g_mapEncSlot[SLOT_LF]];
  g_state.rpmRL  = rpmPhy[g_mapEncSlot[SLOT_LR]];
  g_state.rpmFR  = rpmPhy[g_mapEncSlot[SLOT_RF]];
  g_state.rpmRR  = rpmPhy[g_mapEncSlot[SLOT_RR]];
  g_state.distFL = g_distFL;
  g_state.distRL = g_distRL;
  g_state.distFR = g_distFR;
  g_state.distRR = g_distRR;
  g_dThetaEncRate = 0.f;

  extern uint8_t g_mapMotSlot[4];
  extern uint8_t g_motInv[4];
  uint8_t pL = g_mapMotSlot[0] > 3 ? 0 : g_mapMotSlot[0];
  uint8_t pR = g_mapMotSlot[2] > 3 ? 2 : g_mapMotSlot[2];
  float rawDirL = (float)g_motorDir[pL];
  float rawDirR = (float)g_motorDir[pR];
  float dirL = g_motInv[0] ? -rawDirL : rawDirL;
  float dirR = g_motInv[2] ? -rawDirR : rawDirR;

  float dsL_pwm = dirL * distPhy[g_mapEncSlot[SLOT_LF]];
  float dsR_pwm = dirR * distPhy[g_mapEncSlot[SLOT_RF]];
  locUpdate(dsL_pwm, dsR_pwm, 0);
#endif
}

/** Reset counters + pose (dùng khi nhấn "Đặt lại Odom" trên WebUI). */
inline void odomResetDistance() {
  g_distFL = g_distRL = g_distFR = g_distRR = 0;
  g_state.distFL = g_state.distRL = g_state.distFR = g_state.distRR = 0;
  g_dThetaEncRate = 0.f;
#if USE_ENCODER_HARDWARE
  odomResetTicks();
#endif
  locResetPose();
}

#endif // ODOMETRY_H
