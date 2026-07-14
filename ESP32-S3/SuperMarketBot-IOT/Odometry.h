/* =====================================================================
 *  Odometry.h — 4 encoder FC-03, đếm xung qua attachInterrupt
 *
 *  API:
 *    odomInit()            — Cấu hình INPUT_PULLUP + 4 ISR
 *    odomUpdate()          — Tính RPM & quãng đường (gọi đều mỗi ODOM_PERIOD_MS)
 *    odomResetDistance()   — Reset tổng quãng đường
 * =====================================================================*/
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Config.h"
#include "SensorLayout.h"
#include "Localization.h"

#define ODOM_PERIOD_MS 100   // Cửa sổ tính RPM

// Bộ đếm xung — dùng volatile vì được sửa trong ISR
volatile uint32_t g_ticksFL = 0;
volatile uint32_t g_ticksRL = 0;
volatile uint32_t g_ticksFR = 0;
volatile uint32_t g_ticksRR = 0;

// Tổng xung tích luỹ để tính quãng đường
uint32_t g_totalFL = 0, g_totalRL = 0, g_totalFR = 0, g_totalRR = 0;
volatile uint32_t g_encPhyLastPulseMs[4] = {0, 0, 0, 0};

// Thời điểm ngắt cuối cùng (để lọc nhiễu cơ học/điện từ)
volatile uint32_t g_lastIsrFL = 0;
volatile uint32_t g_lastIsrRL = 0;
volatile uint32_t g_lastIsrFR = 0;
volatile uint32_t g_lastIsrRR = 0;

// ISR phải IRAM_ATTR, càng ngắn càng tốt. Chống nhiễu tần số cao < 1200us.
void IRAM_ATTR isrFL() {
  uint32_t now = micros();
  if (now - g_lastIsrFL > 1200u) {
    g_ticksFL++;
    g_lastIsrFL = now;
  }
}
void IRAM_ATTR isrRL() {
  uint32_t now = micros();
  if (now - g_lastIsrRL > 1200u) {
    g_ticksRL++;
    g_lastIsrRL = now;
  }
}
void IRAM_ATTR isrFR() {
  uint32_t now = micros();
  if (now - g_lastIsrFR > 1200u) {
    g_ticksFR++;
    g_lastIsrFR = now;
  }
}
void IRAM_ATTR isrRR() {
  uint32_t now = micros();
  if (now - g_lastIsrRR > 1200u) {
    g_ticksRR++;
    g_lastIsrRR = now;
  }
}

/* Chỉ tắt ISR nếu cùng chân LED và encoder (ví dụ tùy chỉnh pin) */
#if !SMB_ONBOARD_RGB || (SMB_NEOPIXEL_PIN != ENC_RR)
#define ODOM_HAS_ENC_RR 1
#else
#define ODOM_HAS_ENC_RR 0
#endif

inline void odomInit() {
#if USE_ENCODER_HARDWARE
  pinMode(ENC_FL, INPUT_PULLUP);
  pinMode(ENC_RL, INPUT_PULLUP);
  pinMode(ENC_FR, INPUT_PULLUP);
#if ODOM_HAS_ENC_RR
  pinMode(ENC_RR, INPUT_PULLUP);
#endif
  attachInterrupt(digitalPinToInterrupt(ENC_FL), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RL), isrRL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR), isrFR, RISING);
#if ODOM_HAS_ENC_RR
  attachInterrupt(digitalPinToInterrupt(ENC_RR), isrRR, RISING);
#endif
  Serial.println(F("[Encoder] Encoders initialized on GPIOs with interrupts."));
#else
  Serial.println(F("[Encoder] Encoders disabled (hardware removed)."));
#endif
}

/**
 * Đọc & reset bộ đếm trong vùng critical, sau đó tính:
 *   RPM    = (ticks / ENC_PPR) * (60000 / period_ms)
 *   dist   += (ticks / ENC_PPR) * WHEEL_CIRC_M
 */
inline void odomUpdate() {
  uint32_t fl = 0, rl = 0, fr = 0, rr = 0;

#if USE_ENCODER_HARDWARE
  noInterrupts();
  fl = g_ticksFL; g_ticksFL = 0;
  rl = g_ticksRL; g_ticksRL = 0;
  fr = g_ticksFR; g_ticksFR = 0;
  rr = g_ticksRR; g_ticksRR = 0;
  interrupts();
#else
  // Giả lập Encoder bằng phần mềm dựa trên công suất PWM thực tế đang cấp cho Motor
  // Động cơ vàng DC quay tối đa ~200 RPM ở 6V -> Tần số xung tối đa với đĩa 20 lỗ là ~66.7 xung/giây.
  // Trong cửa sổ ODOM_PERIOD_MS (100ms), số xung tối đa ở 100% PWM là ~6.67 xung.
  // Dùng biến số thực tích lũy để tránh sai số làm tròn khi vận tốc nhỏ.
  static float s_simTicksFL = 0.f;
  static float s_simTicksRL = 0.f;
  static float s_simTicksFR = 0.f;
  static float s_simTicksRR = 0.f;

  // Đọc công suất động cơ hiện tại (sau bộ ramp điều khiển)
  int32_t pwmFL = abs(g_state.lastMotorSpeed[MID_FL]);
  int32_t pwmRL = abs(g_state.lastMotorSpeed[MID_RL]);
  int32_t pwmFR = abs(g_state.lastMotorSpeed[MID_FR]);
  int32_t pwmRR = abs(g_state.lastMotorSpeed[MID_RR]);

  // Quy đổi tỉ lệ tích lũy xung
  constexpr float MAX_TICKS_PER_100MS = 6.67f;
  s_simTicksFL += ((float)pwmFL / (float)PWM_MAX) * MAX_TICKS_PER_100MS;
  s_simTicksRL += ((float)pwmRL / (float)PWM_MAX) * MAX_TICKS_PER_100MS;
  s_simTicksFR += ((float)pwmFR / (float)PWM_MAX) * MAX_TICKS_PER_100MS;
  s_simTicksRR += ((float)pwmRR / (float)PWM_MAX) * MAX_TICKS_PER_100MS;

  // Lấy phần nguyên và trừ đi để giữ phần dư số thực cho chu kỳ sau
  fl = (uint32_t)s_simTicksFL; s_simTicksFL -= (float)fl;
  rl = (uint32_t)s_simTicksRL; s_simTicksRL -= (float)rl;
  fr = (uint32_t)s_simTicksFR; s_simTicksFR -= (float)fr;
  rr = (uint32_t)s_simTicksRR; s_simTicksRR -= (float)rr;
#endif

  g_totalFL += fl; g_totalRL += rl; g_totalFR += fr; g_totalRR += rr;

  uint32_t tPulse = (uint32_t)millis();
  if (fl) g_encPhyLastPulseMs[0] = tPulse;
  if (rl) g_encPhyLastPulseMs[1] = tPulse;
  if (fr) g_encPhyLastPulseMs[2] = tPulse;
  if (rr) g_encPhyLastPulseMs[3] = tPulse;

  const float scaleRpm = 60000.0f / (ENC_PPR * (float)ODOM_PERIOD_MS);
  float rpmPhy[4] = {fl * scaleRpm, rl * scaleRpm, fr * scaleRpm, rr * scaleRpm};
  g_state.rpmFL = rpmPhy[g_mapEncSlot[SLOT_LF]];
  g_state.rpmRL = rpmPhy[g_mapEncSlot[SLOT_LR]];
  g_state.rpmFR = rpmPhy[g_mapEncSlot[SLOT_RF]];
  g_state.rpmRR = rpmPhy[g_mapEncSlot[SLOT_RR]];

  float distPhy[4] = {
      (g_totalFL / ENC_PPR) * WHEEL_CIRC_M * ODOM_CALIB_FACTOR,
      (g_totalRL / ENC_PPR) * WHEEL_CIRC_M * ODOM_CALIB_FACTOR,
      (g_totalFR / ENC_PPR) * WHEEL_CIRC_M * ODOM_CALIB_FACTOR,
      (g_totalRR / ENC_PPR) * WHEEL_CIRC_M * ODOM_CALIB_FACTOR,
  };
  g_state.distFL = distPhy[g_mapEncSlot[SLOT_LF]];
  g_state.distRL = distPhy[g_mapEncSlot[SLOT_LR]];
  g_state.distFR = distPhy[g_mapEncSlot[SLOT_RF]];
  g_state.distRR = distPhy[g_mapEncSlot[SLOT_RR]];

  /* Dead reckoning — dùng tổng ticks vật lý (trước remap) */
  locUpdate(g_totalFL, g_totalFR, g_totalRL, g_totalRR);
}

inline void odomResetDistance() {
  g_totalFL = g_totalRL = g_totalFR = g_totalRR = 0;
  g_state.distFL = g_state.distRL = g_state.distFR = g_state.distRR = 0.0f;
  locResetPose();
}

#endif // ODOMETRY_H
