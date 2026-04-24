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

#define ODOM_PERIOD_MS 100   // Cửa sổ tính RPM

// Bộ đếm xung — dùng volatile vì được sửa trong ISR
volatile uint32_t g_ticksFL = 0;
volatile uint32_t g_ticksRL = 0;
volatile uint32_t g_ticksFR = 0;
volatile uint32_t g_ticksRR = 0;

// Tổng xung tích luỹ để tính quãng đường
uint32_t g_totalFL = 0, g_totalRL = 0, g_totalFR = 0, g_totalRR = 0;

// ISR phải IRAM_ATTR, càng ngắn càng tốt
void IRAM_ATTR isrFL() { g_ticksFL++; }
void IRAM_ATTR isrRL() { g_ticksRL++; }
void IRAM_ATTR isrFR() { g_ticksFR++; }
void IRAM_ATTR isrRR() { g_ticksRR++; }

/* Chỉ tắt ISR nếu cùng chân LED và encoder (ví dụ tùy chỉnh pin) */
#if !SMB_ONBOARD_RGB || (SMB_NEOPIXEL_PIN != ENC_RR)
#define ODOM_HAS_ENC_RR 1
#else
#define ODOM_HAS_ENC_RR 0
#endif

inline void odomInit() {
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
}

/**
 * Đọc & reset bộ đếm trong vùng critical, sau đó tính:
 *   RPM    = (ticks / ENC_PPR) * (60000 / period_ms)
 *   dist   += (ticks / ENC_PPR) * WHEEL_CIRC_M
 */
inline void odomUpdate() {
  noInterrupts();
  uint32_t fl = g_ticksFL; g_ticksFL = 0;
  uint32_t rl = g_ticksRL; g_ticksRL = 0;
  uint32_t fr = g_ticksFR; g_ticksFR = 0;
  uint32_t rr = g_ticksRR; g_ticksRR = 0;
  interrupts();

  g_totalFL += fl; g_totalRL += rl; g_totalFR += fr; g_totalRR += rr;

  const float scaleRpm = 60000.0f / (ENC_PPR * (float)ODOM_PERIOD_MS);
  g_state.rpmFL = fl * scaleRpm;
  g_state.rpmRL = rl * scaleRpm;
  g_state.rpmFR = fr * scaleRpm;
  g_state.rpmRR = rr * scaleRpm;

  g_state.distFL = (g_totalFL / ENC_PPR) * WHEEL_CIRC_M;
  g_state.distRL = (g_totalRL / ENC_PPR) * WHEEL_CIRC_M;
  g_state.distFR = (g_totalFR / ENC_PPR) * WHEEL_CIRC_M;
  g_state.distRR = (g_totalRR / ENC_PPR) * WHEEL_CIRC_M;
}

inline void odomResetDistance() {
  g_totalFL = g_totalRL = g_totalFR = g_totalRR = 0;
  g_state.distFL = g_state.distRL = g_state.distFR = g_state.distRR = 0.0f;
}

#endif // ODOMETRY_H
