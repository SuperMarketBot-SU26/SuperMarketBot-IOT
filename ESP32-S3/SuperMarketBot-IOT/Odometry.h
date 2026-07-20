/* =====================================================================
 *  Odometry.h — RPM & distance telemetry (PWM-simulated vì project không dùng encoder)
 *
 *  Project KHÔNG gắn encoder nữa → RPM và distance chỉ là ước lượng từ PWM.
 *  Pose thật lấy từ IMU (heading) + locUpdate() trong Localization.h.
 *
 *  API:
 *    odomInit()            — In log (không cần ISR)
 *    odomUpdate()          — Tính RPM + distance từ PWM, gọi locUpdate()
 *    odomResetDistance()   — Reset tổng quãng đường + pose
 * =====================================================================*/
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Config.h"
#include "SensorLayout.h"
#include "Localization.h"

#define ODOM_PERIOD_MS 100

/** Tổng quãng đường ước lượng (m) cho từng bánh (logic slot LF/LR/RF/RR). */
float g_distFL = 0, g_distRL = 0, g_distFR = 0, g_distRR = 0;

/* Stub cho RobotTelemetry.h (project không còn dùng encoder thật) */
volatile uint32_t g_encPhyLastPulseMs[4] = {0, 0, 0, 0};

inline void odomInit() {
  Serial.println(F("[Odom] Encoder disabled. RPM/distance simulated from PWM."));
  g_distFL = g_distRL = g_distFR = g_distRR = 0;
}

inline void odomUpdate() {
  // Mỗi tick 100ms: ước lượng số xung dựa trên PWM thực tế đang cấp cho 4 motor.
  // RPM ước lượng: giả sử motor vàng DC 6V đạt ~200 RPM ở 100% PWM → ~6.67 xung/100ms (đĩa 20 lỗ).
  constexpr float MAX_TICKS_PER_TICK = (200.0f * (float)ODOM_PERIOD_MS / 60000.0f) * 20.0f;
  static float s_accTicks[4] = {0, 0, 0, 0};

  int32_t pwm[4] = {
    abs(g_state.lastMotorSpeed[0]),
    abs(g_state.lastMotorSpeed[1]),
    abs(g_state.lastMotorSpeed[2]),
    abs(g_state.lastMotorSpeed[3])
  };

  // Tính rpmPhy theo slot vật lý (FL, RL, FR, RR trước remap)
  float rpmPhy[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) {
    s_accTicks[i] += ((float)pwm[i] / (float)PWM_MAX) * MAX_TICKS_PER_TICK;
    uint32_t intTicks = (uint32_t)s_accTicks[i];
    s_accTicks[i] -= (float)intTicks;
    rpmPhy[i] = ((float)intTicks / (float)ODOM_PERIOD_MS) * (60000.0f / 20.0f);
  }

  // Quãng đường ước lượng (m) theo slot vật lý
  constexpr float M_PER_TICK = (WHEEL_CIRC_M / 20.0f);  // 20 xung = 1 vòng
  for (int i = 0; i < 4; i++) {
    s_accTicks[i] += ((float)pwm[i] / (float)PWM_MAX) * MAX_TICKS_PER_TICK;
  }
  // (Đã cộng acc ở trên; tính quãng đường bằng pwm% × max distance trong 100ms)
  constexpr float MAX_DIST_PER_TICK = (WHEEL_CIRC_M * 200.0f * (float)ODOM_PERIOD_MS) / 60000.0f;
  float distPhy[4];
  for (int i = 0; i < 4; i++) {
    distPhy[i] = ((float)pwm[i] / (float)PWM_MAX) * MAX_DIST_PER_TICK;
  }

  // Cộng dồn distance theo slot logic (LF/LR/RF/RR)
  g_distFL += distPhy[g_mapEncSlot[SLOT_LF]];
  g_distRL += distPhy[g_mapEncSlot[SLOT_LR]];
  g_distFR += distPhy[g_mapEncSlot[SLOT_RF]];
  g_distRR += distPhy[g_mapEncSlot[SLOT_RR]];

  // Ghi ra g_state (slot logic)
  g_state.rpmFL  = rpmPhy[g_mapEncSlot[SLOT_LF]];
  g_state.rpmRL  = rpmPhy[g_mapEncSlot[SLOT_LR]];
  g_state.rpmFR  = rpmPhy[g_mapEncSlot[SLOT_RF]];
  g_state.rpmRR  = rpmPhy[g_mapEncSlot[SLOT_RR]];
  g_state.distFL = g_distFL;
  g_state.distRL = g_distRL;
  g_state.distFR = g_distFR;
  g_state.distRR = g_distRR;

  // Gọi Localization tích phân pose (không truyền ticks vì pose dùng PWM)
  locUpdate();
}

inline void odomResetDistance() {
  g_distFL = g_distRL = g_distFR = g_distRR = 0;
  g_state.distFL = g_state.distRL = g_state.distFR = g_state.distRR = 0;
  locResetPose();
}

#endif // ODOMETRY_H
