/* =====================================================================
 *  Odometry.h — Đọc Encoder thật 2 bánh trước (ISR GPIO) + IMU heading fusion
 *
 *  v2.0 — 2026-07-27 (Root-Cause fix)
 *    - Kích hoạt đọc 2 encoder bánh trước (ENC_L=GPIO35, ENC_R=GPIO36) qua
 *      ngắt ngoài (ISR) — KHÔNG còn mô phỏng PWM ảo.
 *    ⚠️ GPIO 35/36 là input-only trên ESP32-S3 N16R8 (PSRAM chiếm).
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

#define ODOM_PERIOD_MS 100

/** Tổng quãng đường (m) cho từng bánh (slot logic LF/LR/RF/RR). */
float g_distFL = 0, g_distRL = 0, g_distFR = 0, g_distRR = 0;

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
// v2.5 (2026-07-29): ADD 2ms SOFTWARE DEBOUNCE.
//
// Cheap single-channel Hall/slot encoders (the 20-slot disc on this robot)
// can double-count on contact bounce or edge ringing — visible as random RPM
// spikes and distance-integral drift. Reject any pulse that arrives within
// ENC_DEBOUNCE_MS of the previous pulse on the same edge. With max realistic
// wheel speed ~1 m/s and WHEEL_CIRC_M ≈ 0.25 m, max pulse rate ≈ 80/s
// (≈12.5 ms period) — 2 ms leaves an 8× margin and won't drop legitimate
// edges at any practical speed.
static constexpr uint32_t ENC_DEBOUNCE_MS = 2;
static volatile uint32_t s_encLastPulseMsL = 0;
static volatile uint32_t s_encLastPulseMsR = 0;

static inline void IRAM_ATTR isr_enc_left() {
  const uint32_t now = (uint32_t)millis();
  portENTER_CRITICAL_ISR(&s_encMux);
  if ((now - s_encLastPulseMsL) >= ENC_DEBOUNCE_MS) {
    s_encTicksL++;
    g_encPhyLastPulseMs[0] = now;
    s_encLastPulseMsL = now;
  }
  portEXIT_CRITICAL_ISR(&s_encMux);
}

static inline void IRAM_ATTR isr_enc_right() {
  const uint32_t now = (uint32_t)millis();
  portENTER_CRITICAL_ISR(&s_encMux);
  if ((now - s_encLastPulseMsR) >= ENC_DEBOUNCE_MS) {
    s_encTicksR++;
    g_encPhyLastPulseMs[1] = now;
    s_encLastPulseMsR = now;
  }
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
  // Clear debounce baselines too — otherwise the first pulse after reset
  // could be falsely rejected as "too close to the previous pulse" (which
  // was actually a long time ago in real terms but persisted in the static).
  s_encLastPulseMsL = 0;
  s_encLastPulseMsR = 0;
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

// Sticky last-non-zero direction per side. Used to sign coast-phase encoder
// ticks (motor command == 0 but wheel still rolling from momentum). The encoder
// is single-channel so it can't tell forward from backward on its own; we fall
// back to the last commanded direction with a short TTL — past TTL we drop the
// ticks (wheel is genuinely stopped).
static int8_t s_lastNonZeroDirL = 0;
static int8_t s_lastNonZeroDirR = 0;
static uint32_t s_lastNonZeroDirL_ms = 0;
static uint32_t s_lastNonZeroDirR_ms = 0;
static constexpr uint32_t COAST_DIR_TTL_MS = 300;  // past this, treat as stopped

// Delta baselines live at file scope (not inside odomUpdate()) so
// odomResetDistance() can zero them. Previously they were function-local
// statics — that left the baselines stale after a reset, and the very next
// odomUpdate() integrated `0 - lastAccumulatedTicks` as a phantom reverse step.
static int32_t s_lastTicksL = 0;
static int32_t s_lastTicksR = 0;
static uint32_t s_lastUpdateMs = 0;

/** Cập nhật mỗi ODOM_PERIOD_MS (100ms). */
inline void odomUpdate() {
#if USE_ENCODER_HARDWARE
  // ---- 1) Đọc delta xung trong khoảng vừa qua ----
  const uint32_t nowMs = millis();
  const int32_t ticksL_now = odomGetTicksL();
  const int32_t ticksR_now = odomGetTicksR();
  const int32_t dTicksL = ticksL_now - s_lastTicksL;
  const int32_t dTicksR = ticksR_now - s_lastTicksR;
  s_lastTicksL = ticksL_now;
  s_lastTicksR = ticksR_now;

  // Real dt from millis(), clamped to a sane range. Falls back to
  // ODOM_PERIOD_MS for the very first call after boot when s_lastUpdateMs
  // is still 0, or after a Reset Odom. The clamp absorbs FreeRTOS scheduler
  // hiccups so RPM/ds don't blow up on a delayed tick.
  float dt;
  if (s_lastUpdateMs == 0) {
    dt = (float)ODOM_PERIOD_MS / 1000.0f;
  } else {
    uint32_t elapsedMs = (nowMs >= s_lastUpdateMs) ? (nowMs - s_lastUpdateMs) : 0;
    if (elapsedMs == 0 || elapsedMs > 1000u) {
      // 0 = two ticks in same ms (shouldn't happen but defensive); >1000ms =
      // taskControl stalled badly; clamp to ODOM_PERIOD_MS so RPM/ds stay sane.
      dt = (float)ODOM_PERIOD_MS / 1000.0f;
    } else {
      dt = (float)elapsedMs / 1000.0f;
    }
  }
  s_lastUpdateMs = nowMs;

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
  // Fix: LẤY DẤU từ motor direction (g_motorDir[]). Mỗi bánh:
  //   - Số xung = magnitude (encoder đo)
  //   - Dấu     = motor command direction (hệ thống điều khiển biết)
  // Khi 2 bánh quay ngược chiều nhau, ds tự triệt tiêu → ds = 0 cho rotation.
  //
  // Cẩn thận: g_motorDir[motorId] đã bao gồm motor inversion (g_motInv[])
  // và motor scale (g_motorScale[]). Khi lastMotorSpeed[motorId] = 0 (stop),
  // ta giữ ds sign = 0 để không tích phân.
  const float mPerTick = WHEEL_CIRC_M / ENC_PPR;
  // Encoder FL / FR dùng chung xung với left/right (Config.h:151-154):
  //   FL=RL=ENC_L, FR=RR=ENC_R.
  // Ta lấy direction từ motor FL (slot 0) cho bên trái, motor FR (slot 2) cho bên phải.
  // BẢO THỦ: chỉ đổi dấu khi motor đã chạy đủ lâu (>200ms) để tránh trạng thái
  // chuyển tiếp gây dấu sai. Nếu motor command gần 0 (brake/coast) → giữ ds=0.
  //
  // v2.5 (2026-07-29): COAST-PHASE TICK PRESERVATION.
  // Bug trước: motor command = 0 (coast/brake) nhưng bánh vẫn quay do quán
  // tính. Encoder vẫn đếm xung nhưng dsL = 0 → mất tích phân pose vĩnh viễn.
  // Fix: nếu motor dir == 0 nhưng có xung mới và vẫn còn trong COAST_DIR_TTL_MS
  // kể từ lệnh nonzero cuối, dùng sticky last-direction. Sau TTL → drop.
  const int8_t dirL_raw = g_motorDir[MID_FL];
  const int8_t dirR_raw = g_motorDir[MID_FR];
  if (dirL_raw != 0) { s_lastNonZeroDirL = dirL_raw; s_lastNonZeroDirL_ms = nowMs; }
  if (dirR_raw != 0) { s_lastNonZeroDirR = dirR_raw; s_lastNonZeroDirR_ms = nowMs; }
  const bool leftCoast  = (dirL_raw == 0) && (dTicksL > 0)
                          && ((nowMs - s_lastNonZeroDirL_ms) <= COAST_DIR_TTL_MS)
                          && (s_lastNonZeroDirL != 0);
  const bool rightCoast = (dirR_raw == 0) && (dTicksR > 0)
                          && ((nowMs - s_lastNonZeroDirR_ms) <= COAST_DIR_TTL_MS)
                          && (s_lastNonZeroDirR != 0);
  const float dirL = leftCoast  ? (float)s_lastNonZeroDirL : (float)dirL_raw;
  const float dirR = rightCoast ? (float)s_lastNonZeroDirR : (float)dirR_raw;
  const float dsL = (dirL == 0.f) ? 0.f : dirL * ((float)dTicksL * mPerTick);
  const float dsR = (dirR == 0.f) ? 0.f : dirR * ((float)dTicksR * mPerTick);
  const float ds  = (dsL + dsR) * 0.5f;           // translation trung bình (đã signed)

  // ---- 3) RPM thật ----
  // RPM = (xung/100ms) × (60s/100ms) / PPR = (xung × 600) / PPR
  // v2.4: sign theo motor direction (cùng logic với dsL/dsR ở trên).
  // Trước đây rpm_unsigned → RPM thường +6000 (cộng 2 bánh ngược dấu)
  // → twist.twist.linear.x = 32 m/s khi xoay tại chỗ.
  const float rpmL = dirL * ((float)dTicksL / dt) * 60.0f / ENC_PPR;
  const float rpmR = dirR * ((float)dTicksR / dt) * 60.0f / ENC_PPR;

  // Lưu RPM vật lý (slot 0..3 = FL, RL, FR, RR).
  // Vì 4 bánh trái/phải dùng chung xung → set RPM cho cả 2 bánh cùng bên.
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

  // ---- 6) Localization: truyền pose delta thật (x,y,heading) ----
  //   locUpdate encoder-aware: ds/dθ từ encoder, KHÔNG dùng LOC_PWM_TO_MPS.
  locUpdate(dsL, dsR, dt);
#else
  // Fallback: PWM simulation (giữ để debug nếu cần tắt encoder)
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

  locUpdate(0, 0, 0);  // No encoder data — Localization falls back to PWM
#endif
}

/** Reset counters + pose (dùng khi nhấn "Đặt lại Odom" trên WebUI). */
inline void odomResetDistance() {
  g_distFL = g_distRL = g_distFR = g_distRR = 0;
  g_state.distFL = g_state.distRL = g_state.distFR = g_state.distRR = 0;
#if USE_ENCODER_HARDWARE
  odomResetTicks();
#endif
  // Critical: reset the file-scope delta baselines too. Otherwise the next
  // odomUpdate() integrates `0 - s_lastTicksL` as a large phantom reverse
  // step (issue surfaced when WebUI "Reset Odom" caused a pose teleport).
  s_lastTicksL = s_lastTicksR = 0;
  s_lastUpdateMs = 0;       // dt falls back to ODOM_PERIOD_MS for next tick
  s_lastNonZeroDirL = s_lastNonZeroDirR = 0;
  s_lastNonZeroDirL_ms = s_lastNonZeroDirR_ms = 0;
  locResetPose();
}

#endif // ODOMETRY_H
