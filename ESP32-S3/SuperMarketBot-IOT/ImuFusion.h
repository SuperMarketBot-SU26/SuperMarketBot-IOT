/* =====================================================================
 *  ImuFusion.h — Hợp nhất Heading bằng EKF 1-chiều (Gyro + Wheel + SLAM)
 *
 *  Mục tiêu:
 *    - Bù drift của gyro MPU6050 bằng dθ từ differential-drive wheel
 *      (locSetDriveCmd → leftPct/rightPct → dθ_wheel).
 *    - Khi SLAM cung cấp pose tuyệt đối (locSetSlamPose), dùng heading
 *      đó để reset bias dài hạn → hết trôi góc khi chạy lâu.
 *
 *  Mô hình EKF 1D (heading-only):
 *    State   x = [ heading, gyro_bias ]^T
 *    Predict: heading_new = heading + (gyroZ - bias) × dt
 *             bias_new    = bias
 *    Update 1: z = dθ_wheel (rad quay trong dt)
 *              z_hat = (gyroZ - bias) × dt
 *    Update 2: z = heading tuyệt đối từ SLAM
 *              z_hat = heading
 *
 *  Thiết kế đủ nhẹ để chạy ở tần số ODOM_PERIOD_MS (100ms) trên ESP32-S3.
 * =====================================================================*/
#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <math.h>
#include "Localization.h"

#ifndef IMU_FUSION_ENABLE
#define IMU_FUSION_ENABLE  1   // Bật EKF fusion; đặt 0 để fallback về heading cũ (gyro only)
#endif

// Shared encoder-derived dTheta — set once per ODOM_PERIOD_MS (100ms) by Odometry.h,
// consumed by imuFusion::step() every SAFE_LOOP_MS (50ms).
// This replaces the old PWM→dθ approximation that drifted under voltage sag.
extern float g_dThetaEnc;

// --- Covariance khởi tạo & process noise ---
#ifndef IMU_FUSION_P_HEADING
#define IMU_FUSION_P_HEADING    1.0f     // P[0,0] ban đầu (rad^2)
#endif
#ifndef IMU_FUSION_P_BIAS
#define IMU_FUSION_P_BIAS       0.01f    // P[1,1] ban đầu (rad/s)^2
#endif
#ifndef IMU_FUSION_Q_GYRO
#define IMU_FUSION_Q_GYRO       0.0008f  // process noise cho heading (rad^2 / step)
#endif
#ifndef IMU_FUSION_Q_BIAS
#define IMU_FUSION_Q_BIAS       1e-6f    // process noise cho bias drift
#endif

// --- Measurement noise ---
#ifndef IMU_FUSION_R_WHEEL
#define IMU_FUSION_R_WHEEL      0.04f    // (rad^2) wheel-derived dθ — variance mỗi bước 100ms
#endif
#ifndef IMU_FUSION_R_SLAM
#define IMU_FUSION_R_SLAM       0.0025f  // (rad^2) SLAM heading (≈3° sai số 1-sigma)
#endif
#ifndef IMU_FUSION_DEBUG
#define IMU_FUSION_DEBUG        0        // 1: bật serial debug mỗi 1s; 0: tắt
#endif

// --- NaN guard constants ---
#ifndef IMU_FUSION_SANITY_HEADING
#define IMU_FUSION_SANITY_HEADING (2.0f * (float)M_PI)  // heading > 2π → corrupt
#endif
#ifndef IMU_FUSION_SANITY_BIAS
#define IMU_FUSION_SANITY_BIAS    5.0f                    // |bias| > 5 rad/s → corrupt
#endif

// --- Ngưỡng an toàn ---
#ifndef IMU_FUSION_WHEEL_MIN_DIFF
#define IMU_FUSION_WHEEL_MIN_DIFF  5.0f  // % PWM khác biệt tối thiểu để tin wheel-derived dθ
#endif
#ifndef IMU_FUSION_ZUPT_MAX_DTHETA
#define IMU_FUSION_ZUPT_MAX_DTHETA 0.002f  // rad/tick: ngưỡng để coi là "đứng yên" (0.11°/tick)
#endif
#ifndef IMU_FUSION_ZUPT_GYRO_THRESH
#define IMU_FUSION_ZUPT_GYRO_THRESH 0.005f  // rad/s — phát hiện chuyển động qua gyro (nhạy hơn encoder)
#endif
#ifndef IMU_FUSION_ZUPT_SETTLE_TICKS
#define IMU_FUSION_ZUPT_SETTLE_TICKS 10    // ticks đứng yên trước khi kích hoạt ZUPT (10×50ms = 0.5s)
#endif
#ifndef IMU_FUSION_ZUPT_RATE
#define IMU_FUSION_ZUPT_RATE         0.20f  // bias *= (1 - 0.20) mỗi tick khi ZUPT: halving time = ~3 ticks
#endif
#ifndef IMU_FUSION_BIAS_CLAMP
#define IMU_FUSION_BIAS_CLAMP    0.5f    // |bias| không vượt ±0.5 rad/s (≈ ±28°/s)
#endif

namespace imuFusion {

struct State {
  float heading;     // rad, [0, 2π)
  float bias;        // rad/s, gyro bias hiệu chỉnh
  float P00, P01, P10, P11;
  bool  initialized;
};

inline State& getState() {
  static State s = {0.f, 0.f,
                    IMU_FUSION_P_HEADING, 0.f, 0.f, IMU_FUSION_P_BIAS,
                    false};
  return s;
}

/** ZUPT (Zero-Velocity Update) — counters ticks of stillness to freeze heading drift. */
static uint8_t s_zuptStillTicks = 0;  // consecutive ticks with |dThetaEnc| < threshold

// Chuẩn hóa góc về [-π, π]
inline float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.f * (float)M_PI;
  while (a <= -(float)M_PI) a += 2.f * (float)M_PI;
  return a;
}

// Chuẩn hóa góc về [0, 2π)
inline float wrap2Pi(float a) {
  while (a <  0.f)              a += 2.f * (float)M_PI;
  while (a >= 2.f * (float)M_PI) a -= 2.f * (float)M_PI;
  return a;
}

inline void init() {
  State& s = getState();
  s.heading     = wrap2Pi(g_pose.headingRad);
  s.bias        = 0.f;
  s.P00         = IMU_FUSION_P_HEADING;
  s.P01         = 0.f;
  s.P10         = 0.f;
  s.P11         = IMU_FUSION_P_BIAS;
  s.initialized = true;
}

/**
 * Bước Predict: tích lũy heading từ gyro + cập nhật covariance.
 *
 * F = [1, -dt; 0, 1]
 * P_pred = F P F^T + Q
 *
 * @param gyroZRad  Vận tốc góc đo được (rad/s, đã trừ bias thô)
 * @param dt        Bước thời gian (giây)
 */
inline void predict(float gyroZRad, float dt) {
  State& s = getState();
  if (!s.initialized || dt <= 0.f || dt > 1.f) return;

  const float omega = gyroZRad - s.bias;
  s.heading = wrap2Pi(s.heading + omega * dt);

  // NaN guard (no isnan() on ESP32 stdlib — use x!=x trick)
  if (!(s.heading == s.heading) || !(omega == omega)) {
    s.heading = 0.f;
    s.bias = 0.f;
    s.P00 = IMU_FUSION_P_HEADING; s.P01 = 0.f; s.P10 = 0.f; s.P11 = IMU_FUSION_P_BIAS;
    return;
  }

  // F P
  const float Fp00 = s.P00 - dt * s.P10;
  const float Fp01 = s.P01 - dt * s.P11;
  const float Fp10 = s.P10;
  const float Fp11 = s.P11;
  // (F P) F^T
  s.P00 = Fp00 - dt * Fp01 + IMU_FUSION_Q_GYRO;
  s.P01 = Fp01 - dt * Fp11;
  s.P10 = Fp10 - dt * Fp01;
  s.P11 = Fp11 - dt * Fp01 + IMU_FUSION_Q_BIAS;

  if (s.P00 < 1e-9f) s.P00 = 1e-9f;
  if (s.P11 < 1e-9f) s.P11 = 1e-9f;
}

/**
 * Update 1: Complementary filter — gyro-predict + wheel-correct heading.
 *
 * Predict:   heading_gyro = heading + (gyroZ - bias) * dt
 * Correct:   residual = dTheta_wheel - (gyroZ - bias) * dt
 *            heading += K * residual
 *            bias   += K * residual / dt
 *
 * K is a fixed gain (not derived from P matrix — P01=0 in this ESP32 setup
 * because P starts diagonal, which kills the Kalman gain. Complementary filter
 * is equivalent to EKF with a fixed K and is numerically stable.)
 *
 * @param gyroZRad        Vận tốc góc đo được (rad/s) — CẦN truyền vào để tính residual
 * @param dThetaWheelRad   Tổng xoay từ wheel trong dt (rad)
 * @param dt              Bước thời gian (giây)
 */
#ifndef IMU_FUSION_K_CORRECT
#define IMU_FUSION_K_CORRECT 0.15f  // gain: 0.15 = 15% encoder correction per step
                                      // 6.7 steps to close 63% of a 1° error
#endif

inline void updateWheel(float gyroZRad, float dThetaWheelRad, float dt) {
  State& s = getState();
  if (!s.initialized || dt <= 0.f) return;

  // Sanity check: skip if inputs are garbage (x!=x is the portable NaN check)
  if (!(dThetaWheelRad == dThetaWheelRad) || !(gyroZRad == gyroZRad)) return;
  // Clamp to physically impossible values — catch corrupted data
  if (fabsf(dThetaWheelRad) > 10.f) return;

  // Residual: wheel says we rotated dTheta, gyro-predict says we rotated (gyro-bias)*dt
  // If they agree → residual ≈ 0 → no correction.
  // If wheel says more than gyro → positive residual → heading += positive → catch up
  // If wheel says less than gyro → negative residual → heading -= positive → catch up
  float residual = dThetaWheelRad - (gyroZRad - s.bias) * dt;

  // Clamp residual to avoid single-step spikes (e.g. encoder glitch)
  const float MAX_RESIDUAL = 0.1f;  // ~5.7°
  if (residual >  MAX_RESIDUAL) residual =  MAX_RESIDUAL;
  if (residual < -MAX_RESIDUAL) residual = -MAX_RESIDUAL;

  s.heading = wrap2Pi(s.heading + IMU_FUSION_K_CORRECT * residual);

  // Bias update: if wheel consistently disagrees with gyro, nudge bias.
  // bias += (K / dt) * residual — skip if dt is too small (avoid div-by-zero)
  if (dt > 0.001f) {
    s.bias += 0.02f * residual / dt;
  }

  // Clamp bias to physical limits
  if (s.bias >  IMU_FUSION_BIAS_CLAMP) s.bias =  IMU_FUSION_BIAS_CLAMP;
  if (s.bias < -IMU_FUSION_BIAS_CLAMP) s.bias = -IMU_FUSION_BIAS_CLAMP;

  // NaN guard after write
  if (!(s.heading == s.heading) || !(s.bias == s.bias)) {
    s.heading = 0.f;
    s.bias = 0.f;
  }
}

/**
 * Update 2: heading tuyệt đối từ SLAM.
 *
 * H = [1, 0]
 *
 * NaN guard: if headingAbsRad is invalid, skip.
 */
inline void updateSlam(float headingAbsRad) {
  State& s = getState();
  if (!s.initialized) return;
  if (!(headingAbsRad == headingAbsRad)) return;

  const float y = wrapPi(headingAbsRad - s.heading);

  const float S = s.P00 + IMU_FUSION_R_SLAM;
  if (S <= 0.f) return;

  const float K0 = s.P00 / S;
  const float K1 = s.P10 / S;

  s.heading = wrap2Pi(s.heading + K0 * y);
  s.bias   += K1 * y;

  // P = (I - K H) P ; K H = [K0 0; K1 0]
  s.P00 = (1.f - K0) * s.P00;
  s.P01 = (1.f - K0) * s.P01;
  s.P10 = s.P10 - K1 * s.P00;
  s.P11 = s.P11 - K1 * s.P01;

  if (s.P00 < 1e-9f) s.P00 = 1e-9f;

  // Clamp bias
  if (s.bias >  IMU_FUSION_BIAS_CLAMP) s.bias =  IMU_FUSION_BIAS_CLAMP;
  if (s.bias < -IMU_FUSION_BIAS_CLAMP) s.bias = -IMU_FUSION_BIAS_CLAMP;
}

/**
 * Wrapper tiện ích: chạy predict + (optional) updateWheel trong 1 lần gọi.
 *
 * Dùng từ taskControl mỗi khi có gyro reading mới (≈ 100Hz hoặc khi odom update).
 *
 * @param gyroZRad  Vận tốc góc (rad/s, đã trừ bias thô, đã áp deadband)
 * @param dt        Bước thời gian (giây)
 * @param leftPct   PWM trái hiện tại (-100..100)
 * @param rightPct  PWM phải hiện tại (-100..100)
 * @return heading đã fusion (rad, [0, 2π))
 */
inline float step(float gyroZRad, float dt, int leftPct, int rightPct) {
#if IMU_FUSION_ENABLE
  State& s = getState();

  const float dThetaPWM = ((float)rightPct - (float)leftPct) * LOC_PWM_TO_MPS / WHEEL_BASE_M * dt;

  // ZUPT settle-first: đợi đủ ticks đứng yên trước khi freeze
  if (fabsf(dThetaPWM) <= IMU_FUSION_ZUPT_MAX_DTHETA) {
    s_zuptStillTicks++;
    if (s_zuptStillTicks >= IMU_FUSION_ZUPT_SETTLE_TICKS) {
      // Settled: freeze heading + nudge bias về 0
      s.bias -= IMU_FUSION_ZUPT_RATE * s.bias;
      if (fabsf(s.bias) < IMU_FUSION_BIAS_CLAMP * 0.01f) s.bias = 0.f;
      s.P11 *= 0.95f;
      return s.heading;  // heading frozen
    }
  } else {
    s_zuptStillTicks = 0;
  }

  // Predict (chỉ khi chưa settled)
  predict(gyroZRad, dt);

  // Update wheel nếu đang xoay đáng kể
  if (fabsf((float)leftPct - (float)rightPct) >= IMU_FUSION_WHEEL_MIN_DIFF) {
    updateWheel(gyroZRad, dThetaPWM, dt);
  }

  return s.heading;
#else
  // Fallback: trả về heading cũ
  return g_pose.headingRad;
#endif
}

/**
 * EKF step: dùng encoder thực (g_dThetaEnc).
 *
 * Gọi từ taskControl mỗi SAFE_LOOP_MS (50ms). g_dThetaEnc được set bởi
 * odomUpdate() mỗi ODOM_PERIOD_MS (100ms) — dùng chung giá trị trong 100ms.
 *
 * @param gyroZRad  Vận tốc góc Z (rad/s, đã trừ bias, đã IMU_YAW_INVERTED)
 * @param dt        Bước thời gian (giây)
 * @return heading đã fusion (rad, [0, 2π))
 */
/**
 * Reset state if corrupted by NaN (called automatically by step()).
 * Returns true if reset was performed.
 */
inline bool resetIfNan() {
  State& s = getState();
  if (!(s.heading == s.heading) || !(s.bias == s.bias) ||
      fabsf(s.heading) > IMU_FUSION_SANITY_HEADING ||
      fabsf(s.bias) > IMU_FUSION_SANITY_BIAS) {
    Serial.printf("[EKF] NaN/drift detected — resetting. was h=%.3f bias=%.3f\n",
                  s.heading, s.bias);
    // Use 0 as seed heading, NOT g_pose.headingRad (which may also be NaN).
    // On first init, imuFusion::init() already synced from g_pose.headingRad
    // so zero here is safe — the next imuFusion::init() call will re-sync.
    s.heading     = 0.f;
    s.bias       = 0.f;
    s.P00        = IMU_FUSION_P_HEADING;
    s.P01        = 0.f;
    s.P10        = 0.f;
    s.P11        = IMU_FUSION_P_BIAS;
    s.initialized = true;
    s_zuptStillTicks = 0;
    return true;
  }
  return false;
}

inline float step(float gyroZRad, float dt) {
#if IMU_FUSION_ENABLE
  State& s = getState();

  // NaN guard: if heading or bias is corrupted, reset immediately.
  // This catches cases where residual overflow or dt=0 causes NaN propagation.
  if (resetIfNan()) return s.heading;

  // Skip this step if inputs are invalid (x!=x is the portable NaN check)
  if (!(gyroZRad == gyroZRad) || !(g_dThetaEnc == g_dThetaEnc) || dt <= 0.f || dt > 2.f) {
    return s.heading;  // keep last valid heading
  }

  // ZUPT — Zero-Velocity Update: freeze heading khi robot đứng yên.
  // Phát hiện chuyển động bằng 2 nguồn:
  //   1) Encoder dTheta (g_dThetaEnc) — chính xác khi motor quay.
  //   2) Gyro thô (gyroZRad) — nhạy hơn khi bị đẩy tay (motor brake).
  // Cần CẢ 2 điều kiện đều yên mới kích hoạt (OR để reset, AND để settle).
  const bool encoderStill = (fabsf(g_dThetaEnc) <= IMU_FUSION_ZUPT_MAX_DTHETA);
  const bool gyroStill    = (fabsf(gyroZRad)      <= IMU_FUSION_ZUPT_GYRO_THRESH);
  const bool robotMoving  = !encoderStill || !gyroStill;

  if (robotMoving) {
    s_zuptStillTicks = 0;  // Reset counter khi robot chuyển động
  } else {
    s_zuptStillTicks++;
#if IMU_FUSION_DEBUG
    // Log mỗi 2s trong trạng thái settling (trước khi FROZEN)
    static uint32_t s_settleLogMs = 0;
    if (s_zuptStillTicks < IMU_FUSION_ZUPT_SETTLE_TICKS) {
      uint32_t now = millis();
      if (now - s_settleLogMs >= 2000) {
        s_settleLogMs = now;
        Serial.printf("[EKF] h=%.3f zupt=%u/%d settling dThEnc=%.5f gyroZ=%.5f\n",
                      s.heading, s_zuptStillTicks, IMU_FUSION_ZUPT_SETTLE_TICKS,
                      g_dThetaEnc, gyroZRad);
      }
    }
#endif
    if (s_zuptStillTicks >= IMU_FUSION_ZUPT_SETTLE_TICKS) {
      // Settled: freeze hoàn toàn — không predict, không update.
      // Bias vẫn nudge về 0 để khi chuyển động lại, bias ≈ 0.
      s.bias -= IMU_FUSION_ZUPT_RATE * s.bias;
      if (fabsf(s.bias) < IMU_FUSION_BIAS_CLAMP * 0.01f) s.bias = 0.f;
      s.P11 *= 0.95f;

#if IMU_FUSION_DEBUG
  static uint8_t s_debugTick = 0;
  if (++s_debugTick >= 20) {
    s_debugTick = 0;
    Serial.printf("[EKF] h=%.3f bias=%.6f zupt=%u/%d FROZEN\n",
                  s.heading, s.bias, s_zuptStillTicks, IMU_FUSION_ZUPT_SETTLE_TICKS);
  }
#endif
      return s.heading;
    }
  }

  // Predict với gyro (chỉ khi robot đang chuyển động)
  predict(gyroZRad, dt);

  // Update wheel bằng dTheta từ encoder thật.
  if (fabsf(g_dThetaEnc) > IMU_FUSION_ZUPT_MAX_DTHETA) {
    updateWheel(gyroZRad, g_dThetaEnc, dt);
  }

#if IMU_FUSION_DEBUG
  static uint8_t s_debugTick = 0;
  if (++s_debugTick >= 20) {
    s_debugTick = 0;
    // Thêm encoder + gyro diagnostics để debug drift
    Serial.printf("[EKF] h=%.3f bias=%.6f zupt=%u/%d dThEnc=%.4f gyroZ=%.4f\n",
                  s.heading, s.bias, s_zuptStillTicks, IMU_FUSION_ZUPT_SETTLE_TICKS,
                  g_dThetaEnc, gyroZRad);
  }
#endif

  return s.heading;
#else
  return g_pose.headingRad;
#endif
}

/**
 * Đường vào SLAM pose update. Gọi từ locSetSlamPose() khi WebManager gửi pose mới.
 */
inline void applySlamPose(float headingAbsRad) {
#if IMU_FUSION_ENABLE
  updateSlam(headingAbsRad);
#endif
}

/** Debug: in state ra Serial. */
inline void debugPrint(const char* tag = "EKF") {
#if IMU_FUSION_ENABLE
  State& s = getState();
    Serial.printf("[%s] h=%.3f bias=%.6f P00=%.4f P11=%.6f\n",
                tag, s.heading, s.bias, s.P00, s.P11);
#endif
}

}  // namespace imuFusion

#endif // IMU_FUSION_H