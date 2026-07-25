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

// --- Ngưỡng an toàn ---
#ifndef IMU_FUSION_WHEEL_MIN_DIFF
#define IMU_FUSION_WHEEL_MIN_DIFF  5.0f  // % PWM khác biệt tối thiểu để tin wheel-derived dθ
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
 * Update 1: dùng wheel-derived dθ để hiệu chỉnh heading & gyro bias.
 *
 * z_hat = (gyroZ - bias) × dt
 * z     = dθ_wheel
 * H     = [0, -dt]
 *
 * @param gyroZRad       Vận tốc góc đo được (rad/s) — CẦN truyền vào để tính residual
 * @param dThetaWheelRad Tổng xoay từ wheel trong dt (rad)
 * @param dt             Bước thời gian (giây)
 */
inline void updateWheel(float gyroZRad, float dThetaWheelRad, float dt) {
  State& s = getState();
  if (!s.initialized || dt <= 0.f) return;

  // y = z - z_hat = dθ_wheel - (gyroZ - bias) × dt
  const float y = dThetaWheelRad - (gyroZRad - s.bias) * dt;

  // H P H^T = [0 -dt] [P00 P01; P10 P11] [0; -dt] = P11 × dt²
  const float S = s.P11 * dt * dt + IMU_FUSION_R_WHEEL;
  if (S <= 0.f) return;

  // K = P H^T / S ; H^T = [0; -dt]
  const float K0 = -s.P01 * dt / S;
  const float K1 = -s.P11 * dt / S;

  // Update state
  s.heading = wrap2Pi(s.heading + K0 * y);
  s.bias   += K1 * y;

  // P_new = (I - K H) P
  // K H = [K0*0  K0*-dt] = [0  -K0*dt]
  //       [K1*0  K1*-dt]   [0  -K1*dt]
  const float oldP01 = s.P01, oldP11 = s.P11;
  s.P00 = s.P00;
  s.P01 = oldP01 - K0 * dt * oldP11;
  s.P10 = s.P10 - K1 * dt * oldP01;
  s.P11 = oldP11 - K1 * dt * oldP11;

  if (s.P11 < 1e-9f) s.P11 = 1e-9f;

  // Clamp bias
  if (s.bias >  IMU_FUSION_BIAS_CLAMP) s.bias =  IMU_FUSION_BIAS_CLAMP;
  if (s.bias < -IMU_FUSION_BIAS_CLAMP) s.bias = -IMU_FUSION_BIAS_CLAMP;
}

/**
 * Update 2: heading tuyệt đối từ SLAM.
 *
 * H = [1, 0]
 */
inline void updateSlam(float headingAbsRad) {
  State& s = getState();
  if (!s.initialized) return;

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

  // 1) Predict với gyro
  predict(gyroZRad, dt);

  // 2) Update wheel nếu robot đang xoay đáng kể
  if (fabsf((float)leftPct - (float)rightPct) >= IMU_FUSION_WHEEL_MIN_DIFF) {
    // dθ_wheel = (rightPct - leftPct) × LOC_PWM_TO_MPS / WHEEL_BASE_M × dt
    const float dThetaWheel =
        ((float)rightPct - (float)leftPct) * LOC_PWM_TO_MPS / WHEEL_BASE_M * dt;
    updateWheel(gyroZRad, dThetaWheel, dt);
  }

  return s.heading;
#else
  // Fallback: trả về heading cũ
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
  Serial.printf("[%s] h=%.3f bias=%.4f P00=%.4f P11=%.6f\n",
                tag, s.heading, s.bias, s.P00, s.P11);
#endif
}

}  // namespace imuFusion

#endif // IMU_FUSION_H