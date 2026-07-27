/* =====================================================================
 *  Localization.h — Pose estimate dùng IMU (heading) + PWM dead-reckoning (translation)
 *
 *  QUAN TRỌNG: Project này KHÔNG dùng encoder nữa.
 *    - Heading (góc xoay)   ← MPU6050 (cập nhật ở taskControl)
 *    - Translation (x, y)   ← PWM lệnh cuối × hệ số PWM_TO_MPS × dt
 *
 *  Cách hoạt động:
 *    - Motors.h gọi locSetDriveCmd(leftPct, rightPct) mỗi lần botDrive() chạy.
 *    - Odometry.h gọi locUpdate() mỗi ODOM_PERIOD_MS (100ms).
 *      locUpdate() tính ds từ PWM lệnh trong dt kể từ lần cuối.
 *
 *  Hiệu chỉnh:
 *    - LOC_PWM_TO_MPS: hệ số PWM → m/s. Cần đo thực tế:
 *        PWM 100% trong 1 giây đi được bao xa → chia 100 = giá trị mới.
 *      Mặc định 0.0040 = 100% PWM → ~0.40 m/s.
 *
 *  API:
 *    locInit()              — Reset pose về (0,0,0)
 *    locUpdate()            — Tích phân ds/dTheta → cập nhật pose (gọi mỗi ODOM_PERIOD_MS)
 *    locSetDriveCmd(L, R)   — Motors báo lệnh hiện tại (-100..+100 %)
 *    locSetEncoderless(b)   — Bật/tắt fallback PWM (luôn true trong project này)
 *    locResetPose()         — Đặt pose về (0,0,0) + reset heading về 0
 *    g_pose                 — struct {x, y, headingRad} đọc từ mọi module
 * =====================================================================*/
#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "Config.h"
#include <math.h>

// Forward declaration cho ImuFusion EKF để tránh vòng include.
//   ImuFusion.h include "Localization.h" (cần g_pose + LOC_PWM_TO_MPS + WHEEL_BASE_M).
//   Localization.h chỉ cần gọi imuFusion::applySlamPose → forward-declare là đủ.
namespace imuFusion {
  inline void applySlamPose(float headingAbsRad);  // defined in ImuFusion.h
}

#ifndef WHEEL_BASE_M
#define WHEEL_BASE_M    0.365f  // khoảng cách tâm 2 bên bánh (m) — dùng cho dTheta khi lách
#endif

#ifndef LOC_PWM_TO_MPS
// Calibration hệ số tốc độ: 100% PWM → ? m/s
// Đo thực tế: cho robot chạy thẳng 1 giây, đo khoảng cách thực.
// Nếu pose drift quá xa → giảm giá trị này.
// Ví dụ: robot thực tế chạy ~0.15m/s ở 100% PWM → 0.0015
#define LOC_PWM_TO_MPS  0.0015f
#endif

struct Pose2D {
  float x;           // mét, trục X về phía trước lúc boot
  float y;           // mét, trục Y sang phải
  float headingRad;  // radian, [0, 2π)
};

Pose2D g_pose = {0.f, 0.f, 0.f};

/** Lệnh drive cuối cùng (PWM % trái/phải) + thời điểm cập nhật. */
struct LocDriveCmd {
  int16_t  leftPct;
  int16_t  rightPct;
  uint32_t tMs;
};
static LocDriveCmd s_locDriveCmd = {0, 0, 0};
static bool        s_locEnabled  = true;  // luôn true vì project không dùng encoder

inline void locInit() {
  g_pose = {0.f, 0.f, 0.f};
  s_locDriveCmd = {0, 0, 0};
}

inline void locSetDriveCmd(int16_t leftPct, int16_t rightPct) {
  s_locDriveCmd.leftPct  = leftPct;
  s_locDriveCmd.rightPct = rightPct;
  s_locDriveCmd.tMs      = millis();
}

inline void locSetEncoderless(bool enabled) { s_locEnabled = enabled; }

/**
 * Đọc lệnh drive cuối cùng đã ghi vào localization (PWM % trái/phải).
 * Dùng cho ImuFusion EKF updateWheel().
 */
inline void locGetDriveCmd(int16_t &leftPct, int16_t &rightPct) {
  leftPct  = s_locDriveCmd.leftPct;
  rightPct = s_locDriveCmd.rightPct;
}

inline void locResetPose() {
  g_pose = {0.f, 0.f, 0.f};
  s_locDriveCmd = {0, 0, 0};
}

/**
 * Override g_pose bằng tọa độ SLAM từ WebManager (PC chạy Scan Matching + Occupancy Grid).
 * Được gọi khi WebManager gửi lệnh WS { t: "slam_pose", x, y, h }.
 * WaypointNav.h đọc g_pose trực tiếp → tự động dùng SLAM pose chính xác hơn.
 *
 * v2.0 (2026-07-27):
 *   - Thêm rate-limit (chỉ apply mỗi ≥100ms) — tránh spam EKF khi SLAM publish 10Hz.
 *   - Debug log pose feedback khi delta lớn (>20cm hoặc >5°).
 *   - Bỏ dùng wrapPi() (trước đây từ ImuFusion.h) — inline normalize local để
 *     tránh circular include với ImuFusion.h (compile error: 'wrapPi' was not declared).
 */
inline void locSetSlamPose(float x, float y, float headingRad) {
  // Rate-limit: chỉ apply tối đa 10Hz.
  static uint32_t s_lastSlamMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - s_lastSlamMs < 100u) return;
  s_lastSlamMs = nowMs;

  // Debug: in ra khi delta lớn (robot bị SLAM "snap" do loop closure).
  static bool s_first = true;
  if (!s_first) {
    const float dx = x - g_pose.x;
    const float dy = y - g_pose.y;
    const float dist = sqrtf(dx*dx + dy*dy);
    // Tính delta heading về [-π, π] — không phụ thuộc ImuFusion.h (tránh circular include).
    float dH = headingRad - g_pose.headingRad;
    while (dH >  (float)M_PI) dH -= 2.f * (float)M_PI;
    while (dH < -(float)M_PI) dH += 2.f * (float)M_PI;
    const float dh = fabsf(dH);
    if (dist > 0.20f || dh > 0.087f) {  // >20cm hoặc >5°
      Serial.printf("[SLAM] Pose jump: Δpos=%.2fm Δh=%.1f° → (%.2f, %.2f, %.1f°)\n",
                    dist, dh * 180.f / (float)M_PI,
                    x, y, headingRad * 180.f / (float)M_PI);
    }
  }
  s_first = false;

  g_pose.x          = x;
  g_pose.y          = y;
  // Giữ heading trong [0, 2π)
  while (headingRad < 0.f)              headingRad += 2.f * (float)M_PI;
  while (headingRad >= 2.f * (float)M_PI) headingRad -= 2.f * (float)M_PI;
  g_pose.headingRad = headingRad;

  // Cập nhật EKF để reset bias & covariance khi SLAM cung cấp pose tuyệt đối
  imuFusion::applySlamPose(headingRad);
}

/**
 * Tích phân pose từ encoder thật + IMU heading.
 *
 * Project có 2 encoder bánh trước (FL+FR) → đọc được dsL, dsR thật.
 * Localization cũ (v1) dùng LOC_PWM_TO_MPS để tính ds từ PWM% — sai số
 * tích lũy 10-20% / 5m. Bản v2 nhận encoder thẳng, sai số chỉ còn do
 * WHEEL_CIRC_M không chính xác (calibrate 1 lần khi build).
 *
 * Công thức:
 *   ds     = (dsL + dsR)/2
 *   dTheta = (dsR - dsL)/WHEEL_BASE_M      ← dùng dTheta này cho odom pose
 *   x += ds * cos(h)
 *   y += ds * sin(h)
 *   heading += dTheta
 *
 * Sau khi xong, ImuFusion.h predict/updateWheel sẽ fuse với gyro MPU6050
 * → heading EKF ổn định dài hạn, không drift.
 *
 * @param dsL  Quãng đường bánh trái trong tick (m, có dấu)
 * @param dsR  Quãng đường bánh phải trong tick (m, có dấu)
 * @param dt   Khoảng thời gian tích phân (giây) — nếu 0, dùng millis() tự tính
 */
inline void locUpdate(float dsL, float dsR, float dt) {
  if (!s_locEnabled) return;

  // dt an toàn: nếu caller truyền 0 thì tự tính từ millis().
  if (dt <= 0.f) {
    static uint32_t s_lastTMs = 0;
    uint32_t nowMs = millis();
    if (s_lastTMs == 0) { s_lastTMs = nowMs; return; }
    dt = (float)(nowMs - s_lastTMs) * 0.001f;
    s_lastTMs = nowMs;
    if (dt <= 0.f || dt > 2.f) return;
  }

  // Translation trung bình từ 2 encoder.
  const float ds = (dsL + dsR) * 0.5f;

  // dTheta từ chênh lệch 2 bánh (rad).
  //   arc = (dsR - dsL); góc = arc / WHEEL_BASE_M
  //   Dùng trực tiếp dTheta này để update heading (không chờ IMU fusion).
  //   ImuFusion.h vẫn EKF fuse sau để giảm noise + correct gyro bias.
  const float dTheta = (dsR - dsL) / WHEEL_BASE_M;

  // Apply translation (heading dùng IMU đã set ở taskControl).
  const float h = g_pose.headingRad;
  g_pose.x += ds * cosf(h);
  g_pose.y += ds * sinf(h);

  // Apply rotation từ encoder. Nếu chỉ truyền ds=0 (encoder tắt / PWM fallback),
  // bỏ qua để không lan truyền sai số 0.
  if (dsL != 0.f || dsR != 0.f) {
    g_pose.headingRad += dTheta;
    while (g_pose.headingRad <  0.f)              g_pose.headingRad += 2.f * (float)M_PI;
    while (g_pose.headingRad >= 2.f * (float)M_PI) g_pose.headingRad -= 2.f * (float)M_PI;
  }
}

/**
 * Backward-compat overload (giữ cho code cũ gọi locUpdate() không tham số).
 * Nếu không có encoder, fallback về LOC_PWM_TO_MPS (PWM dead-reckoning).
 */
inline void locUpdate() {
  if (!s_locEnabled) return;
  // Fallback: dùng PWM% cuối (project này chỉ chạy khi USE_ENCODER_HARDWARE=0).
  static uint32_t s_lastTMs = 0;
  uint32_t nowMs = millis();
  uint32_t dtMs;
  if (s_lastTMs == 0) { s_lastTMs = nowMs; return; }
  dtMs = nowMs - s_lastTMs;
  s_lastTMs = nowMs;
  if (dtMs == 0 || dtMs > 2000) return;
  const float dt = (float)dtMs * 0.001f;

  const float vL = (float)s_locDriveCmd.leftPct  * LOC_PWM_TO_MPS;
  const float vR = (float)s_locDriveCmd.rightPct * LOC_PWM_TO_MPS;
  const float dLeft  = vL * dt;
  const float dRight = vR * dt;
  locUpdate(dLeft, dRight, dt);
}

#endif // LOCALIZATION_H
