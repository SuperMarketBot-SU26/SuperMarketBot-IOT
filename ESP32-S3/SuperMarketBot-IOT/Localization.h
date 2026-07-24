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

inline void locResetPose() {
  g_pose = {0.f, 0.f, 0.f};
  s_locDriveCmd = {0, 0, 0};
}

/**
 * Override g_pose bằng tọa độ SLAM từ WebManager (PC chạy Scan Matching + Occupancy Grid).
 * Được gọi khi WebManager gửi lệnh WS { t: "slam_pose", x, y, h }.
 * WaypointNav.h đọc g_pose trực tiếp → tự động dùng SLAM pose chính xác hơn.
 */
inline void locSetSlamPose(float x, float y, float headingRad) {
  g_pose.x          = x;
  g_pose.y          = y;
  // Giữ heading trong [0, 2π)
  while (headingRad < 0.f)              headingRad += 2.f * (float)M_PI;
  while (headingRad >= 2.f * (float)M_PI) headingRad -= 2.f * (float)M_PI;
  g_pose.headingRad = headingRad;
}

/**
 * Tích phân pose từ lệnh PWM cuối. Gọi mỗi ODOM_PERIOD_MS (100ms) từ taskControl.
 * - vL = leftPct  × LOC_PWM_TO_MPS  (m/s)
 * - vR = rightPct × LOC_PWM_TO_MPS
 * - ds = (vL + vR)/2 × dt
 * - dTheta = (vR - vL)/WHEEL_BASE_M × dt
 * - x += ds*cos(h), y += ds*sin(h)
 * - Heading update CHỈ từ IMU (g_pose.headingRad đã được taskControl ghi đè).
 */
inline void locUpdate() {
  if (!s_locEnabled) return;

  static uint32_t s_lastTMs = 0;
  uint32_t nowMs = millis();
  uint32_t dtMs;
  if (s_lastTMs == 0) {
    s_lastTMs = nowMs;
    return;  // lần đầu chỉ lưu timestamp
  }
  dtMs = nowMs - s_lastTMs;
  s_lastTMs = nowMs;
  if (dtMs == 0 || dtMs > 2000) return;  // dt bất thường → bỏ qua

  float dt = (float)dtMs * 0.001f;

  // Dùng lệnh drive TẠI THỜI ĐIỂM locUpdate() chạy (không phải locSetDriveCmd tại tick trước)
  // → đơn giản, sai số ~1 tick, chấp nhận được cho waypoint nav vì goal xa vài m.
  float vL = (float)s_locDriveCmd.leftPct  * LOC_PWM_TO_MPS;
  float vR = (float)s_locDriveCmd.rightPct * LOC_PWM_TO_MPS;

  float dLeft  = vL * dt;
  float dRight = vR * dt;
  float ds     = (dLeft + dRight) * 0.5f;

  // Heading đã được IMU set; ở đây ta chỉ update x,y theo heading hiện tại.
  float h = g_pose.headingRad;
  g_pose.x += ds * cosf(h);
  g_pose.y += ds * sinf(h);
}

#endif // LOCALIZATION_H
