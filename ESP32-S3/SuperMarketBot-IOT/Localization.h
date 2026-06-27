/* =====================================================================
 *  Localization.h — Dead Reckoning (2D pose estimate)
 *
 *  Mô hình: Differential Drive — trung bình encoder trái & phải
 *    left  = avg(FL, RL)
 *    right = avg(FR, RR)
 *    ds    = (dLeft + dRight) / 2
 *    dTheta= (dRight - dLeft) / WHEEL_BASE_M
 *    x    += ds * cos(heading + dTheta/2)
 *    y    += ds * sin(heading + dTheta/2)
 *
 *  API:
 *    locInit()          — Reset pose về gốc toạ độ
 *    locUpdate(dTicks)  — Gọi từ odomUpdate() sau mỗi ODOM_PERIOD_MS
 *    locResetPose()     — Reset về (0, 0, 0)
 *    g_pose             — struct { x, y, headingRad } đọc từ các module khác
 * =====================================================================*/
#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "Config.h"
#include <math.h>

/* -------------------- Thông số cơ học ----------------------------- */
/** Khoảng cách tâm bánh trái → tâm bánh phải (m). Đo thực tế trên robot. */
#ifndef WHEEL_BASE_M
#define WHEEL_BASE_M    0.365f  // Khung 25.5x15cm: L_W (track width) ~17.0cm + L_L (axle distance) ~19.5cm = 36.5cm hiệu dụng cho xoay Mecanum
#endif

/* -------------------- Cấu trúc Pose ------------------------------- */
struct Pose2D {
  float x;           // mét, trục X về phía trước lúc boot
  float y;           // mét, trục Y sang phải
  float headingRad;  // radian, [0, 2π), 0 = hướng ban đầu
};

Pose2D g_pose = {0.f, 0.f, 0.f};

/** Chiều vật lý từ Motors.h — ký hiệu hóa ticks để phân biệt tiến/lùi/xoay */
extern volatile int8_t g_motorDir[4];

/* -------------------- Biến tích lũy ticks (Phase 2 bổ sung) ------- */
/** Tổng ticks trên từng bánh kể từ lần locUpdate() cuối — được cộng bởi ISR qua odomUpdate() */
static uint32_t s_locTotalFL = 0;
static uint32_t s_locTotalFR = 0;
static uint32_t s_locTotalRL = 0;
static uint32_t s_locTotalRR = 0;

/* =====================================================================
 *  locInit() — Reset state. Gọi 1 lần sau odomInit() trong setup().
 * ===================================================================*/
inline void locInit() {
  g_pose = {0.f, 0.f, 0.f};
  s_locTotalFL = s_locTotalFR = s_locTotalRL = s_locTotalRR = 0;
}

extern uint8_t g_mapMotSlot[4];
extern uint8_t g_motInv[4];

static inline int8_t locGetPhysicalDir(uint8_t p) {
  for (int s = 0; s < 4; s++) {
    if (g_mapMotSlot[s] == p) {
      int8_t d = g_motorDir[p];
      if (g_motInv[s]) return -d;
      return d;
    }
  }
  return g_motorDir[p];
}

/* =====================================================================
 *  locUpdate() — Cập nhật pose dựa trên Δticks từ lần gọi trước.
 *
 *  Tham số: snapshot tổng ticks hiện tại (từ g_totalFL/FR/RL/RR trong Odometry.h).
 *           Đọc giá trị này trong atomic section trước khi gọi locUpdate().
 * ===================================================================*/
inline void locUpdate(uint32_t totalFL, uint32_t totalFR,
                      uint32_t totalRL, uint32_t totalRR) {
  /* Delta ticks so với lần gọi trước */
  uint32_t dFL = totalFL - s_locTotalFL;
  uint32_t dFR = totalFR - s_locTotalFR;
  uint32_t dRL = totalRL - s_locTotalRL;
  uint32_t dRR = totalRR - s_locTotalRR;
  s_locTotalFL = totalFL;
  s_locTotalFR = totalFR;
  s_locTotalRL = totalRL;
  s_locTotalRR = totalRR;

  /* Ký hiệu hóa delta bằng hướng vật lý (g_motorDir) — phân biệt tiến/lùi (có xét đảo chiều) */
  int32_t sdPhy[4];
  sdPhy[0] = (int32_t)dFL * locGetPhysicalDir(0); // Physical FL
  sdPhy[1] = (int32_t)dRL * locGetPhysicalDir(1); // Physical RL
  sdPhy[2] = (int32_t)dFR * locGetPhysicalDir(2); // Physical FR
  sdPhy[3] = (int32_t)dRR * locGetPhysicalDir(3); // Physical RR

  // Ánh xạ các xung vật lý về các slot logic (LF, LR, RF, RR)
  int32_t sdLF = sdPhy[g_mapEncSlot[SLOT_LF]];
  int32_t sdLR = sdPhy[g_mapEncSlot[SLOT_LR]];
  int32_t sdRF = sdPhy[g_mapEncSlot[SLOT_RF]];
  int32_t sdRR = sdPhy[g_mapEncSlot[SLOT_RR]];

  /* Quãng đường mỗi bên (m) — trung bình trước + sau của các slot logic */
  const float ticksToM = (WHEEL_CIRC_M / ENC_PPR) * ODOM_CALIB_FACTOR;
  float dLeft  = ((float)(sdLF + sdLR) * 0.5f) * ticksToM;
  float dRight = ((float)(sdRF + sdRR) * 0.5f) * ticksToM;

  /* Tính ds và dθ */
  float ds     = (dLeft + dRight) * 0.5f;
  float dTheta = (dRight - dLeft) / WHEEL_BASE_M;

  if (g_imuEnabled) {
    // Nếu có IMU hoạt động, dùng góc headingRad cập nhật từ Gyro ở taskControl độc lập.
    // Ta chỉ dùng ds từ encoder để chiếu dịch chuyển lên hệ tọa độ thế giới.
    float midH = g_pose.headingRad;
    g_pose.x          += ds * cosf(midH);
    g_pose.y          += ds * sinf(midH);
  } else {
    // Nếu không có IMU hoặc IMU chưa sẵn sàng/lỗi, tự động hạ cấp xuống dùng bộ tích lũy Encoder (Dead Reckoning)
    float midH = g_pose.headingRad + dTheta * 0.5f;
    g_pose.x          += ds * cosf(midH);
    g_pose.y          += ds * sinf(midH);
    g_pose.headingRad += dTheta;

    /* Giữ heading trong [0, 2π) */
    while (g_pose.headingRad < 0.f)         g_pose.headingRad += 2.f * (float)M_PI;
    while (g_pose.headingRad >= 2.f * (float)M_PI) g_pose.headingRad -= 2.f * (float)M_PI;
  }
}

/* =====================================================================
 *  locResetPose() — Đặt lại gốc toạ độ (ví dụ: khi dock station).
 * ===================================================================*/
inline void locResetPose() {
  g_pose = {0.f, 0.f, 0.f};
  s_locTotalFL = s_locTotalFR = s_locTotalRL = s_locTotalRR = 0;
}

#endif // LOCALIZATION_H
