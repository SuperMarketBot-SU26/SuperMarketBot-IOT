/* =====================================================================
 *  ObstacleSensors.h — Sensor Fusion: LiDAR (tầm xa) + HC-SR04 (tầm gần)
 *
 *  Quy tắc kết hợp:
 *    obsFrontCm() = min(lidarFront, usFront)  — vật cản gần nhất
 *    obsBackCm()  = min(lidarBack,  usBack)   — vật cản gần nhất phía sau
 *    obsLeftCm()  = usLeft   (HC-SR04 trái)
 *    obsRightCm() = usRight  (HC-SR04 phải)
 *
 *  Khi một cảm biến không hợp lệ → coi như vô cùng → sensor kia chiếm ưu tiên.
 *
 *  LiDAR deceleration (OA cruise):
 *    lidarFrontInDecelZone() → true nếu trong vùng [DECEL_NEAR, DECEL_FAR]
 *    lidarDecelFactor()      → hệ số 0.15–1.0 để nhân với cruisePwm
 * =====================================================================*/
#ifndef OBSTACLE_SENSORS_H
#define OBSTACLE_SENSORS_H

#include "Config.h"

/* -------- Cảm biến nào có giá trị "đáng tin" -------- */
inline bool obsLidarValid() {
#if USE_LIDAR_HARDWARE
  return lidarCmValid(g_state.lidarFront);
#else
  return false;
#endif
}

inline bool obsUsValid() {
#if USE_HC_SR04_HARDWARE
  return g_state.usFront > (int16_t)US_MIN_VALID_CM
      && g_state.usFront < (int16_t)US_PING_MAX_CM;
#else
  return false;
#endif
}

/* -------- FUSION: lấy khoảng cách nguy hiểm nhất (nhỏ nhất) -------- */
/**
 * Khoảng cách phía trước fusion = min(lidarFront, usFront).
 * Nếu cả hai đều không hợp lệ → trả về MAX (coi như không có vật cản).
 */
inline int16_t obsFrontCm() {
#if USE_LIDAR_HARDWARE && USE_HC_SR04_HARDWARE
  int16_t lf = g_state.lidarFront;
  int16_t uf = g_state.usFront;
  bool lfOk = lidarCmValid(lf);
  bool ufOk = uf > (int16_t)US_MIN_VALID_CM && uf < (int16_t)US_PING_MAX_CM;
  if (lfOk && ufOk) return (int16_t)min((int)lf, (int)uf);
  if (lfOk)         return lf;
  if (ufOk)         return uf;
  return (int16_t)US_PING_MAX_CM;

#elif USE_LIDAR_HARDWARE
  return g_state.lidarFront;

#elif USE_HC_SR04_HARDWARE
  return g_state.usFront;

#else
  return (int16_t)US_PING_MAX_CM;
#endif
}

/**
 * Khoảng cách phía sau fusion = min(lidarBack, usBack).
 */
inline int16_t obsBackCm() {
#if USE_LIDAR_HARDWARE && USE_HC_SR04_HARDWARE
  int16_t lb = g_state.lidarBack;
  int16_t ub = g_state.usBack;
  bool lbOk = lidarCmValid(lb);
  bool ubOk = ub > (int16_t)US_MIN_VALID_CM && ub < (int16_t)US_PING_MAX_CM;
  if (lbOk && ubOk) return (int16_t)min((int)lb, (int)ub);
  if (lbOk)         return lb;
  if (ubOk)         return ub;
  return (int16_t)US_PING_MAX_CM;

#elif USE_LIDAR_HARDWARE
  return g_state.lidarBack;

#elif USE_HC_SR04_HARDWARE
  return g_state.usBack;

#else
  return (int16_t)US_PING_MAX_CM;
#endif
}

/**
 * Trái/phải — chỉ HC-SR04 (LiDAR không có hướng trái/phải).
 */
inline int16_t obsLeftCm()  {
#if USE_HC_SR04_HARDWARE
  return g_state.usLeft;
#else
  return (int16_t)US_PING_MAX_CM;
#endif
}

inline int16_t obsRightCm() {
#if USE_HC_SR04_HARDWARE
  return g_state.usRight;
#else
  return (int16_t)US_PING_MAX_CM;
#endif
}

/* -------- Ngưỡng dừng cứng -------- */
inline bool obsFrontBlocked() { return obsFrontCm() < (int16_t)US_STOP_CM; }
inline bool obsRearBlocked()  { return obsBackCm()  < (int16_t)US_STOP_CM; }

/* -------- Bất kỳ góc nào bị chặn (dùng cho OA) -------- */
inline bool obsAnyCornerBlocked() {
  bool blocked = false;
#if USE_HC_SR04_HARDWARE
  blocked = blocked
    || (g_state.usLF > (int16_t)US_MIN_VALID_CM && g_state.usLF < (int16_t)US_STOP_CM)
    || (g_state.usLR > (int16_t)US_MIN_VALID_CM && g_state.usLR < (int16_t)US_STOP_CM)
    || (g_state.usRF > (int16_t)US_MIN_VALID_CM && g_state.usRF < (int16_t)US_STOP_CM)
    || (g_state.usRR > (int16_t)US_MIN_VALID_CM && g_state.usRR < (int16_t)US_STOP_CM);
#endif
  blocked = blocked || obsFrontBlocked() || obsRearBlocked();
  return blocked;
}

/* -------- OA trigger / path clear -------- */
inline bool obsOaTriggered(int16_t frontCm) {
  if (frontCm <= (int16_t)US_MIN_VALID_CM) return false;
  return frontCm < (int16_t)OA_DETECT_CM;
}

inline bool obsPathClear(int16_t frontCm) {
  if (frontCm <= (int16_t)US_MIN_VALID_CM) return true;
  return frontCm >= (int16_t)PATH_CLEAR_MIN_CM;
}

/* -------- LiDAR deceleration zone (tầm xa) -------- */
/**
 * Kiểm tra LiDAR phía trước có nằm trong vùng giảm tốc.
 * Trả về true khi khoảng cách LiDAR nằm giữa DECEL_NEAR và DECEL_FAR.
 * Dùng trong oaCruiseForward() để giảm tốc sớm trước khi SR04 đo được.
 */
inline bool lidarFrontInDecelZone() {
#if USE_LIDAR_HARDWARE
  int16_t d = g_state.lidarFront;
  if (!lidarCmValid(d)) return false;
  return d > (int16_t)LIDAR_DECEL_NEAR_CM && d < (int16_t)LIDAR_DECEL_FAR_CM;
#else
  return false;
#endif
}

/**
 * Hệ số giảm tốc LiDAR tầm xa (0.15–1.0).
 * Khi LiDAR đo được DECEL_FAR → factor ≈ 1.0 (chạy max).
 * Khi LiDAR đo được DECEL_NEAR → factor = 0.15 (rất chậm).
 * Ngoài vùng → factor = 1.0 (không giảm).
 *
 * Công thức nội suy tuyến tính:
 *   factor = (lidarFront - DECEL_NEAR) / (DECEL_FAR - DECEL_NEAR)
 *   clamp factor in [0.15, 1.0]
 */
inline float lidarDecelFactor() {
#if USE_LIDAR_HARDWARE
  int16_t d = g_state.lidarFront;
  if (!lidarCmValid(d)) return 1.0f;
  if (d >= (int16_t)LIDAR_DECEL_FAR_CM) return 1.0f;
  if (d <= (int16_t)LIDAR_DECEL_NEAR_CM) return 0.15f;
  float factor = (float)(d - LIDAR_DECEL_NEAR_CM)
               / (float)(LIDAR_DECEL_FAR_CM - LIDAR_DECEL_NEAR_CM);
  if (factor > 1.0f) factor = 1.0f;
  if (factor < 0.15f) factor = 0.15f;
  return factor;
#else
  return 1.0f;
#endif
}

#endif // OBSTACLE_SENSORS_H
