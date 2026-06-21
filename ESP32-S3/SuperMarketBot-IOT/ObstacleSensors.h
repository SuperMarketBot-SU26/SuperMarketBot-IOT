/* =====================================================================
 *  ObstacleSensors.h — API thống nhất LiDAR hoặc 4× HC-SR04 (4 góc xe)
 * =====================================================================*/
#ifndef OBSTACLE_SENSORS_H
#define OBSTACLE_SENSORS_H

#include "Config.h"

#if USE_HC_SR04_HARDWARE

inline bool obsCmValid(int16_t cm) {
  return cm > (int16_t)US_MIN_VALID_CM && cm < (int16_t)US_PING_MAX_CM;
}

/** Trước = min(trái trước, phải trước, lidar trước); sau = min(trái sau, phải sau, lidar sau). */
inline int16_t obsFrontCm() {
  int16_t f = g_state.usFront;
#if USE_LIDAR_HARDWARE
  if (lidarCmValid(g_state.lidarFront)) {
    if (g_state.lidarFront < f) f = g_state.lidarFront;
  }
#endif
  return f;
}

inline int16_t obsBackCm() {
  int16_t b = g_state.usBack;
#if USE_LIDAR_HARDWARE
  if (lidarCmValid(g_state.lidarBack)) {
    if (g_state.lidarBack < b) b = g_state.lidarBack;
  }
#endif
  return b;
}
inline int16_t obsLeftCm()  { return g_state.usLeft; }
inline int16_t obsRightCm() { return g_state.usRight; }

inline bool obsFrontBlocked() {
  return obsCmValid(obsFrontCm()) && obsFrontCm() < (int16_t)US_STOP_CM;
}

inline bool obsRearBlocked() {
  return obsCmValid(obsBackCm()) && obsBackCm() < (int16_t)US_STOP_CM;
}

inline bool obsAnyCornerBlocked() {
  return (obsCmValid(g_state.usLF) && g_state.usLF < (int16_t)US_STOP_CM)
      || (obsCmValid(g_state.usLR) && g_state.usLR < (int16_t)US_STOP_CM)
      || (obsCmValid(g_state.usRF) && g_state.usRF < (int16_t)US_STOP_CM)
      || (obsCmValid(g_state.usRR) && g_state.usRR < (int16_t)US_STOP_CM);
}

inline bool obsOaTriggered(int16_t frontCm) {
  return obsCmValid(frontCm) && frontCm < (int16_t)US_OA_DETECT_CM;
}

inline bool obsPathClear(int16_t frontCm) {
  return !obsCmValid(frontCm) || frontCm >= (int16_t)US_PATH_CLEAR_CM;
}

inline int16_t obsOaDetectCm()   { return (int16_t)US_OA_DETECT_CM; }
inline int16_t obsPathClearCm()  { return (int16_t)US_PATH_CLEAR_CM; }
inline int16_t obsStopCm()       { return (int16_t)US_STOP_CM; }

#else

inline bool obsCmValid(int16_t cm) {
  return lidarCmValid(cm);
}

inline int16_t obsFrontCm() { return g_state.lidarFront; }
inline int16_t obsBackCm()  { return g_state.lidarBack; }
inline int16_t obsLeftCm()  { return g_state.usLeft; }
inline int16_t obsRightCm() { return g_state.usRight; }

inline bool obsFrontBlocked() {
  return lidarFrontBlocked(obsFrontCm());
}

inline bool obsRearBlocked() {
  return lidarRearBlocked(obsBackCm());
}

inline bool obsAnyCornerBlocked() { return obsFrontBlocked(); }

inline bool obsOaTriggered(int16_t frontCm) {
  return lidarCmValid(frontCm) && frontCm < (int16_t)OA_DETECT_CM;
}

inline bool obsPathClear(int16_t frontCm) {
  return !lidarCmValid(frontCm) || frontCm >= (int16_t)PATH_CLEAR_MIN_CM;
}

inline int16_t obsOaDetectCm()   { return (int16_t)OA_DETECT_CM; }
inline int16_t obsPathClearCm()  { return (int16_t)PATH_CLEAR_MIN_CM; }
inline int16_t obsStopCm()       { return (int16_t)AUTO_LIDAR_BLOCK_CM; }

#endif

#endif // OBSTACLE_SENSORS_H
