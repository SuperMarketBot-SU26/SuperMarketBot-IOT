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

/** Trước = min(trái trước, phải trước); sau = min(trái sau, phải sau). */
inline int16_t obsFrontCm() { return g_state.usFront; }
inline int16_t obsBackCm()  { return g_state.usBack; }
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
