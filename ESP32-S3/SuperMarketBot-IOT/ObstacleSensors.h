/* =====================================================================
 *  ObstacleSensors.h — API thống nhất 4x HC-SR04 (4 góc xe)
 *  TF-Luna đã bỏ hoàn toàn (2026-07-30).
 * =====================================================================*/
#ifndef OBSTACLE_SENSORS_H
#define OBSTACLE_SENSORS_H

#include "Config.h"

inline bool obsCmValid(int16_t cm) {
  return cm > (int16_t)US_MIN_VALID_CM && cm < (int16_t)US_PING_MAX_CM;
}

/** Trước = min(trái trước, phải trước); sau = min(trái sau, phải sau). */
inline int16_t obsFrontCm() { return g_state.usFront; }
inline int16_t obsBackCm()  { return g_state.usBack; }
inline int16_t obsLeftCm()  { return g_state.usLeft; }
inline int16_t obsRightCm() { return g_state.usRight; }

inline bool obsFrontBlocked() {
  return obsCmValid(obsFrontCm()) && obsFrontCm() < (int16_t)g_state.usStopCm;
}

inline bool obsRearBlocked() {
  return obsCmValid(obsBackCm()) && obsBackCm() < (int16_t)g_state.usStopCm;
}

inline bool obsAnyCornerBlocked() {
  return (obsCmValid(g_state.usLF) && g_state.usLF < (int16_t)g_state.usStopCm)
      || (obsCmValid(g_state.usLR) && g_state.usLR < (int16_t)g_state.usStopCm)
      || (obsCmValid(g_state.usRF) && g_state.usRF < (int16_t)g_state.usStopCm)
      || (obsCmValid(g_state.usRR) && g_state.usRR < (int16_t)g_state.usStopCm);
}

inline bool obsOaTriggered(int16_t frontCm) {
  return obsCmValid(frontCm) && frontCm < (int16_t)g_state.usOaDetectCm;
}

inline bool obsPathClear(int16_t frontCm) {
  return !obsCmValid(frontCm) || frontCm >= (int16_t)g_state.usPathClearCm;
}

inline int16_t obsOaDetectCm()   { return (int16_t)g_state.usOaDetectCm; }
inline int16_t obsPathClearCm()  { return (int16_t)g_state.usPathClearCm; }
inline int16_t obsStopCm()       { return (int16_t)g_state.usStopCm; }

#endif // OBSTACLE_SENSORS_H
