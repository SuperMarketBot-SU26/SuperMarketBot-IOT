/* =====================================================================
 *  CtrlJson.h — Lệnh điều khiển JSON dùng chung WebSocket + BLE
 * =====================================================================*/
#ifndef CTRLJSON_H
#define CTRLJSON_H

#include "Config.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <cstring>

extern RobotState g_state;
extern Preferences g_prefs;

inline void robotApplyControlJson(JsonDocument &doc) {
  const char *t = doc["t"];
  if (!t) return;

  if (strcmp(t, "joy") == 0) {
    g_state.cmdX = (int16_t)constrain((int)doc["x"].as<int>(), -100, 100);
    g_state.cmdY = (int16_t)constrain((int)doc["y"].as<int>(), -100, 100);
  } else if (strcmp(t, "spd") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    g_state.baseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("baseSpeed", g_state.baseSpeed);
    g_prefs.end();
  } else if (strcmp(t, "mode") == 0) {
    g_state.mode = (RobotMode)doc["m"].as<uint8_t>();
  } else if (strcmp(t, "estop") == 0) {
    g_state.estop = true;
  } else if (strcmp(t, "odomReset") == 0) {
    extern void odomResetDistance();
    odomResetDistance();
  }
}

#endif // CTRLJSON_H
