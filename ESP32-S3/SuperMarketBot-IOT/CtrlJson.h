/* =====================================================================
 *  CtrlJson.h — Lệnh điều khiển JSON (WebSocket dashboard)
 * =====================================================================*/
#ifndef CTRLJSON_H
#define CTRLJSON_H

#include "Config.h"
#include "Motors.h"
#include "WaypointNav.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <cstring>

extern RobotState g_state;
extern Preferences g_prefs;

/** Dừng motor + về lái tay (gọi khi boot / E-Stop / đổi mode Manual). */
inline void robotForceManualStop() {
  g_state.mode = MODE_MANUAL;
  g_state.cmdX = 0;
  g_state.cmdY = 0;
  g_state.cmdStrafe = 0;
  g_state.estop = false;
  g_usEnabled = false;  // Tắt SR04 khi về lái tay
  botStop();
  wpNavCancel();
}

inline void robotApplyControlJson(JsonDocument &doc) {
  const char *t = doc["t"];
  if (!t) return;

  if (strcmp(t, "joy") == 0) {
    g_state.cmdX = (int16_t)constrain((int)doc["x"].as<int>(), -100, 100);
    g_state.cmdY = (int16_t)constrain((int)doc["y"].as<int>(), -100, 100);
    g_state.cmdStrafe = (int16_t)constrain((int)doc["s"].as<int>(), -100, 100);
  } else if (strcmp(t, "spd") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    g_state.baseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("baseSpeed", g_state.baseSpeed);
    g_prefs.end();
  } else if (strcmp(t, "spdAuto") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    g_state.autoBaseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("autoBaseSpeed", g_state.autoBaseSpeed);
    g_prefs.end();
  } else if (strcmp(t, "mode") == 0) {
    uint8_t m = doc["m"].as<uint8_t>();
    if (m > MODE_WAYPOINT) m = MODE_MANUAL;
    if (m == MODE_MANUAL) {
      robotForceManualStop();
    } else {
      g_state.mode = (RobotMode)m;
      g_state.cmdX = 0;
      g_state.cmdY = 0;
      g_state.cmdStrafe = 0;
      botStop();
    }
  } else if (strcmp(t, "estop") == 0) {
    g_state.estop = true;
    botStop();
    wpNavCancel();
  } else if (strcmp(t, "odomReset") == 0) {
    extern void odomResetDistance();
    odomResetDistance();
  }
}

#endif // CTRLJSON_H
