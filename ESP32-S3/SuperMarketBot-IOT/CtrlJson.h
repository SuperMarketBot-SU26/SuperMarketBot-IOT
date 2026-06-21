/* =====================================================================
 *  CtrlJson.h — Lệnh điều khiển JSON (WebSocket dashboard)
 * =====================================================================*/
#ifndef CTRLJSON_H
#define CTRLJSON_H

#include "Config.h"
// Motors.h bị loại khỏi đây vì kéo theo MotorLayout.h → WebSocketsServer → WiFi
// botStop() được extern định nghĩa trong Motors.h (đã include ở .ino)
extern void botStop();
#include "WaypointNav.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstring>

extern RobotState g_state;
extern SemaphoreHandle_t g_stateMutex;
extern Preferences g_prefs;

/** Dừng motor + về lái tay (gọi khi boot / E-Stop / đổi mode Manual). */
inline void robotForceManualStop() {
  g_state.mode = MODE_MANUAL;
  g_state.cmdX = 0;
  g_state.cmdY = 0;
  g_state.cmdStrafe = 0;
  g_state.estop = false;
  botStop();
  wpNavCancel();
}

inline void robotApplyControlJson(JsonDocument &doc) {
  const char *t = doc["t"];
  if (!t) return;

  if (strcmp(t, "joy") == 0) {
    /* Bọc mutex: đọc từ Core 0 (WebSocket), ghi vào g_state.
     * Core 1 (Control) sẽ đọc giá trị đồng nhất của cmdX/Y/Strafe. */
    if (g_stateMutex != NULL) xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    g_state.cmdX = (int16_t)constrain((int)doc["x"].as<int>(), -100, 100);
    g_state.cmdY = (int16_t)constrain((int)doc["y"].as<int>(), -100, 100);
    g_state.cmdStrafe = (int16_t)constrain((int)doc["s"].as<int>(), -100, 100);
    if (g_stateMutex != NULL) xSemaphoreGive(g_stateMutex);

    // In log debug mỗi 500ms khi lái
    static uint32_t lastJoyLog = 0;
    if (millis() - lastJoyLog > 500u) {
      lastJoyLog = millis();
      Serial.printf("[WS-Joy] X:%d, Y:%d, Strafe:%d\n", g_state.cmdX, g_state.cmdY, g_state.cmdStrafe);
    }
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
  } else if (strcmp(t, "spdSwerve") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    g_state.swerveBaseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("swerveSpeed", g_state.swerveBaseSpeed);
    g_prefs.end();
  } else if (strcmp(t, "mode") == 0) {
    uint8_t m = doc["m"].as<uint8_t>();
    Serial.printf("[WS-Mode] Yeu cau chuyen sang Mode: %d\n", m);
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
    Serial.println(F("[WS-EStop] KICH HOAT ESTOP!"));
    g_state.estop = true;
    botStop();
    wpNavCancel();
  } else if (strcmp(t, "odomReset") == 0) {
    extern void odomResetDistance();
    odomResetDistance();
  }
}

#endif // CTRLJSON_H
