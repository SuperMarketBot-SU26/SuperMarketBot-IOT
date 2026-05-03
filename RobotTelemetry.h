/* =====================================================================
 *  RobotTelemetry.h — JSON telemetry dùng chung WebSocket + BLE
 * =====================================================================*/
#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include "Config.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <cstdio>
#include <esp_heap_caps.h>

/** Đọc nhiệt độ chip (°C). Trả về NAN nếu không đọc được. */
inline float readChipTempCelsius() {
  float t = temperatureRead();
  if (t < -40.f || t > 125.f) return NAN;
  return t;
}

/** 0 = OK, 1 = cảnh báo, 2 = nghiêm trọng (nhiệt + heap SRAM nội bộ). */
inline int computeHealthLevel(float tempC, uint32_t heapIntFree) {
  if (heapIntFree < 20000u) return 2;
  if (tempC == tempC && tempC >= 90.f) return 2;
  if (heapIntFree < 45000u) return 1;
  if (tempC == tempC && tempC >= 80.f) return 1;
  return 0;
}

#if BAT_MONITOR_ENABLE
inline void batteryMonitorInit() {
  pinMode(BAT_ADC_PIN, INPUT);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
}
inline void batteryRead(float &voltsOut, int &pctOut) {
  const float ratio = (BAT_DIV_R1_KOHM + BAT_DIV_R2_KOHM) / BAT_DIV_R2_KOHM;
  uint32_t sum = 0;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    sum += (uint32_t)analogReadMilliVolts(BAT_ADC_PIN);
    delayMicroseconds(250);
  }
  float vPin = (sum / (float)n) / 1000.0f;
  voltsOut = vPin * ratio;
  const float den = (BAT_V_FULL - BAT_V_EMPTY);
  int p = (den > 0.01f)
              ? (int)((voltsOut - BAT_V_EMPTY) / den * 100.0f)
              : 0;
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  pctOut = p;
}
#else
inline void batteryMonitorInit() {}
inline void batteryRead(float &voltsOut, int &pctOut) {
  voltsOut = -1.f;
  pctOut = -1;
}
#endif

/** Payload đầy đủ — gửi qua WebSocket (chunk lớn). */
inline void robotTelemetryFillJson(JsonDocument &doc) {
  doc["lf"] = g_state.lidarFront;
  doc["lb"] = g_state.lidarBack;
  doc["uf"] = g_state.usFront;
  doc["ub"] = g_state.usBack;
  doc["ul"] = g_state.usLeft;
  doc["ur"] = g_state.usRight;
  doc["rFL"] = g_state.rpmFL;
  doc["rRL"] = g_state.rpmRL;
  doc["rFR"] = g_state.rpmFR;
  doc["rRR"] = g_state.rpmRR;
  doc["dFL"] = g_state.distFL;
  doc["dRL"] = g_state.distRL;
  doc["dFR"] = g_state.distFR;
  doc["dRR"] = g_state.distRR;
  doc["mode"] = (uint8_t)g_state.mode;
  doc["estop"] = g_state.estop;

  float tC = readChipTempCelsius();
  if (tC == tC && tC >= -40.f && tC <= 125.f) {
    doc["tempC"] = (double)((int)(tC * 10.f + 0.5f)) / 10.0;
  } else {
    doc["tempC"] = -1.0;
  }
  uint32_t heapInt = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  doc["heap"] = (uint32_t)ESP.getFreeHeap();
  doc["heapIn"] = heapInt;
  if (psramFound()) {
    doc["psFree"] = (uint32_t)ESP.getFreePsram();
    doc["psTot"] = (uint32_t)ESP.getPsramSize();
  } else {
    doc["psFree"] = 0;
    doc["psTot"] = 0;
  }
  doc["upMs"] = (uint32_t)millis();
  doc["apCli"] = WiFi.softAPgetStationNum();
  doc["cpuMHz"] = ESP.getCpuFreqMHz();
  doc["health"] = computeHealthLevel(tC, heapInt);

  doc["chip"] = ESP.getChipModel();
  doc["flashKB"] = (uint32_t)(ESP.getFlashChipSize() / 1024u);
  char buildBuf[48];
  snprintf(buildBuf, sizeof(buildBuf), "%s %s", __DATE__, __TIME__);
  doc["build"] = buildBuf;
  doc["mac"] = WiFi.softAPmacAddress();
  doc["ch"] = (int)WiFi.channel();

  doc["hMin"] = (uint32_t)heap_caps_get_minimum_free_size(
      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  doc["cx"] = (int)g_state.cmdX;
  doc["cy"] = (int)g_state.cmdY;
  uint32_t spdPct =
      (g_state.baseSpeed * 100u) / (uint32_t)(PWM_MAX ? PWM_MAX : 1u);
  doc["spdPct"] = spdPct;

  const uint32_t nowMs = (uint32_t)millis();
  if (g_state.lidarLastUpdateMs == 0u) {
    doc["lfAge"] = -1;
  } else {
    uint32_t age = nowMs - g_state.lidarLastUpdateMs;
    doc["lfAge"] = (int32_t)(age > 86400000u ? 86400000 : age);
  }
  if (g_state.usLastUpdateMs == 0u) {
    doc["usAge"] = -1;
  } else {
    uint32_t ageU = nowMs - g_state.usLastUpdateMs;
    doc["usAge"] = (int32_t)(ageU > 86400000u ? 86400000 : ageU);
  }

  float batVolts = -1.f;
  int batPct = -1;
  batteryRead(batVolts, batPct);
  if (batPct >= 0 && batVolts >= 0.f) {
    doc["batV"] = (double)((int)(batVolts * 10.f + 0.5f)) / 10.0;
    doc["batPct"] = batPct;
  } else {
    doc["batV"] = -1.0;
    doc["batPct"] = -1;
  }

  doc["lr1"] = (uint32_t)g_lunaRxBytes1;
  doc["lr2"] = (uint32_t)g_lunaRxBytes2;
}

/**
 * Telemetry gọn cho BLE notify (MTU ~512 sau trao đổi; giữ < ~480 byte JSON).
 * Cùng ý nghĩa field với web; bỏ chuỗi dài (MAC, build, tên chip đầy đủ).
 */
inline void robotTelemetryFillJsonBle(JsonDocument &doc) {
  doc["v"] = 1;
  doc["lf"] = g_state.lidarFront;
  doc["lb"] = g_state.lidarBack;
  doc["uf"] = g_state.usFront;
  doc["ub"] = g_state.usBack;
  doc["ul"] = g_state.usLeft;
  doc["ur"] = g_state.usRight;
  doc["rFL"] = (double)((int)(g_state.rpmFL * 10.f + 0.5f)) / 10.0;
  doc["rRL"] = (double)((int)(g_state.rpmRL * 10.f + 0.5f)) / 10.0;
  doc["rFR"] = (double)((int)(g_state.rpmFR * 10.f + 0.5f)) / 10.0;
  doc["rRR"] = (double)((int)(g_state.rpmRR * 10.f + 0.5f)) / 10.0;
  doc["dFL"] = (double)((int)(g_state.distFL * 1000.f + 0.5f)) / 1000.0;
  doc["dRL"] = (double)((int)(g_state.distRL * 1000.f + 0.5f)) / 1000.0;
  doc["dFR"] = (double)((int)(g_state.distFR * 1000.f + 0.5f)) / 1000.0;
  doc["dRR"] = (double)((int)(g_state.distRR * 1000.f + 0.5f)) / 1000.0;
  doc["mode"] = (uint8_t)g_state.mode;
  doc["es"] = g_state.estop;
  doc["cx"] = (int)g_state.cmdX;
  doc["cy"] = (int)g_state.cmdY;
  doc["sp"] = (uint32_t)(g_state.baseSpeed * 100u /
                         (uint32_t)(PWM_MAX ? PWM_MAX : 1u));

  float tC = readChipTempCelsius();
  if (tC == tC && tC >= -40.f && tC <= 125.f) {
    doc["tC"] = (double)((int)(tC * 10.f + 0.5f)) / 10.0;
  } else {
    doc["tC"] = -1.0;
  }
  uint32_t heapInt = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  doc["hi"] = heapInt;
  doc["up"] = (uint32_t)millis();
  doc["cli"] = WiFi.softAPgetStationNum();
  doc["hm"] = computeHealthLevel(tC, heapInt);

  const uint32_t nowMs = (uint32_t)millis();
  if (g_state.lidarLastUpdateMs == 0u) {
    doc["lfa"] = -1;
  } else {
    uint32_t age = nowMs - g_state.lidarLastUpdateMs;
    doc["lfa"] = (int32_t)(age > 86400000u ? 86400000 : age);
  }
  if (g_state.usLastUpdateMs == 0u) {
    doc["usa"] = -1;
  } else {
    uint32_t ageU = nowMs - g_state.usLastUpdateMs;
    doc["usa"] = (int32_t)(ageU > 86400000u ? 86400000 : ageU);
  }

  doc["l1"] = (uint32_t)g_lunaRxBytes1;
  doc["l2"] = (uint32_t)g_lunaRxBytes2;

#if BAT_MONITOR_ENABLE
  float batVolts = -1.f;
  int batPct = -1;
  batteryRead(batVolts, batPct);
  if (batPct >= 0 && batVolts >= 0.f) {
    doc["bV"] = (double)((int)(batVolts * 10.f + 0.5f)) / 10.0;
    doc["bP"] = batPct;
  }
#endif
}

#endif // ROBOT_TELEMETRY_H
