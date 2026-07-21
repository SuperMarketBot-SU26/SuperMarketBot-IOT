/* =====================================================================
 *  RobotTelemetry.h — JSON telemetry cho WebSocket / dashboard
 * =====================================================================*/
#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include "Config.h"
#include "SensorLayout.h"
#include "MotorLayout.h"
#include "Sensors.h"
#include "Odometry.h"
#include "MotorTrim.h"   // NV1c — motor trim state accessor for telemetry
#include "LineDecoder.h" // cần extern g_lineSpeedPct (slider mode LINE)
#include "LineSensor.h"  // Phase 9 — line sensor pattern enum
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <cstdio>
#include <esp_heap_caps.h>

// Fallback extern để chắc chắn thấy biến
extern uint8_t g_lineSpeedPct;

/** Đọc nhiệt độ chip (°C). Trả về NAN nếu không đọc được. */
inline float readChipTempCelsius() {
  float t = temperatureRead();
  if (t < -40.f || t > 125.f) return NAN;
  return t;
}

/** Convert LinePattern enum → string cho WebSocket telemetry. */
inline const char* linePatternToStr(LinePattern p) {
  switch (p) {
    case LINE_PAT_UNKNOWN:  return "unknown";
    case LINE_PAT_LOST:     return "lost";
    case LINE_PAT_TRACKING: return "track";
    case LINE_PAT_JUNCTION: return "junc";
    case LINE_PAT_NODE:     return "node";
  }
  return "unknown";
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

/**
 * JSON telemetry WebSocket.
 * @param includeSlow true = thêm heap/nhiệt/pin/map (nặng); false = gói nhẹ cho HMI.
 */
inline void robotTelemetryFillJson(JsonDocument &doc, bool includeSlow = true) {
#if USE_LIDAR_HARDWARE
  doc["lf"] = g_state.lidarFront;
  doc["lb"] = g_state.lidarBack;
  doc["senMode"] = "lidar";
#else
  doc["lf"] = g_state.usFront;
  doc["lb"] = g_state.usBack;
  doc["senMode"] = "us4";
#endif
  doc["uf"] = g_state.usFront;
  doc["ub"] = g_state.usBack;
  doc["ul"] = g_state.usLeft;
  doc["ur"] = g_state.usRight;
  doc["usLF"] = g_state.usLF;
  doc["usLR"] = g_state.usLR;
  doc["usRF"] = g_state.usRF;
  doc["usRR"] = g_state.usRR;
  if (includeSlow) {
    JsonArray mu = doc["mapU"].to<JsonArray>();
    JsonArray me = doc["mapE"].to<JsonArray>();
    for (int i = 0; i < 4; i++) {
      mu.add(g_mapUsSlot[i]);
      me.add(g_mapEncSlot[i]);
    }
    doc["lidF"] = g_lidarFrontUart;
    JsonArray mm = doc["mapMot"].to<JsonArray>();
    JsonArray mi = doc["motInv"].to<JsonArray>();
    JsonArray ms = doc["motSc"].to<JsonArray>();
    for (int i = 0; i < 4; i++) {
      mm.add(g_mapMotSlot[i]);
      mi.add(g_motInv[i]);
      ms.add(g_motorScale[i]);
    }
  }
  doc["rFL"] = g_state.rpmFL;
  doc["rRL"] = g_state.rpmRL;
  doc["rFR"] = g_state.rpmFR;
  doc["rRR"] = g_state.rpmRR;
  doc["dFL"] = g_state.distFL;
  doc["dRL"] = g_state.distRL;
  doc["dFR"] = g_state.distFR;
  doc["dRR"] = g_state.distRR;
  doc["mode"] = (uint8_t)g_state.mode;
  doc["afs"]  = g_autoFsmState;   /* 0=CRUISE 1=STOP 2=SCAN 3=DECEL 4=BACKUP */
  doc["wpSt"] = (const char *)g_wpStatus;
  doc["estop"] = g_state.estop;
 
  doc["upMs"] = (uint32_t)millis();
  doc["apCli"] = WiFi.softAPgetStationNum();

#if USE_LINE_SENSOR
  // Phase 9 — Line sensor telemetry
  JsonArray lr = doc["lineR"].to<JsonArray>();
  for (int i = 0; i < 8; i++) lr.add(g_state.lineRaw[i]);
  doc["lineOff"]    = g_state.lineOffset;
  doc["lineMask"]   = g_state.lineActiveMask;
  doc["lineStab"]   = g_state.lineStableFrames;
  doc["linePat"]    = linePatternToStr((LinePattern)g_state.linePattern);
  doc["linePatRaw"] = (uint8_t)g_state.linePattern;
  doc["lastNode"]   = g_state.lastNodeId;
#endif

  // NV1c — Motor trim telemetry (để web/BE debug drift + auto-cal)
  doc["scaleL"] = (double)g_state.leftMotorScale;
  doc["scaleR"] = (double)g_state.rightMotorScale;
#if AUTO_CAL_ENABLE
  {
    MotorTrimState& mt = motorTrimInstance();
    doc["calDrift"] = (double)mt.lastDriftDegps;
    doc["calAdjCount"] = (uint16_t)mt.adjustCount;
    doc["calDirty"] = (uint8_t)(mt.dirty ? 1 : 0);
  }
#else
  doc["calDrift"] = 0.0;
  doc["calAdjCount"] = 0;
  doc["calDirty"] = 0;
#endif
 
  if (includeSlow) {
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
    doc["cpuMHz"] = ESP.getCpuFreqMHz();
    doc["health"] = computeHealthLevel(tC, heapInt);
    doc["chip"] = ESP.getChipModel();
    doc["flashKB"] = (uint32_t)(ESP.getFlashChipSize() / 1024u);
    doc["mac"] = WiFi.softAPmacAddress();
    doc["ch"] = (int)WiFi.channel();
    doc["hMin"] = (uint32_t)heap_caps_get_minimum_free_size(
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
 
  doc["cx"] = (int)g_state.cmdX;
  doc["cy"] = (int)g_state.cmdY;
  doc["cstr"] = (int)g_state.cmdStrafe;
  doc["HeadingRad"] = g_pose.headingRad;
  doc["xCoord"] = g_pose.x;
  doc["yCoord"] = g_pose.y;
  doc["wheelMode"] = 1; // cố định = Normal (differential drive) — giữ field để tương thích UI cũ
  uint32_t spdPct =
      (g_state.baseSpeed * 100u) / (uint32_t)(PWM_MAX ? PWM_MAX : 1u);
  doc["spdPct"] = spdPct;
  uint32_t autoSpdUse =
      g_state.autoBaseSpeed ? g_state.autoBaseSpeed : g_state.baseSpeed;
  doc["spdAutoPct"] =
      (autoSpdUse * 100u) / (uint32_t)(PWM_MAX ? PWM_MAX : 1u);
  uint32_t swerveSpdUse =
      g_state.swerveBaseSpeed ? g_state.swerveBaseSpeed : (PWM_MAX * 40 / 100);
  doc["spdSwervePct"] =
      (swerveSpdUse * 100u) / (uint32_t)(PWM_MAX ? PWM_MAX : 1u);
  // Line mode speed (slider)
  doc["spdLinePct"] = (int)g_lineSpeedPct;
  uint32_t rotateSpdUse =
      g_state.rotateBaseSpeed ? g_state.rotateBaseSpeed : (PWM_MAX * 10 / 100);
  doc["spdRotatePct"] =
      (rotateSpdUse * 100u) / (uint32_t)(PWM_MAX ? PWM_MAX : 1u);
 
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
 
  if (includeSlow) {
    float batVolts = -1.f;
    int batPct = -1;
    batteryRead(batVolts, batPct);
    g_batPct = batPct; // Cập nhật biến toàn cục để MQTT/AutoDock sử dụng
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
 
  /* HMI: có tín hiệu thật gần đây → web hiển thị ON (không thì OFF) */
  auto recentOk = [nowMs](uint32_t lastMs, uint32_t winMs) -> bool {
    return lastMs != 0u && (nowMs - lastMs) < winMs;
  };
#if USE_LIDAR_HARDWARE
  bool l1ok = recentOk(g_luna1LastOkMs, SENSOR_LINK_MS_LIDAR);
  bool l2ok = recentOk(g_luna2LastOkMs, SENSOR_LINK_MS_LIDAR);
  doc["lfOn"] = (uint8_t)((g_lidarFrontUart == 0u) ? l1ok : l2ok);
  doc["lbOn"] = (uint8_t)((g_lidarFrontUart == 0u) ? l2ok : l1ok);
#else
  bool usFok = recentOk(g_usPhyLastEchoMs[US_PHY_F], SENSOR_LINK_MS_US)
            || recentOk(g_usPhyLastEchoMs[US_PHY_L], SENSOR_LINK_MS_US);
  bool usBok = recentOk(g_usPhyLastEchoMs[US_PHY_B], SENSOR_LINK_MS_US)
            || recentOk(g_usPhyLastEchoMs[US_PHY_R], SENSOR_LINK_MS_US);
  doc["lfOn"] = (uint8_t)usFok;
  doc["lbOn"] = (uint8_t)usBok;
#endif
  JsonArray jUsOn = doc["usOn"].to<JsonArray>();
  JsonArray jEnOn = doc["encOn"].to<JsonArray>();
  for (int s = 0; s < 4; s++) {
    uint8_t pu = g_mapUsSlot[s];
    uint8_t pe = g_mapEncSlot[s];
    if (pu > 3u) pu = (uint8_t)s;
    if (pe > 3u) pe = (uint8_t)s;
    jUsOn.add((uint8_t)recentOk(g_usPhyLastEchoMs[pu], SENSOR_LINK_MS_US));
    jEnOn.add((uint8_t)recentOk(g_encPhyLastPulseMs[pe], SENSOR_LINK_MS_ENC));
  }
}

#endif // ROBOT_TELEMETRY_H
