/* =====================================================================
 *  SensorLayout.h — Ánh xạ logic (4 góc xe / trước-sau LiDAR) → phần cứng cố định
 *  Lưu NVS; team chỉnh trên web, không đổi GPIO trong code.
 *
 *  Siêu âm vật lý (không đổi dây): 0=Echo Trước (10), 1=Sau (11), 2=Trái (12), 3=Phải (13)
 *  Encoder vật lý: 0=FL(15), 1=RL(16), 2=FR(3), 3=RR(48)
 *  Slot logic: 0=Trái trước, 1=Trái sau, 2=Phải trước, 3=Phải sau
 *  LiDAR: lidF=0 → Serial1 = “LiDAR trước xe”; 1 → Serial2 = trước (đổi vai trừu tượng)
 * =====================================================================*/
#ifndef SENSOR_LAYOUT_H
#define SENSOR_LAYOUT_H

#include "Config.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WebSocketsServer.h>
#include <cstring>

/** Slot góc xe (thứ tự cố định UI) */
enum LayoutCorner : uint8_t {
  SLOT_LF = 0,
  SLOT_LR = 1,
  SLOT_RF = 2,
  SLOT_RR = 3,
};

/** Chỉ số siêu âm vật lý (đúng NewPing trong Sensors.h) */
enum UsPhy : uint8_t {
  US_PHY_F = 0,
  US_PHY_B = 1,
  US_PHY_L = 2,
  US_PHY_R = 3,
};

/** Encoder vật lý (đúng ISR Odometry.h) */
enum EncPhy : uint8_t {
  ENC_PHY_FL = 0,
  ENC_PHY_RL,
  ENC_PHY_FR,
  ENC_PHY_RR,
};

uint8_t g_mapUsSlot[4] = {0, 1, 2, 3};
uint8_t g_mapEncSlot[4] = {0, 1, 2, 3};
/** 0 = Serial1 là LiDAR trước xe; 1 = Serial2 là LiDAR trước xe */
uint8_t g_lidarFrontUart = 0;

inline bool layoutIsPermutation4(const uint8_t *m) {
  bool seen[4] = {false, false, false, false};
  for (int i = 0; i < 4; i++) {
    if (m[i] > 3) return false;
    if (seen[m[i]]) return false;
    seen[m[i]] = true;
  }
  return true;
}

inline void sensorLayoutApplyDefaults() {
  for (int i = 0; i < 4; i++) {
    g_mapUsSlot[i] = (uint8_t)i;
    g_mapEncSlot[i] = (uint8_t)i;
  }
  g_lidarFrontUart = 0;
}

/** Đọc NVS (namespace đã mở sẵn read-only) — không gọi begin/end */
inline void sensorLayoutLoad(Preferences &prefs) {
  sensorLayoutApplyDefaults();
  for (int i = 0; i < 4; i++) {
    char k[8];
    snprintf(k, sizeof(k), "mapU%d", i);
    uint8_t v = prefs.getUChar(k, 255);
    if (v <= 3) g_mapUsSlot[i] = v;
    snprintf(k, sizeof(k), "mapE%d", i);
    v = prefs.getUChar(k, 255);
    if (v <= 3) g_mapEncSlot[i] = v;
  }
  uint8_t lf = prefs.getUChar("lidFront", 255);
  if (lf <= 1) g_lidarFrontUart = lf;
  if (!layoutIsPermutation4(g_mapUsSlot) || !layoutIsPermutation4(g_mapEncSlot)) {
    sensorLayoutApplyDefaults();
  }
}

inline bool sensorLayoutSave(Preferences &prefs) {
  if (!layoutIsPermutation4(g_mapUsSlot) || !layoutIsPermutation4(g_mapEncSlot)) return false;
  if (!prefs.begin(NVS_NAMESPACE, false)) return false;
  for (int i = 0; i < 4; i++) {
    char k[8];
    snprintf(k, sizeof(k), "mapU%d", i);
    prefs.putUChar(k, g_mapUsSlot[i]);
    snprintf(k, sizeof(k), "mapE%d", i);
    prefs.putUChar(k, g_mapEncSlot[i]);
  }
  prefs.putUChar("lidFront", g_lidarFrontUart);
  prefs.end();
  return true;
}

/**
 * JSON: { "t":"layout", "us":[0..3]x4, "enc":[..], "lidF":0|1 }
 * Mảng theo thứ tự slot: Trái trước, Trái sau, Phải trước, Phải sau
 */
inline bool sensorLayoutApplyJson(JsonDocument &doc, Preferences &prefs) {
  JsonArray jus = doc["us"];
  JsonArray jen = doc["enc"];
  if (jus.size() != 4 || jen.size() != 4) return false;
  uint8_t tmpU[4], tmpE[4];
  for (uint8_t i = 0; i < 4; i++) {
    int u = jus[i].as<int>();
    int e = jen[i].as<int>();
    if (u < 0 || u > 3 || e < 0 || e > 3) return false;
    tmpU[i] = (uint8_t)u;
    tmpE[i] = (uint8_t)e;
  }
  if (!layoutIsPermutation4(tmpU) || !layoutIsPermutation4(tmpE)) return false;
  int lf = doc["lidF"].as<int>();
  if (lf != 0 && lf != 1) return false;
  for (int i = 0; i < 4; i++) {
    g_mapUsSlot[i] = tmpU[i];
    g_mapEncSlot[i] = tmpE[i];
  }
  g_lidarFrontUart = (uint8_t)lf;
  return sensorLayoutSave(prefs);
}

/** Trả lời 1 client WebSocket (layout hiện tại) */
inline void sensorLayoutReplyToClient(WebSocketsServer &ws, uint8_t clientNum) {
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, true)) return;
  sensorLayoutLoad(p);
  p.end();
  JsonDocument doc;
  doc["t"] = "layout";
  JsonArray a = doc["us"].to<JsonArray>();
  JsonArray b = doc["enc"].to<JsonArray>();
  for (int i = 0; i < 4; i++) {
    a.add(g_mapUsSlot[i]);
    b.add(g_mapEncSlot[i]);
  }
  doc["lidF"] = g_lidarFrontUart;
  char out[192];
  size_t n = serializeJson(doc, out, sizeof(out) - 1);
  if (n > 0) ws.sendTXT(clientNum, out, n);
}

#endif // SENSOR_LAYOUT_H
