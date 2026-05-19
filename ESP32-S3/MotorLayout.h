/* =====================================================================
 *  MotorLayout.h — Góc xe logic → kênh TB6612 vật lý + đảo chiều (NVS)
 *  GPIO không đổi; team hoán vị “bánh nào đang nối kênh nào” + lật hướng.
 *
 *  Slot logic: 0=T.trước, 1=T.sau, 2=P.trước, 3=P.sau (cùng thứ tự cảm biến)
 *  Kênh vật lý (MotorId): 0=FL(PWM4), 1=RL(PWM7), 2=FR(PWM21), 3=RR(PWM40)
 * =====================================================================*/
#ifndef MOTOR_LAYOUT_H
#define MOTOR_LAYOUT_H

#include "Config.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WebSocketsServer.h>
#include <cstring>

uint8_t g_mapMotSlot[4] = {0, 1, 2, 3};
/** Đảo chiều PWM so với logic (tiến/lùi) — theo từng góc xe */
uint8_t g_motInv[4] = {0, 0, 0, 0};

inline bool motorMapIsPermutation4(const uint8_t *m) {
  bool seen[4] = {false, false, false, false};
  for (int i = 0; i < 4; i++) {
    if (m[i] > 3) return false;
    if (seen[m[i]]) return false;
    seen[m[i]] = true;
  }
  return true;
}

inline void motorLayoutApplyDefaults() {
  for (int i = 0; i < 4; i++) {
    g_mapMotSlot[i] = (uint8_t)i;
    g_motInv[i] = 0;
  }
}

inline void motorLayoutLoad(Preferences &prefs) {
  motorLayoutApplyDefaults();
  uint8_t tmp[4];
  for (int i = 0; i < 4; i++) {
    char k[8];
    snprintf(k, sizeof(k), "motM%d", i);
    uint8_t v = prefs.getUChar(k, 255);
    if (v <= 3) tmp[i] = v;
    else tmp[i] = (uint8_t)i;
  }
  if (motorMapIsPermutation4(tmp)) {
    for (int i = 0; i < 4; i++) g_mapMotSlot[i] = tmp[i];
  }
  for (int i = 0; i < 4; i++) {
    char k[8];
    snprintf(k, sizeof(k), "motI%d", i);
    uint8_t inv = prefs.getUChar(k, 255);
    if (inv <= 1) g_motInv[i] = inv;
  }
}

inline bool motorLayoutSave(Preferences &prefs) {
  if (!motorMapIsPermutation4(g_mapMotSlot)) return false;
  if (!prefs.begin(NVS_NAMESPACE, false)) return false;
  for (int i = 0; i < 4; i++) {
    char k[8];
    snprintf(k, sizeof(k), "motM%d", i);
    prefs.putUChar(k, g_mapMotSlot[i]);
    snprintf(k, sizeof(k), "motI%d", i);
    prefs.putUChar(k, g_motInv[i] & 1u);
  }
  prefs.end();
  return true;
}

/**
 * JSON: { "t":"motLayout", "mapMot":[0..3]x4, "motInv":[0|1]x4 }
 */
inline bool motorLayoutApplyJson(JsonDocument &doc, Preferences &prefs) {
  JsonArray jm = doc["mapMot"];
  JsonArray ji = doc["motInv"];
  if (jm.size() != 4 || ji.size() != 4) return false;
  uint8_t tmpM[4], tmpI[4];
  for (uint8_t i = 0; i < 4; i++) {
    int m = jm[i].as<int>();
    int inv = ji[i].as<int>();
    if (m < 0 || m > 3) return false;
    if (inv != 0 && inv != 1) return false;
    tmpM[i] = (uint8_t)m;
    tmpI[i] = (uint8_t)inv;
  }
  if (!motorMapIsPermutation4(tmpM)) return false;
  for (int i = 0; i < 4; i++) {
    g_mapMotSlot[i] = tmpM[i];
    g_motInv[i] = tmpI[i];
  }
  return motorLayoutSave(prefs);
}

inline void motorLayoutReplyToClient(WebSocketsServer &ws, uint8_t clientNum) {
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, true)) return;
  motorLayoutLoad(p);
  p.end();
  JsonDocument doc;
  doc["t"] = "motLayout";
  JsonArray a = doc["mapMot"].to<JsonArray>();
  JsonArray b = doc["motInv"].to<JsonArray>();
  for (int i = 0; i < 4; i++) {
    a.add(g_mapMotSlot[i]);
    b.add(g_motInv[i]);
  }
  char out[160];
  size_t n = serializeJson(doc, out, sizeof(out) - 1);
  if (n > 0) ws.sendTXT(clientNum, out, n);
}

#endif // MOTOR_LAYOUT_H
