/* =====================================================================
 *  MotorLayout.h — Góc xe logic → kênh TB6612 vật lý + đảo chiều (NVS)
 *  GPIO không đổi; team hoán vị "bánh nào đang nối kênh nào" + lật hướng.
 *
 *  Slot logic: 0=T.trước, 1=T.sau, 2=P.trước, 3=P.sau (cùng thứ tự cảm biến)
 *  Kênh vật lý (MotorId): 0=FL(PWM4), 1=RL(PWM7), 2=FR(PWM21), 3=RR(PWM40)
 *
 *  Cải tiến: Hỗ trợ 4 scale riêng cho từng bánh để cân bằng
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

/** Scale riêng cho từng bánh (1.0 = bình thường) */
float g_motorScale[4] = {1.0f, 1.0f, 1.0f, 1.0f};

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
    g_motorScale[i] = 1.0f;
  }
  // Bỏ wheelMode — hệ thống chỉ dùng differential drive (bánh thường)
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

  // Đọc 4 scale riêng
  for (int i = 0; i < 4; i++) {
    char k[8];
    snprintf(k, sizeof(k), "motSc%d", i);
    float sc = prefs.getFloat(k, 1.0f);
    if (sc >= 0.5f && sc <= 1.5f) {
      g_motorScale[i] = sc;
    }
  }

  // Cập nhật left/right scale từ 4 bánh
  g_state.leftMotorScale = (g_motorScale[0] + g_motorScale[1]) / 2.0f;
  g_state.rightMotorScale = (g_motorScale[2] + g_motorScale[3]) / 2.0f;
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
    // Lưu scale riêng
    snprintf(k, sizeof(k), "motSc%d", i);
    prefs.putFloat(k, g_motorScale[i]);
  }
  prefs.end();
  return true;
}

/**
 * JSON: { "t":"motLayout", "mapMot":[0..3]x4, "motInv":[0|1]x4, "motSc":[1.0]x4 }
 * (wheelMode đã bỏ — hệ thống chỉ dùng differential drive)
 */
inline bool motorLayoutApplyJson(JsonDocument &doc, Preferences &prefs) {
  JsonArray jm = doc["mapMot"];
  JsonArray ji = doc["motInv"];
  JsonArray jsc = doc["motSc"];
  
  if (jm.size() == 4 && ji.size() == 4) {
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
  }
  
  // Cập nhật scale nếu có
  if (jsc.size() == 4) {
    for (int i = 0; i < 4; i++) {
      float sc = jsc[i].as<float>();
      if (sc >= 0.5f && sc <= 1.5f) {
        g_motorScale[i] = sc;
      }
    }
    // Cập nhật left/right scale
    g_state.leftMotorScale = (g_motorScale[0] + g_motorScale[1]) / 2.0f;
    g_state.rightMotorScale = (g_motorScale[2] + g_motorScale[3]) / 2.0f;
  }

  // (wheelMode đã bỏ — luôn là differential drive)
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
  JsonArray c = doc["motSc"].to<JsonArray>();
  for (int i = 0; i < 4; i++) {
    a.add(g_mapMotSlot[i]);
    b.add(g_motInv[i]);
    c.add(g_motorScale[i]);
  }
  char out[220];
  size_t n = serializeJson(doc, out, sizeof(out) - 1);
  if (n > 0) ws.sendTXT(clientNum, out, n);
}

/**
 * Đặt wheel mode (gọi từ CtrlJson.h khi nhận lệnh wheelMode)
 * (Hàm đã bỏ — hệ thống chỉ dùng differential drive, không còn wheelMode)
 */
inline void motorLayoutSetWheelMode(uint8_t /*mode*/) {
  // no-op
}

/**
 * Đảo chiều 1 bánh theo slot
 * @param slot   0=Trái trước, 1=Trái sau, 2=Phải trước, 3=Phải sau
 * @param invert 0=không đảo, 1=đảo chiều
 */
inline void motorInvertSlot(uint8_t slot, uint8_t invert) {
  if (slot > 3) return;
  g_motInv[slot] = invert & 1;
}

/**
 * Toggle đảo chiều 1 bánh theo slot
 * @return giá trị mới sau khi toggle
 */
inline uint8_t motorLayoutToggleInvert(uint8_t slot) {
  if (slot > 3) return 0;
  g_motInv[slot] ^= 1;
  return g_motInv[slot];
}

/**
 * Đặt scale cho 1 bánh
 * @param slot   0-3
 * @param scale  0.5-1.5
 */
inline void motorSetScale(uint8_t slot, float scale) {
  if (slot > 3) return;
  scale = constrain(scale, 0.5f, 1.5f);
  g_motorScale[slot] = scale;
  
  // Cập nhật left/right scale
  g_state.leftMotorScale = (g_motorScale[0] + g_motorScale[1]) / 2.0f;
  g_state.rightMotorScale = (g_motorScale[2] + g_motorScale[3]) / 2.0f;
}

/**
 * Lấy scale của 1 bánh
 */
inline float motorGetScale(uint8_t slot) {
  if (slot > 3) return 1.0f;
  return g_motorScale[slot];
}

/**
 * Auto-balance: đặt tất cả scale về trung bình
 */
inline void motorAutoBalance() {
  float avg = (g_motorScale[0] + g_motorScale[1] + g_motorScale[2] + g_motorScale[3]) / 4.0f;
  for (int i = 0; i < 4; i++) {
    g_motorScale[i] = avg;
  }
  g_state.leftMotorScale = (g_motorScale[0] + g_motorScale[1]) / 2.0f;
  g_state.rightMotorScale = (g_motorScale[2] + g_motorScale[3]) / 2.0f;
  Serial.printf("[MotorTrim] Auto-balanced to %.3f\n", avg);
}

/**
 * Cân bằng 2 bên: điều chỉnh scale để 2 bên cùng tốc độ
 */
inline void motorBalanceSides() {
  float avgLeft = (g_motorScale[0] + g_motorScale[1]) / 2.0f;
  float avgRight = (g_motorScale[2] + g_motorScale[3]) / 2.0f;
  float avg = (avgLeft + avgRight) / 2.0f;
  
  // Cân bằng lại: set all = average
  for (int i = 0; i < 4; i++) g_motorScale[i] = avg;
  
  g_state.leftMotorScale = (g_motorScale[0] + g_motorScale[1]) / 2.0f;
  g_state.rightMotorScale = (g_motorScale[2] + g_motorScale[3]) / 2.0f;
  Serial.printf("[MotorTrim] Balanced sides: L=%.3f R=%.3f\n", 
                g_state.leftMotorScale, g_state.rightMotorScale);
}

/**
 * Reset tất cả scale về 1.0
 */
inline void motorResetScales() {
  for (int i = 0; i < 4; i++) {
    g_motorScale[i] = 1.0f;
  }
  g_state.leftMotorScale = g_state.rightMotorScale = 1.0f;
  Serial.println("[MotorTrim] Scales reset to 1.0");
}

/**
 * Lưu motor layout vào NVS
 */
inline bool motorLayoutSaveCurrent(Preferences &prefs) {
  return motorLayoutSave(prefs);
}

#endif // MOTOR_LAYOUT_H
