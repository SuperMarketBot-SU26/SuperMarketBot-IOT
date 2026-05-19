/* =====================================================================
 *  WifiSta.h — STA only, auto reconnect, in IP
 * =====================================================================*/
#ifndef ESP32_CAM_WIFI_STA_H
#define ESP32_CAM_WIFI_STA_H

#include "Config.h"
#include <WiFi.h>

inline void wifiStaBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

#if WIFI_FIXED_CHANNEL > 0
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_FIXED_CHANNEL);
#else
  WiFi.begin(WIFI_SSID, WIFI_PASS);
#endif

  Serial.printf("[WiFi] Connecting to \"%s\"...\n", WIFI_SSID);
}

/** true khi đã có IP */
inline bool wifiStaConnected() {
  return WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0);
}

inline void wifiStaPrintStatus() {
  if (wifiStaConnected()) {
    Serial.printf("[WiFi] Connected — IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.printf("[WiFi] Status: %d\n", (int)WiFi.status());
  }
}

/**
 * Gọi trong loop: chờ kết nối lần đầu hoặc reconnect định kỳ.
 * @return true nếu đang connected
 */
inline bool wifiStaMaintain(uint32_t nowMs, uint32_t &lastAttemptMs) {
  if (wifiStaConnected()) {
    return true;
  }

  if (nowMs - lastAttemptMs < WIFI_RECONNECT_INTERVAL_MS) {
    return false;
  }
  lastAttemptMs = nowMs;

  if (WiFi.status() == WL_CONNECTED) {
    wifiStaPrintStatus();
    return true;
  }

  Serial.println(F("[WiFi] Reconnecting..."));
  WiFi.disconnect(false, false);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    yield();
  }

  if (wifiStaConnected()) {
    wifiStaPrintStatus();
    return true;
  }

  Serial.println(F("[WiFi] Connect timeout"));
  return false;
}

/** Chờ kết nối lúc boot (blocking có timeout). */
inline bool wifiStaWaitConnect() {
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(300);
    Serial.print('.');
    yield();
  }
  Serial.println();
  if (wifiStaConnected()) {
    wifiStaPrintStatus();
    return true;
  }
  Serial.println(F("[WiFi] Boot connect failed — will retry in loop"));
  return false;
}

#endif /* ESP32_CAM_WIFI_STA_H */
