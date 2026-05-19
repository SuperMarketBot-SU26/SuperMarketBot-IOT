/* =====================================================================
 *  WifiSta.h — STA, tối ưu cho stream trên AP robot (S3 kênh 6)
 * =====================================================================*/
#ifndef ESP32_CAM_WIFI_STA_H
#define ESP32_CAM_WIFI_STA_H

#include "Config.h"
#include <WiFi.h>
#include "esp_wifi.h"

inline void wifiStaOptimize() {
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
#if defined(WIFI_POWER_19_5dBm)
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
#endif
}

inline void wifiStaBegin() {
  WiFi.mode(WIFI_STA);
  wifiStaOptimize();
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

#if WIFI_FIXED_CHANNEL > 0
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_FIXED_CHANNEL);
  Serial.printf("[WiFi] Connecting \"%s\" ch=%d ...\n", WIFI_SSID, WIFI_FIXED_CHANNEL);
#else
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] Connecting \"%s\" ...\n", WIFI_SSID);
#endif
}

inline bool wifiStaConnected() {
  return WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0);
}

inline void wifiStaPrintStatus() {
  if (wifiStaConnected()) {
    Serial.printf("[WiFi] IP %s  RSSI %d dBm  ch %d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.channel());
  } else {
    Serial.printf("[WiFi] status %d\n", (int)WiFi.status());
  }
}

inline bool wifiStaMaintain(uint32_t nowMs, uint32_t &lastAttemptMs) {
  if (wifiStaConnected()) return true;
  if (nowMs - lastAttemptMs < WIFI_RECONNECT_INTERVAL_MS) return false;
  lastAttemptMs = nowMs;

  Serial.println(F("[WiFi] Reconnecting..."));
  WiFi.disconnect(false, false);
  delay(80);
#if WIFI_FIXED_CHANNEL > 0
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_FIXED_CHANNEL);
#else
  WiFi.begin(WIFI_SSID, WIFI_PASS);
#endif

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200);
    yield();
  }
  if (wifiStaConnected()) {
    wifiStaOptimize();
    wifiStaPrintStatus();
    return true;
  }
  return false;
}

inline bool wifiStaWaitConnect() {
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print('.');
    yield();
  }
  Serial.println();
  if (wifiStaConnected()) {
    wifiStaOptimize();
    wifiStaPrintStatus();
    return true;
  }
  Serial.println(F("[WiFi] Boot timeout — retry in loop"));
  return false;
}

#endif
