/* =====================================================================
 *  ESP32-CAM.ino — SmartMarketBot vision node (STA only)
 *
 *  Không điều khiển motor / LiDAR — chỉ stream, capture, upload ảnh.
 *  Robotics: ESP32-S3 (folder riêng, không sửa từ project này).
 *
 *  Board: AI-Thinker ESP32-CAM (+ USB programmer shield)
 *  Core: esp32 by Espressif (Arduino)
 *  Lib: esp32-camera (qua board package / esp_camera.h)
 * =====================================================================*/

#include "Config.h"
#include "CamDriver.h"
#include "WifiSta.h"
#include "HttpCamServer.h"

static uint32_t s_wifiRetryMs = 0;
static uint32_t s_bootMs = 0;
static bool s_camOk = false;
static bool s_httpStarted = false;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  s_bootMs = millis();

  Serial.println();
  Serial.println(F("=== SmartMarketBot ESP32-CAM (vision node) ==="));
  Serial.printf("Free heap: %u  PSRAM: %s\n", (unsigned)ESP.getFreeHeap(),
                psramFound() ? "yes" : "no");

  s_camOk = camInit();
  if (!s_camOk) {
    Serial.println(F("[BOOT] Camera init failed — check 5V, ribbon, board model"));
  }

  wifiStaBegin();
  wifiStaWaitConnect();
  s_wifiRetryMs = millis();

  if (wifiStaConnected()) {
    httpCamServerBegin(s_bootMs);
    s_httpStarted = true;
    Serial.println(F("[BOOT] Ready."));
    Serial.printf("  Preview: http://%s/stream\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Capture: http://%s/capture\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println(F("[BOOT] No WiFi yet — HTTP starts after connect"));
  }
}

void loop() {
  const uint32_t now = millis();

  if (!wifiStaConnected()) {
    wifiStaMaintain(now, s_wifiRetryMs);
    if (wifiStaConnected() && !s_httpStarted) {
      httpCamServerBegin(s_bootMs);
      s_httpStarted = true;
      Serial.printf("[HTTP] Started — http://%s/\n", WiFi.localIP().toString().c_str());
    }
    delay(50);
    yield();
    return;
  }

  if (s_httpStarted) {
    httpCamServerLoop();
  }

  static uint32_t lastLog = 0;
  if (now - lastLog > 60000u) {
    lastLog = now;
    Serial.printf("[HEART] heap=%u min=%u cam=%s rssi=%d\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
                  camStatusStr(), WiFi.RSSI());
  }

  yield();
}
