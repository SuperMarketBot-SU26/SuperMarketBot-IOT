/* =====================================================================
 *  HttpCamServer.h — HTTP: /, /stream, /capture, /status, /upload
 * =====================================================================*/
#ifndef ESP32_CAM_HTTP_SERVER_H
#define ESP32_CAM_HTTP_SERVER_H

#include "Config.h"
#include "CamDriver.h"
#include "BackendUpload.h"
#include "WifiSta.h"
#include <WebServer.h>
#include <WiFi.h>

static WebServer g_http(HTTP_SERVER_PORT);
static uint32_t g_bootMs = 0;
static uint32_t g_lastStreamFrameMs = 0;

inline void httpSendJson(const String &json) {
  g_http.send(200, "application/json", json);
}

inline void handleRoot() {
  const bool ok = wifiStaConnected();
  String body = F("{\n  \"device\": \"SmartMarketBot-ESP32-CAM\",\n  \"role\": \"vision-node\",\n");
  body += F("  \"ip\": \"");
  body += ok ? WiFi.localIP().toString() : String("0.0.0.0");
  body += F("\",\n  \"wifi\": \"");
  body += ok ? String("connected") : String("disconnected");
  body += F("\",\n  \"rssi\": ");
  body += ok ? String(WiFi.RSSI()) : String("null");
  body += F(",\n  \"camera\": \"");
  body += camStatusStr();
  body += F("\",\n  \"endpoints\": [\"/\", \"/stream\", \"/capture\", \"/status\"");
#if BACKEND_UPLOAD_ENABLE
  body += F(", \"/upload\"");
#endif
  body += F("]\n}");
  httpSendJson(body);
}

inline void handleStatus() {
  const bool ok = wifiStaConnected();
  const uint32_t up = (millis() - g_bootMs) / 1000u;
  String j = F("{");
  j += F("\"ip\":\"");
  j += ok ? WiFi.localIP().toString() : String("0.0.0.0");
  j += F("\",\"wifi\":");
  j += ok ? String(WiFi.RSSI()) : String("null");
  j += F(",\"heap\":");
  j += String(ESP.getFreeHeap());
  j += F(",\"heapMin\":");
  j += String(ESP.getMinFreeHeap());
  j += F(",\"psram\":");
  j += psramFound() ? String(ESP.getFreePsram()) : String("null");
  j += F(",\"uptime\":");
  j += String(up);
  j += F(",\"camera\":\"");
  j += camStatusStr();
  j += F("\",\"wifiStatus\":");
  j += String((int)WiFi.status());
  j += F("}");
  httpSendJson(j);
}

inline void handleCapture() {
  camera_fb_t *fb = camCapture();
  if (!fb) {
    g_http.send(503, "application/json", F("{\"error\":\"capture_failed\"}"));
    return;
  }
  g_http.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

inline void handleStream() {
  WiFiClient client = g_http.client();
  if (!client.connected()) {
    g_http.send(503, "text/plain", "client disconnected");
    return;
  }

  g_http.setContentLength(CONTENT_LENGTH_UNKNOWN);
  g_http.send(200, "multipart/x-mixed-replace; boundary=frame",
              "SmartMarketBot MJPEG");
  g_lastStreamFrameMs = 0;

  while (client.connected()) {
    const uint32_t now = millis();
    if (g_lastStreamFrameMs != 0 &&
        (now - g_lastStreamFrameMs) < STREAM_MIN_FRAME_MS) {
      delay(1);
      yield();
      continue;
    }

    camera_fb_t *fb = camCapture();
    if (!fb) {
      delay(10);
      yield();
      continue;
    }

    g_http.sendContent(String("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ") +
                       String((unsigned)fb->len) + "\r\n\r\n");
    if (client.write(fb->buf, fb->len) != fb->len) {
      esp_camera_fb_return(fb);
      break;
    }
    g_http.sendContent("\r\n");
    esp_camera_fb_return(fb);
    g_lastStreamFrameMs = now;
    yield();
  }
}

inline void handleUpload() {
#if !BACKEND_UPLOAD_ENABLE
  g_http.send(503, "application/json",
              F("{\"error\":\"upload_disabled\",\"hint\":\"set BACKEND_UPLOAD_ENABLE 1 in Config.h\"}"));
  return;
#else
  camera_fb_t *fb = camCapture();
  if (!fb) {
    g_http.send(503, "application/json", F("{\"error\":\"capture_failed\"}"));
    return;
  }
  int code = -1;
  const bool ok = backendUploadFrame(fb, code);
  esp_camera_fb_return(fb);

  String j = F("{\"uploaded\":");
  j += ok ? "true" : "false";
  j += F(",\"httpCode\":");
  j += String(code);
  j += F("}");
  g_http.send(ok ? 200 : 502, "application/json", j);
#endif
}

inline void handleNotFound() {
  g_http.send(404, "application/json",
              F("{\"error\":\"not_found\",\"paths\":[\"\\/\",\"\\/stream\",\"\\/capture\",\"\\/status\",\"\\/upload\"]}"));
}

inline void httpCamServerBegin(uint32_t bootMs) {
  g_bootMs = bootMs;

  g_http.on("/", HTTP_GET, handleRoot);
  g_http.on("/status", HTTP_GET, handleStatus);
  g_http.on("/capture", HTTP_GET, handleCapture);
  g_http.on("/stream", HTTP_GET, handleStream);
  g_http.on("/upload", HTTP_GET, handleUpload);
  g_http.on("/upload", HTTP_POST, handleUpload);
  g_http.onNotFound(handleNotFound);

  g_http.begin();
  Serial.printf("[HTTP] Server on port %d\n", HTTP_SERVER_PORT);
  Serial.println(F("[HTTP] GET /  /status  /capture  /stream  /upload"));
}

inline void httpCamServerLoop() {
  g_http.handleClient();
}

#endif /* ESP32_CAM_HTTP_SERVER_H */
