/* =====================================================================
 *  HttpCamServer.h — HTTP: /, /stream, /capture, /status, /upload
 * =====================================================================*/
#ifndef ESP32_CAM_HTTP_SERVER_H
#define ESP32_CAM_HTTP_SERVER_H

#include "Config.h"
#include "CamDriver.h"
#include "BackendUpload.h"
#include "WifiSta.h"
#include "WebUI.h"
#include <WebServer.h>
#include <WiFi.h>

static WebServer g_http(HTTP_SERVER_PORT);
static WiFiClient g_streamCli;
static uint32_t g_bootMs = 0;
static uint32_t g_lastStreamFrameMs = 0;
static uint32_t g_streamFrameCount = 0;
static volatile bool g_streamActive = false;

inline void httpSendJson(const String &json) {
  g_http.send(200, "application/json", json);
}

inline void httpSendJpeg(camera_fb_t *fb) {
  g_http.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  g_http.sendHeader("Pragma", "no-cache");
  g_http.sendHeader("Access-Control-Allow-Origin", "*");
  g_http.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
}

inline void handleRoot() {
  g_http.sendHeader("Location", "/view", true);
  g_http.send(302, "text/plain", "");
}

inline void handleApi() {
  const bool ok = wifiStaConnected();
  String body = F("{\"device\":\"SmartMarketBot-ESP32-CAM\",\"role\":\"vision-node\",\"ip\":\"");
  body += ok ? WiFi.localIP().toString() : String("0.0.0.0");
  body += F("\",\"wifi\":\"");
  body += ok ? String("connected") : String("disconnected");
  body += F("\",\"rssi\":");
  body += ok ? String(WiFi.RSSI()) : String("null");
  body += F(",\"camera\":\"");
  body += camStatusStr();
  body += F("\",\"endpoints\":[\"/view\",\"/preview\",\"/capture\",\"/stream\",\"/status\"]}");
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
  j += F(",\"profile\":\"");
  j += camProfileName();
  j += F("\",\"hmirror\":");
  j += String((int)camGetHMirror());
  j += F(",\"vflip\":");
  j += String((int)camGetVFlip());
  j += F(",\"streaming\":");
  j += g_streamActive ? F("true") : F("false");
  j += F("}");
  httpSendJson(j);
}

inline void handleOrientation() {
  if (g_http.hasArg("hmirror") || g_http.hasArg("vflip")) {
    const int h = g_http.hasArg("hmirror") ? g_http.arg("hmirror").toInt() : (int)camGetHMirror();
    const int v = g_http.hasArg("vflip") ? g_http.arg("vflip").toInt() : (int)camGetVFlip();
    camSetOrientation(h, v);
  }
  String j = F("{\"hmirror\":");
  j += String((int)camGetHMirror());
  j += F(",\"vflip\":");
  j += String((int)camGetVFlip());
  j += F("}");
  httpSendJson(j);
}

inline void handleProfile() {
  if (g_http.hasArg("mode")) {
    camSetProfile((uint8_t)g_http.arg("mode").toInt());
  }
  String j = F("{\"profile\":\"");
  j += camProfileName();
  j += F("\",\"mode\":");
  j += String((int)camGetProfile());
  j += F("}");
  httpSendJson(j);
}

inline void handleCapture() {
  camera_fb_t *fb = camCapture();
  if (!fb) {
    g_http.send(503, "application/json", F("{\"error\":\"capture_failed\"}"));
    return;
  }
  httpSendJpeg(fb);
  esp_camera_fb_return(fb);
}

inline void handlePreview() {
  camera_fb_t *fb = camCapturePreview();
  if (!fb) {
    g_http.send(503, "application/json", F("{\"error\":\"preview_failed\"}"));
    return;
  }
  httpSendJpeg(fb);
  esp_camera_fb_return(fb);
}

inline void handleView() {
  webUiSendDashboard(g_http);
}

/** MJPEG không chặn — gửi header rồi từng frame trong httpCamServerPollStream (cho /status chạy song song). */
inline void handleStream() {
  if (g_streamActive) {
    g_http.send(503, "text/plain", "stream busy");
    return;
  }
  g_streamCli = g_http.client();
  if (!g_streamCli.connected()) {
    g_http.send(503, "text/plain", "no client");
    return;
  }

  g_streamCli.setNoDelay(true);
  g_streamCli.println(F("HTTP/1.1 200 OK"));
  g_streamCli.println(F("Content-Type: multipart/x-mixed-replace; boundary=frame"));
  g_streamCli.println(F("Cache-Control: no-cache"));
  g_streamCli.println(F("Access-Control-Allow-Origin: *"));
  g_streamCli.println(F("Connection: keep-alive"));
  g_streamCli.println();
  g_streamCli.flush();

  g_streamActive = true;
  g_streamFrameCount = 0;
  g_lastStreamFrameMs = 0;
  Serial.println(F("[HTTP] /stream start (async)"));
}

inline void httpCamServerPollStream() {
  if (!g_streamActive) return;

  if (!g_streamCli.connected()) {
    g_streamActive = false;
    g_streamCli.stop();
    Serial.printf("[HTTP] /stream end (%u frames)\n", (unsigned)g_streamFrameCount);
    return;
  }

  const uint32_t now = millis();
#if STREAM_MIN_FRAME_MS > 0
  if (g_lastStreamFrameMs != 0 && (now - g_lastStreamFrameMs) < STREAM_MIN_FRAME_MS) {
    return;
  }
#endif

  camera_fb_t *fb = camCapturePreview();
  if (!fb) return;

  g_streamCli.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                     (unsigned)fb->len);
  if (g_streamCli.write(fb->buf, fb->len) != fb->len) {
    esp_camera_fb_return(fb);
    g_streamCli.stop();
    g_streamActive = false;
    return;
  }
  g_streamCli.print(F("\r\n"));
  esp_camera_fb_return(fb);

  g_lastStreamFrameMs = now;
  g_streamFrameCount++;
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
  g_http.on("/api", HTTP_GET, handleApi);
  g_http.on("/status", HTTP_GET, handleStatus);
  g_http.on("/preview", HTTP_GET, handlePreview);
  g_http.on("/capture", HTTP_GET, handleCapture);
  g_http.on("/view", HTTP_GET, handleView);
  g_http.on("/orientation", HTTP_GET, handleOrientation);
  g_http.on("/profile", HTTP_GET, handleProfile);
  g_http.on("/stream", HTTP_GET, handleStream);
  g_http.on("/upload", HTTP_GET, handleUpload);
  g_http.on("/upload", HTTP_POST, handleUpload);
  g_http.onNotFound(handleNotFound);

  g_http.begin();
  Serial.printf("[HTTP] Server on port %d\n", HTTP_SERVER_PORT);
  Serial.println(F("[HTTP] Dashboard: /view  |  /preview  /capture  /stream  /status"));
}

inline void httpCamServerLoop() {
  g_http.handleClient();
  httpCamServerPollStream();
}

inline bool httpCamStreamActive() {
  return g_streamActive;
}

#endif /* ESP32_CAM_HTTP_SERVER_H */
