/* =====================================================================
 *  BackendUpload.h — POST multipart JPEG tới ASP.NET Core API
 * =====================================================================*/
#ifndef ESP32_CAM_BACKEND_UPLOAD_H
#define ESP32_CAM_BACKEND_UPLOAD_H

#include "Config.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include "esp_camera.h"

inline bool backendUploadFrame(const camera_fb_t *fb, int &httpCodeOut) {
  httpCodeOut = -1;
#if !BACKEND_UPLOAD_ENABLE
  (void)fb;
  return false;
#else
  if (!fb || fb->len == 0) return false;

  HTTPClient http;
  http.setTimeout(BACKEND_UPLOAD_TIMEOUT_MS);
  http.setReuse(false);

  if (!http.begin(BACKEND_UPLOAD_URL)) {
    Serial.println(F("[UPLOAD] http.begin failed"));
    return false;
  }

  const String boundary = "esp32camBoundary7f3a9c2e";
  const String hdr =
      String("--") + boundary + "\r\n"
      "Content-Disposition: form-data; name=\"" BACKEND_UPLOAD_FIELD
      "\"; filename=\"capture.jpg\"\r\n"
      "Content-Type: image/jpeg\r\n\r\n";
  const String ftr = String("\r\n--") + boundary + "--\r\n";

  const size_t totalLen = hdr.length() + fb->len + ftr.length();
  uint8_t *body = nullptr;

  if (psramFound()) {
    body = (uint8_t *)ps_malloc(totalLen);
  }
  if (!body) {
    body = (uint8_t *)malloc(totalLen);
  }
  if (!body) {
    Serial.println(F("[UPLOAD] malloc failed"));
    http.end();
    return false;
  }

  memcpy(body, hdr.c_str(), hdr.length());
  memcpy(body + hdr.length(), fb->buf, fb->len);
  memcpy(body + hdr.length() + fb->len, ftr.c_str(), ftr.length());

  http.addHeader("Content-Type",
                 "multipart/form-data; boundary=" + boundary);

  httpCodeOut = http.POST(body, totalLen);
  free(body);
  http.end();

  Serial.printf("[UPLOAD] POST %s — HTTP %d\n", BACKEND_UPLOAD_URL, httpCodeOut);
  return (httpCodeOut > 0 && httpCodeOut < 400);
#endif
}

#endif /* ESP32_CAM_BACKEND_UPLOAD_H */
