/* =====================================================================
 *  VisionHttps.h — HTTPS (443) cho camera tablet (secure context)
 *  HTTP :80 giữ dashboard + WebSocket. Camera: https://192.168.4.1/vision
 * =====================================================================*/
#ifndef VISION_HTTPS_H
#define VISION_HTTPS_H

#include "Config.h"
#include "SslCert.h"
#include "VisionTablet.h"
#include <WiFi.h>
#include <esp_https_server.h>

static httpd_handle_t g_httpsServer = nullptr;

static esp_err_t httpsSendProgmem(httpd_req_t *req, const char *html,
                                  const char *ctype) {
  const size_t len = strlen_P(html);
  char *buf = (char *)malloc(len + 1);
  if (!buf) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  memcpy_P(buf, html, len + 1);
  httpd_resp_set_type(req, ctype);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t r = httpd_resp_send(req, buf, len);
  free(buf);
  return r;
}

static esp_err_t httpsHandleVision(httpd_req_t *req) {
  return httpsSendProgmem(req, VISION_TABLET_HTML, "text/html; charset=utf-8");
}

static esp_err_t httpsHandleStatus(httpd_req_t *req) {
  String j = "{\"ip\":\"" + WiFi.softAPIP().toString() +
             "\",\"wifi\":null,\"camera\":\"tablet\",\"clients\":" +
             String(WiFi.softAPgetStationNum()) + ",\"https\":true}";
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, j.c_str(), j.length());
}

static const char HTTPS_ROOT_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SmartMarketBot HTTPS</title></head><body style="font-family:system-ui;background:#0a1628;color:#e8f0ff;padding:20px">
<h1>Camera tablet (HTTPS)</h1>
<p>Trinh duyet can HTTPS de bat camera. Neu co canh bao chung chi — chon <b>Tiep tuc / Advanced / Proceed</b>.</p>
<p><a href="/vision" style="color:#2ee6a8;font-size:1.1rem">Mo /vision</a> ·
<a href="http://192.168.4.1/" style="color:#3b9eff">Dashboard HTTP</a></p>
</body></html>
)HTML";

static esp_err_t httpsHandleRoot(httpd_req_t *req) {
  return httpsSendProgmem(req, HTTPS_ROOT_HTML, "text/html; charset=utf-8");
}

inline bool visionHttpsBegin() {
#if !VISION_HTTPS_ENABLE
  Serial.println(F("[HTTPS] Tat (VISION_HTTPS_ENABLE=0) — dashboard HTTP nhe hon."));
  return false;
#endif
  if (g_httpsServer) return true;

  static char certRam[2200];
  static char keyRam[2200];
  strncpy_P(certRam, SMB_SSL_CERT, sizeof(certRam) - 1);
  strncpy_P(keyRam, SMB_SSL_KEY, sizeof(keyRam) - 1);
  certRam[sizeof(certRam) - 1] = '\0';
  keyRam[sizeof(keyRam) - 1] = '\0';

  httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
  conf.port_secure = WEB_SSL_PORT;
  conf.servercert = (const uint8_t *)certRam;
  conf.servercert_len = strlen(certRam) + 1;
  conf.prvtkey_pem = (const uint8_t *)keyRam;
  conf.prvtkey_len = strlen(keyRam) + 1;
  conf.httpd.max_uri_handlers = 8;
  conf.httpd.stack_size = 10240;

  esp_err_t err = httpd_ssl_start(&g_httpsServer, &conf);
  if (err != ESP_OK || !g_httpsServer) {
    Serial.printf("[HTTPS] httpd_ssl_start that bai: %d\n", (int)err);
    g_httpsServer = nullptr;
    return false;
  }

  httpd_uri_t uRoot = {.uri = "/", .method = HTTP_GET, .handler = httpsHandleRoot};
  httpd_uri_t uVision = {.uri = "/vision", .method = HTTP_GET, .handler = httpsHandleVision};
  httpd_uri_t uStatus = {.uri = "/status", .method = HTTP_GET, .handler = httpsHandleStatus};
  httpd_register_uri_handler(g_httpsServer, &uRoot);
  httpd_register_uri_handler(g_httpsServer, &uVision);
  httpd_register_uri_handler(g_httpsServer, &uStatus);

  Serial.printf("[HTTPS] Camera tablet: https://%s/vision (port %d)\n",
                WiFi.softAPIP().toString().c_str(), (int)WEB_SSL_PORT);
  Serial.println(F("[HTTPS] Lan dau: trinh duyet hoi chap nhan chung chi tu ky — chon Tiep tuc."));
  return true;
}

#endif
