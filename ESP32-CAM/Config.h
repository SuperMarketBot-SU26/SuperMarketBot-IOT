/* =====================================================================
 *  Config.h — ESP32-CAM (STA vào hotspot ESP32-S3)
 *
 *  PHẢI KHỚP ESP32-S3/Config.h:
 *    AP_SSID         = WIFI_SSID
 *    AP_PASS         = WIFI_PASS
 *    AP_WIFI_CHANNEL = WIFI_FIXED_CHANNEL
 *
 *  Demo vision trên tablet: mở http://192.168.4.1 → mục Vision (S3 web).
 *  ESP32-CAM có thể tắt nguồn nếu chỉ dùng camera tablet.
 * =====================================================================*/
#ifndef ESP32_CAM_CONFIG_H
#define ESP32_CAM_CONFIG_H

#ifndef WIFI_SSID
#define WIFI_SSID "SmartMarketBot"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "12345678"
#endif

#define WIFI_FIXED_CHANNEL  6

#define WIFI_RECONNECT_INTERVAL_MS  8000u
#define WIFI_CONNECT_TIMEOUT_MS     20000u

#define HTTP_SERVER_PORT  80
#define STREAM_MIN_FRAME_MS  0u

#define CAM_LIVE_FRAMESIZE_QVGA  1
#define CAM_HD_FRAMESIZE_VGA     1
#define CAM_JPEG_QUALITY         10
#define CAM_PREVIEW_QUALITY      30
#define CAM_HMIRROR_DEFAULT      1
#define CAM_VFLIP_DEFAULT        1
#define CAM_FB_COUNT             2
#define CAM_XCLK_FREQ_HZ         20000000
#define CAM_GAIN_CEILING_16X     0

#define BACKEND_UPLOAD_ENABLE  0
#define BACKEND_UPLOAD_URL     "http://192.168.1.100:5000/api/vision/upload"
#define BACKEND_UPLOAD_FIELD   "file"
#define BACKEND_UPLOAD_TIMEOUT_MS  15000u

#define SERIAL_BAUD  115200

#endif
