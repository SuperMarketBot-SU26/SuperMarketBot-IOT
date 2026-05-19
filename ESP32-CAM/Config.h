/* =====================================================================
 *  Config.h — ESP32-CAM vision node (STA only, no SoftAP in production)
 * =====================================================================*/
#ifndef ESP32_CAM_CONFIG_H
#define ESP32_CAM_CONFIG_H

/* -------------------- WiFi STA ------------------------------------- */
#ifndef WIFI_SSID
#define WIFI_SSID "SmartMarketBot"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "12345678"
#endif

/** Kênh cố định (0 = auto). Giữ 0 nếu dùng router; hotspot tablet thường auto OK. */
#define WIFI_FIXED_CHANNEL 0

/** Thử lại kết nối WiFi (ms) */
#define WIFI_RECONNECT_INTERVAL_MS  8000u
#define WIFI_CONNECT_TIMEOUT_MS     20000u

/* -------------------- HTTP server ---------------------------------- */
#define HTTP_SERVER_PORT  80

/** Giới hạn tốc độ MJPEG (~ms giữa các frame). 80 ≈ 12 FPS @ VGA. */
#define STREAM_MIN_FRAME_MS  80u

/* -------------------- Camera ----------------------------------------- */
/** 0 = VGA 640×480 (khuyến nghị), 1 = SVGA 800×600 (cần PSRAM ổn định) */
#define CAM_USE_SVGA           0
#define CAM_JPEG_QUALITY       12   /* 10–15: cân bằng kích thước / chất lượng */
#define CAM_FB_COUNT           2
#define CAM_XCLK_FREQ_HZ       20000000

/** Gain ceiling: 8 hoặc 16 (xem CamDriver.h — GAINCEILING_8X / _16X) */
#define CAM_GAIN_CEILING_16X   0    /* 1 = 16x, 0 = 8x */

/* -------------------- Backend upload (ASP.NET) ----------------------- */
#define BACKEND_UPLOAD_ENABLE  0
/** Ví dụ: http://192.168.1.100:5000/api/vision/upload */
#define BACKEND_UPLOAD_URL     "http://192.168.1.100:5000/api/vision/upload"
/** Tên field multipart (khớp IFormFile trên API) */
#define BACKEND_UPLOAD_FIELD   "file"
#define BACKEND_UPLOAD_TIMEOUT_MS  15000u

/* -------------------- Debug ------------------------------------------ */
#define SERIAL_BAUD  115200

#endif /* ESP32_CAM_CONFIG_H */
