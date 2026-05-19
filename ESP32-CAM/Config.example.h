/* Sao chép thành secrets hoặc sửa trực tiếp Config.h trước khi nạp.
 * Không commit mật khẩu WiFi thật lên git public. */

#pragma once

#define WIFI_SSID "YourHotspotOrRouterSSID"
#define WIFI_PASS "YourPassword"

#define BACKEND_UPLOAD_ENABLE  1
#define BACKEND_UPLOAD_URL     "http://192.168.1.100:5000/api/vision/upload"
