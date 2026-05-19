# Demo — WiFi + Vision tablet (khong bat ESP32-CAM)

## WiFi (dong bo voi ESP32-S3)

| ESP32-S3 `Config.h` | ESP32-CAM `Config.h` |
|---------------------|----------------------|
| `AP_SSID` SmartMarketBot | `WIFI_SSID` SmartMarketBot |
| `AP_PASS` 12345678 | `WIFI_PASS` 12345678 |
| `AP_WIFI_CHANNEL` 6 | `WIFI_FIXED_CHANNEL` 6 |

## Demo khuyen nghi (chi tablet + S3)

1. Bat **ESP32-S3** → phat WiFi `SmartMarketBot`.
2. Tablet vao WiFi do.
3. Mo **http://192.168.4.1** → dieu khien robot.
4. Mo **https://192.168.4.1/vision** → camera tablet (chap nhan chung chi tu ky lan dau).
5. **Khong can** bat ESP32-CAM.

## Neu van bat ESP32-CAM

- CAM tu STA vao cung hotspot S3.
- Xem stream rieng: `http://<IP_CAM>/view` (cham hon tablet).

## Sau nay (BE / FE)

- Tablet `/vision` → POST anh len ASP.NET (thay cho CAM `/upload`).
