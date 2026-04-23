# SmartMarketBot-IOT

Robot tự hành Mini 4WD — ESP32-S3-DevKitC N16R8  
Đồ án tốt nghiệp | FPT University CN9

**Phạm vi thư mục này (IoT & edge):** điều khiển, LiDAR/siêu âm, odometry, HMI nội bộ. Toàn bộ hệ thống Capstone còn có Back-end, Front-end Admin, app Android, AI — tích hợp ở các repo tương ứng. **Bản đồ SLAM + điều hướng tối ưu** là bước kế tiếp: dữ liệu LiDAR/odom có thể stream lên server qua MQTT/ROS2.

---

## Cấu trúc mã nguồn

```
SuperMarketBot-IOT/
├── SuperMarketBot-IOT.ino   ← File chính (setup/loop + FreeRTOS tasks)
├── Config.h                 ← Khai báo GPIO, hằng số, struct RobotState
├── Motors.h                 ← Điều khiển 2×TB6612FNG qua LEDC
├── Sensors.h                ← TF-Luna LiDAR UART + HC-SR04 (NewPing)
├── Odometry.h               ← 4×FC-03 encoder (ISR + RPM/distance)
└── WebUI.h                  ← SoftAP + WebServer HTTP + WebSocket Dashboard
```

---

## Thư viện cần cài (Arduino Library Manager)

| Thư viện | Tác giả | Ghi chú |
|---|---|---|
| **ESP32 Arduino core** | Espressif | >= 3.0 — cài qua Board Manager |
| **NewPing** | Tim Eckel | Siêu âm HC-SR04 |
| **WebSockets** | Markus Sattler | Links2004/arduinoWebSockets |
| **ArduinoJson** | Benoit Blanchon | >= v7 |
| **Adafruit NeoPixel** | Adafruit | Đèn WS2812 trên board |

### LED RGB trên ESP32-S3-DevKitC

- Board thường có **WS2812** nối **GPIO 48** (trùng chân **encoder sau phải** trong sơ đồ của bạn).
- `SMB_ONBOARD_RGB = 1`: hiệu ứng theo chế độ (thở teal **Lái tay**, nhanh hơn **Tự hành**, nháy đỏ **E-STOP**), màu gần với HMI web.
- `SMB_ONBOARD_RGB = 0`: tắt LED, encoder 4 bánh hoạt động đủ trên GPIO 48 (nếu đã nối dây).
- Chỉnh độ sáng: `SMB_RGB_BRIGHTNESS` trong `Config.h` (0–255).

---

## Cấu hình Arduino IDE

1. **Board**: `ESP32S3 Dev Module`
2. **Flash Size**: `16MB`
3. **PSRAM**: `OPI PSRAM` (8MB Octal)
4. **Upload Speed**: `921600`
5. **USB CDC On Boot**: `Enabled` (để dùng Serial Monitor)
6. **Partition Scheme**: `16M Flash (3MB APP/9.9MB FATFS)` hoặc `Huge APP`

---

## Nạp code & Debug

```
1. Cắm cáp USB vào cổng COM trên ESP32-S3
2. Chọn đúng COM port trong Arduino IDE
3. Nhấn Upload → chờ Done uploading
4. Mở Serial Monitor, chọn baud 115200
5. Reset board → xem thông số cảm biến xuất hiện
```

Nếu vài dòng đầu Serial có ký tự lạ: do boot ROM in trước, USB CDC còn đồng bộ — đặt đúng **115200**, đóng mọi app khác đang mở COM, hoặc bỏ qua 1–2 dòng; log sau dòng `SmartMarketBot booting` sẽ ổn định.

---

## Dashboard Web

```
1. Điện thoại/PC bắt Wi-Fi: SmartMarketBot (pass: 12345678)
2. Mở trình duyệt → http://192.168.4.1
3. Giao diện hiện: Joystick, LiDAR, cảm biến, RPM
```

**Nếu trang quay mãi / trắng lâu:**

- Bản cũ tải **Google Fonts từ Internet** — trên mạng chỉ có Wi-Fi robot thì **không có mạng ngoài**, trình duyệt dễ bị kẹt chờ font. Bản mới đã bỏ CDN, dùng font hệ thống (xem `WebUI.h`).
- Tắt **dữ liệu di động (4G/5G)** trên điện thoại khi thử, tránh mạ thông minh ưu tiên 3G/4G và lỗi nội bộ mạng.
- Thử trình duyệt khác (Chrome) hoặc mở **http://192.168.4.1** dạng thường, không bắt buộc HTTPS.
- Nạp lại firmware sau khi cập nhật code (Upload).

---

## Tinh chỉnh tham số

Tất cả hằng số nằm trong `Config.h`:

| Hằng số | Mặc định | Ý nghĩa |
|---|---|---|
| `LIDAR_MAX_CM` | 800 cm (8 m) | Trần hiển thị / clamp TF-Luna (theo datasheet) |
| `US_PING_MAX_CM` | 200 cm | Cửa sổ đo NewPing (~2 m, cân bằng tốc độ quét) |
| `US_DISPLAY_MAX_CM` | 160 cm (1,6 m) | Thanh HMI + vùng “xa ổn định” thực tế HC-SR04 trong siêu thị |
| `SAFE_STOP_CM` | 20 cm | Dừng / né gấp (LiDAR + siêu âm) |
| `SAFE_SLOW_CM` | 200 cm | Bắt đầu giảm tốc từ từ theo tầm (phù hợp hành lang rộng) |
| `ENC_PPR` | 20 | Xung/vòng bánh xe |
| `WHEEL_DIAM_M` | 0.065 m | Đường kính bánh |
| `PWM_FREQ` | 20 000 Hz | Tần số PWM động cơ |

---

## Lộ trình nâng cấp SLAM

- Bổ sung IMU (MPU6050) lấy dữ liệu góc quay
- Kết hợp Odometry + IMU → Dead-reckoning pose
- Thêm thư viện `micro-ROS` hoặc giao thức MQTT để xuất bản đồ lên ROS2

---

*Phát triển bởi nhóm SmartMarketBot — FPT University 2026*
