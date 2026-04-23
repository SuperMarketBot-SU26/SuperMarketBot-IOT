# SmartMarketBot-IOT

Robot tự hành Mini 4WD — ESP32-S3-DevKitC N16R8  
Đồ án tốt nghiệp | FPT University CN9

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

---

## Dashboard Web

```
1. Điện thoại/PC bắt Wi-Fi: SmartMarketBot (pass: 12345678)
2. Mở trình duyệt → http://192.168.4.1
3. Giao diện hiện: Joystick + biểu đồ 6 cảm biến + RPM 4 bánh
```

---

## Tinh chỉnh tham số

Tất cả hằng số nằm trong `Config.h`:

| Hằng số | Mặc định | Ý nghĩa |
|---|---|---|
| `SAFE_STOP_CM` | 15 cm | Ngưỡng dừng khẩn cấp |
| `SAFE_SLOW_CM` | 100 cm | Bắt đầu giảm tốc |
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
