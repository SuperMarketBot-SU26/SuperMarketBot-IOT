# SmartMarketBot-IOT

Robot tự hành Mini 4WD — ESP32-S3-DevKitC N16R8  
Đồ án tốt nghiệp | FPT University CN9

**Phạm vi thư mục này (IoT & edge):** điều khiển, LiDAR/siêu âm, odometry, HMI nội bộ. **Không dùng LED ngoài** — LED RGB zin (WS2812, GPIO 38) có thể bật **chỉ khi** Echo siêu âm không trùng chân 38 (mặc định firmware: Echo phải = 38 → `SMB_ONBOARD_RGB = 0`). Phần còn lại của robot: 2× TF-Luna, 4× HC-SR04, 4× encoder, 2× TB6612 như sơ đồ. Toàn bộ hệ thống Capstone còn có Back-end, Front-end, app Android, AI. **Bản đồ SLAM** là bước tích hợp sau (MQTT/ROS2).

---

## Sơ đồ chân (ESP32-S3-DevKitC-1 + linh kiện)

Bảng dưới khớp `Config.h` — **đổi dây** nếu bạn trước đây nối AIN1/AIN2 bánh phải lên pad **WS2812** (sai: dùng **45, 46**). **GPIO 39** hiện là **Echo siêu âm trước**, không dùng cho hướng motor.

| Chức năng | GPIO | Ghi chú |
|-----|-----|-----|
| **TB6612 (trái)** | 4,5,6,7,8,9 | FL/RL, PWM+DIR |
| **TB6612 (phải)** | 21,**45,46**,40,41,42 | AIN1/AIN2 bánh trước phải dùng **45, 46** (không 38,39) |
| **STBY** 2 mạch (chung) | 47 | Cao = chạy |
| **LiDAR trước (UART1)** | TX 17, RX 18 | Nối 5V+GND đúng TF-Luna |
| **LiDAR sau (UART2)** | TX 1, RX 2 | Tránh 19, 20 (USB) |
| **HC-SR04** | Trig **14**, Echo **39**, **43**, **44**, **38** (F,B,L,R) | Cùng hàng J3 với 40–42 motor; Echo 5V → chia áp 3,3V |
| **Encoder (DO x4)** | 15, 16, 3, 48 | Ngắt; **48 = encoder sau phải** |
| **LED zin trên bo** | 38 (WS2812) | `SMB_ONBOARD_RGB=1` chỉ khi **không** dùng GPIO 38 làm Echo (mặc định Echo phải = 38 → RGB tắt trong mã) |

Tránh: **19, 20** (USB), **33–37** (PSRAM gắn module).

### Đã gọn dây chưa? (gợi ý bố cục cáp)

- **Hợp lý sẵn:** 6 chân TB6612 trái liền mạch **4–9**; **4 siêu âm Echo 39, 43, 44, 38** + Trig **14** (Trig có thể gọn cạnh J1 tùy bạn); **LiDAR trước 17+18**; **LiDAR sau 1+2**. Siêu âm và TB6612 phải **cùng hàng 39→46** trên DevKitC-1 → ít băng chéo.
- **Bắt buộc tách cáp:** bánh phải + STBY tập trung cạnh bên kia (**21, 40,41,42, 45,46, 47**): đúng với tài liệu S3, không còn dải 6 số thẳng hàng như bên trái — bạn dùng **một dây 7 ruột** (6 logic + 1 GND) về TB6612 #2, nhãn từng màu theo bảng dưới.
- **Encoder** (15, 16) thường gần cụm J1 với Trig **14**; riêng **3** và **48** nằm vị trí khác trên header — bình thường (encoder gắn 4 bánh, dây vẫn về 4 hướng). **Ghi số GPIO** bằng băng cách nhau đỡ nhầm.
- **Lưu ý strapping (boot):** **GPIO 0, 3, 45, 46** trên S3 bị tài liệu nêu; code dùng **3** (enc), **45, 46** (hướng bánh trước phải). Nên: module encoder DO nối 3,3V hợp lý; dây motor đến TB6612 để driver giữ mức ổn khi cấp nguồn; tránh kéo thử động cơ cắm **trước** lúc boot ổn (giảm rủi ro pin tụt sai trạng thái).

### Bảng theo 6 cáp gợi ý (cùng quấn, gắn nhãn A–F)

| Cáp | Nối tới (GPIO) | Nội dung |
|-----|----------------|----------|
| **A** — TB6612 bên trái | **4,5,6,7,8,9** + GND | 1 cặp 6 tín hiệu + mass |
| **B** — Siêu âm (Trig + 4 Echo) | **14** (Trig), **39, 43, 44, 38** (Echo F,B,L,R) + GND | Dải J3: Echo cạnh 40–42 motor; GPIO 38 trùng pad WS2812 — firmware tắt NeoPixel |
| **C** — LiDAR trước (UART) | **17, 18** + 5V + GND | TX/ RX chéo theo cấu hình cảm biến + nguồn |
| **D** — LiDAR sau (UART) | **1, 2** + 5V + GND | Tương tự, Serial2 |
| **E** — TB6612 bên phải + STBY chung | **21, 40, 41, 42, 45, 46, 47** + GND (và 3,3/5V logic cấp theo sơ đồ driver) | Một ổ 7 tín hiệu + dùng chung 47 tới 2 mạch (theo cách bạn hàn) |
| **F** — Encoder 4 cảm biến (DO) | **3, 15, 16, 48** + GND, Vcc theo lô cảm biến | 4 tín + nguồn cảm biến, nhãn FL/RL/FR/RR theo sơ đồ robot |

**LED zin (GPIO 38):** không dây ngoài; khi dùng **Echo phải → 38**, không cấp thư viện NeoPixel điều khiển đồng thời (đã `SMB_ONBOARD_RGB = 0`).

---

## Cấu trúc mã nguồn

```
SuperMarketBot-IOT/
├── SuperMarketBot-IOT.ino   ← File chính (setup/loop + FreeRTOS tasks)
├── Config.h
├── CtrlJson.h               ← Lệnh JSON WebSocket (điều khiển từ dashboard)
├── RobotTelemetry.h         ← JSON telemetry cho WebSocket
├── Motors.h                 ← Điều khiển 2×TB6612FNG qua LEDC
├── Sensors.h                ← TF-Luna LiDAR UART + HC-SR04 (NewPing)
├── Odometry.h               ← 4× encoder (ISR + RPM/distance)
├── StatusRGB.h              ← Chỉ LED RGB zin bo (GPIO 38), không LED ngoài
└── WebUI.h                  ← SoftAP + Web + WebSocket
```

---

## Thư viện cần cài (Arduino Library Manager)

| Thư viện | Tác giả | Ghi chú |
|---|---|---|
| **ESP32 Arduino core** | Espressif | >= 3.0 — cài qua Board Manager |
| **NewPing** | Tim Eckel | Siêu âm HC-SR04 |
| **WebSockets** | Markus Sattler | Links2004/arduinoWebSockets |
| **ArduinoJson** | Benoit Blanchon | >= v7 |
| **Adafruit NeoPixel** | Adafruit | Chỉ điều khiển **LED zin** GPIO 38 |

### LED — chỉ trên bo DevKit, không thêm phần cứng ngoài

- Một mối **WS2812 (RGB)** tích hợp, **GPIO 38**; không cần hàn thêm bóng.
- **Mặc định hiện tại:** `US_ECHO_R = 38` nên **`SMB_ONBOARD_RGB = 0`** (không dùng NeoPixel; chân 38 là input Echo).
- Muốn **bật LED** lại: đổi `US_ECHO_R` sang GPIO trống (ví dụ **48**), đặt `ENC_RR` sang chân encoder khác (ví dụ **13**), cập nhật dây encoder + siêu âm, rồi `SMB_ONBOARD_RGB = 1`.
- `SMB_RGB_BRIGHTNESS` trong `Config.h` (0–255).

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
| `SAFE_STOP_CM` | 28 cm | Trong **tự lái**: vật trước gần hơn → dừng / né (LiDAR + US) |
| `SAFE_SIDE_AVOID_CM` | 14 cm | Chỉ **tự lái**: HC-SR04 trái/phải — bẻ lái khi thực sự gần (giảm giật) |
| `SAFE_SLOW_CM` | 100 cm | Khoảng bắt đầu **giảm tốc** tiến (nhỏ = ít nhạy từ xa) |
| `ENC_PPR` | 20 | Xung/vòng bánh xe |
| `WHEEL_DIAM_M` | 0.065 m | Đường kính bánh |
| `PWM_FREQ` | 20 000 Hz | Tần số PWM động cơ |
| `TFLUNA_SEND_INIT_CMD` | 1 | Gửi lệnh Benewake lúc boot (bật UART output); tắt nếu module đã cấu hình sẵn |

### TF-Luna có đèn nhưng web vẫn ~800 cm / byte UART = 0

- **Chân Mode (datasheet: pin 5, tuỳ lô hàng):** để **trống hoặc 3,3 V** = UART; **nối GND** = **I2C** → không còn stream `59 59` trên TX/RX nối ESP.
- **Dây:** **TX của Luna → RX ESP** (trước RX=`GPIO18`, sau RX=`GPIO2` theo `Config.h`), **RX Luna ← TX ESP**, GND chung, 5 V.
- Xem Serial Monitor ngay sau dòng `[LiDAR] Raw sniff`: phải có byte và chuỗi `59 59`; nếu `0 byte` → dây/baud/chế độ sai.
- Nạp bản mới: firmware tự gửi **bật output + khung 9 byte + FPS + save** (tắt bằng `TFLUNA_SEND_INIT_CMD 0` nếu cần).

---

## Lộ trình nâng cấp SLAM

- Bổ sung IMU (MPU6050) lấy dữ liệu góc quay
- Kết hợp Odometry + IMU → Dead-reckoning pose
- Thêm thư viện `micro-ROS` hoặc giao thức MQTT để xuất bản đồ lên ROS2

---

*Phát triển bởi nhóm SmartMarketBot — FPT University 2026*
