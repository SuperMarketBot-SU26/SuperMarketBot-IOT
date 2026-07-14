# Tích hợp YDLidar X3 — Kế hoạch

## Vai trò của từng cảm biến

| Sensor | Vai trò | Tần suất | Output |
|--------|---------|----------|--------|
| **YDLidar X3** | SLAM + localization + obstacle backup | 10 Hz scan 360° | Full scan + min(±45° front/back) |
| **HC-SR04 (4x)** | Reactive obstacle avoidance | 20 Hz (mỗi 50ms ping 1 cái) | 4 giá trị khoảng cách |
| **MPU6050** | IMU heading | 100 Hz | góc yaw |
| **Encoder 4x** | Odometry | Theo PID | ticks → distance |

**Lý do giữ HC-SR04 dù đã có X3:**
- HC-SR04 phản hồi ~50ms/sensor → reactive tốt cho tránh va
- X3 cần full 360° (~100ms) → chậm hơn cho obstacle tránh ở tốc độ cao
- HC-SR04 rẻ, đã có sẵn → tận dụng

## Phần cứng kết nối YDLidar X3

| ESP32-S3 Pin | X3 Pin | Note |
|--------------|--------|------|
| GPIO 15 (TX) | X3 RX (optional, nếu cần gửi lệnh) | Chia áp 3.3V |
| GPIO 16 (RX) | X3 TX | Chia áp 3.3V |
| 5V | VCC | |
| GND | GND | |

**Lưu ý quan trọng:** GPIO 16 đang được dùng cho `ENC_RL` (encoder bánh sau trái). Khi bật X3:
- Hoặc **tắt encoder RL** trong `Config.h` (`USE_ENCODER_HARDWARE = 0` hoặc remap sang GPIO khác)
- Hoặc **dùng chân khác** (ví dụ GPIO 11 nếu bỏ US_ECHO_RL).

## Cách bật

Trong `Config.h`:
```cpp
#define USE_YDLIDAR_X3          1   // bật
#define YDLIDAR_X3_TX           15
#define YDLIDAR_X3_RX           16
#define YDLIDAR_X3_BAUD         230400
```

Trong `WaypointNav.h`:
```cpp
#define WP_DISABLE_OBSTACLE_GUARD 1  // (đã bật sẵn để test)
```

## Luồng dữ liệu

```
X3 → Serial1 (230400) → x3Poll() → g_x3Scan (720 points)
                              │
                              ├── x3MinInArc(0°, 45°) → g_state.lidarFront (cm)
                              ├── x3MinInArc(180°, 45°) → g_state.lidarBack (cm)
                              └── Publish lên MQTT topic "robot/<code>/scan" → BE SLAM
```

## Tích hợp SLAM phía Backend

Backend cần subscribe topic `robot/<code>/scan` từ MQTT broker, nhận JSON:
```json
{
  "ts": 12345678,
  "seq": 42,
  "points": [
    {"a": 0.523, "d": 1250},  // a = radian, d = mm
    {"a": 0.531, "d": 1245},
    ...
  ]
```

Sau đó dùng:
- **Hector SLAM** (đơn giản, không cần odometry)
- **Cartographer** (chính xác, có odometry)
- **GMapping** (phổ biến)

Lưu map dưới dạng **PGM/YAML** hoặc **ROS map format**, lưu vào DB → gửi về FE Web Manager để hiển thị.

## Skeleton hiện tại

`YdlidarX3.h` cung cấp:
- `X3Scan g_x3Scan` — buffer scan mới nhất
- `x3Init()` — gọi trong setup(), khởi tạo Serial1 + start scan
- `x3Poll()` — gọi mỗi 50ms trong taskControl, parse bytes
- `x3MinInArc(centerDeg, halfWidthDeg)` — lấy min distance trong cung
- `x3Stop()` — gọi khi tắt

**Cần bổ sung khi có X3 thật:**
1. Verify byte order của X3 protocol (X3 dùng cartographer mode, format chi tiết cần test thực tế)
2. Publish scan qua MQTT (hiện chỉ log Serial)
3. Localization: scan-match với map tĩnh → cập nhật `g_pose` chính xác hơn
4. Visualization trên Web Manager: thêm layer "Lidar scan" lên map
