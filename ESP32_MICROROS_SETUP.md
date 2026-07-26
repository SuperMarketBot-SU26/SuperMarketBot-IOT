# Setup micro-ROS cho ESP32-S3 (hướng dẫn từng bước)

## 1. Cài Arduino IDE + ESP32-S3 Board

```
Arduino IDE → File → Preferences → Additional Board Manager URLs:
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

Tools → Board → Board Manager → Tìm "esp32" → Install (Espressif ESP32 by Espressif Systems)
Tools → Board → ESP32S3 Dev Module
```

## 2. Cài thư viện micro_ros_arduino

```
Sketch → Include Library → Manage Libraries
Search: "micro_ros_arduino"
Install (by eProsima)
```

## 3. Patch file SuperMarketBot-IOT.ino

Mở file `e:\FPT UNIVERSITY\CN9\Sep401\SuperMarketBot-IOT\ESP32-S3\SuperMarketBot-IOT\SuperMarketBot-IOT.ino`

Thêm includes:
```cpp
#include "MicroRos.h"   // <-- THÊM DÒNG NÀY
```

Trong `setup()` (SAU khi Serial + WiFi đã init, SAU khi X3 init):
```cpp
void setup() {
    Serial.begin(115200);
    delay(2000);

    // ... existing setup code (WiFi, Motors, Sensors, X3) ...

    // ★ THÊM micro-ROS:
    // Set IP Ubuntu (đổi 192.168.1.100 thành IP Ubuntu của bạn)
    microRos::setAgent("192.168.1.100", 8888);

    // Đợi X3 có scan ready (scan đầu tiên)
    Serial.println("[micro-ROS] Đợi LiDAR scan đầu tiên...");
    uint32_t t0 = millis();
    while (g_x3Scan.count < 50 && millis() - t0 < 5000) {
        // x3Loop() đã được gọi trong loop thông thường
        delay(50);
    }

    if (!microRos::init()) {
        Serial.println("[micro-ROS] ❌ Init failed - check agent IP");
    }
}
```

Trong `loop()` (SAU cùng, sau taskControl/x3Loop):
```cpp
void loop() {
    // ... existing tasks (x3Loop, odometry, motors, telemetry) ...

    // ★ THÊM micro-ROS:
    microRos::tick();
}
```

## 4. Build & Flash

Click Verify (Ctrl+R) → nếu lỗi library → chỉnh thêm.

Lỗi thường gặp:
- `WiFi.h` not found → cài ESP32 board (bước 1)
- `micro_ros_arduino.h` not found → install thư viện (bước 2)
- Out of memory → tăng Partition Scheme trong Tools → Partition Scheme → "Huge APP (3MB No OTA)"

Click Upload → đợi flash xong.

## 5. Trên Ubuntu 26.04: chạy micro-ros-agent

```bash
# Mở terminal Ubuntu 26.04
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash   # workspace dùng ROS2 lyrical

# Chạy agent (UDP port 8888)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

> Lưu ý: workspace của dự án (`ros2_ws`) chạy ROS2 **lyrical** trên Ubuntu 26.04,
> KHÔNG phải Humble. File `setup_ros2_slam.sh` cũ trong repo này cài Humble —
> giờ đã có `setup_ros2_slam_lyrical.sh` cho Ubuntu 26.04.

Bạn sẽ thấy:
```
[1700000000.000000] info     | UDPv4 Agent listening on port 8888
```

## 6. Test thử

ESP32 flash xong → 5-10s sau, agent sẽ log:
```
[1700000010.000000] info     | New client connected
[1700000011.000000] info     | client REQUEST: Participant
[1700000012.000000] info     | client REQUEST: Endpoint
```

Check topics:
```bash
ros2 topic list
# → /scan, /odom, /imu, /cmd_vel, /tf, /parameter_events, /rosout

ros2 topic echo /scan --once
# → sẽ in 1 message LaserScan đầy đủ
```

## 7. Xem trong RViz2

```bash
ros2 run rviz2 rviz2
```

Trong RViz2:
- Fixed Frame: `odom`
- Add → LaserScan → topic `/scan`
- Add → TF → show frames

Bạn sẽ thấy scan points quay theo thời gian thực.

## 8. Chạy SLAM

```bash
# Terminal 1: micro-ros-agent (đã chạy ở bước 5)
# Terminal 2: SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: RViz2 (đã chạy ở bước 7)
# Add By Topic → /map → Map
```

Trong RViz2:
- Add → Map → topic `/map`
- Fixed Frame: `map`

Bạn sẽ thấy map tự build khi robot di chuyển.

## 9. Lái robot bằng ROS2 cmd_vel

```bash
# Terminal 4: Teleop keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

GIỮ TRONG TERMININAL, dùng:
- `i` = đi tiến
- `,` = đi lùi
- `j` = xoay trái
- `l` = xoay phải
- `k` = dừng
- `q`/Ctrl+C = quit

Lúc này robot sẽ đi theo lệnh ROS2 → ESP32 nhận /cmd_vel → motor chạy.

## 10. Xem map live trong WebManager

```bash
# Terminal 5: rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

WebManager sẽ subscribe `/map` qua WebSocket → vẽ map live (tôi sẽ patch code).

---

## Common Issues

### ESP32 không connect agent
- Check IP Ubuntu: `ip addr show` → inet 192.168.x.x
- Ping từ ESP32: thêm `ping Ubuntu_IP` trong setup
- Firewall: `sudo ufw allow 8888/udp`

### Micro-ROS oom (out of memory)
- Tools → Partition Scheme → "Huge APP (3MB No OTA)"

### /scan rỗng
- Check `g_x3Scan.count` > 0 trước khi microRos::init()
- Đợi 5s cho X3 warm-up

### ROS2 không tìm thấy laser_frame
- TF cần: odom → base_link → laser_frame
- Tôi đã publish /tf nhưng cần fix transform offset
