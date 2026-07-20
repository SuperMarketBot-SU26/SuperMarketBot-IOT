# SmartMarketBot - Roadmap Triển Khai

## Tổng Quan

Dự án được chia thành **8 Phase** với mục tiêu rõ ràng và deliverable cụ thể.

---

## Phase 1: ESP32-S3 Firmware Tối Giản (Motor Controller)

**Mục tiêu**: ESP32 chỉ làm Motor Controller, không làm gì khác

**Deliverables**:
- [x] `MotorOnly.ino` - Firmware mới
- [ ] ESP32 nhận lệnh qua WiFi UDP
- [ ] ESP32 điều khiển 4 động cơ qua TB6612FNG
- [ ] ESP32 chạy PID cho từng động cơ

**Các bước thực hiện**:

```bash
# 1.1. Copy file mới vào thư mục ESP32
cp ESP32-S3/MotorOnly/MotorOnly.ino ESP32-S3/SuperMarketBot-IOT/

# 1.2. Upload firmware
# Arduino IDE: Tools > Port > ESP32-S3 > Upload

# 1.3. Test với app Android
# - Kết nối WiFi: SmartMarketBot
# - Mở app, nhập IP 192.168.4.1
# - Gửi lệnh UDP đến port 4210
```

**Cấu hình mạng**:
- ESP32 AP SSID: `SmartMarketBot`
- ESP32 AP Password: `12345678`
- UDP Port: `4210`

**Packet format (Android → ESP32)**:
```
Byte:  0     1     2-3    4-5    6-7    8
Type:  HEAD  CMD   VX_H   VX_L   VY_H   VY_L  W_H  W_L  CHK
- HEAD = 0xAA
- CMD = 0x01 (velocity)
- VX, VY = int16 mm/s (signed)
- W = int16 mrad/s (signed)
- CHK = XOR checksum
```

**Độ khả thi**: ⭐⭐⭐⭐⭐ (5/5)
- Firmware đơn giản, ít code
- Không cần thay đổi phần cứng
- Đã test với code cũ

---

## Phase 2: Android YDLIDAR Driver

**Mục tiêu**: Android đọc trực tiếp YDLIDAR X3 qua USB OTG

**Deliverables**:
- [x] `YDLIDARX3Manager.kt` - Driver class
- [ ] Parse YDLIDAR X3 protocol
- [ ] Display raw scan data
- [ ] Calculate FPS, latency

**Các bước thực hiện**:

```kotlin
// Sử dụng YDLIDARX3Manager
val lidarManager = YDLIDARX3Manager(
    onScanReady = { points ->
        // points: List<LidarScanPoint>
        // angleRad, distanceMm, quality
        updateScanDisplay(points)
    },
    onError = { error ->
        Log.e("LiDAR", error)
    }
)

// Kết nối qua USB
lidarManager.connect(usbPort)
lidarManager.startScan()
```

**YDLIDAR X3 Protocol**:
```
Header: 0xAA 0x55
Length: 1 byte
Sample count: 1 byte
Start angle: 2 bytes (Q15.7)
Points: [dist_low, dist_high, quality, angle_offset] x N
End angle: 2 bytes
Checksum: 1 byte
```

**Thông số YDLIDAR X3**:
| Thông số | Giá trị |
|----------|---------|
| Scan rate | 5-12 Hz |
| Sample rate | 3000 samples/s |
| Range | 0.1-8m |
| Baudrate | 115200 |
| Resolution | ~0.5° |

**Độ khả thi**: ⭐⭐⭐⭐ (4/5)
- usb-serial-for-android hỗ trợ CDC/ACM
- YDLIDAR protocol có documentation
- Cần test thực tế với thiết bị

---

## Phase 3: SLAM Engine - Occupancy Grid Mapping

**Mục tiêu**: Tạo Occupancy Grid Map từ YDLIDAR data

**Deliverables**:
- [x] `SLAMEngine.h` - SLAM core
- [ ] Occupancy Grid Mapper
- [ ] Ray casting
- [ ] Binary Bayes Filter
- [ ] Map visualization

**Cấu trúc Occupancy Grid**:

```cpp
// Grid size: 400x400 cells = 20m x 20m (5cm/cell)
class OccupancyGrid {
    std::vector<float> cells;  // Log-odds values
    int width = 400;
    int height = 400;
    float resolution = 0.05f;  // 5cm per cell
};
```

**Thuật toán**:
```
1. Với mỗi scan point:
   a. Tính world coordinates từ polar (range, angle)
   b. Ray casting từ robot đến point
   c. Update free cells (log-odds -= 0.7)
   d. Update endpoint cell (log-odds += 1.2)

2. Log-odds to probability:
   P = 1 / (1 + exp(-logodds))
```

**Độ khả thi**: ⭐⭐⭐⭐⭐ (5/5)
- Thuật toán đơn giản, well-documented
- Không cần external libraries
- Có thể chạy real-time trên ARM64

---

## Phase 4: Localization - Scan Matching

**Mục tiêu**: Robot biết chính xác vị trí trên bản đồ

**Deliverables**:
- [ ] ICP scan matching
- [ ] Pose estimation
- [ ] AMCL-like particle filter (optional)

**Thuật toán ICP (Iterative Closest Point)**:
```cpp
// 1. Initial guess (odometry hoặc last pose)
Pose2D initial = odometry_prediction;

// 2. Iterate:
for (int i = 0; i < 10; i++) {
    // a. Transform current scan to world frame
    // b. Find closest points in map
    // c. Calculate transformation
    // d. Update pose estimate
}

// 3. Return refined pose
return refined_pose;
```

**Pure Pursuit cho đi thẳng**:
```cpp
// Để robot đi THẲNG:
// 1. Set waypoint phía trước
// 2. Pure Pursuit tính curvature
// 3. Nếu có drift, scan matching hiệu chỉnh
// 4. LiDAR feedback = độ lệch < 1cm
```

**Độ khả thi**: ⭐⭐⭐⭐ (4/5)
- ICP algorithm well-known
- Performance tốt trên mobile
- Cần tuning parameters

---

## Phase 5: Navigation - Path Planning + Tracking

**Mục tiêu**: Robot đi theo waypoint với quỹ đạo mượt

**Deliverables**:
- [ ] A* Path Planner
- [ ] Pure Pursuit Controller
- [ ] Waypoint management
- [ ] Trajectory smoothing

**A* Algorithm**:
```cpp
// Grid-based A* trên Occupancy Grid
class AStarPlanner {
    // Priority queue (min-heap)
    // Heuristic: Euclidean distance
    // Movement: 8-directional (diagonal)
    // Output: List of (x, y) cells
};
```

**Pure Pursuit Controller**:
```cpp
class PurePursuitController {
    // 1. Tìm lookahead point trên path
    // 2. Tính curvature: κ = 2*ly / L²
    // 3. Output: vx, vy, omega

    float lookaheadDist = 0.5f;  // 50cm
    float targetSpeed = 0.3f;   // 30cm/s
};
```

**Độ khả thi**: ⭐⭐⭐⭐⭐ (5/5)
- A* và Pure Pursuit đơn giản
- Performance tốt trên mobile
- Đã verify trên nhiều robot

---

## Phase 6: Obstacle Avoidance

**Mục tiêu**: Robot tránh vật cản động

**Deliverables**:
- [ ] Dynamic Window Approach (DWA)
- [ ] Real-time obstacle detection
- [ ] Replanning khi có vật cản

**DWA Algorithm**:
```cpp
// 1. Sample velocity space
// 2. For each (v, ω):
//    - Simulate trajectory
//    - Check collision with obstacles
//    - Score = heading + dist + velocity
// 3. Return best velocity

class DWAController {
    std::vector<std::pair<float, float>> sampleVelocities();
    float score(const Trajectory& traj);
    VelocityCommand getBestVelocity();
};
```

**Reactive Navigation**:
```
Waypoint Follow ──┬── Path clear ──► Continue
                  │
                  └── Obstacle ──► DWA → Replan ──► Resume
```

**Độ khả thi**: ⭐⭐⭐⭐ (4/5)
- DWA well-known algorithm
- Performance phụ thuộc scan rate
- Cần buffer để predict

---

## Phase 7: Android UI - SLAM Visualization

**Mục tiêu**: Giao diện Android hiển thị map và điều khiển

**Deliverables**:
- [ ] Live LiDAR Scan display
- [ ] Occupancy Grid Map display
- [ ] Robot Pose visualization
- [ ] Waypoint editor
- [ ] Control panel

**UI Components**:
```
┌─────────────────────────────────────────────┐
│           SMARTMARKETBOT CONTROL            │
├─────────────────────────────────────────────┤
│                                             │
│   ┌───────────────────────────────────┐    │
│   │                                   │    │
│   │         LIVE MAP VIEW             │    │
│   │                                   │    │
│   │    • Scan points (gray dots)      │    │
│   │    • Occupancy grid (black/white) │    │
│   │    • Robot pose (blue arrow)      │    │
│   │    • Waypoints (green circles)    │    │
│   │                                   │    │
│   └───────────────────────────────────┘    │
│                                             │
│   [ LiDAR: ●10Hz ]  [ FPS: 10 ]           │
│   [ Map: 5cm/cell ] [ Points: 360 ]       │
│                                             │
│   ┌─────────────────────────────────────┐  │
│   │ MODE: [SLAM] [NAV] [MANUAL]        │  │
│   └─────────────────────────────────────┘  │
│                                             │
│   ┌─────────────────────────────────────┐  │
│   │ WAYPOINTS                           │  │
│   │ • Waypoint 1: (1.2, 0.5)           │  │
│   │ • Waypoint 2: (2.5, 1.0)           │  │
│   │ [+ Add] [✕ Clear] [⟳ Save] [📂 Load]│  │
│   └─────────────────────────────────────┘  │
│                                             │
│   ┌─────────────────────────────────────┐  │
│   │ STATUS                               │  │
│   │ Pose: x=1.23, y=0.45, θ=45°        │  │
│   │ Battery: 11.8V ●●●●●○○○             │  │
│   │ WiFi: ● Connected                   │  │
│   └─────────────────────────────────────┘  │
│                                             │
│   [ START ]  [ PAUSE ]  [ ■ STOP ]        │
└─────────────────────────────────────────────┘
```

**Độ khả thi**: ⭐⭐⭐⭐⭐ (5/5)
- Jetpack Compose mạnh mẽ
- Canvas API cho custom drawing
- Hiệu năng tốt trên Xiaomi Pad 6

---

## Phase 8: Integration + Testing

**Mục tiêu**: Hệ thống hoàn chỉnh, test trong môi trường thực

**Deliverables**:
- [ ] Full system integration
- [ ] Performance optimization
- [ ] Field testing (3x3m demo area)
- [ ] Documentation

**Test Cases**:
```
1. Mapping Test:
   - Robot đi quanh phòng 3x3m
   - Kiểm tra map accuracy
   - Save/load map

2. Localization Test:
   - Place robot ở vị trí known
   - Kiểm tra pose error < 10cm

3. Navigation Test:
   - Set 4 waypoints
   - Robot đi theo waypoints
   - Độ lệch quỹ đạo < 5cm

4. Obstacle Test:
   - Robot đang đi, đặt vật cản
   - Robot tránh và tiếp tục

5. Full Demo:
   - Autonomous mapping
   - Save map
   - Navigate to waypoints
   - Return to dock
```

---

## Timeline Dự Kiến

```
Week 1-2: Phase 1-2 (ESP32 + YDLIDAR Driver)
Week 3-4: Phase 3 (SLAM Engine)
Week 5-6: Phase 4-5 (Localization + Navigation)
Week 7-8: Phase 6 (Obstacle Avoidance)
Week 9-10: Phase 7 (Android UI)
Week 11-12: Phase 8 (Integration + Testing)
```

---

## Các File Cần Tạo/Update

### ESP32 Firmware
```
ESP32-S3/
├── MotorOnly/
│   └── MotorOnly.ino          ✅ Đã tạo
└── SuperMarketBot-IOT/
    └── SuperMarketBot-IOT.ino  (Backup)
```

### Android Native (C++)
```
Android-Native Hub/app/src/main/cpp/
├── CMakeLists.txt
├── native-lib.cpp
├── include/
│   ├── SLAMEngine.h           ✅ Đã tạo
│   ├── RobotMotorCommand.h     ✅ Đã tạo
│   └── (thêm files khác...)
└── slam/
    ├── slam_engine.cpp
    ├── occupancy_grid.cpp
    └── scan_matching.cpp
```

### Android Native (Kotlin)
```
Android-Native Hub/app/src/main/java/com/smartmarketbot/hub/
├── MainActivity.kt            (Update)
├── lidar/
│   ├── YDLIDARX3Manager.kt   ✅ Đã tạo
│   └── LiDARViewModel.kt      (Tạo mới)
├── slam/
│   └── SLAMViewModel.kt       (Tạo mới)
├── navigation/
│   └── NavViewModel.kt        (Tạo mới)
├── comm/
│   ├── UDPSocket.kt          (Update)
│   └── ESP32Manager.kt         (Tạo mới)
└── ui/
    ├── MapScreen.kt           (Tạo mới)
    └── ControlScreen.kt        (Update)
```

---

## Dependencies

### Native (C++ - NDK)
| Library | Version | Source |
|---------|--------|--------|
| Eigen | 3.4+ | Header-only |
| spdlog | 1.12+ | Prebuilt for NDK |

### Android (Kotlin)
| Library | Version | Purpose |
|---------|---------|---------|
| Jetpack Compose | 1.5+ | UI |
| Hilt | 2.48+ | DI |
| usb-serial-for-android | 3.5+ | USB CDC/ACM |
| OkHttp | 4.12+ | WiFi UDP |

---

## Performance Targets

| Metric | Target | Actual |
|--------|--------|--------|
| LiDAR FPS | 10 Hz | 10 Hz |
| SLAM FPS | 10 Hz | 10 Hz |
| Motor Command Rate | 20 Hz | 20 Hz |
| End-to-End Latency | < 100ms | TBD |
| Map Resolution | 5 cm/cell | 5 cm/cell |
| Localization Accuracy | < 10 cm | TBD |
| Path Tracking Error | < 5 cm | TBD |
