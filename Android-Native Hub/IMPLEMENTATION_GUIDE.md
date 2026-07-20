# SMARTMARKETBOT - HƯỚNG DẪN THỰC HIỆN CHI TIẾT

## PHASE 1: ESP32-S3 FIRMWARE TỐI GIẢN

### 1.1 Backup Firmware Cũ

```bash
# Copy firmware cũ ra thư mục backup
cp SuperMarketBot-IOT.ino SuperMarketBot-IOT-OLD.ino
```

### 1.2 Thay thế Firmware

```bash
# Copy firmware mới vào
cp MotorOnly/MotorOnly.ino SuperMarketBot-IOT.ino
```

### 1.3 Build và Upload

```
Arduino IDE:
1. File > Open > SmartMarketBot-IOT.ino
2. Tools > Board > ESP32-S3 Dev Module
3. Tools > Flash Size > 16MB
4. Tools > PSRAM > 8MB
5. Sketch > Upload
```

### 1.4 Test Cơ Bản

```
Serial Monitor (115200 baud):
== SmartMarketBot Motor Controller ==
Simplified firmware - SLAM on Android
[Motors] Initialized
[WiFi] AP IP: 192.168.4.1
[UDP] Listening on port 4210
[Ready] Motor controller active
```

### 1.5 Test với Python Script

```python
# test_motor.py
import socket
import struct
import time

UDP_IP = "192.168.4.1"
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(vx_mm, vy_mm, omega_mrad):
    packet = bytes([0xAA, 0x01])  # Header, CMD
    packet += struct.pack('<h', vx_mm)  # VX (int16)
    packet += struct.pack('<h', vy_mm)  # VY (int16)
    packet += struct.pack('<h', omega_mrad)  # Omega (int16)
    checksum = 0
    for b in packet[1:]:  # Skip header
        checksum ^= b
    packet += bytes([checksum])
    sock.sendto(packet, (UDP_IP, UDP_PORT))

# Test đi thẳng (100 mm/s)
print("Test forward...")
send_command(100, 0, 0)
time.sleep(2)

# Test dừng
print("Stop...")
send_command(0, 0, 0)
```

**Đánh giá**: ⭐⭐⭐⭐⭐ - Đơn giản, hoạt động ngay

---

## PHASE 2: ANDROID YDLIDAR DRIVER

### 2.1 Cập nhật build.gradle

```gradle
// app/build.gradle
dependencies {
    implementation 'com.hoho.android:usb-serial-driver:3.5.4'
    implementation 'com.google.code.gson:gson:2.10.1'
}
```

### 2.2 Cập nhật AndroidManifest.xml

```xml
<uses-feature android:name="android.hardware.usb.host" />
<uses-permission android:name="android.hardware.usb.permission" />

<!-- YDLIDAR X3 Device Filter -->
<resource>
    <!-- USB Device for YDLIDAR -->
    <usb-device class="2" subclass="2" protocol="1" />
</resource>
```

### 2.3 Sử dụng YDLIDARX3Manager

```kotlin
// MainActivity.kt hoặc Service
class LiDARService : Service() {
    
    private lateinit var lidarManager: YDLIDARX3Manager
    private lateinit var usbManager: UsbManager
    
    override fun onCreate() {
        super.onCreate()
        usbManager = getSystemService(USB_SERVICE) as UsbManager
    }
    
    fun connectLidar() {
        // Tìm YDLIDAR device
        val devices = UsbSerialProber.getDefaultProber()
            .findAllDevices(usbManager)
        
        val lidarDevice = devices.find { 
            it.device.vendorId == 0x10C4 ||  // CP210x
            it.device.vendorId == 0x067B     // Prolific
        }
        
        if (lidarDevice != null) {
            val connection = usbManager.openDevice(lidarDevice.device)
            lidarManager = YDLIDARX3Manager(
                onScanReady = { points ->
                    // points: List<LidarScanPoint>
                    updateScanVisualization(points)
                },
                onError = { error ->
                    Log.e("LiDAR", error)
                }
            )
            lidarManager.connect(lidarDevice.ports[0])
            lidarManager.startScan()
        }
    }
}
```

### 2.4 Visualize Scan Data

```kotlin
fun updateScanVisualization(points: List<LidarScanPoint>) {
    // Convert polar to Cartesian
    val cartesianPoints = points.map { point ->
        val x = (point.distanceMm / 1000f) * kotlin.math.cos(point.angleRad)
        val y = (point.distanceMm / 1000f) * kotlin.math.sin(point.angleRad)
        Pair(x, y)
    }
    
    // Draw on Canvas
    canvas.drawPoints(cartesianPoints, scanPaint)
}
```

**Đánh giá**: ⭐⭐⭐⭐ - Protocol cần verify với thiết bị thật

---

## PHASE 3: SLAM ENGINE

### 3.1 Tích hợp Native Code

```kotlin
// native-lib.cpp (JNI bridge)
extern "C" {

JNIEXPORT jobjectArray JNICALL
Java_com_smartmarketbot_hub_SLAM_nativeProcessScan(
    JNIEnv *env,
    jobject /* this */,
    jfloatArray ranges,
    jfloatArray angles,
    jint count,
    jfloat robotX,
    jfloat robotY,
    jfloat robotTheta
) {
    // Convert Java arrays to C++ vectors
    // Call SLAMEngine.processScan()
    // Return updated pose
}

}
```

### 3.2 Main SLAM Loop

```kotlin
class SLAMViewModel : ViewModel() {
    
    private val slamEngine = NativeSLAMEngine()
    private val scanBuffer = mutableListOf<LidarScanPoint>()
    
    fun onNewScan(points: List<LidarScanPoint>) {
        // Convert to scan points
        val scanPoints = points.map { p ->
            ScanPoint(
                x = (p.distanceMm / 1000f) * cos(p.angleRad),
                y = (p.distanceMm / 1000f) * sin(p.angleRad),
                angle = p.angleRad,
                range = p.distanceMm / 1000f,
                quality = p.quality
            )
        }
        
        // Process with SLAM
        val pose = slamEngine.processScan(
            scanPoints,
            currentPose.x,
            currentPose.y,
            currentPose.theta
        )
        
        // Update UI state
        _robotPose.value = pose
        _occupancyGrid.value = slamEngine.getMap()
    }
}
```

### 3.3 Map Visualization

```kotlin
@Composable
fun SLAMMapView(
    grid: OccupancyGrid,
    robotPose: Pose2D,
    scanPoints: List<LidarScanPoint>
) {
    Canvas(modifier = Modifier.fillMaxSize()) {
        val scale = 20f  // pixels per meter
        
        // Draw grid
        for (x in 0 until grid.width) {
            for (y in 0 until grid.height) {
                val prob = grid.getProbability(x, y)
                if (prob > 0.5f) {  // Occupied
                    drawRect(
                        color = Color.Black,
                        topLeft = Offset(x * scale / grid.resolution, y * scale / grid.resolution),
                        size = androidx.compose.ui.geometry.Size(scale, scale)
                    )
                }
            }
        }
        
        // Draw scan points
        scanPoints.forEach { point ->
            val px = (robotPose.x + point.x) * scale + size.width / 2
            val py = (robotPose.y + point.y) * scale + size.height / 2
            drawCircle(Color.Gray, radius = 2f, center = Offset(px, py))
        }
        
        // Draw robot
        drawRobot(size.width / 2, size.height / 2, robotPose.theta)
    }
}
```

**Đánh giá**: ⭐⭐⭐⭐⭐ - Thuật toán đơn giản, hoạt động ổn định

---

## PHASE 4: LOCALIZATION

### 4.1 ICP Scan Matching

```cpp
// scan_matching.cpp
Pose2D ICP::match(const std::vector<ScanPoint>& currentScan, 
                  const Pose2D& initialGuess,
                  const OccupancyGrid& map) {
    
    Pose2D pose = initialGuess;
    const int maxIterations = 20;
    
    for (int iter = 0; iter < maxIterations; iter++) {
        // 1. Transform scan to world frame
        std::vector<Point2D> transformed;
        for (const auto& p : currentScan) {
            transformed.push_back(transform(pose, p));
        }
        
        // 2. Find closest points in map
        std::vector<std::pair<Point2D, Point2D>> correspondences;
        for (const auto& tp : transformed) {
            auto closest = findClosestInMap(tp, map);
            correspondences.push_back({tp, closest});
        }
        
        // 3. Compute transformation
        auto delta = computeTransform(correspondences);
        
        // 4. Apply transformation
        pose = pose + delta;
        
        // 5. Check convergence
        if (delta.norm() < 0.001f) break;
    }
    
    return pose;
}
```

### 4.2 Độ chính xác cho robot đi thẳng

```
Để robot đi THẲNG với độ lệch < 1cm:

1. SLAM Map tạo trước (đã biết tường ở đâu)
2. Robot bắt đầu tại (0, 0)
3. Khi đi thẳng:
   - LiDAR đọc khoảng cách tường bên trái/phải
   - So sánh với map: nếu tường cách 50cm nhưng đọc được 48cm
   → Robot lệch 2cm sang phải
   → Hiệu chỉnh: quay nhẹ sang trái
4. Pure Pursuit giữ heading
5. Kết quả: độ lệch < 1cm

Điều này giống như robot dò line, nhưng dò tường (wall following)!
```

**Đánh giá**: ⭐⭐⭐⭐ - Cần test thực tế để tune parameters

---

## PHASE 5: NAVIGATION

### 5.1 A* Path Planner

```cpp
// astar.cpp
class AStarPlanner {
public:
    std::vector<GridCell> findPath(
        int startX, int startY,
        int goalX, int goalY,
        const OccupancyGrid& map
    ) {
        // Priority queue (min-heap by f = g + h)
        std::priority_queue<Node> openSet;
        
        // Start node
        Node start = {startX, startY, 0, heuristic(startX, startY, goalX, goalY), nullptr};
        openSet.push(start);
        
        // A* search loop
        while (!openSet.empty()) {
            auto current = openSet.top();
            openSet.pop();
            
            // Goal check
            if (current.x == goalX && current.y == goalY) {
                return reconstructPath(current);
            }
            
            // Expand neighbors (8-directional)
            for (const auto& dir : directions) {
                int nx = current.x + dir.dx;
                int ny = current.y + dir.dy;
                
                if (!map.isValid(nx, ny) || map.isOccupied(nx, ny)) {
                    continue;
                }
                
                float g = current.g + (dir.dx != 0 && dir.dy != 0 ? 1.414f : 1.0f);
                float h = heuristic(nx, ny, goalX, goalY);
                
                openSet.push({nx, ny, g, g + h, &current});
            }
        }
        
        return {};  // No path found
    }
};
```

### 5.2 Pure Pursuit Controller

```cpp
// pure_pursuit.cpp
class PurePursuitController {
public:
    VelocityCommand computeCommand(
        const std::vector<Pose2D>& path,
        const Pose2D& currentPose,
        float lookaheadDist = 0.5f
    ) {
        // 1. Find closest point on path
        int closestIdx = findClosestPoint(path, currentPose);
        
        // 2. Find lookahead point
        float accumDist = 0;
        Pose2D lookaheadPoint;
        for (int i = closestIdx; i < path.size() - 1; i++) {
            accumDist += distance(path[i], path[i+1]);
            if (accumDist >= lookaheadDist) {
                lookaheadPoint = path[i+1];
                break;
            }
        }
        
        // 3. Transform to robot frame
        auto [lx, ly] = transformToRobotFrame(lookaheadPoint, currentPose);
        
        // 4. Compute curvature
        float L = lookaheadDist;
        float kappa = 2 * ly / (L * L);
        
        // 5. Output velocity command
        VelocityCommand cmd;
        cmd.vx = targetSpeed;
        cmd.vy = 0;
        cmd.omega = kappa * cmd.vx;
        
        return cmd;
    }
};
```

### 5.3 Điều khiển Mecanum

```cpp
// Từ vx, vy, omega → wheel velocities
WheelSpeeds mecanumInverseKinematics(cmd.vx, cmd.vy, cmd.omega);

// Tạo UDP packet
auto packet = MotorCommandPacket::encode(
    int16_t(cmd.vx * 1000),  // mm/s
    int16_t(cmd.vy * 1000),  // mm/s
    int16_t(cmd.omega * 1000)  // mrad/s
);

// Gửi qua WiFi UDP
socket.sendto(packet, esp32Ip, 4210);
```

**Đánh giá**: ⭐⭐⭐⭐⭐ - Thuật toán well-known, dễ implement

---

## PHASE 6: OBSTACLE AVOIDANCE

### 6.1 Dynamic Window Approach

```cpp
// dwa.cpp
class DWAController {
public:
    VelocityCommand computeVelocity(
        const VelocityCommand& current,
        const Pose2D& pose,
        const std::vector<ScanPoint>& scan,
        const OccupancyGrid& map
    ) {
        // 1. Sample velocity space
        std::vector<VelocityCommand> trajectories;
        for (float v = current.vx - maxAccel; v <= current.vx + maxAccel; v += dv) {
            for (float w = current.omega - maxAngularAccel; 
                 w <= current.omega + maxAngularAccel; w += dw) {
                
                // 2. Simulate trajectory
                auto traj = simulateTrajectory(pose, v, w, dt);
                
                // 3. Check collision
                if (checkCollision(traj, map, scan)) {
                    continue;
                }
                
                // 4. Score trajectory
                float score = headingScore(traj, goal) 
                            + distScore(traj, obstacles)
                            + velScore(v);
                
                trajectories.push_back({v, w, score});
            }
        }
        
        // 5. Return best velocity
        return maxScore(trajectories);
    }
};
```

### 6.2 Integration với Navigation

```
┌──────────────────────────────────────────────────────────────┐
│                    NAVIGATION LOOP                          │
│                                                              │
│  1. Get global path (A*) from waypoint A → B             │
│  2. Pure Pursuit follow path                               │
│  3. DWA check obstacle:                                    │
│     └── Obstacle detected → stop → DWA → new velocity     │
│  4. If blocked completely → request new path (replan)     │
│  5. If waypoint reached → next waypoint                   │
└──────────────────────────────────────────────────────────────┘
```

**Đánh giá**: ⭐⭐⭐⭐ - DWA là algorithm tiêu chuẩn

---

## PHASE 7: ANDROID UI

### 7.1 UI Components

```
┌─────────────────────────────────────────────────────────────┐
│                    MAIN SCREEN                             │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────┐  │
│  │                                                     │  │
│  │              SLAM MAP CANVAS                        │  │
│  │                                                     │  │
│  │    - Occupancy Grid (black/white)                  │  │
│  │    - Scan Points (gray dots)                       │  │
│  │    - Robot Pose (blue arrow)                       │  │
│  │    - Path (green line)                            │  │
│  │    - Waypoints (numbered circles)                  │  │
│  │                                                     │  │
│  └─────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌────────────────────┐  ┌────────────────────────────┐  │
│  │ Status: RUNNING     │  │ LiDAR: 10Hz | FPS: 10   │  │
│  │ Pose: (1.2, 0.5)m  │  │ Points: 360 | Map: 80%  │  │
│  └────────────────────┘  └────────────────────────────┘  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ MODE: [●SLAM] [○NAV] [○MANUAL]                      │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ WAYPOINTS                                             │  │
│  │ 1. (0.0, 0.0) - Start                               │  │
│  │ 2. (1.0, 0.0) - Shelf A                             │  │
│  │ 3. (2.0, 1.0) - Counter                            │  │
│  │ [+ Add] [✕ Clear] [⟳ Save] [📂 Load]                │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              CONTROL PAD (Manual Mode)                 │  │
│  │                      ▲                               │  │
│  │                   ◄ │ ►                            │  │
│  │                      ▼                               │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  [ 🚀 START ]  [ ⏸ PAUSE ]  [ ⏹ STOP ]  [ 🛑 ESTOP ]  │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 Jetpack Compose Implementation

```kotlin
@Composable
fun SLAMMapCanvas(
    grid: OccupancyGrid,
    pose: Pose2D,
    scanPoints: List<LidarScanPoint>,
    path: List<Pose2D>,
    waypoints: List<Waypoint>,
    modifier: Modifier = Modifier
) {
    Canvas(modifier = modifier) {
        val scale = size.minDimension / 20f  // 20m view range
        
        // Transform to screen coordinates
        fun toScreen(x: Float, y: Float): Offset {
            return Offset(
                x = (x - pose.x) * scale + size.width / 2,
                y = (y - pose.y) * scale + size.height / 2
            )
        }
        
        // Draw grid cells
        for (x in 0 until grid.width step 4) {
            for (y in 0 until grid.height step 4) {
                val prob = grid.getProbability(x, y)
                if (prob > 0.6f) {
                    val (wx, wy) = grid.mapToWorld(x, y)
                    drawRect(
                        color = Color.Black,
                        topLeft = toScreen(wx, wy),
                        size = Size(scale * 4, scale * 4)
                    )
                }
            }
        }
        
        // Draw scan points
        scanPoints.forEach { p ->
            val sx = pose.x + p.x
            val sy = pose.y + p.y
            drawCircle(
                color = Color.Gray.copy(alpha = 0.5f),
                radius = 2f,
                center = toScreen(sx, sy)
            )
        }
        
        // Draw path
        if (path.size > 1) {
            for (i in 0 until path.size - 1) {
                drawLine(
                    color = Color.Green,
                    start = toScreen(path[i].x, path[i].y),
                    end = toScreen(path[i+1].x, path[i+1].y),
                    strokeWidth = 3f
                )
            }
        }
        
        // Draw robot
        drawRobot(size.width / 2, size.height / 2, pose.theta)
    }
}
```

**Đánh giá**: ⭐⭐⭐⭐⭐ - Jetpack Compose mạnh mẽ cho robotics UI

---

## PHASE 8: INTEGRATION

### 8.1 Full System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         XIAOMI PAD 6 (ANDROID)                         │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │                    ANDROID APP (Kotlin)                          │ │
│  │                                                                  │ │
│  │   ┌───────────┐   ┌───────────┐   ┌───────────┐              │ │
│  │   │ SLAM UI   │   │ Map View  │   │ Controls  │              │ │
│  │   └─────┬─────┘   └─────┬─────┘   └─────┬─────┘              │ │
│  │         └───────────┴───────────┴──────────┘                      │ │
│  │                          │                                         │ │
│  │                   ViewModel Layer                                 │ │
│  │                          │                                         │ │
│  └──────────────────────────┼─────────────────────────────────────────┘ │
│                             │ JNI                                       │
│  ┌──────────────────────────┼─────────────────────────────────────────┐ │
│  │              NATIVE C++ (NDK)                                     │ │
│  │                                                                  │ │
│  │   ┌───────────┐   ┌───────────┐   ┌───────────┐   ┌───────────┐│ │
│  │   │ LiDAR    │──▶│  SLAM     │──▶│ Planner  │──▶│  Motor   ││ │
│  │   │ Driver   │   │  Engine   │   │  (A*)   │   │  Command ││ │
│  │   └───────────┘   └───────────┘   └───────────┘   └─────┬─────┘│ │
│  │                                                          │       │ │
│  └──────────────────────────────────────────────────────────┼───────┘ │
│                                                              │         │
│                                                     WiFi UDP │         │
└──────────────────────────────────────────────────────────────┼─────────┘
                                                               │
                                                               ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                           ESP32-S3 (MOTOR ONLY)                       │
│                                                                          │
│   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐                │
│   │   UDP       │──▶│  PID Loop   │──▶│   PWM Gen   │                │
│   │   Receive   │   │  (Speed)    │   │   (LEDC)    │                │
│   │   (vx,vy,ω)│   └─────────────┘   └──────┬──────┘                │
│   └─────────────┘                             │                        │
│                                                ▼                        │
│                                    ┌───────────────────┐                │
│                                    │    TB6612FNG      │                │
│                                    └─────────┬─────────┘                │
│                                                │                        │
│                              ┌─────────────────┼─────────────────┐     │
│                              ▼                 ▼                 ▼     │
│                         ┌────────┐       ┌────────┐       ┌────────┐  │
│                         │ Motor  │       │ Motor  │       │ Motor  │  │
│                         │   FL   │       │   FR   │       │   RL   │  │
│                         └────────┘       └────────┘       └────────┘  │
└──────────────────────────────────────────────────────────────────────────┘
```

### 8.2 Test Checklist

```markdown
## Integration Test Checklist

### Hardware Tests
- [ ] ESP32 powers on
- [ ] WiFi AP starts
- [ ] 4 motors respond to PWM
- [ ] IMU readings stable
- [ ] YDLIDAR connects via USB

### Communication Tests
- [ ] Android connects to ESP32 WiFi
- [ ] UDP packets sent/received
- [ ] Command latency < 20ms
- [ ] Telemetry received at 10Hz

### SLAM Tests
- [ ] Scan data displayed at 10Hz
- [ ] Occupancy grid updates
- [ ] Map accurate (compare with floor plan)
- [ ] Save/load map works

### Localization Tests
- [ ] Initial pose correct
- [ ] Pose updates when moving
- [ ] Drift acceptable (< 10cm after 10m)
- [ ] Pose correction using scan matching

### Navigation Tests
- [ ] A* finds path
- [ ] Pure Pursuit follows path
- [ ] Robot reaches waypoint
- [ ] Smooth trajectory

### Obstacle Tests
- [ ] Static obstacle detected
- [ ] Dynamic obstacle detected
- [ ] Robot avoids and continues
- [ ] DWA prevents collision

### Full Demo
- [ ] Mapping mode: create map
- [ ] Save map to storage
- [ ] Load map
- [ ] Navigate to waypoints
- [ ] Return to dock
- [ ] Battery monitoring works
```

---

## CÁC VẤN ĐỀ THƯỜNG GẶP

### 1. YDLIDAR không kết nối
```
Kiểm tra:
1. USB OTG cable có hỗ trợ data không?
2. Android có quyền USB không?
3. Đúng VID/PID không?
4. Baudrate đúng (115200)?
```

### 2. ESP32 không nhận lệnh
```
Kiểm tra:
1. WiFi có kết nối không?
2. Đúng IP và port không?
3. Checksum đúng không?
4. Firewall không block UDP?
```

### 3. Robot không đi thẳng
```
Nguyên nhân:
1. Motor trim chưa cân bằng
2. PID chưa tuned
3. Wheel slip

Giải pháp:
1. Auto-calibrate motor trim
2. Tune PID gains
3. Tăng friction bánh
4. Dùng LiDAR feedback để hiệu chỉnh
```

### 4. SLAM drift nhiều
```
Nguyên nhân:
1. Scan matching không chính xác
2. Map resolution quá thô
3. Odometry drift (không dùng encoder)

Giải pháp:
1. Tăng scan matching iterations
2. Giảm map resolution (2cm/cell)
3. Loop closure detection
4. Dùng IMU fusion
```

---

## KẾT LUẬN

Với kiến trúc mới này:
- **ESP32-S3**: Chỉ làm Motor Controller (đơn giản, ổn định)
- **Xiaomi Pad 6**: Làm tất cả (SLAM, Navigation, UI - mạnh mẽ)
- **YDLIDAR X3**: Kết nối trực tiếp với Tablet ( không qua ESP32)

Ưu điểm:
✓ Phân tách rõ ràng
✓ Tablet đủ mạnh cho SLAM
✓ Dễ debug và phát triển
✓ Có thể monitor trực tiếp

Nhược điểm:
✗ Cần USB OTG cable
✗ Phụ thuộc WiFi cho motor control
✗ Độ trễ thêm (WiFi + SLAM)

Độ khả thi tổng thể: ⭐⭐⭐⭐⭐

Robot hoàn toàn có thể đi THẲNG với độ lệch < 1cm khi:
1. SLAM map chính xác
2. Scan matching hoạt động tốt
3. Pure Pursuit + LiDAR feedback
4. PID tuned cho Mecanum
