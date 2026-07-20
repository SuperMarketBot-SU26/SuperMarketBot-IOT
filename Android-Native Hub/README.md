# SmartMarketBot - Android Native Hub

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           XIAOMI PAD 6 (ANDROID)                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                    Android Application Layer (Kotlin)                  │ │
│  │                                                                        │ │
│  │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐│ │
│  │   │   SLAM UI   │  │  Map View   │  │   Control   │  │  Waypoint  ││ │
│  │   │  (Compose)  │  │ (Canvas)    │  │   Pad       │  │   Editor   ││ │
│  │   └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬─────┘│ │
│  │          │                │                │                │       │ │
│  │          └────────────────┴────────────────┴────────────────┘       │ │
│  │                               │                                      │ │
│  │                    ┌──────────┴──────────┐                          │ │
│  │                    │   ViewModel/State  │                          │ │
│  │                    │   (StateFlow)       │                          │ │
│  │                    └──────────┬──────────┘                          │ │
│  └───────────────────────────────┼──────────────────────────────────────┘ │
│                                  │ JNI                                    │
│  ┌───────────────────────────────┼──────────────────────────────────────┐ │
│  │                    Native C++ Layer (NDK)                             │ │
│  │                                                                        │ │
│  │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐│ │
│  │   │  LiDAR     │  │   SLAM      │  │  Planner    │  │   Motor     ││ │
│  │   │  Driver    │──│  Engine     │──│  (A* + PP)  │──│  Command    ││ │
│  │   └─────────────┘  └─────────────┘  └─────────────┘  └──────┬─────┘│ │
│  │                                                              │       │ │
│  │   ┌──────────────────────────────────────────────────────────┘       │ │
│  │   │                      WiFi UDP Socket                           │ │
│  │   └──────────────────────────────────────────────────────────┐       │ │
│  │                                                             │       │ │
│  │   Dependencies: Eigen, TinySLAM, OpenCV (optional), spdlog        │ │
│  └─────────────────────────────────────────────────────────────────────┘ │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ USB OTG
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        YDLIDAR X3 (Direct USB)                             │
│                        USB CDC/ACM • 115200 baud • 10Hz                    │
└─────────────────────────────────────────────────────────────────────────────┘

                                    │
                                    │ WiFi UDP (192.168.x.x)
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        ESP32-S3 (Motor Controller Only)                      │
│                                                                             │
│   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │
│   │   UDP       │  │  PWM Gen    │  │  PID Loop   │  │   Motor     │       │
│   │   Receive   │──│  (LEDC)    │──│  (Speed)    │──│   Driver    │       │
│   │   (vx,vy,ω) │  │  20kHz     │  │  per motor  │  │  TB6612     │       │
│   └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘       │
│                                                                             │
│   NO SLAM • NO Navigation • NO WebServer • NO MQTT                         │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
                         ┌─────────────────┐
                         │  TB6612x2 +    │
                         │  4x DC Motor   │
                         │  Mecanum Wheels │
                         └─────────────────┘
```

## Module Descriptions

### 1. LiDAR Driver (`lidar/`)
- Direct USB CDC/ACM communication with YDLIDAR X3
- Protocol parsing (YDLIDAR X3 360° scan data)
- Point cloud generation
- Buffer management for real-time scan data

### 2. SLAM Engine (`slam/`)
- **TinySLAM** - Lightweight 2D laser SLAM
  - ~200 lines C code
  - Works without odometry
  - Occupancy grid mapping
- **Scan Matching** - ICP for localization refinement
- **Pose Estimation** - Particle filter for global localization

### 3. Navigation (`nav/`)
- **A* Path Planner** - Grid-based shortest path
- **Pure Pursuit** - Path tracking controller
- **Dynamic Window Approach** - Real-time obstacle avoidance
- **Pose Correction** - Using LiDAR scan matching

### 4. Motor Command (`motor/`)
- Mecanum inverse kinematics
- Velocity command generation (vx, vy, omega)
- WiFi UDP packet generation

## Communication Protocol

### Android → ESP32 (Motor Command)
```
┌─────────────────────────────────────────────────────────┐
│ Byte:  0    1    2    3    4    5    6    7    8     │
│ Type: HEAD  CMD  VX_H VX_L VY_H VY_L W_H  W_L  CHK   │
│ Desc: 0xAA  0x01 int16  int16  int16  int16  XOR    │
└─────────────────────────────────────────────────────────┘
- VX: forward velocity (mm/s, signed)
- VY: strafe velocity (mm/s, signed)
- W:  angular velocity (mrad/s, signed)
- CHK: XOR checksum of bytes 1-7
```

### ESP32 → Android (Telemetry - Optional)
```
┌─────────────────────────────────────────────────────────┐
│ Byte:  0    1    2    3    4    5    6    7    8     │
│ Type: HEAD  CMD  X_H  X_L  Y_H  Y_L  BAT   CHK       │
└─────────────────────────────────────────────────────────┘
- X, Y: odometry position (mm, signed)
- BAT: battery voltage (0.1V units)
```

## Build Instructions

### Prerequisites
- Android Studio Hedgehog or newer
- Android NDK r25c+
- CMake 3.18+
- USB OTG cable (USB-C to USB-C or USB-A)

### Build
```bash
cd Android-Native\ Hub
./gradlew assembleDebug
```

### Install
```bash
adb install app/build/outputs/apk/debug/app-debug.apk
```

## Hardware Connections

### YDLIDAR X3 → Xiaomi Pad 6
```
YDLIDAR X3 (USB-C) ─── USB OTG Cable ─── Xiaomi Pad 6 (USB-C)
```

### ESP32-S3 → Xiaomi Pad 6 (WiFi)
```
ESP32-S3 (WiFi AP) ◄────────────────── Xiaomi Pad 6 (WiFi Client)
SSID: SmartMarketBot
Password: 12345678
UDP Port: 4210
```

## Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| LiDAR Scan Rate | 10 Hz | YDLIDAR X3 native |
| SLAM Update Rate | 10 Hz | Match scan rate |
| Motor Command Rate | 20 Hz | 50ms cycle |
| End-to-End Latency | < 100ms | Sensor to motor |
| Map Resolution | 5 cm/cell | Suitable for indoor |
| Localization Accuracy | < 10 cm | Using scan matching |

## Dependencies

### Native (C++)
| Library | Version | Purpose |
|---------|--------|---------|
| Eigen | 3.4+ | Linear algebra |
| TinySLAM | - | SLAM core |
| spdlog | 1.12+ | Logging |
| nlohmann/json | 3.11+ | Configuration |

### Android (Kotlin)
| Library | Version | Purpose |
|---------|---------|---------|
| Jetpack Compose | 1.5+ | UI framework |
| Hilt | 2.48+ | Dependency injection |
| Coroutines | 1.7+ | Async operations |
| usb-serial-for-android | 3.5+ | USB CDC/ACM |
| OkHttp | 4.12+ | WiFi UDP (Java) |
