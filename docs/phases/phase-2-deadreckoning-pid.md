# Phase 2 — Dead Reckoning + PID Speed Control

**Thời gian:** 3–5 ngày · **Phụ thuộc:** Phase 1 xong

## Checklist

- [ ] `Localization.h` — pose (x, y, θ) @ 20 Hz
- [ ] `g_encTicksTotal[4]` trong `Odometry.h` (ISR ++, không reset khi odom reset distance)
- [ ] MQTT telemetry: `XCoord`, `YCoord`, `Heading`
- [ ] `PidController.h` — dual PID trái/phải
- [ ] **Chỉ `AN_CRUISE`** dùng PID; scan/backup giữ open-loop
- [ ] `pidReset()` khi `botStop()`
- [ ] Đo `RPM_AT_MAX_PWM`, `TRACK_WIDTH_M` trên robot thật
- [ ] BE: cột `Heading` trong `Robot_Logs`, `GET /api/Robots/{code}/pose`

## IoT — file tạo/sửa

| File | Việc |
|------|------|
| **Localization.h** (mới) | DR từ `g_encTicksTotal`, `TRACK_WIDTH_M`, midpoint integration |
| **Odometry.h** | `g_encTicksTotal[4]`, ISR increment |
| **PidController.h** (mới) | Kp/Ki/Kd, `pwmToTargetRpm` |
| **Motors.h** | `botForwardPid`, `botRotatePid` |
| **SuperMarketBot-IOT.ino** | `localizationUpdate`, PID trong `AN_CRUISE` |
| **MqttClient.h** | Thêm pose + optional pid debug fields |

## Hệ tọa độ (khớp DB NavigationNodes)

- X = East, Y = North, θ=0 hướng Y+
- `x += ΔD*sin(midHeading)`, `y += ΔD*cos(midHeading)`

## Tuning PID (thứ tự)

1. Đo `RPM_AT_MAX_PWM` @ PWM_MAX  
2. Kp only → 3. Ki → 4. Kd nhỏ  

## BE — Prompt 2.3

- `RobotLog.Heading` (float nullable)
- SQL: `ALTER TABLE Robot_Logs ADD Heading FLOAT NULL`
- `RobotsController`: `GET {robotCode}/pose` → log mới nhất

## Prompt Cursor

**PROMPT 2.1** — Localization + Odometry ticks  
**PROMPT 2.2** — PidController + AN_CRUISE only  
**PROMPT 2.3** — BE Heading + pose API
