# Phase 3 — Waypoint Navigation (Full Integration)

**Thời gian:** 5–7 ngày · **Phụ thuộc:** Phase 1 + 2

## Checklist

- [ ] `WaypointNav.h` — FSM: IDLE, HEADING, DRIVING, ARRIVING, OBSTACLE, REROUTE, PAUSED
- [ ] `MODE_WAYPOINT` trong `Config.h` + WebUI/CtrlJson mode `m=2`
- [ ] MQTT `navigate` → `wpLoadRoute` → MODE_WAYPOINT
- [ ] `cancel_nav`, `pause_nav`, `resume_nav`, `stop`
- [ ] Pure Pursuit + PID trong WP_DRIVING; LiDAR safety < 22cm
- [ ] Pose correction tại waypoint (`WP_POSE_CORRECT_ENABLE`)
- [ ] Telemetry: `NavState`, `WaypointProgress`, `TargetNodeId`
- [ ] BE: `NavigationCommandService`, `navigate-robot`, `reroute`, cancel/pause/resume
- [ ] E2E: `POST navigate-robot` → robot tới `TargetNodeId`

## MQTT command navigate

```json
{
  "command": "navigate",
  "payload": "{\"waypoints\":[{\"id\":1,\"x\":0,\"y\":0},{\"id\":3,\"x\":3,\"y\":3}]}"
}
```

Status từ robot: `arrived`, `blocked` (reroute).

## IoT — file tạo/sửa

| File | Việc |
|------|------|
| **WaypointNav.h** (mới) | FSM + `wpLoadRoute` + Pure Pursuit |
| **Config.h** | `MODE_WAYPOINT`, `WP_*` defines |
| **MqttClient.h** | Parse navigate/cancel/pause/resume |
| **SuperMarketBot-IOT.ino** | `wpNavigateUpdate()` khi MODE_WAYPOINT |
| **CtrlJson.h / WebUI** | mode `m=2` |

## BE — Prompt 3.2

| Thành phần | Mô tả |
|------------|--------|
| `INavigationCommandService` | Navigate, Reroute, Cancel, Pause, Resume |
| `NavigationCommandService` | Dijkstra + MQTT publish waypoints |
| `NavigationController` | REST endpoints đầy đủ |
| `MqttClientService` | Log khi status `blocked` |

## Test E2E (tóm tắt)

1. `docker compose up` + `dotnet run` API  
2. ESP32 STA + MQTT connected  
3. `POST /api/Navigation/navigate-robot`  
   `{"RobotCode":"ROBOT-01","TargetNodeId":5}`  
4. Robot: HEADING → DRIVING → ARRIVING … → `arrived`

## Prompt Cursor

**PROMPT 3.1** — WaypointNav.h + tích hợp ino/MQTT/WebUI  
**PROMPT 3.2** — Backend NavigationCommandService + controller

## Lưu ý

- `MODE_AUTO` (reactive) giữ nguyên làm backup  
- `MODE_MANUAL` joystick không đổi  
- Route mới khi đang navigate → hủy route cũ, load mới
