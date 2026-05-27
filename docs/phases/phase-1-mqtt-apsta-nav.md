# Phase 1 — MQTT + AP+STA + Quick Fix Navigation

**Thời gian:** 1–2 ngày · **Phụ thuộc:** Backend Mosquitto + firmware S3 hiện tại

## Checklist

- [ ] ESP32 STA vào router (AP vẫn `SmartMarketBot`)
- [ ] MQTT connected → Serial `MQTT connected`
- [ ] Telemetry mỗi 2s → BE `Robot_Logs` + SignalR
- [ ] Commands: `stop`, `mode_auto`, `mode_manual`, `set_speed`
- [ ] `AUTO_STOP_HOLD_MS` 2800 → **800**
- [ ] Smart scan direction (`s_lastClearDir`)
- [ ] `AN_BACKUP` khi scan fail
- [ ] Encoder stuck detection
- [ ] `POST /api/Navigation/navigate-robot` (BE)

## IoT — file tạo/sửa

| File | Việc |
|------|------|
| **MqttClient.h** (mới) | PubSubClient, callback, telemetry/status, `mqttLoop` |
| **Config.h** | `WIFI_STA_ENABLE`, `STA_SSID/PASS`, MQTT defines |
| **WebUI.h** | `WIFI_AP_STA`, STA connect, `mqttInit()` |
| **SuperMarketBot-IOT.ino** | `#include MqttClient`, `mqttLoop` trong taskWebIO, flags thread-safe |
| Library | **PubSubClient** 2.8 |

## BE — Prompt 1.3

| File | Việc |
|------|------|
| `MqttClientService.cs` | `IncomingRobotPayload`: LidarFront/Rear, Rpm*, NavState, Estop… |
| `RobotTelemetryDto` | Fields mới nullable |
| `NavigationController` | `POST navigate-robot` → Dijkstra + MQTT publish |
| `IRobotService` | `GetRobotByCodeAsync` |

## MQTT JSON telemetry (BE mong đợi)

```json
{
  "Battery": 85,
  "Location": null,
  "Status": "online",
  "Mode": "manual|auto",
  "IsOnline": true,
  "XCoord": null,
  "YCoord": null,
  "lidarFront": 120,
  "lidarRear": 800,
  "rpmFL": 0, "rpmFR": 0, "rpmRL": 0, "rpmRR": 0,
  "navState": "reactive",
  "estop": false
}
```

## Thread safety (bắt buộc)

- Mọi `g_mqttClient.*` chỉ trong **taskWebIO (Core 0)**
- Core 1 set `g_mqttStatusPending` + `g_mqttPendingStatus[32]`

## Prompt Cursor (copy vào Composer)

> Copy nguyên khối **PROMPT 1.1**, **1.2**, **1.3** từ tin nhắn roadmap / file `cursor_prompt.md` Phase 1 trong repo BE (nếu đã lưu).

**PROMPT 1.1** — `MqttClient.h` + Config STA + WebUI + ino + PubSubClient  
**PROMPT 1.2** — Nav fix: STOP_HOLD 800ms, backup, stuck, smart scan  
**PROMPT 1.3** — BE telemetry + `navigate-robot`

## Cấu hình demo

```cpp
#define MQTT_BROKER_HOST  "192.168.1.100"  // IP PC chạy docker Mosquitto
#define MQTT_CLIENT_ID    "ROBOT-01"
#define STA_SSID          "YOUR_ROUTER_SSID"
```

Seed robot trong SQL: `RobotCode = 'ROBOT-01'` (nếu chưa có).
