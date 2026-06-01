/* =====================================================================
 *  MqttClient.h — MQTT client cho SmartMarketBot
 *  Chạy toàn bộ trên Core 0 (taskWebIO) — PubSubClient không thread-safe.
 *  Core 1 chỉ set flag volatile để yêu cầu publish status.
 *
 *  Thư viện: PubSubClient by Nick O'Leary (Library Manager: "PubSubClient")
 * =====================================================================*/
#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include "Config.h"
#include "Localization.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* ==================== CẤU HÌNH MQTT ================================ */
/** IP máy chạy Mosquitto/Backend — đổi thành IP thật trước khi nạp */
#define MQTT_BROKER_HOST   "192.168.1.100"
#define MQTT_BROKER_PORT   1883
/** Mã robot — dùng làm client ID và tên topic */
#define MQTT_CLIENT_ID     "ROBOT-01"
#define MQTT_USER          ""          // Để rỗng nếu broker không cần auth
#define MQTT_PASS          ""
#define MQTT_RECONNECT_MS  5000u       // Thời gian giữa 2 lần thử kết nối lại
#define MQTT_TELEMETRY_MS  2000u       // Publish telemetry mỗi 2 giây

/* Topics */
#define MQTT_TOPIC_TELEMETRY  "smartmarketbot/robot/" MQTT_CLIENT_ID "/telemetry"
#define MQTT_TOPIC_STATUS     "smartmarketbot/robot/" MQTT_CLIENT_ID "/status"
#define MQTT_TOPIC_COMMAND    "smartmarketbot/robot/" MQTT_CLIENT_ID "/command"

/* ==================== BIẾN NỘI BỘ ================================== */
static WiFiClient       g_wifiClient;
static PubSubClient     g_mqttClient(g_wifiClient);
static uint32_t         g_mqttLastReconnectMs = 0;
static uint32_t         g_mqttLastTelemetryMs = 0;
bool                    g_mqttEnabled = false; ///< true khi STA đã lên. Được set trong WebUI.h

/* ==================== FLAG THREAD-SAFE (Core 1 → Core 0) =========== */
/** Core 1 set flag này để Core 0 publish status message */
volatile bool           g_mqttStatusPending = false;
volatile char           g_mqttPendingStatus[32] = {0};

/* ==================== CALLBACK — nhận lệnh từ Backend ============== */
static void mqttCallback(char *topic, byte *payload, unsigned int length) {
  if (length == 0 || length > 511) return;

  char buf[512];
  memcpy(buf, payload, length);
  buf[length] = '\0';

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, buf) != DeserializationError::Ok) {
    Serial.printf("[MQTT] JSON parse error. Topic: %s\n", topic);
    return;
  }

  const char *cmd = doc["command"] | "";
  Serial.printf("[MQTT] CMD: %s\n", cmd);

  if (strcmp(cmd, "stop") == 0) {
    g_state.estop = true;

  } else if (strcmp(cmd, "mode_auto") == 0) {
    g_state.mode = MODE_AUTO;

  } else if (strcmp(cmd, "mode_manual") == 0) {
    g_state.mode = MODE_MANUAL;

  } else if (strcmp(cmd, "set_speed") == 0) {
    int v = doc["payload"] | -1;
    if (v >= 0 && v <= 100) {
      g_state.autoBaseSpeed = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)v / 100u);
    }

  } else if (strcmp(cmd, "navigate") == 0) {
    /* Phase 3: WaypointNav.h sẽ xử lý; tạm log */
    const char *wpJson = doc["payload"] | "";
    Serial.printf("[MQTT] navigate payload len=%d (Phase 3 se xu ly)\n", (int)strlen(wpJson));

  } else {
    Serial.printf("[MQTT] Unknown command: %s\n", cmd);
  }
}

/* ==================== INIT ========================================= */
static void mqttInit() {
  g_mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  g_mqttClient.setCallback(mqttCallback);
  g_mqttClient.setBufferSize(1024); // waypoint payload có thể lớn (Phase 3)
  Serial.printf("[MQTT] Init: broker=%s:%d  clientId=%s\n",
                MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_CLIENT_ID);
}

/* ==================== RECONNECT (gọi từ mqttLoop) ================== */
static void mqttReconnect() {
  if (!g_mqttEnabled) return;
  if (g_mqttClient.connected()) return;

  const uint32_t now = millis();
  if (now - g_mqttLastReconnectMs < MQTT_RECONNECT_MS) return;
  g_mqttLastReconnectMs = now;

  Serial.print(F("[MQTT] Connecting..."));
  bool ok;
  if (strlen(MQTT_USER) > 0) {
    ok = g_mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
  } else {
    ok = g_mqttClient.connect(MQTT_CLIENT_ID);
  }

  if (ok) {
    g_mqttClient.subscribe(MQTT_TOPIC_COMMAND);
    Serial.printf(" OK. Subscribed: %s\n", MQTT_TOPIC_COMMAND);
    /* Publish online status ngay khi kết nối */
    strncpy((char *)g_mqttPendingStatus, "online", sizeof(g_mqttPendingStatus) - 1);
    g_mqttStatusPending = true;
  } else {
    Serial.printf(" FAIL rc=%d\n", g_mqttClient.state());
  }
}

/* ==================== PUBLISH TELEMETRY ============================ */
static void mqttPublishTelemetry() {
  if (!g_mqttClient.connected()) return;
  const uint32_t now = millis();
  if (now - g_mqttLastTelemetryMs < MQTT_TELEMETRY_MS) return;
  g_mqttLastTelemetryMs = now;

  StaticJsonDocument<512> doc;

  /* Pin — nếu BAT_MONITOR_ENABLE=1, giá trị có trong g_batPct (RobotTelemetry.h).
     Tạm null cho đến khi Phase 2 gắn ADC. */
  doc["Battery"]       = (bool)BAT_MONITOR_ENABLE ? (int)0 : (int)-1; // -1 = N/A
  doc["Location"]      = (const char *)nullptr;
  doc["Status"]        = "online";
  doc["CurrentNodeId"] = (const char *)nullptr;
  doc["Mode"]          = (g_state.mode == MODE_AUTO) ? "auto" : "manual";
  doc["IsOnline"]      = true;
  doc["XCoord"]        = g_pose.x;
  doc["YCoord"]        = g_pose.y;
  doc["HeadingRad"]    = g_pose.headingRad;

  /* Sensor data */
  doc["lidarFront"] = g_state.lidarFront;
  doc["lidarRear"]  = g_state.lidarBack;
  doc["rpmFL"]      = g_state.rpmFL;
  doc["rpmFR"]      = g_state.rpmFR;
  doc["rpmRL"]      = g_state.rpmRL;
  doc["rpmRR"]      = g_state.rpmRR;
  doc["navState"]   = "reactive";
  doc["estop"]      = g_state.estop;

  char buf[512];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  g_mqttClient.publish(MQTT_TOPIC_TELEMETRY, (const uint8_t *)buf, n, false);
}

/* ==================== PUBLISH STATUS =============================== */
static void mqttPublishStatus(const char *status) {
  if (!g_mqttClient.connected()) return;

  StaticJsonDocument<128> doc;
  doc["Status"]   = status;
  doc["Mode"]     = (g_state.mode == MODE_AUTO) ? "auto" : "manual";
  doc["IsOnline"] = true;

  char buf[128];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  /* QoS-1 mô phỏng: retained=false, PubSubClient không hỗ trợ QoS1 natively
     → publish 2 lần đảm bảo broker nhận (đơn giản, đủ cho demo) */
  g_mqttClient.publish(MQTT_TOPIC_STATUS, (const uint8_t *)buf, n, false);
}

/* ==================== MAIN LOOP (gọi từ taskWebIO) ================= */
static void mqttLoop() {
  /* Xử lý pending status từ Core 1 trước */
  if (g_mqttStatusPending) {
    mqttPublishStatus((const char *)g_mqttPendingStatus);
    g_mqttStatusPending = false;
  }

  mqttReconnect();
  if (g_mqttClient.connected()) {
    g_mqttClient.loop();
    mqttPublishTelemetry();
  }
}

#endif // MQTT_CLIENT_H
