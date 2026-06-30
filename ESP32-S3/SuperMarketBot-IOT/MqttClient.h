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
#include "WaypointNav.h"
#include "CtrlJson.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* ==================== CẤU HÌNH MQTT ================================
 *
 *  ── CHẾ ĐỘ LOCAL (Mosquitto trên máy lab) ────────────────────────
 *    MQTT_USE_TLS  0
 *    MQTT_BROKER_HOST  "192.168.x.x"   (IP máy chạy Mosquitto)
 *    MQTT_BROKER_PORT  1883
 *    MQTT_USER / MQTT_PASS  ""
 *
 *  ── CHẾ ĐỘ CLOUD (HiveMQ Cloud Free) ─────────────────────────────
 *    MQTT_USE_TLS  1
 *    MQTT_BROKER_HOST  "xxxxxxxx.s2.eu.hivemq.cloud"
 *    MQTT_BROKER_PORT  8883
 *    MQTT_USER  "username_tren_hivemq"
 *    MQTT_PASS  "password_tren_hivemq"
 *
 *  Đổi MQTT_USE_TLS = 0/1 để chọn chế độ rồi nạp lại.
 * ================================================================= */
#define MQTT_USE_TLS       1           // 0 = local Mosquitto | 1 = HiveMQ Cloud TLS
#define MQTT_BROKER_HOST   "60922debd474446a84747b871c4a8182.s1.eu.hivemq.cloud"
#define MQTT_BROKER_PORT   8883
/** Mã robot — dùng làm client ID và tên topic */
#define MQTT_CLIENT_ID     "RB001"
#define MQTT_USER          "Smartmarketbot"
#define MQTT_PASS          "Passsep490"
#define MQTT_RECONNECT_MS  15000u      // Thời gian giữa 2 lần thử kết nối lại (tăng lên 15s để tránh nghẽn)
#define MQTT_TELEMETRY_MS  2000u       // Publish telemetry mỗi 2 giây

/* Topics */
#define MQTT_TOPIC_TELEMETRY  "smartmarketbot/robot/" MQTT_CLIENT_ID "/telemetry"
#define MQTT_TOPIC_STATUS     "smartmarketbot/robot/" MQTT_CLIENT_ID "/status"
#define MQTT_TOPIC_COMMAND    "smartmarketbot/robot/" MQTT_CLIENT_ID "/command"
#define MQTT_TOPIC_LOG        "smartmarketbot/robot/" MQTT_CLIENT_ID "/log"

/* ==================== BIẾN NỘI BỘ ================================== */
#if MQTT_USE_TLS
static WiFiClientSecure g_wifiClient;
#else
static WiFiClient       g_wifiClient;
#endif
extern SemaphoreHandle_t g_mqttMutex;
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
    Serial.printf("[MQTT ERROR] JSON parse failed on topic: %s\n", topic);
    return;
  }

  const char *cmd = doc["command"] | "";
  Serial.printf("\n--- [MQTT RECEIVED COMMAND] ---\nCommand: '%s'\n", cmd);

  if (strcmp(cmd, "stop") == 0) {
    Serial.println(F(">>> LỆNH: DỪNG KHẨN CẤP (STOP) từ Backend! Thiết lập ESTOP = true."));
    g_state.estop = true;

  } else if (strcmp(cmd, "mode_auto") == 0) {
    if (millis() < BOOT_GUARD_MS) {
      Serial.println(F("[MQTT WARNING] Bỏ qua lệnh mode_auto do đang trong thời gian Boot Guard."));
      return;
    }
    Serial.println(F(">>> LỆNH: CHUYỂN MODE TỰ HÀNH (AUTO NE VAT CAN) từ Backend!"));
    g_state.mode = MODE_AUTO;

  } else if (strcmp(cmd, "mode_manual") == 0) {
    Serial.println(F(">>> LỆNH: CHUYỂN MODE LÁI TAY (MANUAL) từ Backend!"));
    robotForceManualStop();

  } else if (strcmp(cmd, "set_speed") == 0 || strcmp(cmd, "set_speed_manual") == 0) {
    int v = -1;
    if (doc["payload"].is<int>()) {
      v = doc["payload"].as<int>();
    } else if (doc["payload"].is<const char*>()) {
      v = atoi(doc["payload"].as<const char*>());
    }
    Serial.printf(">>> LỆNH: ĐẶT TỐC ĐỘ LÁI TAY (SET_SPEED_MANUAL) từ Backend! Tốc độ: %d%%\n", v);
    if (v >= 0 && v <= 100) {
      g_state.baseSpeed = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)v / 100u);
    }

  } else if (strcmp(cmd, "set_speed_auto") == 0) {
    int v = -1;
    if (doc["payload"].is<int>()) {
      v = doc["payload"].as<int>();
    } else if (doc["payload"].is<const char*>()) {
      v = atoi(doc["payload"].as<const char*>());
    }
    Serial.printf(">>> LỆNH: ĐẶT TỐC ĐỘ TỰ HÀNH (SET_SPEED_AUTO) từ Backend! Tốc độ: %d%%\n", v);
    if (v >= 0 && v <= 100) {
      g_state.autoBaseSpeed = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)v / 100u);
    }

  } else if (strcmp(cmd, "set_speed_swerve") == 0) {
    int v = -1;
    if (doc["payload"].is<int>()) {
      v = doc["payload"].as<int>();
    } else if (doc["payload"].is<const char*>()) {
      v = atoi(doc["payload"].as<const char*>());
    }
    Serial.printf(">>> LỆNH: ĐẶT TỐC ĐỘ TRÁNH VẬT (SET_SPEED_SWERVE) từ Backend! Tốc độ: %d%%\n", v);
    if (v >= 0 && v <= 100) {
      g_state.swerveBaseSpeed = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)v / 100u);
    }

  } else if (strcmp(cmd, "set_speed_rotate") == 0) {
    int v = -1;
    if (doc["payload"].is<int>()) {
      v = doc["payload"].as<int>();
    } else if (doc["payload"].is<const char*>()) {
      v = atoi(doc["payload"].as<const char*>());
    }
    Serial.printf(">>> LỆNH: ĐẶT TỐC ĐỘ XOAY (SET_SPEED_ROTATE) từ Backend! Tốc độ: %d%%\n", v);
    if (v >= 0 && v <= 100) {
      g_state.rotateBaseSpeed = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)v / 100u);
    }

  } else if (strcmp(cmd, "set_yaw_scale") == 0) {
    int v = -1;
    if (doc["payload"].is<int>()) {
      v = doc["payload"].as<int>();
    } else if (doc["payload"].is<const char*>()) {
      v = atoi(doc["payload"].as<const char*>());
    }
    Serial.printf(">>> LỆNH: ĐẶT HỆ SỐ BÙ GÓC IMU (SET_YAW_SCALE) từ Backend! Tỉ lệ: %d%%\n", v);
    if (v >= 50 && v <= 150) {
      g_state.imuYawScale = (float)v / 100.0f;
    }

  } else if (strcmp(cmd, "set_strafe") == 0) {
    int v = -1;
    if (doc["payload"].is<int>()) {
      v = doc["payload"].as<int>();
    } else if (doc["payload"].is<const char*>()) {
      v = atoi(doc["payload"].as<const char*>());
    }
    Serial.printf(">>> LỆNH: ĐẶT TRƯỢT NGANG (SET_STRAFE) từ Backend! %d%%\n", v);
    if (v >= 0 && v <= 100) {
      g_state.cmdStrafe = (int16_t)constrain((int)(v - 50) * 2, -100, 100);
    }

  } else if (strcmp(cmd, "test_motor") == 0) {
    const char *payloadStr = doc["payload"] | "";
    int slot = -1;
    int speedPct = 0;
    if (sscanf(payloadStr, "%d_%d", &slot, &speedPct) == 2) {
      robotForceManualStop(); // Chuyển về lái tay và dừng các động cơ khác
      if (slot >= 0 && slot < 4) {
        int32_t speedVal = (int32_t)PWM_MAX * speedPct / 100;
        int32_t sp[4] = {0, 0, 0, 0};
        sp[slot] = speedVal;
        motorApplyLayout(sp);
        Serial.printf(">>> LỆNH: TEST ĐỘNG CƠ SLOT %d (chỉ số %d) ở tốc độ %d%%\n", slot, slot, speedPct);
      }
    }

  } else if (strcmp(cmd, "navigate") == 0) {
    if (millis() < BOOT_GUARD_MS) {
      Serial.println(F("[MQTT WARNING] Bỏ qua lệnh navigate do đang trong thời gian Boot Guard."));
      return;
    }
    const char *wpJson = doc["payload"] | "{}";
    Serial.printf(">>> LỆNH: TỰ HÀNH LỘ TRÌNH (NAVIGATE) từ Backend!\nWaypoints: %s\n", wpJson);
    if (!wpNavParseAndStart(wpJson)) {
      Serial.println(F("[MQTT ERROR] Lỗi parse hoặc start lộ trình Waypoints!"));
    }

  } else {
    Serial.printf("[MQTT WARNING] Lệnh không xác định: %s\n", cmd);
  }
  Serial.println("-------------------------------\n");
}

/* ==================== INIT ========================================= */
static void mqttInit() {
#if MQTT_USE_TLS
  /* HiveMQ Cloud dùng cert CA chính thống (Let's Encrypt) — setInsecure() đủ cho demo.
   * Nếu muốn verify đúng: g_wifiClient.setCACert(HIVEMQ_ROOT_CA_PEM); */
  g_wifiClient.setInsecure();
  Serial.println(F("[MQTT] TLS mode: WiFiClientSecure (setInsecure for demo)"));
#endif
  g_mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  g_mqttClient.setCallback(mqttCallback);
  g_mqttClient.setBufferSize(1024);
  Serial.printf("[MQTT] Init: broker=%s:%d  clientId=%s  TLS=%d\n",
                MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_CLIENT_ID, MQTT_USE_TLS);
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
    ok = g_mqttClient.connect(MQTT_CLIENT_ID); // Local broker không cần auth
  }

  if (ok) {
    g_mqttClient.subscribe(MQTT_TOPIC_COMMAND);
    Serial.printf(" OK. Subscribed to topic: %s\n", MQTT_TOPIC_COMMAND);
    Serial.println(F("\n======================================================="));
    Serial.println(F("[MQTT] >>> ĐÃ KẾT NỐI VỚI BACKEND THÀNH CÔNG! <<<"));
    Serial.println(F("[MQTT] Sẵn sàng nhận lệnh từ: https://interiorly-pinnatisect-adalyn.ngrok-free.dev/"));
    Serial.println(F("=======================================================\n"));
    /* Publish online status ngay khi kết nối */
    strncpy((char *)g_mqttPendingStatus, "online", sizeof(g_mqttPendingStatus) - 1);
    g_mqttStatusPending = true;
  } else {
    Serial.printf(" FAIL rc=%d. Sẽ tự động thử lại sau 15 giây...\n", g_mqttClient.state());
  }
}

/* ==================== AUTO-DOCKING (Phase 3.5) ===================== */
#if BAT_MONITOR_ENABLE
static bool s_dockRequested = false;

static void mqttCheckAutoDock(int batPct) {
  if (batPct <= 0) return; // ADC chưa đo được

  /* Pin yếu → hủy route hiện tại và yêu cầu backend điều hướng về trạm sạc */
  if (batPct < (int)DOCK_LOW_BAT_PCT && !s_dockRequested
      && (g_state.mode == MODE_WAYPOINT || g_state.mode == MODE_AUTO)) {
    s_dockRequested = true;
    wpNavCancel();  // Hủy route đang chạy
    strncpy((char *)g_mqttPendingStatus, "low_battery",
            sizeof(g_mqttPendingStatus) - 1);
    g_mqttStatusPending = true;
    Serial.printf("[DOCK] Battery %d%% < %d%% — requesting return to dock (Node %d).\n",
                  batPct, (int)DOCK_LOW_BAT_PCT, (int)DOCK_NODE_ID);
  }

  /* Pin đã sạc đầy → reset flag */
  if (batPct >= (int)DOCK_FULL_BAT_PCT) {
    s_dockRequested = false;
  }
}
#endif

/* ==================== PUBLISH TELEMETRY ============================ */
static void mqttPublishTelemetry() {
  if (!g_mqttClient.connected()) return;
  const uint32_t now = millis();
  if (now - g_mqttLastTelemetryMs < MQTT_TELEMETRY_MS) return;
  g_mqttLastTelemetryMs = now;

  StaticJsonDocument<512> doc;

  /* Pin — BAT_MONITOR_ENABLE=1: đọc ADC; 0: gửi -1 (N/A) */
#if BAT_MONITOR_ENABLE
  extern int g_batPct;   // Được tính từ ADC trong Sensors.h / BatteryMonitor
  int batPct = g_batPct;
  mqttCheckAutoDock(batPct);
#else
  int batPct = -1;       // Chưa có mạch đo pin
#endif

  doc["Battery"]       = batPct;
  doc["Location"]      = (const char *)nullptr;
  doc["Status"]        = "online";
  doc["CurrentNodeId"] = (const char *)nullptr;
  doc["Mode"]          = (g_state.mode == MODE_WAYPOINT) ? "waypoint"
                       : (g_state.mode == MODE_AUTO)     ? "auto" : "manual";
  doc["IsOnline"]      = true;
  doc["XCoord"]        = g_pose.x;
  doc["YCoord"]        = g_pose.y;
  doc["HeadingRad"]    = g_pose.headingRad;
  doc["Ip"]            = WiFi.localIP().toString();

  /* Sensor data */
  doc["lidarFront"] = obsFrontCm();
  doc["lidarRear"]  = obsBackCm();
#if USE_HC_SR04_HARDWARE
  doc["usLF"] = g_state.usLF;
  doc["usLR"] = g_state.usLR;
  doc["usRF"] = g_state.usRF;
  doc["usRR"] = g_state.usRR;
#endif
  doc["rpmFL"]      = g_state.rpmFL;
  doc["rpmFR"]      = g_state.rpmFR;
  doc["rpmRL"]      = g_state.rpmRL;
  doc["rpmRR"]      = g_state.rpmRR;
  doc["navState"]   = (g_state.mode == MODE_WAYPOINT) ? "waypoint" : "reactive";
  doc["wpStatus"]   = (const char *)g_wpStatus;
  doc["autoFsm"]    = (int)g_autoFsmState;
  doc["wpIndex"]    = (g_state.mode == MODE_WAYPOINT) ? (int)s_wpIndex : -1;
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
  doc["Ip"]       = WiFi.localIP().toString();

  char buf[128];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  /* QoS-1 mô phỏng: retained=false, PubSubClient không hỗ trợ QoS1 natively
     → publish 2 lần đảm bảo broker nhận (đơn giản, đủ cho demo) */
  g_mqttClient.publish(MQTT_TOPIC_STATUS, (const uint8_t *)buf, n, false);
}

/* ==================== MAIN LOOP (gọi từ taskWebIO) ================= */
static void mqttLoop() {
#if !MQTT_ENABLE
  return;
#endif
  if (g_mqttMutex == NULL) return;
  if (xSemaphoreTake(g_mqttMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
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
    xSemaphoreGive(g_mqttMutex);
  }
}

#endif // MQTT_CLIENT_H
