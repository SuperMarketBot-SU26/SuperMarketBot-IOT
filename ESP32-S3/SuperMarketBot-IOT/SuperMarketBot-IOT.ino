/* =====================================================================
 *  SuperMarketBot-IOT.ino
 *  Robot tự hành SmartMarketBot Mini 4WD — Đồ án tốt nghiệp
 *  Board: ESP32-S3-DevKitC N16R8 (16MB Flash / 8MB Octal PSRAM)
 *
 *  Kiến trúc FreeRTOS:
 *    Core 0 → taskWebIO   : WebServer HTTP + WebSocket + broadcast telemetry
 *    Core 1 → taskControl : Đọc cảm biến, Odometry, điều khiển động cơ
 *
 *  Thư viện cần cài (Library Manager):
 *    - ESP32 Arduino core >= 3.0 (espressif/arduino-esp32)
 *    - NewPing (bắt buộc — USE_HC_SR04_HARDWARE=1, 4× HC-SR04)
 *    - WebSockets by Markus Sattler (Links2004/arduinoWebSockets)
 *    - ArduinoJson by Benoit Blanchon
 *    - Adafruit NeoPixel (WS2812 onboard)
 *    - PubSubClient by Nick O'Leary (Phase 1: MQTT)
 *
 *  Phase notes:
 *    Phase 1: MqttClient.h (MQTT + AP+STA WiFi + Nav safety fixes)
 *    Phase 2: Localization.h (Dead Reckoning), PidController.h (Speed PID)
 *    Phase 3: WaypointNav.h (Pure Pursuit waypoint navigation)
 * =====================================================================*/

#include "Config.h"
#include "Motors.h"
#include "Sensors.h"
#include "Odometry.h"
#include "PidController.h"
#include "WaypointNav.h"
#include "StatusRGB.h"
#include "CtrlJson.h"
#include "LocalObstacleAvoid.h"
#include "ObstacleSensors.h"
#include "WebUI.h"
#include "esp_heap_caps.h"

// ── In bộ nhớ lúc chạy (Serial Monitor 115200) ─────────────────────
static void printMemInfo() {
  Serial.println(F("--- Bộ nhớ (ESP, runtime) ---"));
  Serial.printf("Flash chip (tong tren VDK): %u B (~%.2f MB)\n",
                ESP.getFlashChipSize(), ESP.getFlashChipSize() / 1048576.0f);
  Serial.printf("SRAM heap con trong:  %u B  |  khoi lon nhat: %u B\n",
                (unsigned)ESP.getFreeHeap(),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  if (psramFound()) {
    Serial.printf("PSRAM: con %u / tong %u B  |  khoi lon: %u B\n",
                  (unsigned)ESP.getFreePsram(), (unsigned)ESP.getPsramSize(),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
  } else {
    Serial.println(F("PSRAM: chua dung (bat OPI PSRAM + psram o menu Board neu can)"));
  }
  Serial.println(F("Dung luong 'Sketch' khi compile = kich thuoc firmware trong partition app."));
  Serial.println();
}

// ── Định nghĩa biến toàn cục (extern trong các .h) ──────────────────
RobotState g_state = {
  .usFront = LIDAR_MAX_CM, .usBack = LIDAR_MAX_CM, .usLeft = LIDAR_MAX_CM, .usRight = LIDAR_MAX_CM,
  .usLF = LIDAR_MAX_CM, .usLR = LIDAR_MAX_CM, .usRF = LIDAR_MAX_CM, .usRR = LIDAR_MAX_CM,
  .lidarFront = LIDAR_MAX_CM, .lidarBack = LIDAR_MAX_CM,
  .rpmFL = 0, .rpmRL = 0, .rpmFR = 0, .rpmRR = 0,
  .distFL = 0, .distRL = 0, .distFR = 0, .distRR = 0,
  .cmdX = 0, .cmdY = 0, .cmdStrafe = 0,
  .baseSpeed = 0,
  .autoBaseSpeed = 0,
  .mode = MODE_MANUAL,
  .estop = false,
  .lidarLastUpdateMs = 0,
  .usLastUpdateMs = 0
};

// ── Mutex bảo vệ g_state khi đọc/ghi từ 2 core ─────────────────────
SemaphoreHandle_t g_stateMutex;
volatile bool g_usEnabled = false;  // SR04 bật khi vào mode tự hành, tắt khi lái tay

/* =====================================================================
 *  Tự hành Test cảm biến (MODE_AUTO_TEST):
 *    AT_FORWARD → gặp vật → chọn bên trống → AT_TURNING (xoay nhanh)
 *    AT_TURNING → đủ xa phía trước → AT_FORWARD
 *    AT_BACKUP  → sau chặn khi đang lùi
 *
 *  SR04 bật khi g_usEnabled = true (mode tự hành).
 * =================================================================== */

static OaContext s_autoOa;

/** FSM test cảm biến: AT_FORWARD → gặp vật → chọn bên → AT_TURNING → lại FORWARD */
enum AutoTestFsm : uint8_t {
  AT_FORWARD = 0,  // Đi thẳng
  AT_TURNING = 1,  // Xoay tìm đường
  AT_BACKUP  = 2   // Lùi an toàn
};

static AutoTestFsm s_auto_fsm = AT_FORWARD;
volatile uint8_t g_autoFsmState = 0;  /* expose → WebSocket field "afs" */
static uint32_t s_auto_t0 = 0;
/* Phase 1 — Stuck detection */
static uint32_t s_stuckCheckMs = 0;
static float    s_stuckLastDist = 0.f;
static uint8_t  s_stuckCount = 0;
static RobotMode s_ctrl_prev_mode = MODE_MANUAL;

static uint16_t autoSpeedPwm() {
  uint16_t s = g_state.autoBaseSpeed;
  if (s == 0) s = g_state.baseSpeed;
  uint16_t cap = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)AUTO_CRUISE_SPEED_PCT / 100u);
  if (s > cap) s = cap;
  uint16_t lo = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)AUTO_MIN_PWM_FRAC / 100u);
  if (s < lo) s = lo;
  return s;
}

/* Timestamp cho PID CRUISE */
static uint32_t s_pidLastMs = 0;

/** Heading lưu lại khi xoay tìm đường */
static float s_testBaseHeading = 0.f;

/**
 * MODE_AUTO_TEST — Test cảm biến (Roomba-style):
 *
 *  AT_FORWARD: đi thẳng + heading-hold. Gặp vật trước → chọn bên trống
 *              hơn → chuyển AT_TURNING. Sau chặn → AT_BACKUP.
 *
 *  AT_TURNING: xoay tại chỗ nhanh sang bên đã chọn. Khi trước đủ xa
 *              (≥ PATH_CLEAR_MIN_CM) → quay về AT_FORWARD.
 *
 *  AT_BACKUP:  lùi an toàn. Khi sau thông thoáng → AT_FORWARD.
 */
static void autoNavigateAvoidance() {
  const uint16_t spd = autoSpeedPwm();
  const uint32_t now = millis();
  const int16_t f = obsFrontCm();
  const int16_t b = obsBackCm();
  const int16_t l = obsLeftCm();
  const int16_t r = obsRightCm();

  g_autoFsmState = (uint8_t)s_auto_fsm;

  switch (s_auto_fsm) {

    /* ── AT_FORWARD: đi thẳng + heading hold ────────────────────────── */
    case AT_FORWARD: {
      if (obsFrontBlocked() || obsRearBlocked()) {
        botStop();
        pidSpeedReset();
        break;
      }

      /* Gặp vật cản → chọn bên trống hơn → xoay */
      if (obsOaTriggered(f)) {
        bool leftOk  = obsCmValid(l) && l >= (int16_t)US_PATH_CLEAR_CM;
        bool rightOk = obsCmValid(r) && r >= (int16_t)US_PATH_CLEAR_CM;

        int8_t turnDir = 0;
        if (leftOk && rightOk) {
          turnDir = (r >= l) ? 1 : -1;
        } else if (rightOk) {
          turnDir = 1;
        } else if (leftOk) {
          turnDir = -1;
        } else {
          s_auto_fsm = AT_BACKUP;
          s_auto_t0 = now;
          botStop();
          Serial.println(F("[AUTO-TEST] Both sides blocked — backup."));
          break;
        }

        s_testBaseHeading = oaNorm(g_pose.headingRad + (float)turnDir * ((float)M_PI / 2.f));
        s_auto_fsm = AT_TURNING;
        s_auto_t0 = now;
        pidSpeedReset();
        Serial.printf("[AUTO-TEST] Obstacle %dcm — turn %s (L=%d R=%d)\n",
                      (int)f, turnDir > 0 ? "RIGHT" : "LEFT", (int)l, (int)r);
        break;
      }

      /* Heading-hold: bù drift để đi thẳng */
      float dt_s = (float)SAFE_LOOP_MS * 0.001f;
      float holdCorrection = pidHoldCompute(g_pose.headingRad, g_pose.headingRad, dt_s);

      /* Speed PID */
      float pidOut = pidSpeedCompute(pwmToEstMps(spd), robotActualSpeedMps(), dt_s);
      int32_t run = (int32_t)spd + (int32_t)pidOut;
      if (run < 0) run = 0;
      if (run > (int32_t)PWM_MAX) run = (int32_t)PWM_MAX;

      int16_t turn = (int16_t)holdCorrection;
      if (turn >  100) turn =  100;
      if (turn < -100) turn = -100;
      botDriveMecanum(0, 80, turn, (uint16_t)run);
      break;
    }

    /* ── AT_TURNING: xoay tìm đường ──────────────────────────────── */
    case AT_TURNING: {
      /* Sau chặn → dừng xoay, backup */
      if (obsRearBlocked()) {
        botStop();
        s_auto_fsm = AT_BACKUP;
        s_auto_t0 = now;
        Serial.println(F("[AUTO-TEST] Rear blocked while turning — backup."));
        break;
      }

      /* Đủ xa phía trước → quay về đi thẳng */
      if (obsPathClear(f)) {
        s_auto_fsm = AT_FORWARD;
        pidSpeedReset();
        pidHoldReset();
        Serial.printf("[AUTO-TEST] Path clear (%dcm) — resume forward.\n", (int)f);
        break;
      }

      /* Timeout xoay 5s → thử bên kia */
      if (now - s_auto_t0 >= 5000u) {
        s_testBaseHeading = oaNorm(s_testBaseHeading + (float)M_PI);
        s_auto_t0 = now;
        Serial.println(F("[AUTO-TEST] Turn timeout — try opposite direction."));
      }

      /* Xoay tại chỗ nhanh về hướng đã chọn */
      float targetH = s_testBaseHeading;
      float err = oaAngleDiff(targetH, g_pose.headingRad);
      if (fabsf(err) > 0.15f) {
        uint16_t turnPwm = oaPct2Pwm(40);
        if (err > 0) botRotateCW(turnPwm);
        else         botRotateCCW(turnPwm);
      } else {
        /* Đã quay đúng hướng, vẫn tiếp tục cho đến khi đủ xa */
        botStop();
      }
      break;
    }

    /* ── AT_BACKUP: lùi an toàn rồi quay lại ──────────────── */
    case AT_BACKUP: {
      if (now - s_auto_t0 < AUTO_BACKUP_REVERSE_MS) {
        uint16_t bkSpd = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)AUTO_BACKUP_SPEED_PCT / 100u);
        botBackward(bkSpd);
        break;
      }
      if (!obsRearBlocked()) {
        botStop();
        s_auto_fsm = AT_FORWARD;
        pidSpeedReset();
        pidHoldReset();
        Serial.println(F("[AUTO-TEST] Rear clear — resume forward."));
        break;
      }
      /* Sau vẫn chặn sau timeout dài → đảo hướng thử lại */
      if (now - s_auto_t0 >= AUTO_BACKUP_REVERSE_MS * 4u) {
        botStop();
        s_testBaseHeading = oaNorm(g_pose.headingRad + (float)M_PI);
        s_auto_fsm = AT_FORWARD;
        pidSpeedReset();
        pidHoldReset();
        Serial.println(F("[AUTO-TEST] Rear blocked long — flip heading."));
      } else {
        botStop();
      }
      break;
    }

    default:
      s_auto_fsm = AT_FORWARD;
      break;
  }
}

/* =====================================================================
 *  TASK CORE 1 — Điều khiển real-time (cảm biến + động cơ)
 *  Ưu tiên cao để đảm bảo tính thời gian thực
 * =================================================================== */
static void taskControl(void *pvParams) {
  const TickType_t xPeriod = pdMS_TO_TICKS(SAFE_LOOP_MS);
  TickType_t xLastWake = xTaskGetTickCount();

  TickType_t odomTick = xTaskGetTickCount();
  TickType_t usTick   = xTaskGetTickCount();

  while (true) {
    /* 12s đầu: ép MANUAL — tránh MQTT/backend hoặc mode cũ khiến robot tự chạy */
    if (millis() < BOOT_GUARD_MS) {
      if (g_state.mode != MODE_MANUAL) {
        robotForceManualStop();
      } else       if (g_state.cmdX != 0 || g_state.cmdY != 0 || g_state.cmdStrafe != 0) {
        g_state.cmdX = g_state.cmdY = g_state.cmdStrafe = 0;
        botStop();
      }
    }

    if (g_state.mode == MODE_AUTO_TEST && s_ctrl_prev_mode != MODE_AUTO_TEST) {
      g_state.estop = false;
      g_usEnabled = true;
      s_auto_fsm = AT_FORWARD;
      s_auto_t0 = millis();
      s_stuckCount = 0;
      oaReset(s_autoOa);
      s_testBaseHeading = g_pose.headingRad;
      pidSpeedReset();
      pidHoldReset();
      Serial.println(F("[AUTO-TEST] Mode entered — SR04 enabled, forward."));
      strncpy((char *)g_mqttPendingStatus, "auto_test", sizeof(g_mqttPendingStatus) - 1);
      g_mqttStatusPending = true;
    }
    if (g_state.mode != MODE_AUTO_TEST && s_ctrl_prev_mode == MODE_AUTO_TEST) {
      g_usEnabled = false;
      s_auto_fsm = AT_FORWARD;
      oaReset(s_autoOa);
      Serial.println(F("[AUTO-TEST] Mode exited — SR04 disabled."));
      strncpy((char *)g_mqttPendingStatus, "manual", sizeof(g_mqttPendingStatus) - 1);
      g_mqttStatusPending = true;
    }
    /* WAYPOINT: bật SR04 khi cần, tắt khi ra */
    if (g_state.mode == MODE_WAYPOINT && s_ctrl_prev_mode != MODE_WAYPOINT) {
      g_usEnabled = true;
      s_wpFsm = WP_ROUTE_SET;
      s_auto_fsm = AT_FORWARD;
      oaReset(s_wpOa);
      botStop();
      strncpy(g_wpStatus, "route_set", sizeof(g_wpStatus) - 1);
      Serial.println(F("[WP] Mode entered — SR04 enabled, waiting for route from backend."));
    }
    if (g_state.mode != MODE_WAYPOINT && s_ctrl_prev_mode == MODE_WAYPOINT) {
      g_usEnabled = false;
      Serial.println(F("[WP] Mode exited — SR04 disabled."));
    }
    s_ctrl_prev_mode = g_state.mode;

    /* ── Sensor Fusion: poll cả hai hệ thống mỗi vòng loop ─── */
    /* LiDAR (100Hz) — rất nhanh, đọc UART */
    sensorsPollLidar();
    if (wpNavOaActive()) { sensorsPollLidar(); sensorsPollLidar(); }

    /* HC-SR04 (tầm gần) — tuần tự 4 góc, chậm hơn */
    sensorsPollUS();
    if (wpNavOaActive()) { sensorsPollUS(); }

    /* Debug: log cả LiDAR + SR04 mỗi 2 giây */
    static uint32_t s_debugSensorMs = 0;
    if (now - s_debugSensorMs >= 2000u) {
      s_debugSensorMs = now;
      Serial.printf("LiDAR F:%d B:%d | US F:%d B:%d L:%d R:%d\n",
                    (int)g_state.lidarFront, (int)g_state.lidarBack,
                    (int)g_state.usFront,   (int)g_state.usBack,
                    (int)g_state.usLeft,    (int)g_state.usRight);
    }

    if ((xTaskGetTickCount() - odomTick) >= pdMS_TO_TICKS(ODOM_PERIOD_MS)) {
      odomUpdate();
      odomTick = xTaskGetTickCount();
    }

    if (g_state.estop) {
      botStop();
      wpNavStop();
      if (g_state.cmdX == 0 && g_state.cmdY == 0 && g_state.cmdStrafe == 0) g_state.estop = false;
    } else if (g_state.mode == MODE_WAYPOINT) {
      wpNavTick();
    } else if (g_state.mode == MODE_AUTO_TEST) {
      autoNavigateAvoidance();
    } else {
      if (g_state.cmdX == 0 && g_state.cmdY == 0 && g_state.cmdStrafe == 0) {
        botStop();
      } else {
        botDriveMecanum(g_state.cmdStrafe, g_state.cmdY, g_state.cmdX, g_state.baseSpeed);
      }
    }

    vTaskDelayUntil(&xLastWake, xPeriod);
  }
}

/* =====================================================================
 *  TASK CORE 0 — Web IO (HTTP + WebSocket)
 * =================================================================== */
static void taskWebIO(void *pvParams) {
  const TickType_t broadcastPeriod = pdMS_TO_TICKS(WEB_WS_PERIOD_MS);
  TickType_t lastBroadcast = xTaskGetTickCount();

  static uint32_t lastRgbMs = 0;
  const uint32_t rgbPeriod = 35; // ~28 Hz, du mượt cho breathing

  while (true) {
    webUILoop();  // handleClient + ws.loop()

    if ((xTaskGetTickCount() - lastBroadcast) >= broadcastPeriod) {
      webUIBroadcast();
      lastBroadcast = xTaskGetTickCount();
    }
    if (millis() - lastRgbMs >= rgbPeriod) {
      lastRgbMs = millis();
      statusRgbUpdate();
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // nhường CPU ngắn
  }
}

/* =====================================================================
 *  setup() — Chạy trên Core 1 (Arduino default)
 * =================================================================== */
void setup() {
  Serial.begin(115200);
  // USB CDC trên S3 xuất hiện sau vài trăm ms — delay giúp log app không lẫn với boot ROM
  delay(300);
  Serial.println();
  Serial.println(F("== SmartMarketBot booting =="));

  // ── Phần cứng ────────────────────────────────────────────────────
  motorsInit();
  robotForceManualStop();  /* Bật nguồn = luôn dừng, lái tay — không tự Auto/Waypoint */
  sensorsInit();
  sensorsLogBootSample();
  odomInit();
  locInit();

  // LED RGB nội bộ (DevKitC-1: GPIO 38) — sau odom
  statusRgbInit();
#if SMB_ONBOARD_RGB && (SMB_NEOPIXEL_PIN == ENC_RR)
  Serial.println(F("[LED] NeoPixel and ENC_RR same pin: RR encoder ISR disabled."));
#endif

  // ── WiFi + Web ───────────────────────────────────────────────────
  webUIInit();

  // ── Mutex ────────────────────────────────────────────────────────
  g_stateMutex = xSemaphoreCreateMutex();

  // ── Tạo FreeRTOS tasks ───────────────────────────────────────────
  // Core 0: Web IO — stack 8KB, priority 1
  xTaskCreatePinnedToCore(
    taskWebIO, "WebIO",
    10240, nullptr, 1,
    nullptr, 0
  );

  // Core 1: Điều khiển — stack 6KB, priority 5 (real-time)
  xTaskCreatePinnedToCore(
    taskControl, "Control",
    6144, nullptr, 5,
    nullptr, 1
  );

  Serial.println(F("[Boot] Tasks created. Robot ready!"));
  Serial.printf("[Boot] Dashboard:  http://%s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("[Boot] Camera:     https://%s/vision\n", WiFi.softAPIP().toString().c_str());
#if WIFI_STA_ENABLE
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[Boot] STA IP:     %s  (MQTT broker: %s:%d)\n",
                  WiFi.localIP().toString().c_str(), MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  } else {
    Serial.println(F("[Boot] STA: CHUA ket noi — MQTT disabled"));
  }
#endif
  printMemInfo();
}

/* =====================================================================
 *  loop() — Không dùng (FreeRTOS quản lý toàn bộ)
 *  Để trống hoặc bỏ idle task vào đây nếu cần debug
 * =================================================================== */
void loop() {
  vTaskDelay(portMAX_DELAY);
}
