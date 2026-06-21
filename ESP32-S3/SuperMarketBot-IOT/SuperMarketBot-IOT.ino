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
  .swerveBaseSpeed = 0,
  .mode = MODE_MANUAL,
  .estop = false,
  .lidarLastUpdateMs = 0,
  .usLastUpdateMs = 0
};

// ── Mutex bảo vệ g_state khi đọc/ghi từ 2 core ─────────────────────
SemaphoreHandle_t g_stateMutex;

/** Hướng quay vật lý hiện tại của 4 động cơ (MID_FL, MID_RL, MID_FR, MID_RR)
 *  Sử dụng để ký hiệu hóa số xung đếm từ encoder không chiều. */
volatile int8_t g_motorDir[4] = {0, 0, 0, 0};

/* =====================================================================
 *  Tự hành (LiDAR trước/sau): CRUISE → chặn → STOP_HOLD → quét tìm hướng
 *  trống (xoay có ramp, CW rồi CCW tới khi đủ xa phía trước) → hãm nhẹ
 *  → CRUISE. SR04 bật thì thêm bẻ cạnh. (SLAM / bản đồ: sau này thay FSM.)
 * =================================================================== */
enum AutoNavFsm : uint8_t {
  AN_CRUISE = 0,
  AN_BACKUP,   // Lùi khi OA blocked hoàn toàn
  AN_SPIN_SEARCH // Xoay tìm hướng thoát khi kẹt 4 phía
};

static OaContext s_autoOa;

static AutoNavFsm s_auto_fsm = AN_CRUISE;
volatile uint8_t g_autoFsmState = 0;  /* expose → WebSocket field "afs" */
static uint32_t s_auto_t0 = 0;
volatile uint32_t s_settleUntilMs = 0; // Trễ ổn định sau khi đổi hướng/trạng thái
/* Phase 1 — Stuck detection */
static uint32_t s_stuckCheckMs = 0;
static float    s_stuckLastDist = 0.f;
static uint8_t  s_stuckCount = 0;
static RobotMode s_ctrl_prev_mode = MODE_MANUAL;

uint16_t autoSpeedPwm() {
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

static void autoNavigateAvoidance() {
  const uint16_t spd = autoSpeedPwm();
  const uint32_t now = millis();
  const int16_t f = obsFrontCm();
  const int16_t b = obsBackCm();

  /* Telemetry: 0=CRUISE, 10+=OA sub-state */
  g_autoFsmState = (s_autoOa.state == OA_IDLE) ? (uint8_t)s_auto_fsm
                   : (uint8_t)(10 + (uint8_t)s_autoOa.state);

  /* ── Local OA đang chạy (quét → lách → vượt) ─────────────────── */
  if (s_autoOa.state != OA_IDLE) {
    OaTickResult r = oaTick(s_autoOa, f, now);
    if (r == OA_RES_DONE) {
      s_auto_fsm = AN_CRUISE;
      s_settleUntilMs = now + 450;
      usFilterReset();
      Serial.println(F("[AUTO] OA done — cruise (duong truoc du xa). Settle 450ms."));
    } else if (r == OA_RES_BLOCKED) {
      s_auto_fsm = AN_BACKUP;
      s_auto_t0 = now;
      oaReset(s_autoOa); // Reset s_autoOa state to OA_IDLE immediately so we can execute AN_BACKUP!
      Serial.println(F("[AUTO] OA blocked — backup."));
    }
    return;
  }

  switch (s_auto_fsm) {
    case AN_CRUISE: {
      if (s_settleUntilMs > 0) {
        if (now < s_settleUntilMs) {
          botStop();
          pidSpeedReset();
          break;
        } else {
          s_settleUntilMs = 0;
          Serial.println(F("[AUTO] Settle delay done — bat dau di tiep."));
        }
      }

      if (obsFrontBlocked()) {
        Serial.println(F("[AUTO] Phia truoc bi chan sat nut -> Chuyen sang AN_BACKUP de lui lai."));
        botStop();
        pidSpeedReset();
        s_auto_fsm = AN_BACKUP;
        s_auto_t0 = now;
        break;
      }

      /* Gặp vật → quét 2 bên, cần ≥1m mới lách (LocalObstacleAvoid.h) */
      if (obsOaTriggered(f)) {
        s_autoOa.cruiseHeading = g_pose.headingRad;
        if (oaBegin(s_autoOa, f, now)) {
          return;
        }
      }

      /* Chỉ tiến khi phía trước ≥ PATH_CLEAR_MIN_CM (1m) ổn định */
      if (!oaCruiseForward(s_autoOa, f, spd)) {
        if (!obsPathClear(f)) {
          s_autoOa.cruiseHeading = g_pose.headingRad;
          oaBegin(s_autoOa, f, now);
        }
      }
      break;
    }
    case AN_BACKUP: {
      /* Lùi an toàn: kiểm tra Luna sau trước khi lùi */
      if (obsRearBlocked()) {
        /* Sau cũng chặn -> Chuyển sang xoay tại chỗ tìm hướng thoát! */
        s_auto_fsm = AN_SPIN_SEARCH;
        s_auto_t0 = now;
        botStop();
        Serial.println(F("[AUTO] Sau cung bi chan -> Xoay tai cho tim huong thoat."));
        break;
      }
      uint16_t bkSpd = g_state.swerveBaseSpeed;
      if (bkSpd == 0) bkSpd = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)AUTO_BACKUP_SPEED_PCT / 100u);
      botBackward(bkSpd);
      if (now - s_auto_t0 >= AUTO_BACKUP_REVERSE_MS) {
        botStop();
        oaReset(s_autoOa);
        usFilterReset();
        s_auto_fsm = AN_CRUISE;
        s_auto_t0 = now;
        s_settleUntilMs = now + 450;
        s_stuckCount = 0;
        Serial.println(F("[AUTO] Backup done -> Settle 450ms."));
      }
      break;
    }
    case AN_SPIN_SEARCH: {
      // Xoay tại chỗ tìm hướng thoát
      uint16_t spinSpd = g_state.swerveBaseSpeed;
      if (spinSpd == 0) spinSpd = oaPct2Pwm(OA_SCAN_SPEED_PCT);
      botRotateCW(spinSpd);
      // Nếu hướng trước mặt đã thông thoáng
      if (obsPathClear(f)) {
        botStop();
        oaReset(s_autoOa);
        usFilterReset();
        s_auto_fsm = AN_CRUISE;
        s_auto_t0 = now;
        s_settleUntilMs = now + 450;
        Serial.println(F("[AUTO] Tim thay loi thoat khi xoay quet -> Settle 450ms."));
      }
      // Giới hạn xoay tối đa 6s tránh xoay vòng vô hạn nếu bị nhốt kín
      if (now - s_auto_t0 >= 6000u) {
        botStop();
        oaReset(s_autoOa);
        usFilterReset();
        s_auto_fsm = AN_CRUISE;
        s_auto_t0 = now;
        s_settleUntilMs = now + 450;
        Serial.println(F("[AUTO] Het 6s spin search -> Settle 450ms."));
      }
      break;
    }
    default:
      s_auto_fsm = AN_CRUISE;
      break;
  }

  /* ── Stuck detection (chạy CRUISE mà không tiến được) ───────────── */
  if (s_auto_fsm == AN_CRUISE && g_state.mode == MODE_AUTO) {
    if (now - s_stuckCheckMs >= STUCK_CHECK_INTERVAL_MS) {
      s_stuckCheckMs = now;
      float currDist = g_state.distFL + g_state.distFR;
      float currentPwm = (float)autoSpeedPwm();
      if (currentPwm >= (float)STUCK_MIN_PWM) {
        float moved = currDist - s_stuckLastDist;
        if (moved < 0.f) moved = -moved;
        if (moved < 0.001f) {
          s_stuckCount++;
          if (s_stuckCount >= STUCK_THRESHOLD) {
            s_stuckCount = 0;
            s_auto_fsm = AN_BACKUP;
            s_auto_t0 = now;
            botStop();
          }
        } else {
          s_stuckCount = 0;
        }
      }
      s_stuckLastDist = currDist;
    }
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
      } else if (g_state.cmdX != 0 || g_state.cmdY != 0 || g_state.cmdStrafe != 0) {
        g_state.cmdX = g_state.cmdY = g_state.cmdStrafe = 0;
        botStop();
      }
    }

    if (g_state.mode == MODE_AUTO && s_ctrl_prev_mode != MODE_AUTO) {
      g_state.estop = false;
      s_auto_fsm = AN_CRUISE;
      s_auto_t0 = millis();
      s_pidLastMs = millis();
      s_stuckCount = 0;
      oaReset(s_autoOa);
      s_autoOa.cruiseHeading = g_pose.headingRad;
      pidSpeedReset();
      usFilterReset();
      s_settleUntilMs = millis() + 450;
      /* Báo backend chuyển mode */
      strncpy((char *)g_mqttPendingStatus, "auto", sizeof(g_mqttPendingStatus) - 1);
      g_mqttStatusPending = true;
    }
    if (g_state.mode != MODE_AUTO && s_ctrl_prev_mode == MODE_AUTO) {
      s_auto_fsm = AN_CRUISE;
      oaReset(s_autoOa);
      strncpy((char *)g_mqttPendingStatus, "manual", sizeof(g_mqttPendingStatus) - 1);
      g_mqttStatusPending = true;
    }
    s_ctrl_prev_mode = g_state.mode;

#if USE_LIDAR_HARDWARE
    sensorsPollLidar();
#endif

    sensorsPollUS();

    if ((xTaskGetTickCount() - odomTick) >= pdMS_TO_TICKS(ODOM_PERIOD_MS)) {
      odomUpdate();
      odomTick = xTaskGetTickCount();
    }

    if (g_state.estop) {
      botStop();
      wpNavStop();   // Cũng abort waypoint nav nếu đang chạy
      if (g_state.cmdX == 0 && g_state.cmdY == 0) g_state.estop = false;
    } else if (g_state.mode == MODE_WAYPOINT) {
      wpNavTick();
    } else if (g_state.mode == MODE_AUTO) {
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
  // Core 0: Web IO — stack 10KB, priority 2 (tăng để WiFi/WS được xử lý kịp)
  xTaskCreatePinnedToCore(
    taskWebIO, "WebIO",
    10240, nullptr, 2,
    nullptr, 0
  );

  // Core 1: Điều khiển — stack 8KB, priority 5 (real-time)
  xTaskCreatePinnedToCore(
    taskControl, "Control",
    8192, nullptr, 5,
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
