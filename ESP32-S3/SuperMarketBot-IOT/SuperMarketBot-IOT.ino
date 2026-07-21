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
#include <esp_task_wdt.h>
#include "CtrlJson.h"
#include "LocalObstacleAvoid.h"
#include "ObstacleSensors.h"
#include "MqttClient.h"
#include "YdlidarX3.h"       // ← YDLidar X3 driver (SLAM + localization + obstacle backup)
#include "WebUI.h"
#include "LineSensor.h"      // Phase 9 — TCRT5000 8-ch line sensor
#include "LineDecoder.h"     // Phase 9 — line state machine + steering PID
#include "LidarStreamWS.h"   // ← Stream LiDAR thô sang Tablet (port 82)
#include "ImuMpu6050.h"      // ← Đọc góc xoay từ MPU6050
#include "MotorTrim.h"       // ← NV1c — Auto-calibrate motor trim dựa trên yaw drift
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
#undef Serial
LoggerSerial logger(Serial0);
#define Serial logger
QueueHandle_t g_logQueue = NULL;

// Waypoint Navigation globals
Waypoint            s_wpRoute[WP_MAX_WAYPOINTS];
volatile uint8_t    s_wpCount = 0;
volatile uint8_t    s_wpIndex = 0;
volatile WpFsmState s_wpFsm   = WP_IDLE;
volatile uint32_t   s_wpT0    = 0;
volatile uint32_t   s_wpObstHoldStart = 0;
OaContext            s_wpOa;
volatile uint32_t   s_wpSettleUntilMs = 0;

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
  .rotateBaseSpeed = 0,
  .imuYawScale = 1.0f,
  .mode = MODE_MANUAL,
  .estop = false,
  .lidarLastUpdateMs = 0,
  .usLastUpdateMs = 0,
  .alignThresholdDeg = 10.0f,
  .rotateSpeedMinPct = 10,
  .usStopCm = 30,
  .usOaDetectCm = 42,
  .usPathClearCm = 35,            // Hành lang hẹp: giảm từ 48→35cm để tránh OA trigger liên tục
  .usPathClearStreak = 18,
  .yawKp = 40.0f,
  .yawKi = 0.0f,
  .yawKd = 2.0f,
  .leftMotorScale = LEFT_MOTOR_SCALE_DEFAULT,
  .rightMotorScale = RIGHT_MOTOR_SCALE_DEFAULT
};

// ── Mutex bảo vệ g_state khi đọc/ghi từ 2 core ─────────────────────
SemaphoreHandle_t g_stateMutex;
SemaphoreHandle_t g_mqttMutex;

/** Hướng quay vật lý hiện tại của 4 động cơ (MID_FL, MID_RL, MID_FR, MID_RR)
 *  Sử dụng để ký hiệu hóa số xung đếm từ encoder không chiều. */
volatile int8_t g_motorDir[4] = {0, 0, 0, 0};

// ── Line Sensor & Decoder Global Definitions (Phase 9) ─────────────
LineState        g_lineState;
float            g_lineOffsetEMA = 0.0f;
float            g_lineOffsetVariance = 0.0f;
LineDecoderState g_ldState = LD_IDLE;
uint32_t         g_ldStateEnterMs = 0;
uint32_t         g_lostSinceMs = 0;
uint8_t          g_lineSpeedPct = 60;   // 0..100 — slider mode LINE (lưu NVS)


/* =====================================================================
 *  Tự hành (AUTO MODE) — Reactive obstacle avoidance, đơn giản và rõ ràng.
 *
 *  3 trạng thái:
 *    AN_CRUISE       — đi thẳng giữ heading bằng IMU
 *    AN_BACKUP       — lùi khi sát vật trước (front ≤ stopCm)
 *    AN_SPIN_SEARCH  — xoay tại chỗ CW cho tới khi front đủ xa
 *
 *  Mỗi trạng thái đều giữ heading bằng IMU + PID Yaw khi có movement.
 *  Đây là bản viết lại đơn giản, thay thế toàn bộ FSM phức tạp trước.
 * =================================================================== */
enum AutoNavFsm : uint8_t {
  AN_CRUISE = 0,
  AN_BACKUP,
  AN_SPIN_SEARCH
};

static AutoNavFsm s_auto_fsm = AN_CRUISE;
static uint32_t    s_auto_t0 = 0;
volatile uint8_t   g_autoFsmState = 0;
volatile uint32_t  s_settleUntilMs = 0;

/* Biến OA cũ — giữ để tương thích với WebUI (nếu có telemetry field liên quan),
   nhưng autoNavigateAvoidance() đơn giản mới KHÔNG dùng đến nó nữa. */
OaContext g_oaCtx;

/** Lấy PWM cruise từ cấu hình. (extern trong LocalObstacleAvoid.h) */
uint16_t autoSpeedPwm() {
  uint16_t s = g_state.autoBaseSpeed;
  if (s == 0) s = g_state.baseSpeed;
  uint16_t cap = (uint16_t)((uint32_t)PWM_MAX * AUTO_CRUISE_SPEED_PCT / 100u);
  if (s > cap) s = cap;
  uint16_t lo = (uint16_t)((uint32_t)PWM_MAX * AUTO_MIN_PWM_FRAC / 100u);
  if (s < lo) s = lo;
  return s;
}

/** AUTO mode handler — đơn giản: giữ heading IMU + né vật cản US. */
static void autoNavigateAvoidance() {
  const uint32_t now = millis();
  const int16_t  f   = obsFrontCm();
  const uint16_t spd = autoSpeedPwm();

  /* Expose FSM ra WebSocket: 0=CRUISE, 1=BACKUP, 2=SPIN_SEARCH */
  g_autoFsmState = (uint8_t)s_auto_fsm;

  switch (s_auto_fsm) {
    case AN_CRUISE: {
      /* 1) Vật cản quá gần → dừng + lùi */
      if (obsFrontBlocked()) {
        botStop();
        pidYawReset();
        s_auto_fsm = AN_BACKUP;
        s_auto_t0  = now;
        Serial.println(F("[AUTO] Front blocked → BACKUP"));
        break;
      }

      /* 2) Vật cản trong vùng detect (≤ usOaDetectCm) → xoay tìm hướng thoát */
      if (obsOaTriggered(f)) {
        botStop();
        pidYawReset();
        s_auto_fsm = AN_SPIN_SEARCH;
        s_auto_t0  = now;
        Serial.println(F("[AUTO] Obstacle detected → SPIN_SEARCH"));
        break;
      }

      /* 3) Đường trống → đi thẳng + giữ heading */
      {
        static float s_targetHeading = 0.f;
        static bool  s_haveHeading   = false;

        /* Heading đã bị reset khi vừa chuyển sang AUTO hoặc sau OA → lấy lại 1 lần */
        if (!s_haveHeading) {
          s_targetHeading = g_pose.headingRad;
          s_haveHeading   = true;
        }

        /* Nếu drift quá lớn (≥ 25°) do OA → re-lock heading */
        float dh = wpNormalizeAngle(g_pose.headingRad - s_targetHeading);
        if (fabsf(dh) > 0.436f) {  // 25°
          s_targetHeading = g_pose.headingRad;
          pidYawReset();
        }

        float dt_s = (float)SAFE_LOOP_MS / 1000.f;
        float steer = pidYawCompute(s_targetHeading, g_pose.headingRad, dt_s);
        steer = constrain(steer, -85.f, 85.f);
        botDrive((int16_t)steer, 100, spd);
      }
      break;
    }

    case AN_BACKUP: {
      /* Lùi thẳng (giữ heading) trong AUTO_BACKUP_REVERSE_MS */
      {
        static float s_backHeading = 0.f;
        static bool  s_backHave    = false;
        if (!s_backHave) {
          s_backHeading = g_pose.headingRad;
          s_backHave    = true;
          pidYawReset();
        }
        float dt_s = (float)SAFE_LOOP_MS / 1000.f;
        float steer = pidYawCompute(s_backHeading, g_pose.headingRad, dt_s);
        steer = constrain(steer, -85.f, 85.f);
        botDrive((int16_t)steer, -100, spd);
      }

      if (now - s_auto_t0 >= AUTO_BACKUP_REVERSE_MS) {
        botStop();
        s_auto_fsm = AN_SPIN_SEARCH;
        s_auto_t0  = now;
        Serial.println(F("[AUTO] Backup done → SPIN_SEARCH"));
      }
      break;
    }

    case AN_SPIN_SEARCH: {
      /* Xoay CW cho tới khi đường trước thông (≥ usPathClearCm) hoặc hết timeout */
      botRotateCW(oaPct2Pwm(OA_SCAN_SPEED_PCT));

      if (obsPathClear(f)) {
        botStop();
        s_auto_fsm = AN_CRUISE;
        /* Reset heading lock cho cruise mới */
        pidYawReset();
        Serial.println(F("[AUTO] Found clear path → CRUISE"));
      }
      if (now - s_auto_t0 >= 6000u) {
        botStop();
        s_auto_fsm = AN_BACKUP;
        s_auto_t0  = now;
        Serial.println(F("[AUTO] Spin timeout 6s → BACKUP"));
      }
      break;
    }
  }
}

/* =====================================================================
 *  TASK CORE 1 — Điều khiển real-time (cảm biến + IMU + động cơ)
 *
 *  Quy trình mỗi tick (SAFE_LOOP_MS):
 *    1. Đọc IMU (heading), LiDAR, US.
 *    2. Gọi odomUpdate() mỗi ODOM_PERIOD_MS → locUpdate() tích phân pose.
 *    3. Tùy mode:
 *       - MODE_MANUAL  : lái thẳng từ joystick (cmdX/cmdY). KHÔNG heading lock.
 *                        Nếu cmdY != 0 và cmdX == 0 → bật heading lock nhẹ cho dễ lái.
 *       - MODE_AUTO    : autoNavigateAvoidance() — 3 state CRUISE/BACKUP/SPIN_SEARCH.
 *       - MODE_WAYPOINT: wpNavTick() — Pure Pursuit + OA + Align.
 *       - MODE_LINE    : lineDecoderUpdate() — TCRT5000 8-ch line tracking.
 * =================================================================== */
static void taskControl(void *pvParams) {
  const TickType_t xPeriod = pdMS_TO_TICKS(SAFE_LOOP_MS);
  TickType_t xLastWake = xTaskGetTickCount();

  TickType_t odomTick = xTaskGetTickCount();

  while (true) {
    /* ── Guard: 12s đầu luôn MANUAL, không cho tự chạy ───────────── */
    if (millis() < BOOT_GUARD_MS) {
      if (g_state.mode != MODE_MANUAL) {
        robotForceManualStop();
      } else if (g_state.cmdX != 0 || g_state.cmdY != 0 || g_state.cmdStrafe != 0) {
        g_state.cmdX = g_state.cmdY = g_state.cmdStrafe = 0;
        botStop();
      }
    }

    /* ── Transition callbacks ─────────────────────────────────────── */
    static RobotMode s_prevMode = MODE_MANUAL;
    if (g_state.mode != s_prevMode) {
      /* Vừa chuyển mode → dừng motor + reset heading lock + PID */
      botStop();
      pidYawReset();
      s_prevMode = g_state.mode;
      Serial.printf("[Mode] Switched → %s\n",
        g_state.mode == MODE_MANUAL ? "MANUAL" :
        g_state.mode == MODE_AUTO ? "AUTO" :
        g_state.mode == MODE_WAYPOINT ? "WAYPOINT" : "LINE");
    }

    /* ── 0) Line sensor read (50Hz) — chạy trước để LineDecoder có data mới ─ */
#if USE_LINE_SENSOR
    {
      uint32_t nowMs = millis();
      if (nowMs - g_state.lineLastUpdateMs >= LINE_READ_MS) {
        lineSensorUpdate();
        lineSensorPublishState();
      }
    }
#endif

    /* ── 1) IMU → heading ─────────────────────────────────────────── */
#if USE_IMU_MPU6050
    {
      float imuHeading = g_pose.headingRad;
      if (imuMpu6050Update(imuHeading)) {
        // Heading rate limiter — chống spike từ IMU noise.
        static float s_prevHeading = 0.f;
        static bool  s_firstHeading = true;
        if (!s_firstHeading) {
          float dHeading = wpNormalizeAngle(imuHeading - s_prevHeading);
          const float MAX_DHEADING = 2.5f * (float)SAFE_LOOP_MS * 0.001f;
          if (fabsf(dHeading) > MAX_DHEADING) {
            float clamped = s_prevHeading + copysignf(MAX_DHEADING, dHeading);
            imuHeading = wpNormalizeAngle(clamped);
          }
        }
        s_firstHeading = false;
        s_prevHeading  = imuHeading;
        g_pose.headingRad = imuHeading;
      }
    }
#endif

    /* ── 2) Sensors: LiDAR + US ───────────────────────────────────── */
#if USE_LIDAR_HARDWARE
    sensorsPollLidar();
#endif
#if USE_YDLIDAR_X3
    x3Poll();
#endif
    sensorsPollUS();

    /* ── 3) Odom + Localization ───────────────────────────────────── */
    if ((xTaskGetTickCount() - odomTick) >= pdMS_TO_TICKS(ODOM_PERIOD_MS)) {
      odomUpdate();
      odomTick = xTaskGetTickCount();
    }

    /* ── 4) EStop ─────────────────────────────────────────────────── */
    if (g_state.estop) {
      botStop();
      wpNavStop();
      if (g_state.cmdX == 0 && g_state.cmdY == 0) g_state.estop = false;
      vTaskDelayUntil(&xLastWake, xPeriod);
      continue;
    }

    /* ── 5) Mode dispatch ─────────────────────────────────────────── */
    switch (g_state.mode) {
      case MODE_MANUAL: {
        if (g_state.cmdX == 0 && g_state.cmdY == 0 && g_state.cmdStrafe == 0) {
          botStop();
        } else {
          /* Lái thẳng tự do. Có heading lock nhẹ khi cmdY!=0 và cmdX==0
             để robot đi thẳng không bị lệch do sai lệch cơ khí. */
          static float s_tgtH = 0.f;
          static bool  s_have = false;
          if (g_imuEnabled && g_state.cmdY != 0 && g_state.cmdX == 0 && g_state.cmdStrafe == 0) {
            if (!s_have) {
              s_tgtH = g_pose.headingRad;
              pidYawReset();
              s_have = true;
            }
            /* Nếu drift quá 25° → re-lock */
            float dh = wpNormalizeAngle(g_pose.headingRad - s_tgtH);
            if (fabsf(dh) > 0.436f) {
              s_tgtH = g_pose.headingRad;
              pidYawReset();
            }
            float dt_s = (float)SAFE_LOOP_MS / 1000.f;
            float steer = constrain(pidYawCompute(s_tgtH, g_pose.headingRad, dt_s), -85.f, 85.f);
            botDrive((int16_t)steer, g_state.cmdY, g_state.baseSpeed);
          } else {
            s_have = false;
            botDrive(g_state.cmdX, g_state.cmdY, g_state.baseSpeed);
          }
        }
        break;
      }

      case MODE_AUTO: {
        autoNavigateAvoidance();
        break;
      }

      case MODE_WAYPOINT: {
        wpNavTick();
        break;
      }

#if USE_LINE_SENSOR
      case MODE_LINE: {
        float dtS = (float)SAFE_LOOP_MS * 0.001f;
        lineDecoderUpdate(dtS);
        break;
      }
#endif
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
    lidarStreamLoop(); // ← Gửi Lidar frame sang Tablet (10 Hz, ~8KB/s)

    // Đọc hàng đợi log và truyền đi
    if (g_logQueue != NULL) {
      LogMessage logMsg;
      int processed = 0;
      while (processed < 8 && xQueueReceive(g_logQueue, &logMsg, 0) == pdTRUE) {
        processed++;
        // 1. Gửi qua WebSocket (Cục bộ)
        if (g_wsServer.connectedClients() > 0) {
          StaticJsonDocument<256> doc;
          doc["type"] = "log";
          doc["message"] = logMsg.text;
          char buf[256];
          serializeJson(doc, buf);
          g_wsServer.broadcastTXT(buf);
        }
        // 2. Gửi qua MQTT (Cloud/Backend)
        if (g_mqttMutex != NULL && xSemaphoreTake(g_mqttMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (g_mqttClient.connected()) {
            StaticJsonDocument<256> doc;
            doc["msg"] = logMsg.text;
            char buf[256];
            serializeJson(doc, buf);
            g_mqttClient.publish(MQTT_TOPIC_LOG, buf);
          }
          xSemaphoreGive(g_mqttMutex);
        }
      }
    }

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

static void taskMQTT(void *pvParams) {
  while (true) {
    #if WIFI_STA_ENABLE
    mqttLoop();
    #endif
    vTaskDelay(pdMS_TO_TICKS(100)); // Chạy tần số 10Hz
  }
}

/* =====================================================================
 *  setup() — Chạy trên Core 1 (Arduino default)
 * =================================================================== */
void setup() {
  // Vô hiệu hóa Task Watchdog toàn hệ thống để tránh bị reset do TLS handshake quá lâu
  esp_task_wdt_deinit();

  g_logQueue = xQueueCreate(64, sizeof(LogMessage));
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
  imuMpu6050Init();
#if USE_YDLIDAR_X3
  x3Init();
#endif

#if USE_LINE_SENSOR
  lineSensorInit();
  lineDecoderInit();
  g_state.lineActiveMask = 0;
  g_state.linePattern = (uint8_t)LINE_PAT_UNKNOWN;
#endif

  // LED RGB nội bộ (DevKitC-1: GPIO 38) — sau odom
  statusRgbInit();
#if SMB_ONBOARD_RGB && (SMB_NEOPIXEL_PIN == ENC_RR)
  Serial.println(F("[LED] NeoPixel and ENC_RR same pin: RR encoder ISR disabled."));
#endif

  // ── WiFi + Web ───────────────────────────────────────────────────
  webUIInit();

  // ── LiDAR Stream WebSocket (port 82 → Tablet Android) ────────────
  //    Chạy sau webUIInit() để WiFi SoftAP đã sẵn sàng.
  //    Tablet kết nối ws://192.168.4.1:82 để nhận Lidar + Pose.
  lidarStreamInit();
  lidarStreamRegisterHttpEndpoint(g_httpServer); // truyền trực tiếp, không extern

  // ── Mutex ────────────────────────────────────────────────────────
  g_stateMutex = xSemaphoreCreateMutex();
  g_mqttMutex = xSemaphoreCreateMutex();

  // ── Tạo FreeRTOS tasks ───────────────────────────────────────────
  // Core 0: Web IO — stack 10KB, priority 2 (tăng để WiFi/WS được xử lý kịp)
  xTaskCreatePinnedToCore(
    taskWebIO, "WebIO",
    10240, nullptr, 2,
    nullptr, 0
  );

  // Core 1: MQTT Task — priority 1 (chạy trên Core 1 để tránh ảnh hưởng đến sóng Wi-Fi/Bluetooth ở Core 0)
  xTaskCreatePinnedToCore(
    taskMQTT, "MQTTTask",
    8192, nullptr, 1,
    nullptr, 1
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
    Serial.printf(F("[Boot] STA IP:     %s  (MQTT broker: %s:%d)\n"),
                  WiFi.localIP().toString().c_str(), MQTT_BROKER_HOST, (int)MQTT_BROKER_PORT);
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
