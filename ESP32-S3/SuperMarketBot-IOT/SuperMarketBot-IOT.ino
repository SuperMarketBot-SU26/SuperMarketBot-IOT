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
#include "MotorControlPro.h"
#include "Sensors.h"
#include "Odometry.h"
#include "PidController.h"
#include "Localization.h"   // [Bước 5 - 2026-07-27] include trước CtrlJson.h để thấy locSetSlamPose()
#include "ImuFusion.h"          // EKF 1D heading fusion (gyro + wheel + SLAM)
#include "ImuMpu6050.h"      // ← Đọc góc xoay từ MPU6050 (must be before ImuFusion.h: imuFusion ns wraps getGyroZ)
#include "AutoExplore.h"        // Mode Tự đi quét Map (frontier exploration)
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
#if defined(USE_MICRO_ROS) && (USE_MICRO_ROS == 1)
#include "MicroRos.h"        // ← micro-ROS WiFi UDP — bridges to ROS2 agent
#endif
#include "esp_heap_caps.h"

// Forward declarations for task functions (defined below setup() / loop())
static void taskControl(void *pvParams);
static void taskWebIO(void *pvParams);
static void taskMQTT(void *pvParams);
static void taskMicroRos(void *pvParams);  // forward — called by taskWifiConnect
#if USE_YDLIDAR_X3
// taskX3 is defined inline in YdlidarX3.h — no forward declaration needed here.
#endif
// taskWifiConnect: handles WiFi STA + micro-ROS init in background on Core 1.
// Runs at priority 1 — below taskControl (5) so motor control always preempts.
// Robot is ready to drive within ~2s of boot; network features connect async.
static void taskWifiConnect(void *pvParams) {
  (void)pvParams;
  Serial.println(F("[taskWifiConnect] Started — connecting WiFi STA in background..."));

  struct { const char* ssid; const char* pass; } staList[] = {
    { STA_SSID,   STA_PASS   },
    { STA_SSID_2, STA_PASS_2 },
    { STA_SSID_3, STA_PASS_3 },
    { STA_SSID_4, STA_PASS_4 },
    { STA_SSID_5, STA_PASS_5 },
  };
  constexpr int STA_LIST_COUNT = sizeof(staList) / sizeof(staList[0]);
  bool staOk = false;

  for (int si = 0; si < STA_LIST_COUNT && !staOk; si++) {
    if (staList[si].ssid == nullptr || strlen(staList[si].ssid) == 0) continue;
    Serial.printf("[WiFi] STA [%d/%d]: \"%s\"...\n", si + 1, STA_LIST_COUNT, staList[si].ssid);
    WiFi.begin(staList[si].ssid, staList[si].pass);

    uint32_t staStart = millis();
    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(500));
      if (millis() - staStart > STA_CONNECT_TIMEOUT_MS) {
        Serial.printf("[WiFi] \"%s\" timeout.\n", staList[si].ssid);
        WiFi.disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
        break;
      }
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] STA OK! \"%s\" = %s\n",
                    staList[si].ssid, WiFi.localIP().toString().c_str());
      g_mqttEnabled = true;
      staOk = true;
    }
  }

  if (!staOk) {
    Serial.println(F("[WiFi] All SSIDs failed — AP-only mode."));
  }

  // WiFi ready — now init micro-ROS. This takes ~1-3s (UDP + DDS handshake).
  // If agent isn't running yet, it will retry each spin_some() call.
#if USE_MICRO_ROS
  Serial.println(F("[taskWifiConnect] Spawning micro-ROS task (will connect in background)..."));
  BaseType_t mr = xTaskCreatePinnedToCore(
      taskMicroRos, "MicroRos",
      8192, nullptr, 4,
      nullptr, 1
  );
  Serial.printf("[taskWifiConnect] taskMicroRos %s on Core 1.\n",
                (mr == pdPASS) ? "created" : "FAILED");
#endif

  vTaskDelete(nullptr);  // Task done — WiFi stays connected via ESP32 WiFi driver
}

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
  .joyLastMs = 0,
  .cmd_velLastMs = 0,
  .baseSpeed = 0,
  .autoBaseSpeed = 0,
  .waypointBaseSpeed = 0,
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

// ── Task: micro-ROS DDS spin (runs on Core 1, priority 4) ───────────────
// Shares Core 1 with taskControl (priority 5) and taskMQTT (priority 1).
// We run at a lower priority than taskControl so the motor loop always preempts
// us when needed. The rclc executor's internal timeout (5 ms per spin_some)
// is short enough that it yields frequently and doesn't starve taskControl.
static void taskMicroRos(void *pvParams) {
  (void)pvParams;
  Serial.println(F("[taskMicroRos] Started on Core 1"));

  while (true) {
    if (!microRos::isInitialized()) {
      if (microRos::init()) {
        Serial.println(F("[taskMicroRos] micro-ROS successfully connected & initialized!"));
      } else {
        vTaskDelay(pdMS_TO_TICKS(2000));  // Retry background connection every 2s without blocking Core 1
        continue;
      }
    }
    // microRos::tick() calls rclc_executor_spin_some() (5 ms timeout),
    // then runs the 500 ms /cmd_vel watchdog, then publishes /scan @ 20 Hz,
    // /odom @ 50 Hz, /imu/data @ 50 Hz.
    microRos::tick();
    // Small yield (1ms) prevents this task from hogging Core 1 when
    // micro-ROS work is light. Without it, the tight spin loop starves
    // other Core-1 tasks (WiFi stack, MQTT) and causes publish jitter.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void taskControl(void *pvParams) {
  const TickType_t xPeriod = pdMS_TO_TICKS(SAFE_LOOP_MS);
  TickType_t xLastWake = xTaskGetTickCount();
  uint32_t s_lastLoopMs = millis();  // track actual loop period for PID dt

  TickType_t odomTick = xTaskGetTickCount();

  // Register this task with the hardware watchdog. esp_task_wdt_init()
  // was called in setup() after microRos::init(). The watchdog resets every
  // SAFE_LOOP_MS = 50 ms in the loop below — well inside the 5 s window.
  TaskHandle_t me = xTaskGetCurrentTaskHandle();
  esp_err_t add_err = esp_task_wdt_add(me);
  Serial.printf("[WDT] taskControl add result: %s (handle=%p)\n", esp_err_to_name(add_err), me);

  while (true) {
    uint32_t loopStartMs = millis();
    uint32_t dtMs = (loopStartMs >= s_lastLoopMs) ? (loopStartMs - s_lastLoopMs) : 0;
    if (dtMs > 200) dtMs = 200;   // cap: avoid huge dt on scheduler hiccup
    const float dt_s = (float)dtMs * 0.001f;

    s_lastLoopMs = loopStartMs;

    // DEBUG: periodic diagnostics (every 5s)
    {
      static uint32_t s_lastDiag = 0;
      if (loopStartMs - s_lastDiag > 5000) {
        s_lastDiag = loopStartMs;
        Serial.printf("[DIAG] dtMs=%lu dt_s=%.4f freeheap=%u wifi=%s\n",
            (unsigned long)dtMs, (double)dt_s,
            (unsigned)esp_get_free_heap_size(),
            WiFi.status() == WL_CONNECTED ? "conn" : "DISC");
      }
    }

    // Feed hardware watchdog — catches cases where taskControl is delayed
    // (e.g., rclc_executor_spin_some blocking in micro-ROS task) for >5 s.
    esp_err_t rst_err = esp_task_wdt_reset();
    if (rst_err != ESP_OK) {
      static uint32_t s_last_warn = 0;
      if (millis() - s_last_warn > 5000) {
        Serial.printf("[WDT] reset error: %s\n", esp_err_to_name(rst_err));
        s_last_warn = millis();
      }
    }
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

    /* ── 1) IMU → EKF heading fusion ──────────────────────────────────
     *  Chuỗi xử lý:
     *    1) Đọc gyro Z từ MPU6050 (đã trừ bias thô + deadband)
     *    2) Tính dt giữa 2 lần update
     *    3) Chạy EKF: predict(gyroZ, dt) + updateWheel(gyroZ, dθ_wheel, dt)
     *    4) heading fusioned → rate-limiter → g_pose.headingRad
     */
#if USE_IMU_MPU6050
    {
      // Pipeline IMU EKF (v2.0 — dùng gyroZ thô, không còn "delta_heading/dt"):
      //   1) Đọc heading tích lũy từ IMU (giữ backward-compat cho telemetry).
      //   2) Đọc gyroZ thô (rad/s) — dùng cho EKF predict (chính xác hơn nhiều).
      //   3) Đọc encoder delta dsL/dsR (m) — dùng cho EKF updateWheel (chính xác tuyệt đối).
      //   4) locUpdate(dsL, dsR, dt) — Localization tích phân pose thật từ encoder.
      //
      // Đây là fusion CHUẨN công nghiệp: gyroZ thô + encoder thô + EKF 1D heading.
      // Sai số heading sau 10 phút: < 2° (đo được khi SLAM có /amcl_pose feedback).
      float imuHeadingBuf = g_pose.headingRad;
      imuMpu6050Update(imuHeadingBuf);  // Giữ tích lũy heading IMU (cho telemetry/debug)
      const uint32_t nowMs = millis();

      // 2) Đọc gyroZ thô (rad/s) — ImuMpu6050.h đã trừ bias + IMU_YAW_INVERTED.
      // Dùng gyroZ này làm input cho EKF predict (trước đây tính delta_heading/dt → sai số).
      float gyroZRaw = 0.f;
      const bool gyroOk = imuMpu6050GetGyroZ(gyroZRaw);

      // 3) EKF step:
      //    - predict bằng gyroZ thô (bias EKF tự ước lượng).
      //    - updateWheel bằng dθ_wheel tính từ encoder delta + WHEEL_BASE_M.
      //    - Cả hai đều có dt chính xác = SAFE_LOOP_MS/1000 (giả định loop ổn định).
      const float dt = (float)SAFE_LOOP_MS / 1000.f;

      // deadband nhỏ để tránh predict khi gyro < 0.0022 rad/s (~0.13°/s) — IMU đã làm.
      // (EKF step() đã có IMU_FUSION_Q_GYRO xử lý; ta chỉ cần gyro OK.)
      if (!gyroOk) {
        gyroZRaw = 0.f;  // mất IMU → gyro=0, chỉ encoder dẫn heading
      }

      // 4) updateWheel: imuFusion::step() dùng g_dThetaEncRate (encoder thật)
      //    được compute mỗi ODOM_PERIOD_MS (100ms) bởi odomUpdate().
      //    Không cần gọi locGetDriveCmd nữa cho heading fusion.

      // EKF step: predict từ gyro + update từ encoder thật
      float fusedHeading = imuFusion::step(gyroZRaw, dt, gyroOk);

      // Rate limiter — bảo vệ EKF heading khỏi single-tick SPIKE (IMU spike, EKF divergence).
      // Cap ở mức ~5.7°/tick (≈114°/s) — đủ cho mọi rotation hợp lệ của robot
      // (tốc độ quay tối đa ~1.5 rad/s = 86°/s khi dùng motor 12V).
      // EKF đã có IMU_FUSION_Q_GYRO process noise lọc noise ở layer riêng;
      // rate limiter này chỉ cắt spike cực lớn (>5σ IMU noise).
      static float s_prevFused = 0.f;
      static bool  s_firstFused = true;

      // NaN guard: if fusedHeading is corrupt, keep last valid heading
      if (!(fusedHeading == fusedHeading) || fabsf(fusedHeading) > 2.f * (float)M_PI) {
        fusedHeading = s_prevFused;  // revert to last good value
      }

      if (!s_firstFused) {
        float dHeading = wpNormalizeAngle(fusedHeading - s_prevFused);
        const float MAX_DHEADING = 0.10f;  // ~5.7°
        if (fabsf(dHeading) > MAX_DHEADING) {
          float clamped = s_prevFused + copysignf(MAX_DHEADING, dHeading);
          fusedHeading = wpNormalizeAngle(clamped);
        }
      }
      s_firstFused = false;
      s_prevFused  = fusedHeading;

      g_pose.headingRad = fusedHeading;
    }
#endif

    /* ── 2) Sensors: LiDAR + US ───────────────────────────────────── */
#if USE_LIDAR_HARDWARE
    sensorsPollLidar();
#endif
#if USE_YDLIDAR_X3
    // NOTE: LiDAR drain đã chuyển sang taskX3 riêng (xem setup()).
    // Task chạy priority 6, period 5ms — drain UART không block control loop.
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
        // Check if teleop (via micro-ROS /cmd_vel) is actively driving.
        // cmd_vel_callback updates cmd_velLastMs each time it processes a
        // non-gated /cmd_vel and directly calls motorApplyLayout/botRotate*.
        // When teleop is active we must NOT call botStop() or botDrive() —
        // the 2 control paths would fight each other and cause 20Hz jitter.
        const uint32_t nowMs = millis();
        const uint32_t cvAgeMs = (g_state.cmd_velLastMs != 0)
            ? (nowMs - g_state.cmd_velLastMs) : 0xFFFFFFFFu;
        // Two conditions needed:
        // 1. teleop is alive (fresh timestamp) — prevents race with cmd_vel_callback
        // 2. teleop is actually driving (cmd_velMoving=true) — ensures botStop() still
        //    fires when teleop sends lin=0,ang=0 (stop command) even while still sending.
        // When teleop goes silent entirely, cmd_velMoving stays false → botStop() fires
        // after 500ms. When teleop sends a stop command while still alive, cmd_velMoving
        // becomes false immediately → botStop() fires right away.
        const bool teleopActive = g_state.cmd_velMoving && (cvAgeMs < 500u);
        // Debug: log control task decision every 500ms
        static uint32_t s_lastCtrlLog = 0;
        if (nowMs - s_lastCtrlLog > 500u) {
            s_lastCtrlLog = nowMs;
            Serial.printf("[Ctrl-MANUAL] cmdX=%d cmdY=%d teleopActive=%d cvAgeMs=%u\n",
                g_state.cmdX, g_state.cmdY, teleopActive, (unsigned)cvAgeMs);
        }

        // [Safety Watchdog] Nếu joystick WebUI quá lâu (> 500ms) không gởi cập nhật, tự động về 0
        if (g_state.joyLastMs != 0 && (nowMs - g_state.joyLastMs > 500u)) {
          if (g_state.cmdX != 0 || g_state.cmdY != 0 || g_state.cmdStrafe != 0) {
            if (g_stateMutex != NULL) xSemaphoreTake(g_stateMutex, portMAX_DELAY);
            g_state.cmdX = 0;
            g_state.cmdY = 0;
            g_state.cmdStrafe = 0;
            if (g_stateMutex != NULL) xSemaphoreGive(g_stateMutex);
            Serial.println(F("[Safety Watchdog] WebUI joystick timeout (>500ms) -> STOP"));
          }
        }

        if (g_state.cmdX == 0 && g_state.cmdY == 0 && g_state.cmdStrafe == 0) {
          if (!teleopActive) botStop();
        } else if (!teleopActive) {
          /* Lái thẳng tự do. Có heading lock nhẹ khi cmdY!=0 và cmdX==0
             để robot đi thẳng không bị lệch do sai lệch cơ khí. */
          static float s_tgtH = 0.f;
          static bool  s_have = false;
          if (g_imuEnabled && g_state.cmdY != 0 && g_state.cmdX == 0 && g_state.cmdStrafe == 0) {
            if (!s_have) {
              s_tgtH = g_pose.headingRad;
              pidYawReset();
              s_have = true;
              Serial.printf("[HLK] activated — tgtH=%.3f (IMU enabled=%d)\n",
                  (double)s_tgtH, g_imuEnabled);
            }
            /* Nếu drift quá 25° → re-lock */
            float dh = wpNormalizeAngle(g_pose.headingRad - s_tgtH);
            if (fabsf(dh) > 0.436f) {
              s_tgtH = g_pose.headingRad;
              pidYawReset();
            }
            float steer = constrain(pidYawCompute(s_tgtH, g_pose.headingRad, dt_s), -85.f, 85.f);
            // DEBUG: log every 2s when heading lock is active
            {
              static uint32_t s_lastDebug = 0;
              uint32_t now = millis();
              if (now - s_lastDebug > 2000) {
                float err = wpNormalizeAngle(s_tgtH - g_pose.headingRad);
                Serial.printf("[HLK] tgt=%.3f cur=%.3f err=%.3f steer=%.1f s_have=%d\n",
                    (double)s_tgtH, (double)g_pose.headingRad, (double)err, (double)steer, s_have);
                s_lastDebug = now;
              }
            }
            botDrive((int16_t)steer, g_state.cmdY, g_state.baseSpeed);
          } else {
            // Heading lock only active for pure fwd/back. Any turn or stop → reset.
            if (s_have) {
              pidYawReset();
              s_have = false;
              Serial.println(F("[HLK] deactivated — PID reset"));
            }
            // Nếu có xoay hướng (cmdX != 0), ưu tiên nâng công suất theo g_state.rotateBaseSpeed (slider Xoay Hướng)
            uint16_t baseSpd = g_state.baseSpeed;
            uint16_t activeSpeed = baseSpd;
            if (g_state.cmdX != 0) {
              uint16_t rotSpeed = (g_state.rotateBaseSpeed > 0) ? g_state.rotateBaseSpeed : baseSpd;
              activeSpeed = (g_state.cmdY == 0) ? rotSpeed : ((rotSpeed > baseSpd) ? rotSpeed : baseSpd);
            }
            botDrive(g_state.cmdX, g_state.cmdY, activeSpeed);
          }
        }
        break;
      }

      case MODE_AUTO_EXPLORE: {
        autoExplore::tick();
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
    uint32_t nowMs = millis();
    uint32_t loopMs = nowMs - s_lastLoopMs;
    s_lastLoopMs = nowMs;
    (void)loopMs;  // future: expose via diagnostics if needed
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
  // Vô hiệu hóa Task Watchdog trong quá trình boot để tránh reset do
  // TLS handshake hoặc WiFi kết nối lâu. Sẽ được bật lại SAU khi mọi thứ
  // đã initialize xong (sau microRos::init).
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

#if USE_LINE_SENSOR
  lineSensorInit();
  lineDecoderInit();
  g_state.lineActiveMask = 0;
  g_state.linePattern = (uint8_t)LINE_PAT_UNKNOWN;
#endif

#if USE_YDLIDAR_X3
  Serial.println(F("[Boot] Initializing YDLIDAR X3..."));
  x3Init();
  Serial.println(F("[Boot] YDLIDAR X3 initialization step passed."));
#endif

  Serial.println(F("[Boot] Initializing IMU..."));
  imuMpu6050Init();
  Serial.println(F("[Boot] IMU initialization step passed."));

#if IMU_FUSION_ENABLE
  // Khởi tạo EKF sau khi IMU đã calibrate xong
  imuFusion::init();
  Serial.println(F("[Boot] IMU EKF fusion initialized."));
#endif

  // LED RGB nội bộ (DevKitC-1: GPIO 38) — sau odom
  Serial.println(F("[Boot] Initializing RGB..."));
  statusRgbInit();
  Serial.println(F("[Boot] RGB initialization step passed."));

  // ── WiFi + Web ───────────────────────────────────────────────────
  Serial.println(F("[Boot] Initializing WebUI..."));
  webUIInit();
  Serial.println(F("[Boot] WebUI initialization step passed."));

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

#if USE_YDLIDAR_X3
  // Core 1: LiDAR X3 — priority 6 (cao nhất, real-time UART drain 5ms period)
  // Fix lỗi "tụt pts khi ROS2 lên": drain UART trong task riêng, không bị
  // micro-ROS agent hay MQTT block. FIFO đầy = mất bytes = X3 resync = pts giảm.
  xTaskCreatePinnedToCore(
    taskX3, "LidarX3",
    4096, nullptr, 6,
    nullptr, 1
  );
#endif

  // WDT init BEFORE all tasks so add() always succeeds. FreeRTOS may schedule
  // as soon as a task is created, so init must be first.
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 5000,
      .idle_core_mask = 0,
      .trigger_panic = false,
  };
  esp_err_t wdt_err = esp_task_wdt_init(&wdt_config);
  Serial.printf("[Boot] WDT init: %s\n", esp_err_to_name(wdt_err));

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

  // Core 1: Điều khiển — stack 8KB, priority 5 (real-time)
  xTaskCreatePinnedToCore(
    taskControl, "Control",
    8192, nullptr, 5,
    nullptr, 1
  );
  Serial.println(F("[Boot] taskControl created on Core 1."));

#if WIFI_STA_ASYNC
  // WiFi STA + micro-ROS init chạy background — robot lái được ngay sau ~2s boot.
  xTaskCreatePinnedToCore(taskWifiConnect, "WiFiConnect", 8192, nullptr, 1, nullptr, 1);
  Serial.println(F("[Boot] taskWifiConnect spawned (non-blocking)."));
#elif WIFI_STA_ENABLE
  if (WiFi.status() == WL_CONNECTED) {
#if USE_MICRO_ROS
    Serial.println(F("[Boot] Spawning micro-ROS task..."));
    BaseType_t mr = xTaskCreatePinnedToCore(
        taskMicroRos, "MicroRos",
        8192, nullptr, 4,
        nullptr, 1
    );
    Serial.printf(F("[Boot] taskMicroRos %s on Core 1.\n"),
        (mr == pdPASS) ? "created" : "FAILED");
#endif
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
