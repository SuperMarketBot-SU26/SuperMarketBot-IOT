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
 *    - NewPing (chỉ khi USE_HC_SR04_HARDWARE=1 trong Config.h)
 *    - WebSockets by Markus Sattler (Links2004/arduinoWebSockets)
 *    - ArduinoJson by Benoit Blanchon
 *    - Adafruit NeoPixel (WS2812 onboard)
 *    - PubSubClient by Nick O'Leary (Phase 1: MQTT)
 * =====================================================================*/

#include "Config.h"
#include "Motors.h"
#include "Sensors.h"
#include "Odometry.h"
#include "PidController.h"
#include "StatusRGB.h"
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
  .cmdX = 0, .cmdY = 0,
  .baseSpeed = 0,
  .autoBaseSpeed = 0,
  .mode = MODE_MANUAL,
  .estop = false,
  .lidarLastUpdateMs = 0,
  .usLastUpdateMs = 0
};

// ── Mutex bảo vệ g_state khi đọc/ghi từ 2 core ─────────────────────
SemaphoreHandle_t g_stateMutex;

/* =====================================================================
 *  Tự hành (LiDAR trước/sau): CRUISE → chặn → STOP_HOLD → quét tìm hướng
 *  trống (xoay có ramp, CW rồi CCW tới khi đủ xa phía trước) → hãm nhẹ
 *  → CRUISE. SR04 bật thì thêm bẻ cạnh. (SLAM / bản đồ: sau này thay FSM.)
 * =================================================================== */
enum AutoNavFsm : uint8_t {
  AN_CRUISE = 0,
  AN_STOP_HOLD,
  AN_SCAN_SEEK,
  AN_SCAN_DECEL,
  AN_BACKUP       // Phase 1: lùi khi scan CW+CCW đều thất bại
};

static AutoNavFsm s_auto_fsm = AN_CRUISE;
static uint32_t s_auto_t0 = 0;
static int8_t s_scan_dir = 1;   // +1 = CW, -1 = CCW
static uint8_t s_scan_pass = 0; // 0 = chiều đầu, 1 = đã đổi chiều một lần
static uint8_t s_clear_streak = 0;

/* Phase 1 — Stuck detection */
static uint32_t s_stuckCheckMs = 0;
static float    s_stuckLastDist = 0.f;
static uint8_t  s_stuckCount = 0;
static RobotMode s_ctrl_prev_mode = MODE_MANUAL;

static uint16_t autoSpeedPwm() {
  uint16_t s = g_state.autoBaseSpeed;
  if (s == 0) s = g_state.baseSpeed;
  uint16_t lo = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)AUTO_MIN_PWM_FRAC / 100u);
  if (s < lo) s = lo;
  return s;
}

static uint16_t autoScanTurnPwm() {
  uint32_t p = ((uint32_t)PWM_MAX * (uint32_t)AUTO_SCAN_PWM_PCT) / 100u;
  uint16_t tmin = (uint16_t)(PWM_MAX / 11u);
  if (p < (uint32_t)tmin) p = (uint32_t)tmin;
  if (p > (uint32_t)PWM_MAX) p = (uint32_t)PWM_MAX;
  return (uint16_t)p;
}

/** PWM xoay phần đầu mỗi lần đổi hướng — tránh giật; sàn ~22% mục tiêu tối thiểu. */
static uint16_t autoScanSeekPwm(uint32_t segElapsedMs, uint16_t scanPwm) {
  if (segElapsedMs >= AUTO_SCAN_RAMP_UP_MS) return scanPwm;
  uint32_t p = (uint32_t)scanPwm * segElapsedMs / AUTO_SCAN_RAMP_UP_MS;
  uint32_t lo = (uint32_t)scanPwm * 22u / 100u;
  if (lo < (uint32_t)(PWM_MAX / 14u)) lo = (uint32_t)(PWM_MAX / 14u);
  if (p < lo) p = lo;
  if (p > (uint32_t)scanPwm) p = (uint32_t)scanPwm;
  if (p > (uint32_t)PWM_MAX) p = (uint32_t)PWM_MAX;
  return (uint16_t)p;
}

/* Timestamp cho PID CRUISE */
static uint32_t s_pidLastMs = 0;

static void autoNavigateAvoidance() {
  const uint16_t spd = autoSpeedPwm();
  const uint16_t scanPwm = autoScanTurnPwm();
  const uint32_t now = millis();
  const int16_t f = g_state.lidarFront;
  const int16_t b = g_state.lidarBack;
  const int16_t L = g_state.usLeft;
  const int16_t R = g_state.usRight;

  switch (s_auto_fsm) {
    case AN_CRUISE: {
      const bool blockFront = (f < AUTO_LIDAR_BLOCK_CM);
      const bool blockRear =
          (AUTO_LIDAR_BLOCK_USE_REAR != 0) && (b < AUTO_LIDAR_BLOCK_CM);
      if (blockFront || blockRear) {
        botStop();
        pidSpeedReset();
        s_auto_fsm = AN_STOP_HOLD;
        s_auto_t0 = now;
        s_pidLastMs = now;
        break;
      }

      /* Giảm tốc khi gần vật */
      uint16_t targetSpd = spd;
      if (f < AUTO_LIDAR_SLOW_CM && f >= AUTO_LIDAR_BLOCK_CM) {
        float den = (float)(AUTO_LIDAR_SLOW_CM - AUTO_LIDAR_BLOCK_CM);
        float t = den > 1.f ? (float)(f - AUTO_LIDAR_BLOCK_CM) / den : 1.f;
        if (t < 0.2f) t = 0.2f;
        targetSpd = (uint16_t)((float)spd * t);
      }

      /* Speed PID — chỉ áp dụng khi encoder có xung (tránh phantom PID khi trượt) */
      float dt_s = (float)(now - s_pidLastMs) * 0.001f;
      s_pidLastMs = now;
      if (dt_s < 0.001f) dt_s = 0.001f;

      float actualMps = robotActualSpeedMps();
      float targetMps = pwmToEstMps(targetSpd);
      float pidOut = pidSpeedCompute(targetMps, actualMps, dt_s);

      /* Áp dụng PID delta vào target PWM */
      int32_t run = (int32_t)targetSpd + (int32_t)pidOut;
      if (run < 0)           run = 0;
      if (run > (int32_t)PWM_MAX) run = (int32_t)PWM_MAX;
      uint16_t runPwm = (uint16_t)run;

#if USE_HC_SR04_HARDWARE
      if (R < AUTO_US_SIDE_CM && L >= R - 2) {
        botDrive(-40, 70, runPwm);
      } else if (L < AUTO_US_SIDE_CM && R >= L - 2) {
        botDrive(40, 70, runPwm);
      } else
#endif
      {
        botForward(runPwm);
      }
      break;
    }
    case AN_STOP_HOLD: {
      botStop();
      if (now - s_auto_t0 >= AUTO_STOP_HOLD_MS) {
        s_auto_fsm = AN_SCAN_SEEK;
        s_auto_t0 = now;
        s_scan_dir = 1;
        s_scan_pass = 0;
        s_clear_streak = 0;
      }
      break;
    }
    case AN_SCAN_SEEK: {
      const uint32_t seg = now - s_auto_t0;

      const bool frontOk = (f >= (int16_t)AUTO_LIDAR_CLEAR_CM) && (f > 3);
      if (frontOk) {
        if (s_clear_streak < 255) s_clear_streak++;
      } else {
        s_clear_streak = 0;
      }
      if (s_clear_streak >= AUTO_SCAN_CLEAR_STREAK) {
        s_auto_fsm = AN_SCAN_DECEL;
        s_auto_t0 = now;
        s_clear_streak = 0;
        break;
      }
      if (seg >= AUTO_SCAN_SEEK_MS_PER_DIR) {
        botStop();
        s_clear_streak = 0;
        if (s_scan_pass >= 1) {
          /* Cả CW và CCW đều không tìm được hướng trống → lùi */
          s_auto_fsm = AN_BACKUP;
          s_auto_t0 = now;
          s_scan_pass = 0;
        } else {
          s_scan_pass = 1;
          s_scan_dir = (int8_t)(-s_scan_dir);
          s_auto_t0 = now;
        }
        break;
      }

      const uint16_t turn = autoScanSeekPwm(seg, scanPwm);
      if (s_scan_dir > 0) {
        botRotateCW(turn);
      } else {
        botRotateCCW(turn);
      }
      break;
    }
    case AN_SCAN_DECEL: {
      const uint32_t dt = now - s_auto_t0;
      if (dt >= AUTO_SCAN_DECEL_MS) {
        botStop();
        s_auto_fsm = AN_CRUISE;
        break;
      }
      uint32_t pwm = ((uint32_t)scanPwm * (AUTO_SCAN_DECEL_MS - dt)) / AUTO_SCAN_DECEL_MS;
      uint16_t d = (uint16_t)pwm;
      if (s_scan_dir > 0) {
        botRotateCW(d);
      } else {
        botRotateCCW(d);
      }
      break;
    }
    case AN_BACKUP: {
      /* Lùi an toàn: kiểm tra Luna sau trước khi lùi */
      if (b <= (int16_t)AUTO_LIDAR_BLOCK_CM) {
        /* Sau cũng chặn → dừng hẳn, chờ người can thiệp */
        botStop();
        if (now - s_auto_t0 >= AUTO_BACKUP_REVERSE_MS * 3u) {
          s_auto_fsm = AN_CRUISE;  // timeout → thử lại từ đầu
        }
        break;
      }
      uint16_t bkSpd = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)AUTO_BACKUP_SPEED_PCT / 100u);
      botBackward(bkSpd);
      if (now - s_auto_t0 >= AUTO_BACKUP_REVERSE_MS) {
        botStop();
        s_auto_fsm = AN_SCAN_SEEK;
        s_auto_t0 = now;
        s_scan_dir = 1;
        s_scan_pass = 0;
        s_clear_streak = 0;
        s_stuckCount = 0;
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
    if (g_state.mode == MODE_AUTO && s_ctrl_prev_mode != MODE_AUTO) {
      g_state.estop = false;
      s_auto_fsm = AN_CRUISE;
      s_auto_t0 = millis();
      s_pidLastMs = millis();
      s_scan_dir = 1;
      s_scan_pass = 0;
      s_clear_streak = 0;
      s_stuckCount = 0;
      pidSpeedReset();
      /* Báo backend chuyển mode */
      strncpy((char *)g_mqttPendingStatus, "auto", sizeof(g_mqttPendingStatus) - 1);
      g_mqttStatusPending = true;
    }
    if (g_state.mode != MODE_AUTO && s_ctrl_prev_mode == MODE_AUTO) {
      s_auto_fsm = AN_CRUISE;
      s_scan_pass = 0;
      s_clear_streak = 0;
      strncpy((char *)g_mqttPendingStatus, "manual", sizeof(g_mqttPendingStatus) - 1);
      g_mqttStatusPending = true;
    }
    s_ctrl_prev_mode = g_state.mode;

    sensorsPollLidar();
    /* Dừng + xoay: đọc Luna kép để cả Serial1 & Serial2 kịp byte (PWM dễ làm UART2 chậm). */
    if (g_state.mode == MODE_AUTO && s_auto_fsm != AN_CRUISE) {
      sensorsPollLidar();
      sensorsPollLidar();
    }

#if USE_HC_SR04_HARDWARE
    if ((xTaskGetTickCount() - usTick) >= pdMS_TO_TICKS(60)) {
      sensorsPollUS();
      usTick = xTaskGetTickCount();
    }
#else
    sensorsPollUS();
#endif

    if ((xTaskGetTickCount() - odomTick) >= pdMS_TO_TICKS(ODOM_PERIOD_MS)) {
      odomUpdate();
      odomTick = xTaskGetTickCount();
    }

    if (g_state.estop) {
      botStop();
      if (g_state.cmdX == 0 && g_state.cmdY == 0) g_state.estop = false;
    } else if (g_state.mode == MODE_AUTO) {
      autoNavigateAvoidance();
    } else {
      if (g_state.cmdX == 0 && g_state.cmdY == 0) {
        botStop();
      } else {
        botDrive(g_state.cmdX, g_state.cmdY, g_state.baseSpeed);
      }
    }

    vTaskDelayUntil(&xLastWake, xPeriod);
  }
}

/* =====================================================================
 *  TASK CORE 0 — Web IO (HTTP + WebSocket)
 * =================================================================== */
static void taskWebIO(void *pvParams) {
  const TickType_t broadcastPeriod = pdMS_TO_TICKS(100); // 10 Hz telemetry
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
    8192, nullptr, 1,
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
