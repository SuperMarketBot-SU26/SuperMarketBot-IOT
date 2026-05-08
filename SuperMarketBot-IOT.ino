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
 *    - NewPing by Tim Eckel
 *    - WebSockets by Markus Sattler (Links2004/arduinoWebSockets)
 *    - ArduinoJson by Benoit Blanchon
 *    - Adafruit NeoPixel (WS2812 onboard)
 * =====================================================================*/

#include "Config.h"
#include "Motors.h"
#include "Sensors.h"
#include "Odometry.h"
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
  .usFront = US_PING_MAX_CM, .usBack = US_PING_MAX_CM, .usLeft = US_PING_MAX_CM, .usRight = US_PING_MAX_CM,
  .usLF = US_PING_MAX_CM, .usLR = US_PING_MAX_CM, .usRF = US_PING_MAX_CM, .usRR = US_PING_MAX_CM,
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
 *  Tự hành demo (siêu thị): chỉ HC-SR04 — đi thẳng, né bằng lùi + xoay.
 *  Không vTaskDelay; không bật estop khi gặp vật (đó là lỗi cũ làm auto “chết”).
 * =================================================================== */
enum AutoNavFsm : uint8_t { AN_CRUISE = 0, AN_BACKUP, AN_TURN };

static AutoNavFsm s_auto_fsm = AN_CRUISE;
static uint32_t s_auto_t0 = 0;
static int8_t s_auto_turn = 1; // +1 CW, -1 CCW
static uint8_t s_auto_alt = 0;
static RobotMode s_ctrl_prev_mode = MODE_MANUAL;

static uint16_t autoSpeedPwm() {
  uint16_t s = g_state.autoBaseSpeed;
  if (s == 0) s = g_state.baseSpeed;
  uint16_t lo = (uint16_t)((uint32_t)PWM_MAX * (uint32_t)AUTO_MIN_PWM_FRAC / 100u);
  if (s < lo) s = lo;
  return s;
}

static void autoNavigateUltrasonic() {
  const uint16_t spd = autoSpeedPwm();
  const uint32_t now = millis();
  const int16_t f = g_state.usFront;
  const int16_t L = g_state.usLeft;
  const int16_t R = g_state.usRight;
  const int16_t B = g_state.usBack;

  switch (s_auto_fsm) {
    case AN_CRUISE: {
      if (f < AUTO_US_BLOCK_CM) {
        s_auto_fsm = AN_BACKUP;
        s_auto_t0 = now;
        break;
      }
      uint16_t run = spd;
      if (f < AUTO_US_SLOW_CM && f > AUTO_US_BLOCK_CM) {
        float den = (float)(AUTO_US_SLOW_CM - AUTO_US_BLOCK_CM);
        float t = den > 1.f ? (float)(f - AUTO_US_BLOCK_CM) / den : 1.f;
        if (t < 0.2f) t = 0.2f;
        run = (uint16_t)((float)spd * t);
      }
      if (R < AUTO_US_SIDE_CM && L >= R - 2) {
        botDrive(-40, 70, run);
      } else if (L < AUTO_US_SIDE_CM && R >= L - 2) {
        botDrive(40, 70, run);
      } else {
        botForward(run);
      }
      break;
    }
    case AN_BACKUP: {
      uint16_t bspd = (uint16_t)(((uint32_t)spd * 12u) / 25u);
      uint16_t bmin = (uint16_t)(PWM_MAX / 14u);
      if (bspd < bmin) bspd = bmin;

      if (B < AUTO_US_BACK_STOP_CM) {
        if (L > R + 3) s_auto_turn = -1;
        else if (R > L + 3) s_auto_turn = 1;
        else s_auto_turn = ((s_auto_alt & 1u) != 0) ? 1 : -1;
        s_auto_alt++;
        s_auto_fsm = AN_TURN;
        s_auto_t0 = now;
        botStop();
        break;
      }
      botBackward(bspd);
      if (now - s_auto_t0 >= AUTO_BACKUP_MS) {
        if (L > R + 3) s_auto_turn = -1;
        else if (R > L + 3) s_auto_turn = 1;
        else s_auto_turn = ((s_auto_alt & 1u) != 0) ? 1 : -1;
        s_auto_alt++;
        s_auto_fsm = AN_TURN;
        s_auto_t0 = now;
        botStop();
      }
      break;
    }
    case AN_TURN: {
      uint16_t tspd = (uint16_t)(((uint32_t)spd * 4u) / 10u);
      uint16_t tmin = (uint16_t)(PWM_MAX / 11u);
      if (tspd < tmin) tspd = tmin;
      if (s_auto_turn > 0) botRotateCW(tspd);
      else botRotateCCW(tspd);
      if (now - s_auto_t0 >= AUTO_TURN_MS) {
        botStop();
        s_auto_fsm = AN_CRUISE;
      }
      break;
    }
    default:
      s_auto_fsm = AN_CRUISE;
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
    if (g_state.mode == MODE_AUTO && s_ctrl_prev_mode != MODE_AUTO) {
      g_state.estop = false;
      s_auto_fsm = AN_CRUISE;
      s_auto_t0 = millis();
      s_auto_alt = 0;
    }
    if (g_state.mode != MODE_AUTO && s_ctrl_prev_mode == MODE_AUTO) {
      s_auto_fsm = AN_CRUISE;
    }
    s_ctrl_prev_mode = g_state.mode;

    sensorsPollLidar();

    if ((xTaskGetTickCount() - usTick) >= pdMS_TO_TICKS(60)) {
      sensorsPollUS();
      usTick = xTaskGetTickCount();
    }

    if ((xTaskGetTickCount() - odomTick) >= pdMS_TO_TICKS(ODOM_PERIOD_MS)) {
      odomUpdate();
      odomTick = xTaskGetTickCount();
    }

    if (g_state.estop) {
      botStop();
      if (g_state.cmdX == 0 && g_state.cmdY == 0) g_state.estop = false;
    } else if (g_state.mode == MODE_AUTO) {
      autoNavigateUltrasonic();
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
  Serial.printf("[Boot] Dashboard: http://%s\n",
                WiFi.softAPIP().toString().c_str());
  printMemInfo();
}

/* =====================================================================
 *  loop() — Không dùng (FreeRTOS quản lý toàn bộ)
 *  Để trống hoặc bỏ idle task vào đây nếu cần debug
 * =================================================================== */
void loop() {
  vTaskDelay(portMAX_DELAY);
}
