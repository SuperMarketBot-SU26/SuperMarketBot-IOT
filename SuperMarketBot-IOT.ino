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
 *  • BLE: xem tools/ble-dashboard.html (Web Bluetooth demo, Chrome localhost)
 * =====================================================================*/

#include "Config.h"
#include "Motors.h"
#include "Sensors.h"
#include "Odometry.h"
#include "StatusRGB.h"
#include "WebUI.h"
#include "BleUi.h"
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
  .lidarFront = LIDAR_MAX_CM, .lidarBack = LIDAR_MAX_CM,
  .rpmFL = 0, .rpmRL = 0, .rpmFR = 0, .rpmRR = 0,
  .distFL = 0, .distRL = 0, .distFR = 0, .distRR = 0,
  .cmdX = 0, .cmdY = 0,
  .baseSpeed = 0,
  .mode = MODE_MANUAL,
  .estop = false,
  .lidarLastUpdateMs = 0,
  .usLastUpdateMs = 0
};

// ── Mutex bảo vệ g_state khi đọc/ghi từ 2 core ─────────────────────
SemaphoreHandle_t g_stateMutex;

/* =====================================================================
 *  Logic tự hành né vật cản đơn giản (State Machine)
 * =================================================================== */
static void autoAvoid() {
  const int16_t frontMin = min(g_state.lidarFront, g_state.usFront);
  const int16_t backMin  = min(g_state.lidarBack,  g_state.usBack);
  const int16_t leftDist  = g_state.usLeft;
  const int16_t rightDist = g_state.usRight;

  // Dừng khẩn cấp — ưu tiên tuyệt đối
  if (frontMin < SAFE_STOP_CM) {
    botStop();
    // Lùi nhẹ để thoát
    vTaskDelay(pdMS_TO_TICKS(200));
    botBackward(g_state.baseSpeed / 2);
    vTaskDelay(pdMS_TO_TICKS(400));
    botStop();
    // Xoay về hướng nhiều không gian hơn
    if (rightDist > leftDist) botRotateCW(g_state.baseSpeed / 2);
    else                      botRotateCCW(g_state.baseSpeed / 2);
    vTaskDelay(pdMS_TO_TICKS(500));
    botStop();
    return;
  }

  // Giảm tốc khi vật cản gần (tuyến tính: 100% → 30% trong dải SAFE_STOP..SAFE_SLOW)
  uint16_t spd = g_state.baseSpeed;
  if (frontMin < SAFE_SLOW_CM) {
    float ratio = (float)(frontMin - SAFE_STOP_CM) / (SAFE_SLOW_CM - SAFE_STOP_CM);
    ratio = max(0.3f, ratio);
    spd = (uint16_t)(g_state.baseSpeed * ratio);
  }

  // Vừa tiến vừa tránh sang trái/phải nếu bên đó trống
  if (rightDist < SAFE_STOP_CM) {
    // Nguy hiểm bên phải → vừa tiến vừa lệch trái
    botDrive(-40, 80, spd);
  } else if (leftDist < SAFE_STOP_CM) {
    botDrive(40, 80, spd);
  } else {
    // Đường thẳng
    botForward(spd);
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
    // ── Đọc LiDAR (mỗi vòng, non-blocking) ────────────────────────
    sensorsPollLidar();

    // ── Đọc siêu âm (~4×5ms = 20ms) mỗi 60ms để giảm ảnh hưởng ──
    if ((xTaskGetTickCount() - usTick) >= pdMS_TO_TICKS(60)) {
      sensorsPollUS();
      usTick = xTaskGetTickCount();
    }

    // ── Odometry mỗi 100ms ─────────────────────────────────────────
    if ((xTaskGetTickCount() - odomTick) >= pdMS_TO_TICKS(ODOM_PERIOD_MS)) {
      odomUpdate();
      odomTick = xTaskGetTickCount();
    }

    // ── Đánh giá dừng khẩn cấp ────────────────────────────────────
    bool frontDanger = (g_state.lidarFront < SAFE_STOP_CM) ||
                       (g_state.usFront    < SAFE_STOP_CM);
    if (frontDanger) g_state.estop = true;

    // ── Ra lệnh động cơ ───────────────────────────────────────────
    if (g_state.estop) {
      botStop();
      // Tự xoá estop sau khi người dùng gửi lại lệnh joystick
      if (g_state.cmdX == 0 && g_state.cmdY == 0) {
        // chờ confirm release — reset estop
        g_state.estop = false;
      }
    } else if (g_state.mode == MODE_AUTO) {
      autoAvoid();
    } else {
      // Chế độ lái tay
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
      bleUiNotifyIfConnected();
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
  bleUiInit();

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
