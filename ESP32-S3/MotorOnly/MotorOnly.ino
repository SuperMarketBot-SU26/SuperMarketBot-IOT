/* =====================================================================
 *  MotorOnly.ino - ESP32-S3 Firmware (Motor Controller Only)
 *  SmartMarketBot - Simplified for SLAM on Xiaomi Pad 6
 *
 *  This firmware ONLY handles:
 *  - Receive velocity commands via WiFi UDP
 *  - Generate PWM signals for motors
 *  - Run PID speed control
 *  - Read IMU for telemetry
 *
 *  ALL SLAM/Navigation processing happens on Xiaomi Pad 6
 *
 *  Board: ESP32-S3-DevKitC-1 (N16R8)
 *  =====================================================================*/

#include "Config.h"
#include "Motors.h"
#include "PidController.h"
#include "ImuMpu6050.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>

// ============================================================================
// NETWORK CONFIGURATION
// ============================================================================
const char* AP_SSID = "SmartMarketBot";
const char* AP_PASS = "12345678";
const int UDP_PORT = 4210;

// ============================================================================
// MOTOR COMMAND PACKET FORMAT (from Android)
// Byte:  0     1     2-3    4-5    6-7    8
// Type:  HEAD  CMD   VX_H   VX_L   VY_H   VY_L  W_H  W_L  CHK
// HEAD = 0xAA, CMD = 0x01, VX/VY = int16 (mm/s), W = int16 (mrad/s)
constexpr uint8_t PACKET_HEADER = 0xAA;
constexpr uint8_t CMD_VELOCITY = 0x01;
constexpr uint8_t CMD_STOP = 0x02;
constexpr uint8_t PACKET_SIZE = 9;

// ============================================================================
// GLOBAL STATE
// ============================================================================
WiFiUDP udp;
bool wifiConnected = false;

// Velocity command (from Android)
volatile float cmdVx = 0.0f;      // m/s
volatile float cmdVy = 0.0f;      // m/s
volatile float cmdOmega = 0.0f;   // rad/s
volatile uint32_t lastCommandMs = 0;

// PID controllers for each wheel
PidController pidFL(10.0f, 0.0f, 2.0f);   // Front-left
PidController pidRL(10.0f, 0.0f, 2.0f);   // Rear-left
PidController pidFR(10.0f, 0.0f, 2.0f);   // Front-right
PidController pidRR(10.0f, 0.0f, 2.0f);   // Rear-right

// Motor RPM (from encoder feedback)
volatile float rpmFL = 0, rpmRL = 0, rpmFR = 0, rpmRR = 0;

// ============================================================================
// ROBOT PHYSICAL CONSTANTS
// ============================================================================
constexpr float WHEEL_RADIUS = 0.0325f;    // 65mm wheel
constexpr float ROBOT_LENGTH = 0.25f;      // Wheelbase
constexpr float ROBOT_WIDTH = 0.20f;

// ============================================================================
// MOTOR LAYOUT (same as original)
// ============================================================================
enum MotorId : uint8_t { MID_FL = 0, MID_RL = 1, MID_FR = 2, MID_RR = 3 };

// ============================================================================
// FORWARD KINEMATICS - Mecanum
// ============================================================================
void mecanumInverseKinematics(float vx, float vy, float omega,
                              float& wFL, float& wRL, float& wFR, float& wRR) {
    float L = ROBOT_LENGTH / 2.0f;
    float W = ROBOT_WIDTH / 2.0f;
    float r = WHEEL_RADIUS;

    // Mecanum X-pattern inverse kinematics
    wFL = (vx - vy - (L + W) * omega) / r;  // Front-left
    wRL = (vx + vy - (L + W) * omega) / r;  // Rear-left
    wFR = (vx + vy + (L + W) * omega) / r;  // Front-right
    wRR = (vx - vy + (L + W) * omega) / r;  // Rear-right
}

// ============================================================================
// VELOCITY TO PWM CONVERSION
// ============================================================================
constexpr float MAX_RPM = 300.0f;          // Max wheel RPM
constexpr float MAX_PWM = 1023.0f;        // 10-bit PWM

float rpmToPwm(float rpm) {
    // Linear mapping (tune these parameters)
    float pwm = rpm * 3.4f;  // ~1023 at 300 RPM
    return constrain(pwm, -MAX_PWM, MAX_PWM);
}

// ============================================================================
// PARSE UDP PACKET
// ============================================================================
bool parseCommandPacket(const uint8_t* data, size_t len) {
    if (len < PACKET_SIZE) return false;
    if (data[0] != PACKET_HEADER) return false;

    uint8_t cmd = data[1];

    if (cmd == CMD_STOP) {
        cmdVx = 0;
        cmdVy = 0;
        cmdOmega = 0;
        lastCommandMs = millis();
        return true;
    }

    if (cmd == CMD_VELOCITY) {
        // Parse velocity command
        int16_t vx_mm = data[2] | (data[3] << 8);
        int16_t vy_mm = data[4] | (data[5] << 8);
        int16_t omega_mrad = data[6] | (data[7] << 8);

        // Verify checksum
        uint8_t checksum = 0;
        for (int i = 1; i < 8; i++) {
            checksum ^= data[i];
        }
        if (checksum != data[8]) {
            Serial.println("[UDP] Checksum error!");
            return false;
        }

        // Convert to float (m/s and rad/s)
        cmdVx = vx_mm / 1000.0f;
        cmdVy = vy_mm / 1000.0f;
        cmdOmega = omega_mrad / 1000.0f;
        lastCommandMs = millis();

        return true;
    }

    return false;
}

// ============================================================================
// PID MOTOR CONTROL LOOP
// ============================================================================
void updateMotorControl() {
    // Calculate target wheel velocities from command
    float wFL_target, wRL_target, wFR_target, wRR_target;
    mecanumInverseKinematics(cmdVx, cmdVy, cmdOmega,
                              wFL_target, wRL_target, wFR_target, wRR_target);

    // Convert rad/s to RPM for PID
    float rpmFL_target = wFL_target * 60.0f / (2.0f * PI);
    float rpmRL_target = wRL_target * 60.0f / (2.0f * PI);
    float rpmFR_target = wFR_target * 60.0f / (2.0f * PI);
    float rpmRR_target = wRR_target * 60.0f / (2.0f * PI);

    // Compute PID outputs
    float dt = 0.05f;  // 50ms loop
    int pwmFL = pidFL.compute(rpmFL_target, rpmFL, dt);
    int pwmRL = pidRL.compute(rpmRL_target, rpmRL, dt);
    int pwmFR = pidFR.compute(rpmFR_target, rpmFR, dt);
    int pwmRR = pidRR.compute(rpmRR_target, rpmRR, dt);

    // Drive motors
    motorDrive(MID_FL, pwmFL);
    motorDrive(MID_RL, pwmRL);
    motorDrive(MID_FR, pwmFR);
    motorDrive(MID_RR, pwmRR);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println(F("== SmartMarketBot Motor Controller =="));
    Serial.println(F("Simplified firmware - SLAM on Android"));

    // Initialize motors
    motorsInit();
    botStop();
    Serial.println(F("[Motors] Initialized"));

    // Initialize IMU
#if USE_IMU_MPU6050
    imuMpu6050Init();
    Serial.println(F("[IMU] Initialized"));
#endif

    // Setup WiFi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    IPAddress IP = WiFi.softAPIP();
    Serial.print(F("[WiFi] AP IP: "));
    Serial.println(IP);

    // Start UDP
    udp.begin(UDP_PORT);
    Serial.print(F("[UDP] Listening on port "));
    Serial.println(UDP_PORT);

    Serial.println(F("[Ready] Motor controller active"));
    Serial.println(F("[Info] Waiting for commands from Android..."));
}

// ============================================================================
// MAIN LOOP
// ============================================================================
unsigned long lastControlUpdate = 0;
unsigned long lastTelemetry = 0;

void loop() {
    unsigned long now = millis();

    // =========================================
    // 1. READ UDP COMMANDS
    // =========================================
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        uint8_t buffer[64];
        int len = udp.read(buffer, sizeof(buffer));
        if (len > 0) {
            parseCommandPacket(buffer, len);
        }
    }

    // =========================================
    // 2. SAFETY: TIMEOUT CHECK
    // =========================================
    // If no command for 500ms, emergency stop
    if (now - lastCommandMs > 500 && lastCommandMs > 0) {
        cmdVx = 0;
        cmdVy = 0;
        cmdOmega = 0;
        botStop();
    }

    // =========================================
    // 3. MOTOR CONTROL (50Hz)
    // =========================================
    if (now - lastControlUpdate >= 20) {  // 50Hz
        lastControlUpdate = now;

        // Update motor control
        updateMotorControl();
    }

    // =========================================
    // 4. TELEMETRY (10Hz) - Optional
    // =========================================
    if (now - lastTelemetry >= 100) {  // 10Hz
        lastTelemetry = now;

        // Send telemetry back to Android
        // Format: 0xBB, CMD, X_H, X_L, Y_H, Y_L, BAT, STATUS, CHK
        uint8_t reply[9];
        reply[0] = 0xBB;
        reply[1] = 0x10;  // Telemetry

        // Placeholder odometry (from encoder if available)
        int16_t xPos = 0;
        int16_t yPos = 0;
        reply[2] = xPos & 0xFF;
        reply[3] = (xPos >> 8) & 0xFF;
        reply[4] = yPos & 0xFF;
        reply[5] = (yPos >> 8) & 0xFF;

        // Battery (placeholder)
        reply[6] = 120;  // 12.0V

        // Status
        reply[7] = 0x01;  // Running

        // Checksum
        reply[8] = 0;
        for (int i = 1; i < 8; i++) reply[8] ^= reply[i];

        udp.beginPacket(udp.remoteIP(), 4211);
        udp.write(reply, 9);
        udp.endPacket();
    }

    // Small delay to prevent watchdog issues
    delay(5);
}
