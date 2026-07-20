/**
 * RobotMotorCommand.h
 * Motor command generator for ESP32 — 4WD Differential Drive.
 *
 * Project dùng 4WD bánh thường (2 trái + 2 phải). Vy = 0 luôn.
 *
 * Module này handle:
 * 1. Velocity command generation (vx, omega) — vy = 0
 * 2. Differential inverse kinematics (left/right wheel speed)
 * 3. Mecanum IK alias (back-compat) — internally routes to differential
 *
 * Communication: project dùng WebSocket JSON (xem MotorLink.kt).
 * UDP packet format giữ làm LEGACY — không còn được sử dụng.
 */

#ifndef ROBOT_MOTOR_COMMAND_H
#define ROBOT_MOTOR_COMMAND_H

#include <cstdint>
#include <cmath>
#include <vector>
#include <array>

// ============================================================================
// ROBOT PHYSICAL CONSTANTS
// ============================================================================

// Robot dimensions (in meters)
constexpr float ROBOT_LENGTH = 0.25f;      // Wheelbase length
constexpr float ROBOT_WIDTH = 0.20f;      // Wheelbase width
constexpr float WHEEL_RADIUS = 0.0325f;   // Wheel radius (65mm diameter)

// 4WD differential drive — không có roller angle.
// Mecanum roller angle (45°) giữ làm deprecated constant để back-compat.

// ============================================================================
// MOTOR COMMAND STRUCTURES
// ============================================================================

struct VelocityCommand {
    float vx;      // Forward velocity (m/s)
    float vy;      // Strafe velocity (m/s)
    float omega;   // Angular velocity (rad/s)
    uint32_t timestamp_ms;
};

struct WheelSpeeds {
    float w_fl;  // Front-left wheel (rad/s)
    float w_rl;  // Rear-left wheel
    float w_fr;  // Front-right wheel
    float w_rr;  // Rear-right wheel
};

// ============================================================================
// INVERSE KINEMATICS — 4WD DIFFERENTIAL DRIVE (bánh thường)
// ============================================================================
//
// Layout: 2 bánh trái (FL, RL) + 2 bánh phải (FR, RR).
//   left_wheel  = vx + omega * W
//   right_wheel = vx - omega * W
// Trong đó W = half-width (wheelbase).
//
// Project KHÔNG dùng mecanum. Hàm [mecanumInverseKinematics] được giữ làm
// alias cho [differentialInverseKinematics] để không break native code cũ.

/**
 * 4WD Differential IK (PRIMARY — dùng cho project).
 *
 * @param vx Forward velocity (m/s)
 * @param omega Angular velocity (rad/s)
 * @param L  unused (kept cho API compat)
 * @param W  half-width wheelbase (m)
 * @return WheelSpeeds (rad/s) — FL/RL = left, FR/RR = right
 */
inline WheelSpeeds differentialInverseKinematics(
    float vx, float omega,
    float /*L*/ = ROBOT_LENGTH / 2.0f,
    float W = ROBOT_WIDTH / 2.0f
) {
    float left  = vx + omega * W;
    float right = vx - omega * W;

    WheelSpeeds ws;
    ws.w_fl = left  / WHEEL_RADIUS;
    ws.w_rl = left  / WHEEL_RADIUS;
    ws.w_fr = right / WHEEL_RADIUS;
    ws.w_rr = right / WHEEL_RADIUS;
    return ws;
}

/**
 * [DEPRECATED] Alias cho differentialInverseKinematics (back-compat).
 * Project đổi từ mecanum sang 4WD — function này chỉ ignore vy.
 */
inline WheelSpeeds mecanumInverseKinematics(
    float vx, float /*vy*/,
    float omega,
    float L = ROBOT_LENGTH / 2.0f,
    float W = ROBOT_WIDTH / 2.0f
) {
    return differentialInverseKinematics(vx, omega, L, W);
}

/**
 * Forward kinematics cho 4WD differential — average 2 bánh mỗi bên.
 */
inline VelocityCommand mecanumForwardKinematics(
    float w_fl, float w_rl, float w_fr, float w_rr,
    float /*L*/ = ROBOT_LENGTH / 2.0f,
    float W = ROBOT_WIDTH / 2.0f
) {
    float r = WHEEL_RADIUS;

    VelocityCommand cmd;

    // 4WD differential: left wheel = (w_fl + w_rl)/2, right wheel = (w_fr + w_rr)/2
    float w_left  = (w_fl + w_rl) * 0.5f;
    float w_right = (w_fr + w_rr) * 0.5f;

    cmd.vx    = r * (w_left + w_right) * 0.5f;
    cmd.vy    = 0;   // 4WD không có lateral velocity
    cmd.omega = (r / W) * (w_left - w_right) * 0.5f;

    return cmd;
}

// ============================================================================
// VELOCITY COMMAND GENERATION
// ============================================================================

class MotorCommandGenerator {
public:
    MotorCommandGenerator() {
        cmd_.vx = 0;
        cmd_.vy = 0;
        cmd_.omega = 0;
        cmd_.timestamp_ms = 0;
    }

    /**
     * Set target velocity
     * @param vx Forward velocity (m/s)
     * @param vy Strafe velocity (m/s)
     * @param omega Angular velocity (rad/s)
     */
    void setVelocity(float vx, float vy, float omega) {
        // Clamp velocities to safe limits
        cmd_.vx = clamp(vx, -MAX_VELOCITY, MAX_VELOCITY);
        cmd_.vy = clamp(vy, -MAX_VELOCITY, MAX_VELOCITY);
        cmd_.omega = clamp(omega, -MAX_OMEGA, MAX_OMEGA);
        cmd_.timestamp_ms = millis();
    }

    /**
     * Set velocity from Pure Pursuit or navigation
     * @param linear Linear velocity (m/s)
     * @param angular Angular velocity (rad/s)
     */
    void setLinearAngular(float linear, float angular) {
        cmd_.vx = linear;
        cmd_.vy = 0;
        cmd_.omega = angular;
        cmd_.timestamp_ms = millis();
    }

    /**
     * Emergency stop - zero all velocities
     */
    void emergencyStop() {
        cmd_.vx = 0;
        cmd_.vy = 0;
        cmd_.omega = 0;
        cmd_.timestamp_ms = millis();
    }

    /**
     * Get current velocity command
     */
    const VelocityCommand& getCommand() const { return cmd_; }

    /**
     * Get wheel velocities for ESP32 PWM control
     * Returns array of 4 wheel speeds in rad/s
     */
    std::array<float, 4> getWheelVelocities() const {
        auto ws = mecanumInverseKinematics(cmd_.vx, cmd_.vy, cmd_.omega);
        return {ws.w_fl, ws.w_rl, ws.w_fr, ws.w_rr};
    }

    /**
     * Get wheel velocities in RPM (for telemetry)
     */
    std::array<float, 4> getWheelRPM() const {
        auto rpm = getWheelVelocities();
        for (auto& r : rpm) {
            r = r * 60.0f / (2.0f * M_PI);  // rad/s to RPM
        }
        return rpm;
    }

private:
    VelocityCommand cmd_;

    static constexpr float MAX_VELOCITY = 0.5f;   // 0.5 m/s max
    static constexpr float MAX_OMEGA = 3.0f;     // 3 rad/s max

    static inline float clamp(float val, float min, float max) {
        return std::max(min, std::min(max, val));
    }

    static inline uint32_t millis() {
        return (uint32_t)(esp_timer_get_time() / 1000);
    }
};

// ============================================================================
// UDP PACKET PROTOCOL
// ============================================================================

/**
 * Motor command packet structure for ESP32
 *
 * Format: Binary, little-endian
 * Bytes:  0     1     2-3    4-5    6-7    8
 * Type:  HEAD  CMD   VX_H   VX_L   VY_H   VY_L  ...
 *        ...   W_H   W_L    CHK
 *
 * - HEAD: 0xAA (sync byte)
 * - CMD:  0x01 (velocity command)
 * - VX:   int16 (velocity_x in mm/s, signed)
 * - VY:   int16 (velocity_y in mm/s, signed)
 * - W:    int16 (omega in mrad/s, signed)
 * - CHK:  XOR checksum of bytes 1-7
 */
class MotorCommandPacket {
public:
    static constexpr uint8_t PACKET_HEADER = 0xAA;
    static constexpr uint8_t CMD_VELOCITY = 0x01;
    static constexpr uint8_t CMD_STOP = 0x02;
    static constexpr uint8_t CMD_CONFIG = 0x03;

    static constexpr size_t PACKET_SIZE = 9;

    /**
     * Encode velocity command to binary packet
     */
    static std::array<uint8_t, PACKET_SIZE> encode(
        int16_t vx_mm, int16_t vy_mm, int16_t omega_mrad
    ) {
        std::array<uint8_t, PACKET_SIZE> packet;

        packet[0] = PACKET_HEADER;
        packet[1] = CMD_VELOCITY;
        packet[2] = (uint8_t)(vx_mm & 0xFF);
        packet[3] = (uint8_t)((vx_mm >> 8) & 0xFF);
        packet[4] = (uint8_t)(vy_mm & 0xFF);
        packet[5] = (uint8_t)((vy_mm >> 8) & 0xFF);
        packet[6] = (uint8_t)(omega_mrad & 0xFF);
        packet[7] = (uint8_t)((omega_mrad >> 8) & 0xFF);
        packet[8] = calculateChecksum(packet.data(), 8);

        return packet;
    }

    /**
     * Encode stop command
     */
    static std::array<uint8_t, PACKET_SIZE> encodeStop() {
        return encode(0, 0, 0);
    }

    /**
     * Calculate XOR checksum
     */
    static uint8_t calculateChecksum(const uint8_t* data, size_t len) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < len; ++i) {
            checksum ^= data[i];
        }
        return checksum;
    }

    /**
     * Verify packet checksum
     */
    static bool verify(const uint8_t* packet, size_t len) {
        if (len < PACKET_SIZE) return false;
        uint8_t expected = calculateChecksum(packet, 8);
        return packet[8] == expected;
    }
};

// ============================================================================
// TELEMETRY PARSER (ESP32 → Android)
// ============================================================================

/**
 * Telemetry packet structure from ESP32
 *
 * Format: Binary, little-endian
 * Bytes:  0     1     2-3    4-5    6-7    8     9
 * Type:  HEAD  CMD   X_H    X_L    Y_H    Y_L   BAT   CHK
 *
 * - HEAD: 0xBB (different from command)
 * - CMD:  0x10 (telemetry)
 * - X:    int16 (position_x in mm, signed)
 * - Y:    int16 (position_y in mm, signed)
 * - BAT:  uint8 (battery voltage in 0.1V units, e.g., 125 = 12.5V)
 * - CHK:  XOR checksum
 */
struct RobotTelemetry {
    int16_t pos_x;       // mm
    int16_t pos_y;       // mm
    int16_t heading;     // mrad
    uint8_t battery;     // 0.1V units
    uint8_t status;      // Status flags

    bool parse(const uint8_t* packet, size_t len) {
        if (len < 9) return false;
        if (packet[0] != 0xBB) return false;

        uint8_t expected = MotorCommandPacket::calculateChecksum(packet, 8);
        if (packet[8] != expected) return false;

        pos_x = packet[2] | (packet[3] << 8);
        pos_y = packet[4] | (packet[5] << 8);
        heading = packet[6] | (packet[7] << 8);
        battery = packet[9];
        status = packet[10];

        return true;
    }

    float getBatteryVoltage() const { return battery / 10.0f; }
    float getPosXMeters() const { return pos_x / 1000.0f; }
    float getPosYMeters() const { return pos_y / 1000.0f; }
    float getHeadingRadians() const { return heading / 1000.0f; }
};

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Apply exponential smoothing to velocity commands
 * Helps reduce jerky movements
 */
class VelocitySmoother {
public:
    VelocitySmoother(float alpha = 0.8f) : alpha_(alpha) {}

    VelocityCommand smooth(const VelocityCommand& raw) {
        VelocityCommand out;
        out.timestamp_ms = raw.timestamp_ms;

        if (!initialized_) {
            out = raw;
            initialized_ = true;
        } else {
            out.vx = alpha_ * raw.vx + (1 - alpha_) * prev_.vx;
            out.vy = alpha_ * raw.vy + (1 - alpha_) * prev_.vy;
            out.omega = alpha_ * raw.omega + (1 - alpha_) * prev_.omega;
        }

        prev_ = out;
        return out;
    }

    void reset() {
        initialized_ = false;
        prev_ = {0, 0, 0, 0};
    }

private:
    float alpha_;
    bool initialized_ = false;
    VelocityCommand prev_;
};

#endif // ROBOT_MOTOR_COMMAND_H
