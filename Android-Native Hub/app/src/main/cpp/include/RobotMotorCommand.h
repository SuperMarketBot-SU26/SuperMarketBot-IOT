/**
 * RobotMotorCommand.h
 * Motor command generator and UDP communication for ESP32
 *
 * This module handles:
 * 1. Velocity command generation (vx, vy, omega)
 * 2. Mecanum inverse kinematics
 * 3. WiFi UDP packet transmission
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

// Mecanum wheel configuration (X-pattern)
// Angles of rollers relative to wheel axis
constexpr float MECANUM_ROLLER_ANGLE = 45.0f;  // degrees

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
// INVERSE KINEMATICS - Mecanum Wheels
// ============================================================================

/**
 * Convert robot velocity (vx, vy, omega) to individual wheel velocities
 * For mecanum wheels with X-pattern configuration
 *
 * @param vx Robot forward velocity (m/s)
 * @param vy Robot strafe velocity (m/s)
 * @param omega Robot angular velocity (rad/s)
 * @param L Robot half-length (wheelbase)
 * @param W Robot half-width (wheelbase)
 * @return WheelSpeeds in rad/s for each wheel
 */
inline WheelSpeeds mecanumInverseKinematics(
    float vx, float vy, float omega,
    float L = ROBOT_LENGTH / 2.0f,
    float W = ROBOT_WIDTH / 2.0f
) {
    // Standard mecanum inverse kinematics
    // Assuming wheel arrangement:
    //   FL   FR
    //   RL   RR
    //
    // Wheel layout (X-pattern):
    //   /    \
    //  FL    FR
    //   \    /
    //   /    \
    //  RL    RR
    //   \    /

    float r = WHEEL_RADIUS;

    // Calculate wheel angular velocities
    // For X-pattern mecanum:
    // v_fl = (vx - vy - (L + W) * omega) / r
    // v_rl = (vx + vy - (L + W) * omega) / r
    // v_fr = (vx + vy + (L + W) * omega) / r
    // v_rr = (vx - vy + (L + W) * omega) / r

    float sum_lw = (L + W);

    WheelSpeeds ws;
    ws.w_fl = (vx - vy - sum_lw * omega) / r;
    ws.w_rl = (vx + vy - sum_lw * omega) / r;
    ws.w_fr = (vx + vy + sum_lw * omega) / r;
    ws.w_rr = (vx - vy + sum_lw * omega) / r;

    return ws;
}

/**
 * Convert wheel velocities back to robot velocity (forward kinematics)
 * Used for odometry verification
 */
inline VelocityCommand mecanumForwardKinematics(
    float w_fl, float w_rl, float w_fr, float w_rr,
    float L = ROBOT_LENGTH / 2.0f,
    float W = ROBOT_WIDTH / 2.0f
) {
    float r = WHEEL_RADIUS;

    VelocityCommand cmd;

    // Average to reduce noise
    float w_sum1 = w_fl + w_rr;
    float w_sum2 = w_rl + w_fr;
    float w_diff1 = w_fl - w_rr;
    float w_diff2 = w_fr - w_rl;

    cmd.vx = (r / 4.0f) * (w_sum1 + w_sum2);
    cmd.vy = (r / 4.0f) * (-w_sum1 + w_sum2);
    cmd.omega = (-r / (4.0f * (L + W))) * (-w_diff1 + w_diff2);

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
