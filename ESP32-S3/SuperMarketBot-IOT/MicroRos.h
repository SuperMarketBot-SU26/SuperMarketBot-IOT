// ===========================================================
// MicroRos.h — ESP32-S3 = micro-ROS node
// Publish: /scan (LiDAR), /odom (wheel encoder), /imu/data (MPU6050)
// Subscribe: /cmd_vel (motor speed command)
//
// Yêu cầu Arduino IDE:
//   - Board: ESP32S3 Dev Module
//   - Cài thư viện qua Library Manager:
//       • micro_ros_arduino (https://github.com/micro-ROS/micro_ros_arduino)
//   - Hoặc PlatformIO: lib_deps = micro_ros_arduino
//
// Compatible với ROS2 Lyrical (Ubuntu 26.04) và Humble (Ubuntu 22.04).
// ===========================================================

#ifndef MICROROS_H
#define MICROROS_H

#include "Config.h"
#include "PidController.h"   // heading lock: pidYawReset / pidYawCompute
#include "WaypointNav.h"     // heading lock: wpNormalizeAngle
#include "Localization.h"    // g_pose, g_imuEnabled (for IMU heading lock)
#include <Arduino.h>

#if defined(USE_MICRO_ROS) && (USE_MICRO_ROS == 1)
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>  // v2.1: rmw_uros_sync_session + epoch_millis

// Message types
#include <sensor_msgs/msg/laser_scan.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

// WiFi transport (UDP)
#include <WiFi.h>

// Forward declarations — actual types defined in YdlidarX3.h / Localization.h
// Placed at GLOBAL scope (NOT inside microRos namespace) so the linker
// correctly finds the definitions in YdlidarX3.h / Localization.h.
extern struct X3Scan g_x3Scan;
extern struct Pose2D g_pose;

namespace microRos {

// micro-ROS objects
static rcl_node_t       g_node;
static rcl_allocator_t  g_allocator;
static rclc_support_t   g_support;
static rclc_executor_t  g_executor;

// Publishers
static rcl_publisher_t  g_scan_pub;
static rcl_publisher_t  g_odom_pub;
static rcl_publisher_t  g_imu_pub;

// Subscribers
static rcl_subscription_t g_cmd_vel_sub;

// Messages
static sensor_msgs__msg__LaserScan  g_scan_msg;
static nav_msgs__msg__Odometry      g_odom_msg;
static sensor_msgs__msg__Imu        g_imu_msg;
static geometry_msgs__msg__Twist    g_cmd_vel_msg;

// State
static bool   g_initialized = false;
static String g_agent_ip = MICRO_ROS_AGENT_IP;
static int    g_agent_port = MICRO_ROS_AGENT_PORT;
static uint32_t g_last_scan_ms = 0;
static uint32_t g_last_odom_ms = 0;
static uint32_t g_last_imu_ms = 0;

// Odometry angular velocity state
static uint32_t s_odomPrevMs = 0;
static float    s_odomPrevHeading = 0.f;

// ============================================================
// CALLBACK: nhận /cmd_vel từ ROS2 → điều khiển motor
// ============================================================
static void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    float lin = msg->linear.x;
    float ang = msg->angular.z;

    // Gate: WebUI joystick còn tươi → bỏ qua /cmd_vel từ ROS2.
    // Threshold 300ms phù hợp với WebUI publish joystick ở ~10Hz.
    const uint32_t nowMs = millis();
    const uint32_t joyAgeMs = (g_state.joyLastMs != 0)
        ? (nowMs - g_state.joyLastMs) : 0xFFFFFFFFu;
    if (joyAgeMs < 300u) {
        return;
    }

    constexpr float ROS2_ANG_MIN = 0.02f;
    constexpr float ROS2_ANG_MAX_ROT = 1.50f;  // Aligned with Nav2 operational turning profile (was 4.00)
    constexpr float ROS2_ANG_MAX_FWD = 1.00f;
    constexpr float ROS2_LIN_MIN = 0.005f;
    constexpr float ROS2_LIN_MAX = 0.40f;
    constexpr int32_t ROS2_PWM_MIN = 320;  // Raised from 280 to 320: increase linear driving power for crisper and faster movement responsiveness
    constexpr int32_t ROS2_PWM_MIN_ROT = 560;  // 560 (~55% power): robust initial pulse torque to overcome skid-steer static floor friction
    constexpr int32_t ROS2_PWM_MAX_ROT = 700;  // 700 (~68% power): controlled burst momentum during stepwise rotation pulses
    constexpr int32_t ROS2_PWM_MAX = (int32_t)PWM_MAX;

    g_state.cmd_velLastMs = nowMs;
    // Set moving flag: true if this cmd_vel is above deadzone minimums
    g_state.cmd_velMoving = (fabs(lin) > ROS2_LIN_MIN || fabs(ang) > ROS2_ANG_MIN);
    // Removed repetitive cmd_vel serial logging to conserve CPU cycles and prevent UART buffer stalling

    enum class MotionMode : uint8_t { STOPPED, LINEAR, ANGULAR };
    static MotionMode s_lastMotionMode = MotionMode::STOPPED;
    static uint32_t s_transitionStartMs = 0;
    static uint32_t s_stoppedStartMs = 0;
    static uint32_t s_stepCycleStartMs = 0;
    // Stepwise linear driving tracking variables
    static float s_linStartX = 0.0f;
    static float s_linStartY = 0.0f;
    static bool s_linTracking = false;
    static uint32_t s_linPauseStartMs = 0;

    if (fabs(ang) > ROS2_ANG_MIN && fabs(lin) < 0.005f) {
        // ── Xoay tại chỗ (chỉ khi linear thực sự bằng 0) ─────────────
        s_linTracking = false;
        s_linPauseStartMs = 0;
        s_stoppedStartMs = 0;
        if (s_lastMotionMode == MotionMode::LINEAR) {
            if (s_transitionStartMs == 0) {
                s_transitionStartMs = nowMs;
            }
            if (nowMs - s_transitionStartMs < 800u) {
                // Pause 800ms when transitioning from linear driving to rotation so kinetic momentum settles and SLAM stabilizes
                ::botStop();
                return;
            }
            s_transitionStartMs = 0;
        }
        s_lastMotionMode = MotionMode::ANGULAR;

        // ── Stepwise "Pulse-and-Wait" Turning Controller ─────────────
        // Overcomes skid-steer static friction via short torque pulses while
        // inserting zero-velocity pauses between steps. During stationary intervals,
        // LiDAR scans remain unblurred so Ceres scan matching reliably aligns orientation in RViz.
        if (s_stepCycleStartMs == 0 || (nowMs - s_stepCycleStartMs > 850u)) {
            s_stepCycleStartMs = nowMs;
        }
        const uint32_t elapsed = nowMs - s_stepCycleStartMs;
        if (elapsed >= 210u) {
            // Pause phase (210ms .. 850ms): hold completely still for 640ms so LiDAR mirror completes
            // four clean stationary 360° sweeps and Ceres solver firmly snaps scan orientation onto the map.
            ::botStop();
            return;
        }

        const float angMag = fabsf(ang);
        int32_t pwm = (int32_t)(((angMag - ROS2_ANG_MIN) /
                                 (ROS2_ANG_MAX_ROT - ROS2_ANG_MIN)) *
                                (ROS2_PWM_MAX_ROT - ROS2_PWM_MIN_ROT) + ROS2_PWM_MIN_ROT);
        if (pwm < ROS2_PWM_MIN_ROT) pwm = ROS2_PWM_MIN_ROT;
        if (pwm > ROS2_PWM_MAX_ROT) pwm = ROS2_PWM_MAX_ROT;
        if (ang > 0) ::botRotateCCW((uint16_t)pwm);
        else         ::botRotateCW((uint16_t)pwm);
    } else if (fabs(lin) >= 0.005f || fabs(ang) > ROS2_ANG_MIN) {
        // ── Đi thẳng / rẽ / cong (Arcade Mix) ─────────────────────────
        s_stepCycleStartMs = 0;
        s_stoppedStartMs = 0;
        if (s_lastMotionMode == MotionMode::ANGULAR) {
            if (s_transitionStartMs == 0) {
                s_transitionStartMs = nowMs;
            }
            if (nowMs - s_transitionStartMs < 800u) {
                // Pause 800ms when transitioning from rotation to linear advancement so SLAM locks final heading transform
                ::botStop();
                return;
            }
            s_transitionStartMs = 0;
        }
        s_lastMotionMode = MotionMode::LINEAR;

        // ── Stepwise "Pulse-and-Wait" Linear Driving Controller ────────
        // Halts the robot after every 35cm (0.35m) of linear travel for a 350ms pause.
        // During stationary settling, kinetic damping finishes immediately and SLAM Toolbox
        // captures clean, zero-speed LiDAR sweeps to maintain pinpoint map alignment.
        if (s_linPauseStartMs > 0) {
            if (nowMs - s_linPauseStartMs < 350u) {
                // Hold completely still for 350ms so LiDAR scans snap onto map walls
                ::botStop();
                return;
            }
            // Pause window completed: reset position tracking for next 35cm movement step
            s_linPauseStartMs = 0;
            s_linTracking = false;
        }

        if (!s_linTracking) {
            s_linStartX = g_pose.x;
            s_linStartY = g_pose.y;
            s_linTracking = true;
        } else {
            float distTraveled = hypotf(g_pose.x - s_linStartX, g_pose.y - s_linStartY);
            if (distTraveled >= 0.35f) {
                // Reached 35cm target: initiate stationary scan pause
                s_linPauseStartMs = nowMs;
                ::botStop();
                return;
            }
        }

        // ── Đi thẳng / rẽ (Arcade Mix với Deadband Mapping) ────────────────
        float normFwd = lin / ROS2_LIN_MAX;
        float normRot = (ang / ROS2_ANG_MAX_FWD) * 1.25f;  // Apply 1.25x rotational torque gain to ensure smooth cornering against floor friction

        // Trong ROS, ang > 0 là xoay TRÁI (CCW).
        // Xoay trái -> Bánh phải phải quay nhanh hơn bánh trái.
        float normLeft  = normFwd - normRot;
        float normRight = normFwd + normRot;

        float maxNorm = max(fabsf(normLeft), fabsf(normRight));
        if (maxNorm > 1.0f) {
            normLeft  /= maxNorm;
            normRight /= maxNorm;
        }

        // Map the normalized speed to physical PWM, ensuring it bypasses static gear/carpet resistance during curves
        auto mapPwm = [&](float norm) -> int32_t {
            if (fabsf(norm) < 0.01f) return 0;
            // Balanced starting voltage during turning curves (460 vs 280) to maintain momentum without wheel slippage
            int32_t activeMinPwm = (fabsf(normRot) > 0.05f) ? 460 : ROS2_PWM_MIN;
            int32_t p = (int32_t)(fabsf(norm) * (ROS2_PWM_MAX - activeMinPwm) + activeMinPwm);
            if (p > ROS2_PWM_MAX) p = ROS2_PWM_MAX;
            return (norm >= 0) ? p : -p;
        };

        int32_t leftPwm  = mapPwm(normLeft);
        int32_t rightPwm = mapPwm(normRight);
        locSetDriveCmd(
            (int16_t)constrain((int)(leftPwm  * 100L / ROS2_PWM_MAX), -100, 100),
            (int16_t)constrain((int)(rightPwm * 100L / ROS2_PWM_MAX), -100, 100));
        const int32_t sp[4] = {leftPwm, leftPwm, rightPwm, rightPwm};
        // ROS2 đã thực sự điều khiển motor → đánh dấu để controlTask
        // MODE_MANUAL bỏ qua (xem SuperMarketBot-IOT.ino case MODE_MANUAL).
        g_state.cmd_velLastMs = nowMs;
        ::motorApplyLayout(sp);
    } else {
        // lin quá nhỏ và ang quá nhỏ → không lái. Reset heading lock để
        // lần tới di chuyển có target tươi.
        if (s_stoppedStartMs == 0) s_stoppedStartMs = nowMs;
        if (nowMs - s_stoppedStartMs >= 800u) {
            // Once stationary for 800ms+, reset motion mode so future initial driving commands start immediately without extra transition delay
            s_lastMotionMode = MotionMode::STOPPED;
        }
        s_stepCycleStartMs = 0;
        s_linTracking = false;
        s_linPauseStartMs = 0;
        s_transitionStartMs = 0;
        ::botStop();
    }
}

// ============================================================
// TIMESTAMP HELPER: Dùng epoch wall-clock đã sync từ agent
// ============================================================
static void set_synced_stamp(int32_t &sec, uint32_t &nanosec, int32_t offset_ms = 0) {
    int64_t epoch_ms = rmw_uros_epoch_millis() + offset_ms;
    if (epoch_ms > 1000000000LL) {
        sec = (int32_t)(epoch_ms / 1000);
        nanosec = (uint32_t)((epoch_ms % 1000) * 1000000UL);
    } else {
        const int64_t nowMs = (int64_t)millis() + offset_ms;
        const uint32_t posMs = (nowMs > 0) ? (uint32_t)nowMs : 0;
        sec = (int32_t)(posMs / 1000);
        nanosec = (posMs % 1000) * 1000000UL;
    }
}

// ============================================================
// FILL LaserScan message từ g_x3Scan
// ============================================================
static void fill_scan_msg() {
    // Update header string fields on every publish
    g_scan_msg.header.frame_id.data = (char*)"laser_frame";
    g_scan_msg.header.frame_id.size = 11;
    g_scan_msg.header.frame_id.capacity = 12;
    // Align scan timestamps directly with real-time odometry instantly so point clouds do not lag behind the robot in RViz
    set_synced_stamp(g_scan_msg.header.stamp.sec, g_scan_msg.header.stamp.nanosec, 0);

    g_scan_msg.angle_min = 0.0f;
    g_scan_msg.angle_max = 2.0f * M_PI;
    g_scan_msg.angle_increment = (2.0f * M_PI) / 360.0f;  // Restored to 360 beams (1.0 deg/beam) to prevent WiFi UDP fragmentation
    g_scan_msg.time_increment = 0.0f;
    g_scan_msg.scan_time = 0.1f;
    g_scan_msg.range_min = 0.12f;
    g_scan_msg.range_max = 8.0f;

    // Init ranges array — capacity fields REQUIRED for CDR deserialization
    if (g_scan_msg.ranges.data == NULL) {
        g_scan_msg.ranges.data = (float*)malloc(360 * sizeof(float));
        g_scan_msg.ranges.size = 360;
        g_scan_msg.ranges.capacity = 360;
        if (g_scan_msg.ranges.data == NULL) {
            Serial.println("[scan] malloc FAILED");
            return;
        }
    }

    for (size_t i = 0; i < 360; i++) g_scan_msg.ranges.data[i] = 0.0f;

    // intensities — explicitly zero-length to avoid garbage in CDR payload.
    // The YDLIDAR X3 does not return intensity values; leaving the struct
    // fields at their zero-initialized values is safe because micro-ROS
    // CDR serializes the size/capacity fields.  Explicitly setting them
    // avoids any ambiguity between library versions on how to handle an
    // uninitialized sequence.
    g_scan_msg.intensities.data     = NULL;
    g_scan_msg.intensities.size     = 0;
    g_scan_msg.intensities.capacity = 0;

    for (uint16_t i = 0; i < g_x3Scan.count; i++) {
        const LidarPoint &p = g_x3Scan.points[i];
        if (p.distanceMm < 120 || p.distanceMm > 8000) continue;
        if (p.quality < 10) continue;

        // YDLidar X3 spins clockwise, but ROS expects counter-clockwise.
        // If we don't invert, physical right appears on RViz left!
        float ros_angle = (2.0f * (float)M_PI) - p.angleRad;
        if (ros_angle < 0.0f) ros_angle += 2.0f * (float)M_PI;
        if (ros_angle >= 2.0f * (float)M_PI) ros_angle -= 2.0f * (float)M_PI;

        int idx = (int)(ros_angle / (2.0f * M_PI) * (float)g_scan_msg.ranges.size);
        if (idx < 0) idx += g_scan_msg.ranges.size;
        if (idx >= (int)g_scan_msg.ranges.size) idx = g_scan_msg.ranges.size - 1;

        float dist_m = (float)p.distanceMm / 1000.0f;
        float &slot = g_scan_msg.ranges.data[idx];
        if (slot == 0.0f || dist_m < slot) slot = dist_m;
    }
}

// ============================================================
// FILL Odometry message
// ============================================================
static void fill_odom_msg() {
    const Pose2D &pose = g_pose;

    // ---- Header ----
    g_odom_msg.header.frame_id.data = (char*)"odom";
    g_odom_msg.header.frame_id.size = 4;
    g_odom_msg.header.frame_id.capacity = 5;
    g_odom_msg.child_frame_id.data = (char*)"base_link";
    g_odom_msg.child_frame_id.size = 10;
    g_odom_msg.child_frame_id.capacity = 11;
    const uint32_t nowMs = millis();
    set_synced_stamp(g_odom_msg.header.stamp.sec, g_odom_msg.header.stamp.nanosec);

    g_odom_msg.pose.pose.position.x = pose.x;
    g_odom_msg.pose.pose.position.y = pose.y;
    g_odom_msg.pose.pose.position.z = 0.0f;

    float h = pose.headingRad * 0.5f;
    g_odom_msg.pose.pose.orientation.x = 0.0f;
    g_odom_msg.pose.pose.orientation.y = 0.0f;
    g_odom_msg.pose.pose.orientation.z = sinf(h);
    g_odom_msg.pose.pose.orientation.w  = cosf(h);

    // Kinematic Slip Guard: prevent single-channel encoder scrubbing during turns from spiking forward twist velocity in RViz
    float avgRpm = 0.f;
    if ((g_state.rpmFL * g_state.rpmFR) < 0.f) {
        avgRpm = 0.f;
    } else {
        const float diffRpm = fabsf(g_state.rpmFR - g_state.rpmFL);
        const float meanRpm = (g_state.rpmFL + g_state.rpmFR) * 0.5f;
        if (diffRpm > fabsf(meanRpm) && diffRpm > 5.0f) {
            avgRpm = (fabsf(g_state.rpmFL) < fabsf(g_state.rpmFR)) ? g_state.rpmFL : g_state.rpmFR;
        } else {
            avgRpm = meanRpm;
        }
    }
    const float wheelRps = avgRpm / 60.0f;
    const float linearMps = wheelRps * (float)WHEEL_CIRC_M;
    g_odom_msg.twist.twist.linear.x = linearMps;
    g_odom_msg.twist.twist.linear.y = 0.0f;
    g_odom_msg.twist.twist.linear.z = 0.0f;

    // Angular velocity from delta heading / dt
    if (s_odomPrevMs != 0) {
        const float dt = (float)(nowMs - s_odomPrevMs) / 1000.0f;
        if (dt > 0.001f && dt < 5.0f) {
            float dH = pose.headingRad - s_odomPrevHeading;
            while (dH >  (float)M_PI) dH -= 2.f * (float)M_PI;
            while (dH < -(float)M_PI) dH += 2.f * (float)M_PI;
            g_odom_msg.twist.twist.angular.x = 0.0f;
            g_odom_msg.twist.twist.angular.y = 0.0f;
            g_odom_msg.twist.twist.angular.z = dH / dt;
        }
    } else {
        g_odom_msg.twist.twist.angular.x = 0.0f;
        g_odom_msg.twist.twist.angular.y = 0.0f;
        g_odom_msg.twist.twist.angular.z = 0.0f;
    }
    s_odomPrevMs = nowMs;
    s_odomPrevHeading = pose.headingRad;

    for (int i = 0; i < 36; i++) {
        g_odom_msg.pose.covariance[i] = 0.0f;
        g_odom_msg.twist.covariance[i] = 0.0f;
    }
    g_odom_msg.pose.covariance[0] = 0.05f;
    g_odom_msg.pose.covariance[7] = 0.05f;
    g_odom_msg.pose.covariance[35] = 0.10f;
    g_odom_msg.twist.covariance[0] = 0.02f;
    g_odom_msg.twist.covariance[35] = 0.10f;
}

// ============================================================
// FILL Imu message
// ============================================================
static uint32_t s_imuPrevGyroZ_ms __attribute__((unused)) = 0;
static float    s_imuPrevGyroZ_radps __attribute__((unused)) = 0.f;

static void fill_imu_msg() {
    const Pose2D &pose = g_pose;

    float h = pose.headingRad * 0.5f;
    g_imu_msg.orientation.x = 0.0f;
    g_imu_msg.orientation.y = 0.0f;
    g_imu_msg.orientation.z = sinf(h);
    g_imu_msg.orientation.w = cosf(h);

    float gyroZ = 0.f;
    bool gyroOk = false;
#if USE_IMU_MPU6050
    gyroOk = ::imuMpu6050GetGyroZ(gyroZ);
#endif
    g_imu_msg.angular_velocity.x = 0.0f;
    g_imu_msg.angular_velocity.y = 0.0f;
    g_imu_msg.angular_velocity.z = gyroOk ? gyroZ : 0.0f;

    // Only log hardware sensor errors; suppress periodic status output during normal operation to preserve throughput
    static uint32_t s_lastImuPubLog = 0;
    const uint32_t nowMs = millis();
    if (nowMs - s_lastImuPubLog >= 5000) {
        s_lastImuPubLog = nowMs;
        if (!gyroOk) {
            Serial.println("[uROS-IMU] gyroOk=0 — MPU6050 read FAILED, angular_velocity.z = 0");
        }
    }

    g_imu_msg.linear_acceleration.x = 0.0f;
    g_imu_msg.linear_acceleration.y = 0.0f;
    g_imu_msg.linear_acceleration.z = 9.81f;

    for (int i = 0; i < 9; i++) {
        g_imu_msg.orientation_covariance[i] = 0.0f;
        g_imu_msg.angular_velocity_covariance[i] = 0.0f;
        g_imu_msg.linear_acceleration_covariance[i] = 0.0f;
    }
    g_imu_msg.orientation_covariance[0] = -1.0f;
    g_imu_msg.orientation_covariance[4] = -1.0f;
    g_imu_msg.orientation_covariance[8] = 0.02f;
    g_imu_msg.angular_velocity_covariance[0] = -1.0f;
    g_imu_msg.angular_velocity_covariance[4] = -1.0f;
    g_imu_msg.angular_velocity_covariance[8] = gyroOk ? 0.001f : -1.0f;
    g_imu_msg.linear_acceleration_covariance[0] = -1.0f;
    g_imu_msg.linear_acceleration_covariance[4] = -1.0f;
    g_imu_msg.linear_acceleration_covariance[8] = 0.01f;

    g_imu_msg.header.frame_id.data = (char*)"imu_link";
    g_imu_msg.header.frame_id.size = 9;
    g_imu_msg.header.frame_id.capacity = 10;
    set_synced_stamp(g_imu_msg.header.stamp.sec, g_imu_msg.header.stamp.nanosec);
}

// ============================================================
// INIT: Khởi tạo micro-ROS node
// ============================================================
inline bool init() {
    if (g_initialized) return true;

    set_microros_wifi_transports(
        (char*)WiFi.SSID().c_str(),
        (char*)WiFi.psk().c_str(),
        (char*)g_agent_ip.c_str(),
        (uint16_t)g_agent_port
    );

    // Ping agent before initializing XRCE-DDS session to avoid blocking or creating invalid handles
    if (rmw_uros_ping_agent(500, 2) != RMW_RET_OK) {
        return false;
    }

    g_allocator = rcl_get_default_allocator();
    if (rclc_support_init(&g_support, 0, NULL, &g_allocator) != RCL_RET_OK) {
        Serial.println(F("[micro-ROS] rclc_support_init failed"));
        return false;
    }

    // Synchronize clock timestamps after establishing active XRCE-DDS connection
    const int SYNC_TIMEOUT_MS = 1000;
    if (!rmw_uros_sync_session(SYNC_TIMEOUT_MS)) {
        Serial.printf("[micro-ROS] WARNING: clock sync failed (agent=%s:%d). Map drag may occur.\n",
                      g_agent_ip.c_str(), g_agent_port);
    } else {
        Serial.println(F("[micro-ROS] Clock synced with agent."));
    }

    rclc_node_init_default(&g_node, "supermarketbot_esp32", "", &g_support);

    // Publish /scan as RELIABLE (default). 
    // A 1500-byte LaserScan requires XRCE-DDS fragmentation (default MTU 512). 
    // Micro-XRCE-DDS only supports fragmentation on the Reliable stream.
    // If published as BEST_EFFORT, the message is silently dropped.
    rclc_publisher_init_default(
        &g_scan_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "/scan"
    );
    rclc_publisher_init_default(
        &g_odom_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"
    );
    rclc_publisher_init_default(
        &g_imu_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );

    // /cmd_vel: BEST_EFFORT, depth=1. A stale velocity command that arrives
    // late (WiFi jitter, DDS retransmit) is unsafe to replay — it could
    // resume a high-throttle PWM that was issued before a stop or ESTOP.
    // The watchdog in spin() (500 ms) and MODE_MANUAL cvAgeMs gate (300 ms)
    // are the safety net when commands stop arriving.
    // NOTE: rclc_subscription_init_best_effort uses depth=1 by default.
    rclc_subscription_init_best_effort(
        &g_cmd_vel_sub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    );

    rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator);
    rclc_executor_add_subscription(
        &g_executor, &g_cmd_vel_sub, &g_cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA
    );

    // Init messages — capacity fields REQUIRED for CDR deserialization
    g_scan_msg.header.frame_id.data = (char*)"laser_frame";
    g_scan_msg.header.frame_id.size = 11;
    g_scan_msg.header.frame_id.capacity = 12;

    g_odom_msg.header.frame_id.data = (char*)"odom";
    g_odom_msg.header.frame_id.size = 4;
    g_odom_msg.header.frame_id.capacity = 5;
    g_odom_msg.child_frame_id.data = (char*)"base_link";
    g_odom_msg.child_frame_id.size = 10;
    g_odom_msg.child_frame_id.capacity = 11;

    g_imu_msg.header.frame_id.data = (char*)"imu_link";
    g_imu_msg.header.frame_id.size = 9;
    g_imu_msg.header.frame_id.capacity = 10;

    g_initialized = true;
    Serial.printf("[micro-ROS] Node ready. agent=%s:%d\n",
                  g_agent_ip.c_str(), g_agent_port);
    return true;
}

// ============================================================
// SPIN
// ============================================================
inline void spin() {
    if (!g_initialized) return;

    rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(5));

    // Re-sync every 10 seconds to prevent clock drift (ESP32 vs agent clocks
    // can diverge ~10-50ms/minute depending on WiFi jitter). A short ping
    // keeps timestamps aligned without blocking the control loop.
    static uint32_t s_last_sync_ms = 0;
    uint32_t now = millis();
    if (now - s_last_sync_ms >= 10000) {
        s_last_sync_ms = now;
        rmw_uros_sync_session(100);  // 100ms timeout for ping
    }

    // Publish scan when a brand new 360° rotation finishes, throttled to 120ms (~8 Hz) to match physical sensor RPM without flooding Wi-Fi UDP buffers
    if (g_x3Scan.scanReady && (now - g_last_scan_ms >= 120)) {
        g_last_scan_ms = now;
        g_x3Scan.scanReady = false; // consume fresh scan
        fill_scan_msg();
        rcl_publish(&g_scan_pub, &g_scan_msg, NULL);
    }

    if (now - g_last_odom_ms >= 20) {
        g_last_odom_ms = now;
        fill_odom_msg();
        rcl_publish(&g_odom_pub, &g_odom_msg, NULL);
    }

    if (now - g_last_imu_ms >= 20) {
        g_last_imu_ms = now;
        fill_imu_msg();
        rcl_publish(&g_imu_pub, &g_imu_msg, NULL);
    }
}

inline void tick() { spin(); }
inline bool isInitialized() { return g_initialized; }

}  // namespace microRos
#else
namespace microRos {
  inline bool init() { return false; }
  inline void spin() {}
  inline void tick() {}
  inline bool isInitialized() { return false; }
}
#endif

#endif  // MICROROS_H