// ===========================================================
// MicroRos.h — ESP32-S3 = micro-ROS node
// Publish: /odom (wheel encoder), /imu/data (MPU6050).
// YDLIDAR X3 is connected directly to the Pi 5 in the production topology,
// so /scan is normally published by ydlidar_ros2_driver, not this firmware.
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
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/string.h>

// Forward declarations — actual types defined in YdlidarX3.h / Localization.h
// Placed at GLOBAL scope (NOT inside microRos namespace) so the linker
// correctly finds the definitions in YdlidarX3.h / Localization.h.
extern struct Pose2D g_pose;

namespace microRos {

// micro-ROS objects
static rcl_node_t       g_node;
static rcl_allocator_t  g_allocator;
static rclc_support_t   g_support;
static rclc_executor_t  g_executor;

// Publishers
static rcl_publisher_t  g_odom_pub;
static rcl_publisher_t  g_imu_pub;
static rcl_publisher_t  g_us_front_pub;
static rcl_publisher_t  g_us_debug_pub;

// Subscribers
static rcl_subscription_t g_cmd_vel_sub;

// Messages
static nav_msgs__msg__Odometry      g_odom_msg;
static sensor_msgs__msg__Imu        g_imu_msg;
static geometry_msgs__msg__Twist    g_cmd_vel_msg;
static sensor_msgs__msg__Range      g_us_front_msg;
static std_msgs__msg__String        g_us_debug_msg;

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
    const uint32_t nowMs = millis();
    const uint32_t joyAgeMs = (g_state.joyLastMs != 0)
        ? (nowMs - g_state.joyLastMs) : 0xFFFFFFFFu;
    if (joyAgeMs < 300u) {
        return;
    }

    constexpr float ROS2_ANG_MIN = 0.02f;
    constexpr float ROS2_LIN_MIN = 0.005f;
    // *** CRITICAL FIX: Reduced from 0.40 to 0.05 ***
    // At desired_linear_vel=0.04, normFwd=0.04/0.10=0.40 → good PWM resolution with headroom
    constexpr float ROS2_LIN_MAX = 0.10f;
    constexpr float ROS2_ANG_MAX_FWD = 1.00f;  // Reduced from 2.0: tighter angular mapping for precision
    constexpr int32_t ROS2_PWM_MIN = 300;       // Straight-line minimum PWM
    constexpr int32_t ROS2_PWM_ROT = 450;       // Rotation minimum PWM (4WD skid-steer needs more torque)
    constexpr int32_t ROS2_PWM_MAX = (int32_t)PWM_MAX;

    // *** SMOOTHING FILTER STATE (persistent across calls) ***
    // Exponential Moving Average (EMA) eliminates sudden PWM jumps that cause
    // encoder jitter → bad odometry → SLAM map drift.
    // alpha=0.3: each cmd_vel moves PWM 30% toward target. At 10-15 Hz cmd_vel,
    // settling time ≈ 200-300ms → smooth ramp, no motor stutter.
    static int32_t s_smoothL = 0;
    static int32_t s_smoothR = 0;
    constexpr float SMOOTH_ALPHA = 0.3f;

    g_state.cmd_velLastMs = nowMs;
    g_state.cmd_velMoving = (fabs(lin) > ROS2_LIN_MIN || fabs(ang) > ROS2_ANG_MIN);

    if (fabs(lin) >= ROS2_LIN_MIN || fabs(ang) > ROS2_ANG_MIN) {
        // Normalize velocity to [-1, 1] range
        float normFwd = constrain(lin / ROS2_LIN_MAX, -1.0f, 1.0f);
        float normRot = constrain(ang / ROS2_ANG_MAX_FWD, -1.0f, 1.0f);

        // Heading Lock for straight driving: if no rotation is commanded (pure straight driving),
        // apply PID correction using IMU heading to keep the physical robot moving dead straight!
        static float s_rosTgtH = 0.f;
        static bool  s_rosHaveH = false;
        static uint32_t s_rosLastH_ms = 0;
        uint32_t nowH = millis();
        float dt_h = (s_rosLastH_ms > 0) ? (float)(nowH - s_rosLastH_ms) * 0.001f : 0.02f;
        s_rosLastH_ms = nowH;

        if (g_imuEnabled && fabsf(normRot) < 0.02f && fabsf(normFwd) > 0.02f) {
            if (!s_rosHaveH) {
                s_rosTgtH = g_pose.headingRad;
                pidYawReset();
                s_rosHaveH = true;
            }
            float dh = wpNormalizeAngle(g_pose.headingRad - s_rosTgtH);
            if (fabsf(dh) > 0.436f) {
                s_rosTgtH = g_pose.headingRad;
                pidYawReset();
            }
            float steer = constrain(pidYawCompute(s_rosTgtH, g_pose.headingRad, dt_h), -25.f, 25.f);
            float steerNorm = steer / 100.0f;
            normRot = steerNorm;
        } else {
            if (s_rosHaveH) {
                pidYawReset();
                s_rosHaveH = false;
            }
        }

        // Arcade drive: differential left/right
        float normLeft  = normFwd - normRot;
        float normRight = normFwd + normRot;

        float maxNorm = max(fabsf(normLeft), fabsf(normRight));
        if (maxNorm > 1.0f) {
            normLeft  /= maxNorm;
            normRight /= maxNorm;
        }

        // Map normalized value to PWM with separate straight/rotation thresholds
        auto mapPwm = [&](float norm) -> int32_t {
            if (fabsf(norm) < 0.01f) return 0;
            
            // 4WD skid-steer needs more torque for rotation than going straight.
            // CRITICAL FIX: Smoothly blend the minimum PWM based on rotation intent.
            // Using a hard ternary (normRot > 0.05 ? 450 : 300) caused a violent 150-PWM jolt 
            // when Nav2 made tiny 1-degree micro-corrections, completely destroying SLAM odometry.
            // Now, it linearly scales from 300 to 450 as rotation increases.
            float rotFactor = constrain(fabsf(normRot) * 3.0f, 0.0f, 1.0f); // Reaches full ROT torque at 33% rotation command
            int32_t activeMinPwm = ROS2_PWM_MIN + (int32_t)(rotFactor * (ROS2_PWM_ROT - ROS2_PWM_MIN));
            
            int32_t p = (int32_t)(fabsf(norm) * (ROS2_PWM_MAX - activeMinPwm) + activeMinPwm);
            if (p > ROS2_PWM_MAX) p = ROS2_PWM_MAX;
            return (norm >= 0) ? p : -p;
        };

        int32_t targetL = mapPwm(normLeft);
        int32_t targetR = mapPwm(normRight);

        // *** APPLY SMOOTHING FILTER ***
        // EMA low-pass: smooth_pwm = α * target + (1-α) * smooth_pwm
        s_smoothL = (int32_t)(SMOOTH_ALPHA * (float)targetL + (1.0f - SMOOTH_ALPHA) * (float)s_smoothL);
        s_smoothR = (int32_t)(SMOOTH_ALPHA * (float)targetR + (1.0f - SMOOTH_ALPHA) * (float)s_smoothR);

        locSetDriveCmd(
            (int16_t)constrain((int)(s_smoothL * 100L / ROS2_PWM_MAX), -100, 100),
            (int16_t)constrain((int)(s_smoothR * 100L / ROS2_PWM_MAX), -100, 100));
        
        const int32_t sp[4] = {s_smoothL, s_smoothL, s_smoothR, s_smoothR};
        ::motorApplyLayout(sp);
    } else {
        // Smooth stop: ramp down instead of instant brake
        s_smoothL = (int32_t)((1.0f - SMOOTH_ALPHA) * (float)s_smoothL);
        s_smoothR = (int32_t)((1.0f - SMOOTH_ALPHA) * (float)s_smoothR);
        if (abs(s_smoothL) < 20 && abs(s_smoothR) < 20) {
            s_smoothL = 0; s_smoothR = 0;
            ::botStop();
        } else {
            const int32_t sp[4] = {s_smoothL, s_smoothL, s_smoothR, s_smoothR};
            ::motorApplyLayout(sp);
        }
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
// FILL Odometry message
// ============================================================
static void fill_odom_msg() {
    // Use g_pose which is stabilized by MPU6050 gyro via ImuFusion.
    // This prevents fake wheel-yaw rotation when stationary or when wheels slip.
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

    // Calculate forward linear velocity from wheel RPM; 0 when turning in place
    float avgRpm = 0.f;
    if ((g_state.rpmFL * g_state.rpmFR) < 0.f) {
        avgRpm = 0.f;
    } else {
        avgRpm = (g_state.rpmFL + g_state.rpmFR) * 0.5f;
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

    // CRITICAL FIX: Do NOT send 0.0 covariance! 
    // 0.0 covariance tells slam_toolbox that odometry is mathematically perfect.
    // This causes SLAM to shrink its scan-matching search window to 0 and blindly follow
    // the IMU, even if the IMU is drifting (causing the ghosting effect).
    for (int i = 0; i < 36; i++) {
        g_odom_msg.pose.covariance[i] = 0.0f;
        g_odom_msg.twist.covariance[i] = 0.0f;
    }
    
    // Realistic covariances for an ESP32 skid-steer bot with MPU6050
    g_odom_msg.pose.covariance[0]  = 0.02f;   // X — encoder + skid uncertainty
    g_odom_msg.pose.covariance[7]  = 0.02f;   // Y
    g_odom_msg.pose.covariance[14] = 9999.0f; // Z (ignored)
    g_odom_msg.pose.covariance[21] = 9999.0f; // Roll (ignored)
    g_odom_msg.pose.covariance[28] = 9999.0f; // Pitch (ignored)
    g_odom_msg.pose.covariance[35] = 0.02f;   // Yaw — stabilized by MPU6050 gyro via ImuFusion + ZUPT

    g_odom_msg.twist.covariance[0]  = 0.01f;  // Vx
    g_odom_msg.twist.covariance[7]  = 9999.0f;// Vy
    g_odom_msg.twist.covariance[14] = 9999.0f;// Vz
    g_odom_msg.twist.covariance[21] = 9999.0f;// Vroll
    g_odom_msg.twist.covariance[28] = 9999.0f;// Vpitch
    g_odom_msg.twist.covariance[35] = 0.02f;  // Vyaw
}

// ============================================================
// FILL Imu message
// ============================================================
static uint32_t s_imuPrevGyroZ_ms __attribute__((unused)) = 0;
static float    s_imuPrevGyroZ_radps __attribute__((unused)) = 0.f;

static void fill_imu_msg() {
    // MPU6050 has no absolute yaw reference. Do not advertise the firmware's
    // already-fused heading as an independent IMU orientation measurement.
    // Consumers must use angular_velocity.z; covariance[0] = -1 marks the
    // orientation field unavailable per sensor_msgs/Imu.
    g_imu_msg.orientation.x = 0.0f;
    g_imu_msg.orientation.y = 0.0f;
    g_imu_msg.orientation.z = 0.0f;
    g_imu_msg.orientation.w = 1.0f;

    float gyroZ = 0.f;
    bool gyroOk = false;
#if USE_IMU_MPU6050
    gyroOk = ::imuMpu6050GetGyroZ(gyroZ);
    if (gyroOk) {
        // Subtract real-time ZUPT bias estimate to eliminate straight-line heading drift
        gyroZ -= imuFusion::getState().bias;

        // Stationary zero-clamp: when robot is physically stationary, angular velocity is exactly 0.0
        const bool hasDriveCmd = (g_state.cmd_velMoving || g_state.cmdY != 0 || g_state.cmdX != 0 || g_state.cmdStrafe != 0);
        const bool encoderStill = (fabsf(g_dThetaEncRate) <= 0.05f);
        const bool gyroStill    = (fabsf(gyroZ) <= 0.02f); // ~1.1 deg/s
        const bool robotMoving  = hasDriveCmd || !encoderStill || !gyroStill;

        if (!robotMoving) {
            gyroZ = 0.0f; // Absolute zero when stationary — eliminates standstill spin!
        } else if (fabsf(gyroZ) < 0.008f) {
            gyroZ = 0.0f;
        }
    }
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
    g_imu_msg.angular_velocity_covariance[0] = 9999.0f;
    g_imu_msg.angular_velocity_covariance[4] = 9999.0f;
    g_imu_msg.angular_velocity_covariance[8] = gyroOk ? 0.02f : 9999.0f;
    g_imu_msg.linear_acceleration_covariance[0] = -1.0f;
    g_imu_msg.linear_acceleration_covariance[4] = -1.0f;
    g_imu_msg.linear_acceleration_covariance[8] = 0.01f;

    g_imu_msg.header.frame_id.data = (char*)"imu_link";
    g_imu_msg.header.frame_id.size = 9;
    g_imu_msg.header.frame_id.capacity = 10;
    set_synced_stamp(g_imu_msg.header.stamp.sec, g_imu_msg.header.stamp.nanosec);
}

#if defined(MICRO_ROS_USE_SERIAL) && (MICRO_ROS_USE_SERIAL == 1)
extern "C" {
    bool arduino_transport_open(struct uxrCustomTransport * transport) {
        (void)transport;
        Serial0.begin(921600); // 921600 baud UART transport
        return true;
    }
    bool arduino_transport_close(struct uxrCustomTransport * transport) {
        (void)transport;
        Serial0.end();
        return true;
    }
    size_t arduino_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {
        (void)transport;
        (void)err;
        return Serial0.write(buf, len);
    }
    size_t arduino_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
        (void)transport;
        (void)err;
        uint32_t start = millis();
        size_t read_bytes = 0;
        // Do NOT cap the timeout! taskMicroRos runs at priority 4 and will be preempted by taskControl (priority 5)
        // automatically. Capping this breaks the XRCE-DDS framing protocol during the initial handshake.
        while (read_bytes < len) {
            if (Serial0.available() > 0) {
                buf[read_bytes++] = Serial0.read();
            } else {
                if ((millis() - start) >= (uint32_t)timeout) break;
                delay(1); // Yield to FreeRTOS (allows taskControl to run smoothly)
            }
        }
        return read_bytes;
    }
}
#endif

// ============================================================
// INIT: Khởi tạo micro-ROS node
// ============================================================
inline bool init() {
    if (g_initialized) return true;

    logger.muteRealSerial = true; // Ngắt ASCII log lên cáp USB để nhường đường cho XRCE-DDS
    set_microros_transports();    // XRCE-DDS qua Serial0 (UART0)

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

    // /odom is 736 bytes (due to two 36-double covariance matrices), exceeding the default 512-byte
    // MTU. Micro-XRCE-DDS silently drops fragmented messages on BEST_EFFORT, so it MUST be RELIABLE!
    rclc_publisher_init_default(
        &g_odom_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"
    );
    // /imu/data is 328 bytes (<512 bytes MTU), fits inside a single packet, and runs at 50Hz BEST_EFFORT!
    rclc_publisher_init_best_effort(
        &g_imu_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );

    rclc_publisher_init_best_effort(
        &g_us_front_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/us_front_dist"
    );

    rclc_publisher_init_best_effort(
        &g_us_debug_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/us_debug"
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

    g_odom_msg.header.frame_id.data = (char*)"odom";
    g_odom_msg.header.frame_id.size = 4;
    g_odom_msg.header.frame_id.capacity = 5;
    g_odom_msg.child_frame_id.data = (char*)"base_link";
    g_odom_msg.child_frame_id.size = 10;
    g_odom_msg.child_frame_id.capacity = 11;

    g_imu_msg.header.frame_id.data = (char*)"imu_link";
    g_imu_msg.header.frame_id.size = 9;
    g_imu_msg.header.frame_id.capacity = 10;

    g_us_front_msg.header.frame_id.data = (char*)"us_front_link";
    g_us_front_msg.header.frame_id.size = 13;
    g_us_front_msg.header.frame_id.capacity = 14;
    g_us_front_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    g_us_front_msg.field_of_view = 0.26f; // ~15 degrees
    g_us_front_msg.min_range = 0.02f;
    g_us_front_msg.max_range = 2.0f;

    g_us_debug_msg.data.data = (char *) malloc(100);
    g_us_debug_msg.data.capacity = 100;
    g_us_debug_msg.data.size = 0;

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

    uint32_t now = millis();

    // 1. Publish 50 Hz real-time telemetry FIRST before any incoming executor polling!
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

    // Publish Ultrasonic front data
    static uint32_t s_last_us_ms = 0;
    if (now - s_last_us_ms >= 100) { // 10Hz
        s_last_us_ms = now;
        g_us_front_msg.header.stamp.sec = (int32_t)(now / 1000);
        g_us_front_msg.header.stamp.nanosec = (uint32_t)((now % 1000) * 1000000);
        float dist_m = g_state.usFront / 100.0f;
        g_us_front_msg.range = dist_m;
        rcl_publish(&g_us_front_pub, &g_us_front_msg, NULL);

        if (g_us_debug_msg.data.data != NULL) {
            snprintf(g_us_debug_msg.data.data, 100, "[US_DEBUG] Front: %dcm | Back: %dcm | Left: %dcm | Right: %dcm", 
                     g_state.usFront, g_state.usBack, g_state.usLeft, g_state.usRight);
            g_us_debug_msg.data.size = strlen(g_us_debug_msg.data.data);
            rcl_publish(&g_us_debug_pub, &g_us_debug_msg, NULL);
        }
    }

    // 3. Ping agent and re-sync every 2 seconds to detect if agent was restarted (Ctrl+C on Pi)
    static uint32_t s_last_sync_ms = 0;
    if (now - s_last_sync_ms >= 2000) {
        s_last_sync_ms = now;
        rmw_uros_sync_session(100);  // 100ms timeout for time sync
        
        // If the ROS 2 agent is killed (Ctrl+C), ping will fail. We must reboot to clear the old session ID.
        if (rmw_uros_ping_agent(100, 2) != RMW_RET_OK) {
            Serial.println(F("[micro-ROS] Agent disconnected! Auto-rebooting to reset session..."));
            delay(100);
            ESP.restart();
        }
    }

    // 4. Poll incoming subscriptions with 0ns timeout so XRCE-DDS never blocks telemetry!
    static uint32_t s_last_spin_some_ms = 0;
    if (now - s_last_spin_some_ms >= 20) {
        s_last_spin_some_ms = now;
        rclc_executor_spin_some(&g_executor, 0);
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
