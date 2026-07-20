/**
 * jni_wrapper.cpp
 * JNI Bridge between Kotlin and Native C++ SLAM + MotorLink
 */

#include <jni.h>
#include <android/log.h>
#include <cstdio>
#include "SLAMEngine.h"
#include "RobotMotorCommand.h"
#include "udp_socket.h"

#define LOG_TAG "SLAM_JNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

static SLAMEngine* g_slamEngine = nullptr;
static UdpSocket* g_motorUdp = nullptr;

extern "C" {

// ============================================================================
// SLAM ENGINE JNI
// ============================================================================

JNIEXPORT void JNICALL
Java_com_smartmarketbot_hub_slam_SLAMEngine_nativeInit(
    JNIEnv *env,
    jobject /* this */,
    jfloat startX,
    jfloat startY,
    jfloat startTheta
) {
    if (g_slamEngine) {
        delete g_slamEngine;
    }
    g_slamEngine = new SLAMEngine();
    g_slamEngine->initialize(startX, startY, startTheta);
    LOGI("SLAM Engine initialized at (%.2f, %.2f, %.2f)", startX, startY, startTheta);
}

JNIEXPORT void JNICALL
Java_com_smartmarketbot_hub_slam_SLAMEngine_nativeDestroy(
    JNIEnv *env,
    jobject /* this */
) {
    if (g_slamEngine) {
        delete g_slamEngine;
        g_slamEngine = nullptr;
        LOGI("SLAM Engine destroyed");
    }
}

JNIEXPORT jfloatArray JNICALL
Java_com_smartmarketbot_hub_slam_SLAMEngine_nativeProcessScan(
    JNIEnv *env,
    jobject /* this */,
    jfloatArray ranges,
    jfloatArray angles,
    jintArray qualities,
    jint count,
    jfloat robotX,
    jfloat robotY,
    jfloat robotTheta
) {
    if (!g_slamEngine) {
        LOGE("SLAM Engine not initialized");
        return nullptr;
    }

    // Get arrays
    jfloat* rangeData = env->GetFloatArrayElements(ranges, nullptr);
    jfloat* angleData = env->GetFloatArrayElements(angles, nullptr);
    jint* qualityData = env->GetIntArrayElements(qualities, nullptr);

    // Build scan points
    std::vector<ScanPoint> scanPoints;
    scanPoints.reserve(count);
    for (int i = 0; i < count; i++) {
        ScanPoint p;
        p.range = rangeData[i] / 1000.0f;  // mm to m
        p.angle = angleData[i];
        p.quality = qualityData[i];
        p.x = p.range * cosf(p.angle);
        p.y = p.range * sinf(p.angle);
        scanPoints.push_back(p);
    }

    // Release arrays
    env->ReleaseFloatArrayElements(ranges, rangeData, 0);
    env->ReleaseFloatArrayElements(angles, angleData, 0);
    env->ReleaseIntArrayElements(qualities, qualityData, 0);

    // Process scan
    Pose2D pose = g_slamEngine->processScan(scanPoints);

    // Return pose as float array
    jfloatArray result = env->NewFloatArray(3);
    jfloat poseData[3] = {pose.x, pose.y, pose.theta};
    env->SetFloatArrayRegion(result, 0, 3, poseData);

    return result;
}

JNIEXPORT jbyteArray JNICALL
Java_com_smartmarketbot_hub_slam_SLAMEngine_nativeGetMapImage(
    JNIEnv *env,
    jobject /* this */
) {
    if (!g_slamEngine) {
        return nullptr;
    }

    std::vector<uint8_t> image = g_slamEngine->getMapImage();
    
    jbyteArray result = env->NewByteArray(image.size());
    env->SetByteArrayRegion(result, 0, image.size(), (jbyte*)image.data());
    
    return result;
}

JNIEXPORT void JNICALL
Java_com_smartmarketbot_hub_slam_SLAMEngine_nativeSaveMap(
    JNIEnv *env,
    jobject /* this */,
    jstring filepath
) {
    // [Phase 8] Map serialization được làm ở Kotlin (MapPersistence.kt) vì
    // SLAMEngine native chưa có impl grid serialization.
    // JNI này giữ làm stub để tương thích nếu sau này port sang C++.
    const char* path = env->GetStringUTFChars(filepath, nullptr);
    LOGI("nativeSaveMap: stub, use Kotlin MapPersistence instead (%s)", path);
    env->ReleaseStringUTFChars(filepath, path);
}

JNIEXPORT jboolean JNICALL
Java_com_smartmarketbot_hub_slam_SLAMEngine_nativeLoadMap(
    JNIEnv *env,
    jobject /* this */,
    jstring filepath
) {
    const char* path = env->GetStringUTFChars(filepath, nullptr);
    LOGI("nativeLoadMap: stub, use Kotlin MapPersistence instead (%s)", path);
    env->ReleaseStringUTFChars(filepath, path);
    return JNI_FALSE;
}

// ============================================================================
// NAVIGATION JNI — A*, Pure Pursuit, DWA chạy Kotlin (performance đủ tốt cho indoor robot).
// JNI wrappers cho planning/control bị bỏ qua vì Kotlin code đã cover. Bridge với
// native chỉ dùng cho SLAM scan matching + MotorLink UDP.
// ============================================================================

// ============================================================================
// MOTOR LINK JNI (Phase 5.3 — UDP bridge xuống ESP32)
// ============================================================================

JNIEXPORT void JNICALL
Java_com_smartmarketbot_hub_navigation_MotorLink_nativeOpen(
    JNIEnv *env,
    jobject /* this */
) {
    if (!g_motorUdp) {
        g_motorUdp = new UdpSocket();
    }
    bool ok = g_motorUdp->open();
    LOGI("MotorLink nativeOpen: %s", ok ? "OK" : "FAIL");
}

JNIEXPORT void JNICALL
Java_com_smartmarketbot_hub_navigation_MotorLink_nativeClose(
    JNIEnv *env,
    jobject /* this */
) {
    if (g_motorUdp) {
        g_motorUdp->close();
        delete g_motorUdp;
        g_motorUdp = nullptr;
    }
    LOGI("MotorLink nativeClose");
}

/**
 * Gửi velocity command packet (vx_mm/s, vy_mm/s, omega_mrad/s) tới ESP32.
 * Packet format: MotorCommandPacket::encode (9 bytes).
 */
JNIEXPORT jboolean JNICALL
Java_com_smartmarketbot_hub_navigation_MotorLink_nativeSendVelocity(
    JNIEnv *env,
    jobject /* this */,
    jstring host,
    jint port,
    jint vxMmS,
    jint vyMmS,
    jint omegaMradS
) {
    if (!g_motorUdp || !g_motorUdp->isOpen()) {
        LOGE("MotorLink: socket not open");
        return JNI_FALSE;
    }

    // Encode packet (clamp int16)
    auto clamp16 = [](jint v) -> int16_t {
        if (v > 32767) return 32767;
        if (v < -32768) return -32768;
        return static_cast<int16_t>(v);
    };

    auto pkt = MotorCommandPacket::encode(
        clamp16(vxMmS),
        clamp16(vyMmS),
        clamp16(omegaMradS)
    );

    const char* hostStr = env->GetStringUTFChars(host, nullptr);
    bool ok = g_motorUdp->sendTo(pkt.data(), pkt.size(), hostStr, port);
    env->ReleaseStringUTFChars(host, hostStr);

    if (!ok) {
        LOGE("MotorLink: sendTo %s:%d failed", hostStr, port);
    }
    return ok ? JNI_TRUE : JNI_FALSE;
}

/**
 * Compute mecanum IK ở native (test/telemetry) — trả về mảng 4 wheel rad/s.
 */
JNIEXPORT jfloatArray JNICALL
Java_com_smartmarketbot_hub_navigation_MotorLink_nativeMecanumIK(
    JNIEnv *env,
    jobject /* this */,
    jfloat vx,
    jfloat vy,
    jfloat omega
) {
    auto ws = mecanumInverseKinematics(vx, vy, omega);
    jfloatArray out = env->NewFloatArray(4);
    jfloat data[4] = {ws.w_fl, ws.w_rl, ws.w_fr, ws.w_rr};
    env->SetFloatArrayRegion(out, 0, 4, data);
    return out;
}

} // extern "C"
