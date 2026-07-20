package com.smartmarketbot.hub.navigation

import android.util.Log

/**
 * MotorLink — Phase 5.3: UDP bridge xuống ESP32.
 *
 * Gửi 9-byte velocity command packet (vx, vy, omega) tới ESP32 qua WiFi UDP
 * (port 4210 mặc định — phải khớp với ESP32 firmware).
 *
 * Packet format (xem [com.smartmarketbot.hub.cpp.RobotMotorCommand]):
 *   [0]    0xAA        header
 *   [1]    0x01        cmd velocity
 *   [2-3]  vx int16 LE (mm/s)
 *   [4-5]  vy int16 LE (mm/s)
 *   [6-7]  omega int16 LE (mrad/s)
 *   [8]    XOR checksum
 *
 * Dùng native (JNI) thay cho DatagramSocket Kotlin vì:
 *  - Tránh GC pause trong control loop 10Hz
 *  - Dùng chung code với native SLAM stack
 *
 * Đơn vị (Phase 5.3 cũ dùng rad/s, đổi sang mm/s / mrad/s cho khớp ESP32 protocol):
 *  - vx, vy: m/s → mm/s khi gửi
 *  - omega: rad/s → mrad/s khi gửi
 */
class MotorLink(
    private val esp32Host: String = "192.168.4.1",
    private val esp32Port: Int = 4210
) {
    companion object {
        private const val TAG = "MotorLink"

        init {
            try {
                System.loadLibrary("native_slam")
            } catch (e: UnsatisfiedLinkError) {
                Log.e(TAG, "Failed to load native_slam: ${e.message}")
            }
        }
    }

    @Volatile
    private var open: Boolean = false

    /** Mở UDP socket native. Idempotent. */
    @Synchronized
    fun open() {
        if (open) return
        nativeOpen()
        open = true
        Log.i(TAG, "MotorLink opened → $esp32Host:$esp32Port")
    }

    /** Đóng UDP socket. */
    @Synchronized
    fun close() {
        if (!open) return
        nativeClose()
        open = false
    }

    /**
     * Gửi velocity command.
     *
     * @param vx    forward velocity (m/s)
     * @param vy    strafe velocity (m/s)
     * @param omega angular velocity (rad/s)
     * @return true nếu gửi thành công
     */
    fun sendVelocity(vx: Float, vy: Float, omega: Float): Boolean {
        if (!open) open()
        val vxMm = (vx * 1000f).toInt()
        val vyMm = (vy * 1000f).toInt()
        val omegaMrad = (omega * 1000f).toInt()
        return nativeSendVelocity(esp32Host, esp32Port, vxMm, vyMm, omegaMrad)
    }

    /** Emergency stop. */
    fun stop() = sendVelocity(0f, 0f, 0f)

    /** Tính Mecanum IK ở native (cho debug/telemetry). Trả về [w_fl, w_rl, w_fr, w_rr] (rad/s). */
    fun mecanumIK(vx: Float, vy: Float, omega: Float): FloatArray {
        return nativeMecanumIK(vx, vy, omega)
    }

    // ----- JNI -----
    private external fun nativeOpen()
    private external fun nativeClose()
    private external fun nativeSendVelocity(
        host: String,
        port: Int,
        vxMmS: Int,
        vyMmS: Int,
        omegaMradS: Int
    ): Boolean

    private external fun nativeMecanumIK(vx: Float, vy: Float, omega: Float): FloatArray
}
