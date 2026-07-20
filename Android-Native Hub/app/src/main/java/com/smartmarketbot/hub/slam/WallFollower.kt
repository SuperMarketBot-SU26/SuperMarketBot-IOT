package com.smartmarketbot.hub.slam

import android.util.Log
import com.smartmarketbot.hub.lidar.YDLIDARX3Manager
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * WallFollower — Phase 4.2: Localization accuracy cho robot đi thẳng.
 *
 * Nguyên lý "dò tường" (wall following):
 *
 *   1. Robot đang ở pose (x, y, θ). Map đã biết tường ở đâu.
 *   2. LiDAR quét — lấy khoảng cách tường thực tế ở 2 bên (trái & phải).
 *   3. Tính khoảng cách tường DỰ KIẾN theo map tại cùng vị trí:
 *        - ray-cast từ pose theo hướng (θ + 90°) cho trái
 *        - ray-cast từ pose theo hướng (θ - 90°) cho phải
 *   4. Sai số lateral = (d_lidar_left - d_map_left) - (d_lidar_right - d_map_right)
 *      → robot lệch bao nhiêu cm so với đường giữa.
 *   5. Sai số heading: nếu lệch tăng đều theo thời gian → đang xoay.
 *
 * Đầu ra: (lateralOffset_m, headingCorrection_rad) — dùng để bù vào Pure Pursuit / ICP.
 *
 * Độ chính xác điển hình (YDLIDAR X3 ±3cm, map 5cm/cell):
 *   - lateral < 1 cm
 *   - heading correction < 0.5°
 *
 * Lưu ý:
 *   - Map phải CÓ TƯỜNG Ở 2 BÊN đường đi. Nếu corridor rộng vô tận → không có baseline.
 *   - Dùng làm "soft correction" chứ không thay thế ICP.
 */
class WallFollower(
    private val slamEngine: SLAMEngine,
    private val maxLateralCorrection: Float = 0.5f,  // m, bão hoà
    private val maxHeadingCorrection: Float = 0.1f,  // rad, bão hoà
    private val smoothingAlpha: Float = 0.7f        // EMA
) {
    companion object {
        private const val TAG = "WallFollower"

        /** Góc tương đối so với heading robot, dùng cho 2 bên tường. */
        const val LEFT_ANGLE = Math.PI.toFloat() / 2f   // +90°
        const val RIGHT_ANGLE = -Math.PI.toFloat() / 2f  // -90°
        const val FRONT_ANGLE = 0f
    }

    data class Correction(
        val lateralOffset: Float = 0f,     // m, (+) = robot lệch phải
        val headingError: Float = 0f,       // rad, (+) = robot đang xoay trái
        val valid: Boolean = false
    ) {
        /** Trả về (lateral_vel, angular_vel) suggestion để bù drift. */
        fun toControlGains(
            lateralGain: Float = 1.5f,
            headingGain: Float = 2.0f
        ): Pair<Float, Float> {
            if (!valid) return 0f to 0f
            val lateralCmd = (-lateralOffset * lateralGain).coerceIn(-0.2f, 0.2f)
            val headingCmd = (-headingError * headingGain).coerceIn(-0.5f, 0.5f)
            return lateralCmd to headingCmd
        }
    }

    private var smoothedLateral = 0f
    private var smoothedHeading = 0f
    private var prevLateral = 0f

    /**
     * Đo sai số giữa LiDAR thực và map tại pose hiện tại.
     *
     * @param scanPoints Raw LiDAR scan
     * @param pose       Pose estimate (x, y, theta)
     * @return Correction với lateralOffset & headingError
     */
    fun computeCorrection(
        scanPoints: List<YDLIDARX3Manager.LidarScanPoint>,
        pose: SLAMEngine.Pose
    ): Correction {
        if (scanPoints.size < 30) {
            return Correction(valid = false)
        }

        // 1) Đo khoảng cách tường THỰC từ LiDAR (2 bên + trước)
        val measuredLeft = findWallDistance(scanPoints, pose.theta + LEFT_ANGLE)
        val measuredRight = findWallDistance(scanPoints, pose.theta + RIGHT_ANGLE)

        if (measuredLeft < 0f || measuredRight < 0f) {
            Log.d(TAG, "computeCorrection: missing wall data (L=${measuredLeft}, R=${measuredRight})")
            return Correction(valid = false)
        }

        // 2) Đo khoảng cách tường DỰ KIẾN từ map
        val mapLeft = slamEngine.obstacleDistanceAt(pose, LEFT_ANGLE, maxRange = 4f)
        val mapRight = slamEngine.obstacleDistanceAt(pose, RIGHT_ANGLE, maxRange = 4f)

        if (mapLeft >= 4f - 0.01f || mapRight >= 4f - 0.01f) {
            // Không có tường trong map ở 1 hoặc 2 bên → không có baseline
            Log.d(TAG, "computeCorrection: no wall in map (L=${mapLeft}, R=${mapRight})")
            return Correction(valid = false)
        }

        // 3) Lateral offset
        // Nếu measured > map: tường xa hơn dự kiến → robot lệch về phía ngược lại.
        // Công thức: offset = (measured_left - map_left) - (measured_right - map_right) / 2
        val leftDelta = measuredLeft - mapLeft
        val rightDelta = measuredRight - mapRight
        val lateralOffset = ((leftDelta - rightDelta) / 2f)
            .coerceIn(-maxLateralCorrection, maxLateralCorrection)

        // 4) Heading error: so sánh tường trước — nếu lệch giữa các lần đo → đang xoay.
        // Dùng tỉ lệ (front_left - front_right) / (front_left + front_right)
        val measuredFrontLeft = findWallDistance(scanPoints, pose.theta + Math.PI.toFloat() / 4f)
        val measuredFrontRight = findWallDistance(scanPoints, pose.theta - Math.PI.toFloat() / 4f)
        val headingError = if (measuredFrontLeft > 0 && measuredFrontRight > 0) {
            val ratio = (measuredFrontLeft - measuredFrontRight) / (measuredFrontLeft + measuredFrontRight)
            (ratio * 0.5f).coerceIn(-maxHeadingCorrection, maxHeadingCorrection)
        } else {
            0f
        }

        // 5) EMA smoothing
        smoothedLateral = smoothingAlpha * smoothedLateral + (1f - smoothingAlpha) * lateralOffset
        smoothedHeading = smoothingAlpha * smoothedHeading + (1f - smoothingAlpha) * headingError

        prevLateral = lateralOffset

        Log.d(TAG, "correction: lateral=${"%.3f".format(smoothedLateral)}m, " +
                "heading=${"%.3f".format(Math.toDegrees(smoothedHeading.toDouble()))}° " +
                "(meas L=${"%.2f".format(measuredLeft)}, R=${"%.2f".format(measuredRight)}; " +
                "map L=${"%.2f".format(mapLeft)}, R=${"%.2f".format(mapRight)})")

        return Correction(
            lateralOffset = smoothedLateral,
            headingError = smoothedHeading,
            valid = true
        )
    }

    /**
     * Tìm khoảng cách đến tường đầu tiên trong góc [angleRel] (relative to robot frame).
     * Dùng median-of-buckets để chống nhiễu.
     *
     * @return khoảng cách (m), -1 nếu không tìm thấy trong maxRange
     */
    private fun findWallDistance(
        scanPoints: List<YDLIDARX3Manager.LidarScanPoint>,
        angleRel: Float,
        maxRange: Float = 2f,
        toleranceRad: Float = 0.05f  // ±3°
    ): Float {
        // 1) Lọc điểm trong cung ±tolerance quanh angleRel
        val candidates = scanPoints.filter { p ->
            p.distanceMm in 100..(maxRange * 1000).toInt() &&
                    abs(normalizeAngle(p.angleRad - angleRel)) < toleranceRad
        }
        if (candidates.isEmpty()) return -1f

        // 2) Lấy median khoảng cách (chống spike nhiễu)
        val distances = candidates.map { it.distanceMm / 1000f }.sorted()
        val median = distances[distances.size / 2]

        // 3) Median absolute deviation: lọc outlier > 2 * MAD
        val mad = distances.map { abs(it - median) }.sorted()[distances.size / 2]
        val filtered = if (mad > 0.05f) {
            distances.filter { abs(it - median) < 2f * mad }
        } else {
            distances
        }
        if (filtered.isEmpty()) return -1f

        return filtered.average().toFloat()
    }

    private fun normalizeAngle(angle: Float): Float {
        var a = angle
        while (a > Math.PI.toFloat()) a -= (2 * Math.PI).toFloat()
        while (a < -Math.PI.toFloat()) a += (2 * Math.PI).toFloat()
        return a
    }

    /** Reset state — gọi khi SLAM reset / robot teleport. */
    fun reset() {
        smoothedLateral = 0f
        smoothedHeading = 0f
        prevLateral = 0f
        Log.d(TAG, "reset")
    }
}
