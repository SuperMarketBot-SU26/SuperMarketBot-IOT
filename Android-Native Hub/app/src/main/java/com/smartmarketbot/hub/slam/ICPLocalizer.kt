package com.smartmarketbot.hub.slam

import android.util.Log
import com.smartmarketbot.hub.lidar.YDLIDARX3Manager
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * ICP (Iterative Closest Point) Scan Matching — Phase 4: Localization.
 *
 * Mục tiêu: tìm phép biến đổi (dx, dy, dθ) để overlap scan hiện tại với scan trước
 * một cách chính xác nhất. Đây là thuật toán chuẩn thay thế cho centroid-based
 * matching trong [SLAMEngine.estimateMotion] (vốn chỉ là heuristic).
 *
 * Thuật toán (point-to-point ICP):
 *  1. Khởi tạo transform = initialGuess (thường = odometry estimate).
 *  2. Lặp tối đa maxIterations:
 *     a. Với mỗi điểm trong current scan, tìm điểm gần nhất trong previous scan.
 *     b. Tính transform (dx, dy, dθ) tối ưu bằng SVD-2D closed-form
 *        (cross-covariance matrix → angle, rồi translation).
 *     c. Apply transform lên current scan.
 *     d. Nếu delta < convergenceThreshold → hội tụ.
 *  3. Trả về transform cuối cùng.
 *
 * Độ chính xác điển hình (YDLIDAR X3, ±3cm noise, range 0.1-8m):
 *   - Phòng kín 5×5m, scan 360°: < 1cm translation, < 0.5° rotation.
 *
 * Khuyến nghị: dùng ICP kết hợp odometry (dead-reckoning từ IMU + wheel PWM).
 * ICP sửa drift của odometry.
 */
class ICPLocalizer(
    private val maxIterations: Int = 20,
    private val convergenceThreshold: Float = 0.001f,    // m (translation delta)
    private val maxCorrespondenceDist: Float = 1.0f,    // m (lọc outlier)
    private val minPoints: Int = 20
) {
    companion object {
        private const val TAG = "ICPLocalizer"

        /** Transform 2D: rotation (rad) + translation (m). */
        data class Transform(
            val dx: Float = 0f,
            val dy: Float = 0f,
            val dtheta: Float = 0f
        ) {
            fun norm(): Float = sqrt(dx * dx + dy * dy)
            fun compose(other: Transform): Transform = Transform(
                dx = dx + other.dx * cos(dtheta) - other.dy * sin(dtheta),
                dy = dy + other.dx * sin(dtheta) + other.dy * cos(dtheta),
                dtheta = normalizeAngle(dtheta + other.dtheta)
            )
        }
    }

    /**
     * Chạy ICP giữa 2 scan.
     *
     * @param prevScan      Scan trước (Cartesian, world frame, danh sách (x, y))
     * @param currScan      Scan hiện tại (Cartesian, world frame)
     * @param initialGuess  Odometry estimate (dx, dy, dθ) — dùng làm init.
     * @return Transform tìm được. norm() nhỏ = pose không đổi (robot đứng yên).
     */
    fun match(
        prevScan: List<Pair<Float, Float>>,
        currScan: List<Pair<Float, Float>>,
        initialGuess: Transform = Transform()
    ): Transform {
        if (prevScan.size < minPoints || currScan.size < minPoints) {
            Log.d(TAG, "match: skip, not enough points (prev=${prevScan.size}, curr=${currScan.size})")
            return initialGuess
        }

        var transform = initialGuess
        var prevError = Float.MAX_VALUE

        for (iter in 0 until maxIterations) {
            // 1) Apply current transform to currScan
            val transformedCurr = currScan.map { (x, y) ->
                val c = cos(transform.dtheta)
                val s = sin(transform.dtheta)
                Pair(
                    x * c - y * s + transform.dx,
                    x * s + y * c + transform.dy
                )
            }

            // 2) For each transformed point, find closest point in prevScan
            val correspondences = ArrayList<Pair<Pair<Float, Float>, Pair<Float, Float>>>(transformedCurr.size)
            var totalError = 0f
            var pairCount = 0

            for (p in transformedCurr) {
                val closest = findClosest(p, prevScan, maxCorrespondenceDist)
                if (closest != null) {
                    val dx = p.first - closest.first
                    val dy = p.second - closest.second
                    val distSq = dx * dx + dy * dy
                    correspondences.add(Pair(p, closest))
                    totalError += sqrt(distSq)
                    pairCount++
                }
            }

            if (pairCount < 3) {
                Log.w(TAG, "iter=$iter: only $pairCount correspondences, abort")
                break
            }

            val meanError = totalError / pairCount

            // 3) Compute optimal transform via 2D SVD-style closed-form
            val delta = computeTransform(correspondences) ?: run {
                Log.w(TAG, "iter=$iter: SVD failed, abort")
                break
            }

            // 4) Update
            transform = Transform(
                dx = transform.dx + delta.dx * cos(transform.dtheta) - delta.dy * sin(transform.dtheta),
                dy = transform.dy + delta.dx * sin(transform.dtheta) + delta.dy * cos(transform.dtheta),
                dtheta = normalizeAngle(transform.dtheta + delta.dtheta)
            )

            // 5) Convergence check
            if (delta.norm() < convergenceThreshold && abs(meanError - prevError) < 0.0005f) {
                Log.d(TAG, "iter=$iter: converged, meanError=${"%.4f".format(meanError)}, delta=${"%.4f".format(delta.norm())}")
                return transform
            }
            prevError = meanError

            if (iter == maxIterations - 1) {
                Log.d(TAG, "iter=$iter: max iterations, meanError=${"%.4f".format(meanError)}")
            }
        }

        return transform
    }

    /** Tìm điểm trong [scan] gần [p] nhất, trong bán kính [maxDist]. */
    private fun findClosest(
        p: Pair<Float, Float>,
        scan: List<Pair<Float, Float>>,
        maxDist: Float
    ): Pair<Float, Float>? {
        var bestDistSq = maxDist * maxDist
        var best: Pair<Float, Float>? = null

        for (q in scan) {
            val dx = p.first - q.first
            val dy = p.second - q.second
            val dSq = dx * dx + dy * dy
            if (dSq < bestDistSq) {
                bestDistSq = dSq
                best = q
            }
        }
        return best
    }

    /**
     * Tính transform (R, t) tối ưu giữa 2 tập điểm correspondences (p_i, q_i)
     * sao cho q_i ≈ R*p_i + t, dùng closed-form SVD-2D.
     *
     * Cross-covariance H = (1/N) * Σ (q_i - q_mean) * (p_i - p_mean)^T
     * SVD-2D:
     *   H = [[A, B], [C, D]]
     *   angle = atan2(B - C, A + D)   // sau khi flip nếu det < 0
     */
    private fun computeTransform(
        correspondences: List<Pair<Pair<Float, Float>, Pair<Float, Float>>>
    ): Transform? {
        val n = correspondences.size
        if (n < 3) return null

        // Means
        var pmx = 0f; var pmy = 0f
        var qmx = 0f; var qmy = 0f
        for ((p, q) in correspondences) {
            pmx += p.first; pmy += p.second
            qmx += q.first; qmy += q.second
        }
        pmx /= n; pmy /= n
        qmx /= n; qmy /= n

        // Cross-covariance matrix H = Σ (q - q_mean) (p - p_mean)^T
        // H = [[Sxx, Sxy], [Syx, Syy]]  trong đó Sij = Σ (q_i - q_mean_i)(p_j - p_mean_j)
        var sxx = 0f; var sxy = 0f
        var syx = 0f; var syy = 0f
        for ((p, q) in correspondences) {
            val px = p.first - pmx
            val py = p.second - pmy
            val qx = q.first - qmx
            val qy = q.second - qmy
            sxx += qx * px
            sxy += qx * py
            syx += qy * px
            syy += qy * py
        }

        // 2D rotation angle (closed-form, equivalent to SVD khi det >= 0)
        // Nếu det(H) = sxx*syy - sxy*syx < 0, phải flip để tránh reflection.
        val detH = sxx * syy - sxy * syx
        val theta = if (detH >= 0) {
            atan2(sxy - syx, sxx + syy)
        } else {
            // Reflection case: flip trục y của q hoặc x của p
            atan2(sxy + syx, sxx - syy)
        }

        // Translation: t = q_mean - R(theta) * p_mean
        val c = cos(theta)
        val s = sin(theta)
        val tx = qmx - (c * pmx - s * pmy)
        val ty = qmy - (s * pmx + c * pmy)

        return Transform(dx = tx, dy = ty, dtheta = theta)
    }

    private fun normalizeAngle(angle: Float): Float {
        var a = angle
        while (a > Math.PI.toFloat()) a -= (2 * Math.PI).toFloat()
        while (a < -Math.PI.toFloat()) a += (2 * Math.PI).toFloat()
        return a
    }

    /** Hàm tiện ích: convert từ [YDLIDARX3Manager.LidarScanPoint] → (x, y) Cartesian. */
    fun toCartesian(scanPoints: List<YDLIDARX3Manager.LidarScanPoint>): List<Pair<Float, Float>> {
        return scanPoints
            .filter { it.distanceMm in 100..8000 }  // 0.1m..8m
            .map { p ->
                val r = p.distanceMm / 1000f
                Pair(r * cos(p.angleRad), r * sin(p.angleRad))
            }
    }
}
