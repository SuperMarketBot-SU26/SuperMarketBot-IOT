package com.smartmarketbot.hub.slam

import android.util.Log
import kotlin.math.*
import com.smartmarketbot.hub.lidar.YDLIDARX3Manager

/**
 * SLAM Engine - Pure Kotlin Implementation
 * 
 * Implements Grid-based SLAM with:
 * - Occupancy Grid Map
 * - Scan Matching (Simple ICP variant)
 * - Particle Filter for localization
 * 
 * Optimized for YDLIDAR X3 on Android
 */
class SLAMEngine(
    private val mapResolution: Float = 0.05f,  // 5cm per cell
    private val mapWidth: Int = 400,            // 20m x 20m
    private val mapHeight: Int = 400,
    private val maxRange: Float = 8.0f          // YDLIDAR X3 max range
) {
    companion object {
        private const val TAG = "SLAMEngine"
        
        // Log-odds values for occupancy
        private const val LOG_ODDS_FREE = -0.7f      // P=0.33
        private const val LOG_ODDS_OCCUPIED = 1.5f   // P=0.82
        private const val LOG_ODDS_MAX = 5.0f
        private const val LOG_ODDS_MIN = -5.0f
        
        // Scan matching
        private const val MAX_ITERATIONS = 10
        private const val CONVERGENCE_THRESHOLD = 0.001f
    }

    // Robot pose
    data class Pose(
        var x: Float = 0f,
        var y: Float = 0f,
        var theta: Float = 0f  // radians
    )

    // Occupancy Grid
    private val grid = IntArray(mapWidth * mapHeight)  // Log-odds values
    
    // Map center (robot starts here)
    private val mapCenterX = mapWidth / 2
    private val mapCenterY = mapHeight / 2
    
    // Current pose estimate
    private var currentPose = Pose()
    
    // Previous scan for odometry
    private var prevScanPoints: List<ScanPoint>? = null
    
    // Map statistics
    var totalUpdates = 0
        private set
    var lastUpdateTime = 0L
        private set

    // Scan point structure
    data class ScanPoint(
        val x: Float,      // Cartesian x (meters)
        val y: Float,      // Cartesian y (meters)
        val range: Float,  // Range (meters)
        val angle: Float,  // Angle (radians)
        val quality: Int   // Signal quality
    )

    /**
     * Update SLAM with new scan data
     * @param scanPoints Raw scan points from LiDAR
     * @param dtMs Time since last update (ms)
     * @return Updated pose
     */
    fun update(scanPoints: List<YDLIDARX3Manager.LidarScanPoint>, dtMs: Long): Pose {
        if (scanPoints.isEmpty()) return currentPose

        // Convert polar to Cartesian
        val cartesianPoints = scanPoints.map { p ->
            ScanPoint(
                x = (p.distanceMm / 1000f) * cos(p.angleRad),
                y = (p.distanceMm / 1000f) * sin(p.angleRad),
                range = p.distanceMm / 1000f,
                angle = p.angleRad,
                quality = p.quality
            )
        }.filter { it.range < maxRange && it.quality > 10 }

        if (cartesianPoints.isEmpty()) return currentPose

        // Scan matching: dùng ICP (Phase 4) thay cho centroid heuristic.
        // initialGuess = 0 (odometry bù drift bằng ICP ở pose hiện tại).
        val motion = icpLocalizer.match(
            prevScan = prevCartesianPairs ?: emptyList(),
            currScan = cartesianPoints.map { it.x to it.y },
            initialGuess = ICPLocalizer.Companion.Transform()
        )
        prevCartesianPairs = cartesianPoints.map { it.x to it.y }
        prevScanPoints = cartesianPoints

        // Update pose estimate
        currentPose.x += motion.dx
        currentPose.y += motion.dy
        currentPose.theta = normalizeAngle(currentPose.theta + motion.dtheta)

        // [Phase 4.2] Wall-following correction: bù lateral offset + heading drift
        // dựa trên so sánh tường LiDAR thực vs map.
        val wf = wallFollower
        if (wf != null) {
            val corr = wf.computeCorrection(scanPoints, currentPose)
            if (corr.valid) {
                // Apply correction (small, EMA-smoothed)
                val (lateralCmd, headingCmd) = corr.toControlGains()
                currentPose.x += -lateralCmd * sin(currentPose.theta) * 0.05f  // 50ms lateral step
                currentPose.y += lateralCmd * cos(currentPose.theta) * 0.05f
                currentPose.theta = normalizeAngle(currentPose.theta + headingCmd * 0.05f)
            }
        }

        // Update occupancy grid
        updateGrid(cartesianPoints, currentPose)

        totalUpdates++
        lastUpdateTime = System.currentTimeMillis()

        return currentPose
    }

    // [Phase 4] ICP Localizer instance — dùng trong update() thay cho centroid heuristic cũ
    private val icpLocalizer = ICPLocalizer()
    private var prevCartesianPairs: List<Pair<Float, Float>>? = null

    // [Phase 4.2] Wall-follower — optional, bật qua enableWallFollowing()
    private var wallFollower: WallFollower? = null
    fun enableWallFollowing(enable: Boolean = true) {
        wallFollower = if (enable) {
            if (wallFollower == null) WallFollower(this)
            wallFollower
        } else null
    }

    /**
     * Compute centroid of scan points
     */
    private fun computeCentroid(points: List<ScanPoint>): PointF {
        if (points.isEmpty()) return PointF(0f, 0f)
        
        var sumX = 0f
        var sumY = 0f
        var weight = 0f
        
        for (p in points) {
            val w = p.quality.toFloat()
            sumX += p.x * w
            sumY += p.y * w
            weight += w
        }
        
        return if (weight > 0) {
            PointF(sumX / weight, sumY / weight)
        } else {
            PointF(0f, 0f)
        }
    }

    data class PointF(var x: Float, var y: Float)

    /**
     * Estimate rotation between two scans
     */
    private fun estimateRotation(prev: List<ScanPoint>, curr: List<ScanPoint>): Float {
        if (prev.size < 5 || curr.size < 5) return 0f
        
        // Compute angle histogram
        val prevAngles = prev.filter { it.range > 0.1f }.map { atan2(it.y, it.x) }
        val currAngles = curr.filter { it.range > 0.1f }.map { atan2(it.y, it.x) }
        
        // Find median angle difference
        val diffs = mutableListOf<Float>()
        for (pa in prevAngles.take(10)) {
            for (ca in currAngles.take(10)) {
                diffs.add(normalizeAngle(ca - pa))
            }
        }
        
        return if (diffs.isNotEmpty()) {
            diffs.sorted()[diffs.size / 2]  // Median
        } else 0f
    }

    /**
     * Normalize angle to [-PI, PI]
     */
    private fun normalizeAngle(angle: Float): Float {
        var a = angle
        while (a > PI.toFloat()) a -= (2 * PI).toFloat()
        while (a < -PI.toFloat()) a += (2 * PI).toFloat()
        return a
    }

    /**
     * Update occupancy grid with new scan
     */
    private fun updateGrid(scanPoints: List<ScanPoint>, pose: Pose) {
        val robotMapX = mapCenterX + (pose.x / mapResolution).toInt()
        val robotMapY = mapCenterY + (pose.y / mapResolution).toInt()
        
        for (point in scanPoints) {
            // Transform point to world frame
            val worldX = pose.x + point.x * cos(pose.theta) - point.y * sin(pose.theta)
            val worldY = pose.y + point.x * sin(pose.theta) + point.y * cos(pose.theta)
            
            val pointMapX = mapCenterX + (worldX / mapResolution).toInt()
            val pointMapY = mapCenterY + (worldY / mapResolution).toInt()
            
            // Check bounds
            if (pointMapX < 0 || pointMapX >= mapWidth ||
                pointMapY < 0 || pointMapY >= mapHeight) continue
            
            // Update cell at point as occupied
            val idx = pointMapY * mapWidth + pointMapX
            grid[idx] = clampLogOdds(grid[idx] + LOG_ODDS_OCCUPIED)
            
            // Update cells along the ray as free
            val steps = (point.range / mapResolution).toInt()
            for (i in 1 until steps) {
                val ratio = i.toFloat() / steps
                val freeX = robotMapX + ((pointMapX - robotMapX) * ratio).toInt()
                val freeY = robotMapY + ((pointMapY - robotMapY) * ratio).toInt()
                
                if (freeX in 0 until mapWidth && freeY in 0 until mapHeight) {
                    val freeIdx = freeY * mapWidth + freeX
                    grid[freeIdx] = clampLogOdds(grid[freeIdx] + LOG_ODDS_FREE)
                }
            }
        }
    }

    private fun clampLogOdds(v: Float): Int {
        return maxOf(LOG_ODDS_MIN.toInt(), minOf(LOG_ODDS_MAX.toInt(), v.toInt()))
    }

    /**
     * Get occupancy probability for a cell
     * @param wx World x (meters)
     * @param wy World y (meters)
     * @return Probability 0..1, or null if out of bounds
     */
    fun getProbability(wx: Float, wy: Float): Float? {
        val mx = mapCenterX + (wx / mapResolution).toInt()
        val my = mapCenterY + (wy / mapResolution).toInt()
        
        if (mx < 0 || mx >= mapWidth || my < 0 || my >= mapHeight) {
            return null
        }
        
        val idx = my * mapWidth + mx
        return logOddsToProbability(grid[idx])
    }

    /**
     * Check if a cell is occupied
     */
    fun isOccupied(wx: Float, wy: Float, threshold: Float = 0.5f): Boolean {
        return getProbability(wx, wy)?.let { it > threshold } ?: false
    }

    /**
     * Get latest scan points for UI rendering
     */
    fun getLatestScanPoints(): List<android.graphics.PointF> {
        val pts = prevScanPoints ?: return emptyList()
        return pts.map { android.graphics.PointF(it.x, it.y) }
    }

    /**
     * Get raw log-odds grid for zero-allocation ultra-fast UI rendering
     */
    fun getRawGrid(): IntArray = grid

    /**
     * Get occupancy grid for visualization
     * @return Grid as 2D array of probabilities (0..1)
     */
    fun getGrid(): Array<FloatArray> {
        return Array(mapHeight) { y ->
            FloatArray(mapWidth) { x ->
                val idx = y * mapWidth + x
                logOddsToProbability(grid[idx])
            }
        }
    }

    /**
     * Get grid cell value (log-odds)
     */
    fun getGridCell(mx: Int, my: Int): Float? {
        if (mx < 0 || mx >= mapWidth || my < 0 || my >= mapHeight) {
            return null
        }
        return grid[my * mapWidth + mx].toFloat()
    }

    private fun logOddsToProbability(lo: Int): Float {
        return 1f / (1f + exp(-lo.toFloat()))
    }

    /**
     * Reset the map
     */
    fun reset() {
        grid.fill(0)
        currentPose = Pose()
        prevScanPoints = null
        prevCartesianPairs = null
        totalUpdates = 0
    }

    // ============== Map persistence (Phase 8) ==============

    /**
     * Export raw log-odds grid + dimensions để serialize.
     */
    fun exportRawLogOdds(): Triple<IntArray, Int, Int> = Triple(
        grid.copyOf(), mapWidth, mapHeight
    )

    /**
     * Import raw log-odds grid (thường từ file đã save).
     * Chỉ hoạt động nếu dimensions khớp với map hiện tại.
     */
    fun importRawLogOdds(rawGrid: IntArray, width: Int, height: Int): Boolean {
        if (rawGrid.size != width * height || width != mapWidth || height != mapHeight) {
            Log.w(TAG, "importRawLogOdds: dim mismatch (got ${width}x${height}, expected ${mapWidth}x${mapHeight})")
            return false
        }
        System.arraycopy(rawGrid, 0, grid, 0, rawGrid.size)
        return true
    }

    /**
     * Set robot pose (for initialization)
     */
    fun setPose(x: Float, y: Float, theta: Float) {
        currentPose = Pose(x, y, theta)
    }

    /**
     * Get current pose estimate
     */
    fun getPose(): Pose = currentPose

    /**
     * Get map statistics
     */
    fun getStats(): String {
        val occupiedCells = grid.count { it > 0 }
        val freeCells = grid.count { it < 0 }
        
        return buildString {
            appendLine("=== SLAM Engine Statistics ===")
            appendLine("Updates: $totalUpdates")
            appendLine("Pose: (${"%.2f".format(currentPose.x)}, ${"%.2f".format(currentPose.y)}, ${"%.2f".format(Math.toDegrees(currentPose.theta.toDouble()))}°)")
            appendLine("Occupied cells: $occupiedCells")
            appendLine("Free cells: $freeCells")
            appendLine("Resolution: ${mapResolution * 100}cm/cell")
        }
    }

    /**
     * Check if pose is valid (inside map bounds)
     */
    fun isPoseValid(): Boolean {
        val mx = mapCenterX + (currentPose.x / mapResolution).toInt()
        val my = mapCenterY + (currentPose.y / mapResolution).toInt()
        
        return mx in 20 until mapWidth - 20 && my in 20 until mapHeight - 20
    }

    /**
     * Find nearest obstacle in direction
     * @param angle Direction to check (radians, 0 = forward, relative to robot frame)
     * @param maxRange Maximum range to check
     * @return Distance to obstacle or maxRange if none
     */
    fun obstacleDistance(angle: Float, maxRange: Float = 4f): Float =
        obstacleDistanceAt(currentPose, angle, maxRange)

    /**
     * Overload: tìm obstacle từ pose cho trước (dùng cho WallFollower — Phase 4.2).
     * @param pose Pose bất kỳ (thường = currentPose, nhưng cho phép evaluate "what-if")
     */
    fun obstacleDistanceAt(pose: Pose, angle: Float, maxRange: Float = 4f): Float {
        val cosA = cos(angle + pose.theta)
        val sinA = sin(angle + pose.theta)

        for (r in 0 until (maxRange / mapResolution).toInt()) {
            val wx = pose.x + r * mapResolution * cosA
            val wy = pose.y + r * mapResolution * sinA

            if (isOccupied(wx, wy, 0.6f)) {
                return r * mapResolution
            }
        }
        return maxRange
    }

    /**
     * Check if path is clear between two points
     */
    fun isPathClear(fromX: Float, fromY: Float, toX: Float, toY: Float): Boolean {
        val dx = toX - fromX
        val dy = toY - fromY
        val dist = sqrt(dx * dx + dy * dy)
        val steps = (dist / mapResolution).toInt()
        
        for (i in 0..steps) {
            val ratio = i.toFloat() / steps
            val wx = fromX + dx * ratio
            val wy = fromY + dy * ratio
            
            if (isOccupied(wx, wy, 0.7f)) {
                return false
            }
        }
        return true
    }
}
