package com.smartmarketbot.hub.slam

import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.Handler
import android.os.Looper
import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.smartmarketbot.hub.lidar.YDLIDARX3Manager
import com.smartmarketbot.hub.UsbSerialService
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch

/**
 * SLAM ViewModel - Manages SLAM state and coordinates LiDAR + SLAM
 */
class SLAMViewModel : ViewModel() {
    companion object {
        private const val TAG = "SLAMViewModel"
    }

    // SLAM Engine instance
    private val slamEngine = SLAMEngine()

    // State flows
    private val _robotPose = MutableStateFlow(Pose2D(0f, 0f, 0f))
    val robotPose: StateFlow<Pose2D> = _robotPose

    private val _occupancyGrid = MutableStateFlow<Array<FloatArray>?>(null)
    val occupancyGrid: StateFlow<Array<FloatArray>?> = _occupancyGrid

    private val _scanPoints = MutableStateFlow<List<Point2D>>(emptyList())
    val scanPoints: StateFlow<List<Point2D>> = _scanPoints

    private val _slamState = MutableStateFlow(SLAMState.IDLE)
    val slamState: StateFlow<SLAMState> = _slamState

    private val _lidarConnected = MutableStateFlow(false)
    val lidarConnected: StateFlow<Boolean> = _lidarConnected

    private val _logMessages = MutableStateFlow<List<String>>(emptyList())
    val logMessages: StateFlow<List<String>> = _logMessages

    // LiDAR Manager
    private var lidarManager: YDLIDARX3Manager? = null

    // Timing
    private var lastScanTime = 0L
    private val handler = Handler(Looper.getMainLooper())

    // Pose data class
    data class Pose2D(
        val x: Float,
        val y: Float,
        val theta: Float  // radians
    )

    // Point data class
    data class Point2D(
        val x: Float,
        val y: Float,
        val quality: Int = 0
    )

    // SLAM States
    enum class SLAMState {
        IDLE,
        INITIALIZING,
        MAPPING,
        LOCALIZING,
        NAVIGATING,
        ERROR
    }

    /**
     * Initialize SLAM with LiDAR connection
     */
    fun initialize(context: Context, lidarManager: YDLIDARX3Manager) {
        this.lidarManager = lidarManager
        _slamState.value = SLAMState.INITIALIZING
        addLog("SLAM Initializing...")

        // Set callback for scan data
        lidarManager.onScanReady = { points ->
            onScanReceived(points)
        }

        lidarManager.onError = { error ->
            addLog("LiDAR Error: $error")
            _slamState.value = SLAMState.ERROR
        }

        _lidarConnected.value = true
        addLog("LiDAR connected")
        _slamState.value = SLAMState.MAPPING
    }

    /**
     * Start SLAM mapping
     */
    fun startMapping() {
        _slamState.value = SLAMState.MAPPING
        slamEngine.reset()
        addLog("Started SLAM Mapping")
    }

    /**
     * Stop SLAM
     */
    fun stopSLAM() {
        _slamState.value = SLAMState.IDLE
        addLog("SLAM Stopped")
    }

    /**
     * Reset map
     */
    fun resetMap() {
        slamEngine.reset()
        _occupancyGrid.value = slamEngine.getGrid()
        addLog("Map reset")
    }

    /**
     * Set robot pose (for localization mode)
     */
    fun setPose(x: Float, y: Float, theta: Float) {
        slamEngine.setPose(x, y, theta)
        _robotPose.value = Pose2D(x, y, theta)
        addLog("Pose set to (${"%.2f".format(x)}, ${"%.2f".format(y)}, ${"%.1f".format(Math.toDegrees(theta.toDouble()))}°)")
    }

    /**
     * Process incoming scan data
     */
    private fun onScanReceived(points: List<YDLIDARX3Manager.LidarScanPoint>) {
        val now = System.currentTimeMillis()
        val dtMs = now - lastScanTime
        lastScanTime = now

        viewModelScope.launch {
            try {
                // Update SLAM
                val pose = slamEngine.update(points, dtMs)
                _robotPose.value = Pose2D(pose.x, pose.y, pose.theta)

                // Convert scan points for visualization
                val visPoints = points
                    .filter { it.distanceMm > 100 && it.distanceMm < 8000 && it.quality > 10 }
                    .map { p ->
                        val x = (p.distanceMm / 1000f) * kotlin.math.cos(p.angleRad)
                        val y = (p.distanceMm / 1000f) * kotlin.math.sin(p.angleRad)
                        Point2D(x, y, p.quality)
                    }
                _scanPoints.value = visPoints

                // Update grid periodically (every 10 scans)
                if (slamEngine.totalUpdates % 10 == 0) {
                    _occupancyGrid.value = slamEngine.getGrid()
                }

            } catch (e: Exception) {
                Log.e(TAG, "Error processing scan: ${e.message}")
            }
        }
    }

    /**
     * Get obstacle distance in direction
     */
    fun getObstacleDistance(angleRad: Float): Float {
        return slamEngine.obstacleDistance(angleRad)
    }

    /**
     * Check if path is clear
     */
    fun isPathClear(fromX: Float, fromY: Float, toX: Float, toY: Float): Boolean {
        return slamEngine.isPathClear(fromX, fromY, toX, toY)
    }

    /**
     * Get current map as bytes for transmission
     */
    fun getMapData(): ByteArray? {
        val grid = _occupancyGrid.value ?: return null
        return encodeMapForTransmission(grid)
    }

    /**
     * Encode map as compact byte array for transmission
     */
    private fun encodeMapForTransmission(grid: Array<FloatArray>): ByteArray {
        val height = grid.size
        val width = if (grid.isNotEmpty()) grid[0].size else 0
        
        // Simple encoding: 1 byte per cell, 0-255 mapped to 0-1 probability
        val data = ByteArray(width * height)
        
        for (y in 0 until height) {
            for (x in 0 until width) {
                val prob = grid[y][x]
                data[y * width + x] = (prob * 255).toInt().coerceIn(0, 255).toByte()
            }
        }
        
        return data
    }

    /**
     * Add log message
     */
    private fun addLog(msg: String) {
        viewModelScope.launch {
            val timestamp = java.text.SimpleDateFormat(
                "HH:mm:ss",
                java.util.Locale.getDefault()
            ).format(java.util.Date())
            
            _logMessages.value = _logMessages.value.takeLast(99) + "[$timestamp] $msg"
            Log.i(TAG, msg)
        }
    }

    /**
     * Get SLAM statistics
     */
    fun getStats(): String {
        return slamEngine.getStats()
    }

    override fun onCleared() {
        super.onCleared()
        stopSLAM()
    }
}
