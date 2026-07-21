package com.smartmarketbot.hub.slam

import android.app.Service
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.hardware.usb.UsbManager
import android.os.Binder
import android.os.IBinder
import android.util.Log
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.smartmarketbot.hub.lidar.YDLIDARX3Manager
import com.smartmarketbot.hub.navigation.AStarPlanner
import com.smartmarketbot.hub.navigation.WaypointNavigatorService

/**
 * SLAM Service - Background service for LiDAR + SLAM + Navigation
 * 
 * Manages:
 * - USB connection to YDLIDAR
 * - SLAM processing
 * - Navigation (Path planning + Control)
 * - Broadcasts status to UI
 */
class SLAMService : Service() {
    
    companion object {
        private const val TAG = "SLAMService"
        
        // Actions for broadcast
        const val ACTION_LIDAR_CONNECTED = "com.smartmarketbot.hub.slam.LIDAR_CONNECTED"
        const val ACTION_LIDAR_DISCONNECTED = "com.smartmarketbot.hub.slam.LIDAR_DISCONNECTED"
        const val ACTION_LIDAR_ERROR = "com.smartmarketbot.hub.slam.LIDAR_ERROR"
        const val ACTION_SCAN_READY = "com.smartmarketbot.hub.slam.SCAN_READY"
        const val ACTION_POSE_UPDATE = "com.smartmarketbot.hub.slam.POSE_UPDATE"
        const val ACTION_NAV_STATE_CHANGED = "com.smartmarketbot.hub.slam.NAV_STATE_CHANGED"
        const val ACTION_TARGET_REACHED = "com.smartmarketbot.hub.slam.TARGET_REACHED"
        
        // Extras
        const val EXTRA_SCAN_COUNT = "scan_count"
        const val EXTRA_POSE_X = "pose_x"
        const val EXTRA_POSE_Y = "pose_y"
        const val EXTRA_POSE_THETA = "pose_theta"
        const val EXTRA_ERROR_MSG = "error_msg"
        const val EXTRA_NAV_STATE = "nav_state"
    }

    // LiDAR Manager
    private var lidarManager: YDLIDARX3Manager? = null
    private var isConnected = false

    // SLAM Engine
    val slamEngine = SLAMEngine()
    
    // Navigation — bind thay vì tạo raw instance (NavigatorCore nằm trong Service)
    private var navigator: WaypointNavigatorService? = null
    private var navigatorBound = false

    private val navigatorConnection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, service: IBinder?) {
            val binder = service as WaypointNavigatorService.NavigatorBinder
            navigator = binder.getService()
            // Pass slamEngine tới navigator (Service sẽ tạo NavigatorCore)
            navigator?.initialize(slamEngine)
            navigatorBound = true

            // Forward state changes broadcast
            binder.getNavigator()?.onStateChanged = { state ->
                val intent = Intent(ACTION_NAV_STATE_CHANGED).apply {
                    putExtra(EXTRA_NAV_STATE, state.toString())
                }
                sendBroadcast(intent)
            }
            Log.i(TAG, "WaypointNavigatorService bound")
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            navigator = null
            navigatorBound = false
            Log.i(TAG, "WaypointNavigatorService disconnected")
        }
    }

    // Last scan time for timing
    private var lastScanTime = 0L

    // Binder for local binding
    private val binder = SLAMServiceBinder()

    inner class SLAMServiceBinder : Binder() {
        fun getService(): SLAMService = this@SLAMService
    }

    override fun onCreate() {
        super.onCreate()
        Log.i(TAG, "SLAM Service created")

        // Bind tới WaypointNavigatorService
        val navIntent = Intent(this, WaypointNavigatorService::class.java)
        bindService(navIntent, navigatorConnection, Context.BIND_AUTO_CREATE)
    }

    override fun onBind(intent: Intent?): IBinder {
        return binder
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        connectLidar()
        return START_STICKY
    }

    /**
     * Connect to YDLIDAR via USB
     */
    fun connectLidar(): Boolean {
        if (isConnected) {
            Log.w(TAG, "Already connected")
            return true
        }

        try {
            val usbManager = getSystemService(USB_SERVICE) as UsbManager
            
            // Find YDLIDAR device
            val availableDevices = UsbSerialProber.getDefaultProber()
                .findAllDrivers(usbManager)
            
            if (availableDevices.isEmpty()) {
                Log.e(TAG, "No USB serial devices found")
                broadcastError("No USB devices found")
                return false
            }

            // Try to find YDLIDAR (common VID/PID)
            val lidarDevice = availableDevices.find { 
                it.device.vendorId == 0x10C4 ||  // Silicon Labs CP210x
                it.device.vendorId == 0x067B ||  // Prolific
                it.device.vendorId == 0x0483 ||  // STMicroelectronics
                it.device.vendorId == 0x1D6B     // Linux Foundation
            }

            val deviceToUse = lidarDevice ?: availableDevices.firstOrNull()
            
            if (deviceToUse == null) {
                Log.e(TAG, "No suitable device found")
                broadcastError("No suitable device found")
                return false
            }

            Log.i(TAG, "Found device: ${deviceToUse.device.vendorId}:${deviceToUse.device.productId}")

            // Request permission if needed
            if (!usbManager.hasPermission(deviceToUse.device)) {
                Log.w(TAG, "No USB permission, requesting...")
                val permissionIntent = android.app.PendingIntent.getBroadcast(
                    this,
                    0,
                    Intent(ACTION_LIDAR_CONNECTED),
                    android.app.PendingIntent.FLAG_IMMUTABLE
                )
                usbManager.requestPermission(deviceToUse.device, permissionIntent)
                return false
            }

            // Open device
            val connection = usbManager.openDevice(deviceToUse.device)
            if (connection == null) {
                Log.e(TAG, "Failed to open USB device")
                broadcastError("Failed to open device")
                return false
            }

            // Create LiDAR manager
            lidarManager = YDLIDARX3Manager(
                onScanReady = { points ->
                    onScanReceived(points)
                },
                onError = { error ->
                    Log.e(TAG, "LiDAR error: $error")
                    broadcastError(error)
                }
            )

            // Connect
            if (lidarManager!!.connect(deviceToUse.ports[0], connection)) {
                isConnected = true
                Log.i(TAG, "YDLIDAR connected successfully")
                
                // Start scan
                lidarManager!!.startScan()
                
                broadcastConnected()
                return true
            } else {
                Log.e(TAG, "Failed to connect to LiDAR")
                broadcastError("Connection failed")
                return false
            }

        } catch (e: Exception) {
            Log.e(TAG, "Error connecting LiDAR: ${e.message}")
            broadcastError("Error: ${e.message}")
            return false
        }
    }

    /**
     * Disconnect LiDAR
     */
    fun disconnectLidar() {
        try {
            lidarManager?.disconnect()
            lidarManager = null
            isConnected = false
            broadcastDisconnected()
            Log.i(TAG, "LiDAR disconnected")
        } catch (e: Exception) {
            Log.e(TAG, "Error disconnecting: ${e.message}")
        }
    }

    /**
     * Start LiDAR scanning
     */
    fun startScan(): Boolean {
        return lidarManager?.startScan() ?: false
    }

    /**
     * Stop LiDAR scanning
     */
    fun stopScan(): Boolean {
        return lidarManager?.stopScan() ?: false
    }

    /**
     * Process incoming scan data
     */
    private fun onScanReceived(points: List<YDLIDARX3Manager.LidarScanPoint>) {
        val now = System.currentTimeMillis()
        val dtMs = now - lastScanTime
        lastScanTime = now

        try {
            // Update SLAM
            val pose = slamEngine.update(points, dtMs)
            
            // Broadcast pose update
            val intent = Intent(ACTION_POSE_UPDATE).apply {
                putExtra(EXTRA_POSE_X, pose.x)
                putExtra(EXTRA_POSE_Y, pose.y)
                putExtra(EXTRA_POSE_THETA, pose.theta)
                putExtra(EXTRA_SCAN_COUNT, slamEngine.totalUpdates)
            }
            sendBroadcast(intent)
            
            // Broadcast scan ready to update UI map
            sendBroadcast(Intent(ACTION_SCAN_READY))

        } catch (e: Exception) {
            Log.e(TAG, "Error processing scan: ${e.message}")
        }
    }

    // === Navigation Methods ===

    /**
     * Navigate to target position
     */
    fun navigateTo(x: Float, y: Float): Boolean {
        return navigator?.navigateTo(x, y) ?: false
    }

    /**
     * Navigate through multiple waypoints
     */
    fun navigateWaypoints(waypoints: List<Pair<Float, Float>>): Boolean {
        return navigator?.navigateWaypoints(waypoints) ?: false
    }

    /**
     * Cancel navigation
     */
    fun cancelNavigation() {
        navigator?.cancel()
    }

    /**
     * Pause navigation
     */
    fun pauseNavigation() {
        navigator?.pause()
    }

    /**
     * Resume navigation
     */
    fun resumeNavigation() {
        navigator?.resume()
    }

    /**
     * Get navigation state
     */
    fun getNavigationState(): Int {
        return navigator?.getState() ?: WaypointNavigatorService.STATE_IDLE
    }

    /**
     * Get current path
     */
    fun getCurrentPath(): List<com.smartmarketbot.hub.navigation.AStarPlanner.Waypoint> {
        return navigator?.getCurrentPath() ?: emptyList()
    }

    /**
     * Set navigation velocity limit
     */
    fun setNavigationSpeed(linearVel: Float, angularVel: Float) {
        navigator?.setMaxVelocity(linearVel, angularVel)
    }

    /**
     * Reset SLAM map
     */
    fun resetMap() {
        slamEngine.reset()
        Log.i(TAG, "SLAM map reset")
    }

    /**
     * Set robot pose
     */
    fun setPose(x: Float, y: Float, theta: Float) {
        slamEngine.setPose(x, y, theta)
    }

    /**
     * Get SLAM Engine instance
     */
    fun getSLAMEngine(): SLAMEngine = slamEngine

    /**
     * Get current pose
     */
    fun getCurrentPose(): SLAMEngine.Pose = slamEngine.getPose()

    /**
     * Get LiDAR statistics
     */
    fun getStats(): String {
        return buildString {
            appendLine("=== SLAM Service Statistics ===")
            appendLine("LiDAR: ${if (isConnected) "Connected" else "Disconnected"}")
            appendLine(lidarManager?.getStats() ?: "N/A")
            appendLine()
            appendLine(slamEngine.getStats())
            appendLine()
            appendLine("=== Navigation ===")
            appendLine("State: ${getNavStateName(getNavigationState())}")
            appendLine("Remaining distance: ${"%.2f".format(navigator?.getRemainingDistance() ?: 0f)}m")
        }
    }

    private fun getNavStateName(state: Int): String {
        return when (state) {
            WaypointNavigatorService.STATE_IDLE -> "Idle"
            WaypointNavigatorService.STATE_PLANNING -> "Planning"
            WaypointNavigatorService.STATE_NAVIGATING -> "Navigating"
            WaypointNavigatorService.STATE_OBSTACLE_AVOID -> "Avoiding Obstacle"
            WaypointNavigatorService.STATE_PAUSED -> "Paused"
            WaypointNavigatorService.STATE_COMPLETED -> "Completed"
            WaypointNavigatorService.STATE_FAILED -> "Failed"
            else -> "Unknown"
        }
    }

    /**
     * Check if LiDAR is connected
     */
    fun isLidarConnected(): Boolean = isConnected

    private fun broadcastConnected() {
        sendBroadcast(Intent(ACTION_LIDAR_CONNECTED))
    }

    private fun broadcastDisconnected() {
        sendBroadcast(Intent(ACTION_LIDAR_DISCONNECTED))
    }

    private fun broadcastError(msg: String) {
        val intent = Intent(ACTION_LIDAR_ERROR).apply {
            putExtra(EXTRA_ERROR_MSG, msg)
        }
        sendBroadcast(intent)
    }

    override fun onDestroy() {
        super.onDestroy()
        disconnectLidar()
        if (navigatorBound) {
            unbindService(navigatorConnection)
            navigatorBound = false
        }
        navigator?.cancel()
        Log.i(TAG, "SLAM Service destroyed")
    }
}
