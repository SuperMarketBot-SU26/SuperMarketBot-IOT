package com.smartmarketbot.hub.navigation

import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Intent
import android.os.Binder
import android.os.Build
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.util.Log
import androidx.core.app.NotificationCompat
import com.smartmarketbot.hub.slam.SLAMEngine

/**
 * Waypoint Navigator Service — Phase 8 wrapper around [NavigatorCore].
 *
 * Đã tách logic navigation thành POJO [NavigatorCore] để:
 *  - Dễ test (không cần Service/MockContext)
 *  - Tái sử dụng cho MainActivity / Background Worker
 *  - Service chỉ quản lý lifecycle + foreground notification + control loop ticker
 *
 * Flow:
 *   MainActivity.startService(ACTION_NAVIGATE_TO)
 *     → Service.onStartCommand
 *     → navigator.navigateTo(x, y)
 *     → handler.post(tickRunnable) mỗi CONTROL_PERIOD_MS
 *     → navigator.tick() → motorLink.sendVelocity()
 */
class WaypointNavigatorService : Service() {

    companion object {
        private const val TAG = "WaypointNavigator"

        // Actions for broadcast
        const val ACTION_NAV_STATE_CHANGED = "com.smartmarketbot.hub.NAV_STATE_CHANGED"
        const val ACTION_TARGET_REACHED = "com.smartmarketbot.hub.TARGET_REACHED"
        const val ACTION_PATH_UPDATE = "com.smartmarketbot.hub.PATH_UPDATE"
        const val ACTION_OBSTACLE_DETECTED = "com.smartmarketbot.hub.OBSTACLE_DETECTED"

        // [Phase 7] Actions từ MainActivity
        const val ACTION_NAVIGATE_TO = "com.smartmarketbot.hub.NAVIGATE_TO"
        const val ACTION_EMERGENCY_STOP = "com.smartmarketbot.hub.EMERGENCY_STOP"

        // Extras
        const val EXTRA_NAV_STATE = "nav_state"
        const val EXTRA_PATH_WAYPOINTS = "path_waypoints"
        const val EXTRA_OBSTACLE_DISTANCE = "obstacle_distance"
        const val EXTRA_TARGET_X = "target_x"
        const val EXTRA_TARGET_Y = "target_y"

        // Notification
        private const val NAV_NOTIFICATION_ID = 0x4E41  // "NA"

        // State aliases — forward từ NavigatorCore
        const val STATE_IDLE = NavigatorCore.STATE_IDLE
        const val STATE_PLANNING = NavigatorCore.STATE_PLANNING
        const val STATE_NAVIGATING = NavigatorCore.STATE_NAVIGATING
        const val STATE_OBSTACLE_AVOID = NavigatorCore.STATE_OBSTACLE_AVOID
        const val STATE_PAUSED = NavigatorCore.STATE_PAUSED
        const val STATE_COMPLETED = NavigatorCore.STATE_COMPLETED
        const val STATE_FAILED = NavigatorCore.STATE_FAILED
    }

    // Core logic (POJO)
    private var navigator: NavigatorCore? = null

    // Control loop
    private val handler = Handler(Looper.getMainLooper())
    private var isRunning = false
    private var controlRunnable: Runnable? = null

    // Binder
    private val binder = NavigatorBinder()

    inner class NavigatorBinder : Binder() {
        fun getService(): WaypointNavigatorService = this@WaypointNavigatorService
        fun getNavigator(): NavigatorCore? = navigator
    }

    override fun onCreate() {
        super.onCreate()
        // [Phase 7] Foreground notification
        try {
            val channelId = "waypoint_navigator"
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                val mgr = getSystemService(NotificationManager::class.java)
                val channel = NotificationChannel(
                    channelId, "Waypoint Navigator",
                    NotificationManager.IMPORTANCE_LOW
                )
                mgr?.createNotificationChannel(channel)
            }
            val notif = NotificationCompat.Builder(this, channelId)
                .setContentTitle("SmartMarketBot Navigation")
                .setContentText("A* + Pure Pursuit + DWA active")
                .setSmallIcon(android.R.drawable.ic_menu_compass)
                .setOngoing(true)
                .build()
            startForeground(NAV_NOTIFICATION_ID, notif)
        } catch (e: Exception) {
            Log.w(TAG, "startForeground failed: ${e.message}")
        }
        Log.i(TAG, "WaypointNavigatorService created")
    }

    override fun onBind(intent: Intent?): IBinder = binder

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_NAVIGATE_TO -> {
                val x = intent.getFloatExtra(EXTRA_TARGET_X, Float.NaN)
                val y = intent.getFloatExtra(EXTRA_TARGET_Y, Float.NaN)
                if (!x.isNaN() && !y.isNaN() && navigator != null) {
                    Log.i(TAG, "onStartCommand: navigateTo($x, $y)")
                    if (navigator!!.navigateTo(x, y)) {
                        startControlLoop()
                    }
                } else {
                    Log.w(TAG, "onStartCommand: navigator not initialized or invalid coords")
                }
            }
            ACTION_EMERGENCY_STOP -> {
                Log.w(TAG, "onStartCommand: EMERGENCY STOP")
                stopControlLoop()
                navigator?.cancel()
            }
        }
        return START_STICKY
    }

    /**
     * Khởi tạo navigator với SLAM engine. Gọi từ SLAMService.onCreate() qua binder.
     */
    fun initialize(slamEngine: SLAMEngine) {
        if (navigator != null) return
        navigator = NavigatorCore(slamEngine).also {
            it.onStateChanged = { state ->
                val intent = Intent(ACTION_NAV_STATE_CHANGED).apply {
                    putExtra(EXTRA_NAV_STATE, state)
                }
                sendBroadcast(intent)
            }
            it.onTargetReached = {
                val intent = Intent(ACTION_TARGET_REACHED).apply {
                    putExtra(EXTRA_NAV_STATE, STATE_COMPLETED)
                }
                sendBroadcast(intent)
                stopControlLoop()
            }
            it.onObstacle = { dist ->
                val intent = Intent(ACTION_OBSTACLE_DETECTED).apply {
                    putExtra(EXTRA_OBSTACLE_DISTANCE, dist)
                }
                sendBroadcast(intent)
            }
            it.openMotorLink()
        }
        Log.i(TAG, "Navigator initialized")
    }

    fun setMaxVelocity(linear: Float, angular: Float) {
        navigator?.setMaxVelocity(linear, angular)
    }

    fun navigateTo(x: Float, y: Float): Boolean {
        val ok = navigator?.navigateTo(x, y) ?: false
        if (ok) startControlLoop()
        return ok
    }

    fun navigateWaypoints(waypoints: List<Pair<Float, Float>>): Boolean {
        val ok = navigator?.navigateWaypoints(waypoints) ?: false
        if (ok) startControlLoop()
        return ok
    }

    fun pause() = navigator?.pause()
    fun resume() = navigator?.resume()
    fun cancel() {
        stopControlLoop()
        navigator?.cancel()
    }
    fun replan(): Boolean = navigator?.replan() ?: false
    fun getState(): Int = navigator?.navState ?: STATE_IDLE
    fun getCurrentPath(): List<AStarPlanner.Waypoint> = navigator?.currentPath ?: emptyList()
    fun getRemainingDistance(): Float = navigator?.getRemainingDistance() ?: 0f
    fun getProgress(): Float = navigator?.getProgress() ?: 0f

    private fun startControlLoop() {
        if (isRunning) return
        isRunning = true
        controlRunnable = object : Runnable {
            override fun run() {
                if (isRunning) {
                    navigator?.tick()
                    handler.postDelayed(this, NavigatorCore.CONTROL_PERIOD_MS)
                }
            }
        }
        handler.post(controlRunnable!!)
    }

    private fun stopControlLoop() {
        isRunning = false
        controlRunnable?.let { handler.removeCallbacks(it) }
        controlRunnable = null
    }

    override fun onDestroy() {
        super.onDestroy()
        stopControlLoop()
        navigator?.closeMotorLink()
        navigator?.cancel()
        Log.i(TAG, "WaypointNavigatorService destroyed")
    }
}