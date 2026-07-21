package com.smartmarketbot.hub.navigation

import android.util.Log
import com.smartmarketbot.hub.slam.SLAMEngine
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * NavigatorCore — Phase 8: Navigation logic thuần (POJO, không phải Service).
 *
 * Tách khỏi [WaypointNavigatorService] để dễ test, không bị ràng buộc Android Service
 * lifecycle. Service chỉ wrap POJO này + handle foreground notification.
 *
 * Bao gồm:
 *  - A* path planner
 *  - Pure Pursuit controller
 *  - DWA obstacle avoidance
 *  - MotorLink (UDP bridge xuống ESP32)
 *  - Control loop 10Hz với state machine
 */
class NavigatorCore(val slamEngine: SLAMEngine) {

    companion object {
        private const val TAG = "NavigatorCore"
        const val STATE_IDLE = 0
        const val STATE_PLANNING = 1
        const val STATE_NAVIGATING = 2
        const val STATE_OBSTACLE_AVOID = 3
        const val STATE_PAUSED = 4
        const val STATE_COMPLETED = 5
        const val STATE_FAILED = 6

        const val CONTROL_PERIOD_MS = 100L
        const val DEFAULT_MAX_LINEAR = 0.3f
        const val DEFAULT_MAX_ANGULAR = 2.0f
    }

    val pathPlanner = AStarPlanner()
    val pursuitController = PurePursuitController(
        wheelBase = 0.3f,
        maxSpeed = DEFAULT_MAX_LINEAR
    )
    val obstacleAvoider = DWAObstacleAvoider(slamEngine)
    val motorLink = MotorLink()

    // Phase 9 — Line + node fusion
    val lineBridge = LineSensorBridge()
    val nodeRegistry = NodeRegistry()
    private var lineTrackingEnabled = false

    // State
    @Volatile var navState: Int = STATE_IDLE
        private set
    var currentPath: List<AStarPlanner.Waypoint> = emptyList()
        private set
    var goalX = 0f; private set
    var goalY = 0f; private set
    var hasGoal = false
        private set

    var maxLinearVel = DEFAULT_MAX_LINEAR
    var maxAngularVel = DEFAULT_MAX_ANGULAR

    // Callbacks
    var onVelocityCommand: ((linear: Float, angular: Float) -> Unit)? = null
    var onStateChanged: ((state: Int) -> Unit)? = null
    var onTargetReached: (() -> Unit)? = null
    var onObstacle: ((distance: Float) -> Unit)? = null

    init {
        pathPlanner.setSLAMEngine(slamEngine)
    }

    /** Mở motor UDP socket. Idempotent. */
    fun openMotorLink() {
        motorLink.open()
    }

    fun closeMotorLink() {
        motorLink.close()
    }

    fun setMaxVelocity(linear: Float, angular: Float) {
        maxLinearVel = linear
        maxAngularVel = angular
        pursuitController.setSpeed(linear)
    }

    /**
     * Navigate tới (x, y). Trả về true nếu plan path thành công.
     * Caller cần gọi [tick] định kỳ (mỗi CONTROL_PERIOD_MS) sau khi navigateTo() thành công.
     */
    fun navigateTo(x: Float, y: Float): Boolean {
        val pose = slamEngine.getPose()
        Log.i(TAG, "navigateTo from (${pose.x}, ${pose.y}) → ($x, $y)")

        setState(STATE_PLANNING)
        replanAttempts = 0  // reset khi navigate mới

        val result = pathPlanner.findPath(slamEngine, pose.x, pose.y, x, y)

        if (!result.success || result.waypoints.isEmpty()) {
            Log.e(TAG, "Path planning failed: ${result.message}")
            setState(STATE_FAILED)
            return false
        }

        currentPath = result.waypoints
        pursuitController.setPath(result.waypoints)
        obstacleAvoider.setTarget(x, y)

        goalX = x
        goalY = y
        hasGoal = true

        setState(STATE_NAVIGATING)
        return true
    }

    fun navigateWaypoints(waypoints: List<Pair<Float, Float>>): Boolean {
        if (waypoints.isEmpty()) return false
        val (x, y) = waypoints.first()
        val ok = navigateTo(x, y)
        if (ok && waypoints.size > 1) {
            onTargetReached = { navigateWaypoints(waypoints.drop(1)) }
        }
        return ok
    }

    /** Tick control loop. Gọi mỗi CONTROL_PERIOD_MS từ Service/Activity. */
    fun tick() {
        if (!hasGoal) return

        val pose = slamEngine.getPose()

        val obstacleDist = checkObstacles(pose.x, pose.y, pose.theta)
        if (obstacleDist < 0.3f) {
            handleObstacle(obstacleDist)
            return
        }

        val robotState = PurePursuitController.RobotState(pose.x, pose.y, pose.theta)
        val control = pursuitController.compute(robotState)

        if (control.isGoalReached) {
            handleGoalReached()
            return
        }

        obstacleAvoider.setTarget(goalX, goalY)
        val safeVel = obstacleAvoider.findSafeVelocity(
            pose.x, pose.y, pose.theta,
            control.linearVel, control.angularVel
        )

        val finalLinear = safeVel.linearVel.coerceIn(-maxLinearVel, maxLinearVel)
        val finalAngular = safeVel.angularVel.coerceIn(-maxAngularVel, maxAngularVel)

        // Send to ESP32 qua WebSocket JSON (4WD differential: linear + angular)
        motorLink.sendVelocity(finalLinear, finalAngular)
        onVelocityCommand?.invoke(finalLinear, finalAngular)
    }

    fun pause() {
        if (navState == STATE_NAVIGATING || navState == STATE_OBSTACLE_AVOID) {
            motorLink.stop()
            setState(STATE_PAUSED)
        }
    }

    fun resume() {
        if (navState == STATE_PAUSED && hasGoal) {
            setState(STATE_NAVIGATING)
        }
    }

    fun cancel() {
        motorLink.stop()
        pursuitController.reset()
        currentPath = emptyList()
        hasGoal = false
        replanAttempts = 0
        lineTrackingEnabled = false
        motorLink.setMode(0)
        lineBridge.reset()
        setState(STATE_IDLE)
        onVelocityCommand?.invoke(0f, 0f)
        Log.i(TAG, "Navigation cancelled")
    }

    fun replan(): Boolean {
        if (!hasGoal) return false
        val pose = slamEngine.getPose()
        val result = pathPlanner.findPath(slamEngine, pose.x, pose.y, goalX, goalY)
        if (result.success) {
            currentPath = result.waypoints
            pursuitController.setPath(result.waypoints)
            obstacleAvoider.setTarget(goalX, goalY)
            setState(STATE_NAVIGATING)
            return true
        }
        return false
    }

    fun getRemainingDistance(): Float {
        val pose = slamEngine.getPose()
        return pursuitController.getRemainingDistance(
            PurePursuitController.RobotState(pose.x, pose.y, pose.theta)
        )
    }

    fun getProgress(): Float = pursuitController.getProgress()

    // ---------- Internal ----------

    /** Last time we tried replan. Backoff để tránh spam. */
    private var lastReplanAttemptMs = 0L
    private var replanAttempts = 0
    private val REPLAN_BACKOFF_MS = 2000L
    private val MAX_REPLAN_ATTEMPTS = 5

    private fun checkObstacles(x: Float, y: Float, theta: Float): Float {
        val forward = obstacleAvoider.getClearanceAtAngle(x, y, theta)
        val left = obstacleAvoider.getClearanceAtAngle(x, y, theta - 0.5f)
        val right = obstacleAvoider.getClearanceAtAngle(x, y, theta + 0.5f)
        val minDist = minOf(forward, left, right)
        if (minDist < 0.5f) onObstacle?.invoke(minDist)
        return minDist
    }

    private fun handleObstacle(dist: Float) {
        Log.w(TAG, "Obstacle detected at ${"%.2f".format(dist)}m")
        motorLink.stop()
        onVelocityCommand?.invoke(0f, 0f)

        if (navState == STATE_OBSTACLE_AVOID) {
            // [Phase 8] Dynamic replanning với exponential backoff
            tryDynamicReplan()
        } else {
            setState(STATE_OBSTACLE_AVOID)
        }
    }

    /**
     * Thử replan path. Backoff 2s giữa các lần thử. Sau 5 lần fail → STATE_FAILED.
     * Trả về true nếu đã chuyển state (replanning hoặc failed).
     */
    private fun tryDynamicReplan() {
        val now = System.currentTimeMillis()
        if (now - lastReplanAttemptMs < REPLAN_BACKOFF_MS) {
            // Chưa đến lúc retry — chờ
            return
        }
        lastReplanAttemptMs = now
        replanAttempts++

        if (replanAttempts > MAX_REPLAN_ATTEMPTS) {
            Log.e(TAG, "Replan failed after $MAX_REPLAN_ATTEMPTS attempts — abort")
            motorLink.stop()
            setState(STATE_FAILED)
            return
        }

        Log.i(TAG, "Dynamic replan attempt $replanAttempts/$MAX_REPLAN_ATTEMPTS")
        if (replan()) {
            replanAttempts = 0
            Log.i(TAG, "Replan successful — resuming navigation")
        } else {
            Log.w(TAG, "Replan attempt $replanAttempts failed, will retry in ${REPLAN_BACKOFF_MS}ms")
        }
    }

    /**
     * [Phase 9] Khi robot đi qua 1 node (dấu +) → snap pose về node pose.
     *
     * Triết lý: line là backbone cứng nên position error tích lũy chỉ do
     * SLAM/odometry drift. Tại node (vị trí tuyệt đối đã biết trước),
     * ta "sửa" pose của robot về giá trị node. Heading vẫn giữ từ IMU/odometry
     * (chính xác hơn) trừ khi user cố ý set nodePose.theta.
     */
    fun onNodeCrossed(nodeId: Int) {
        val node = nodeRegistry.get(nodeId)
        if (node == null) {
            // Chưa biết node này ở đâu → use SLAM pose hiện tại
            val cur = slamEngine.getPose()
            nodeRegistry.addFromPose(nodeId, cur.x, cur.y, cur.theta,
                label = "auto-$nodeId")
            Log.i(TAG, "Node #$nodeId auto-registered at (${"%.2f".format(cur.x)}, ${"%.2f".format(cur.y)})")
            return
        }

        // Snap pose
        slamEngine.setPose(node.x, node.y, node.theta)
        Log.i(TAG, "Snap pose to node #$nodeId → (${"%.2f".format(node.x)}, ${"%.2f".format(node.y)})")

        // Nếu có goal → check xem đã tới chưa
        if (hasGoal) {
            val cur = slamEngine.getPose()
            val dx = goalX - cur.x
            val dy = goalY - cur.y
            val dist = kotlin.math.sqrt(dx * dx + dy * dy)
            if (dist < GOAL_RADIUS_M) {
                handleGoalReached()
            }
        }
    }

    /**
     * [Phase 9] Auto-register node từ SLAM pose khi bridge báo node mới
     * mà registry chưa có. Cho phép dựng map tự động.
     */
    fun syncBridgeToSLAMPose() {
        val line = lineBridge.snapshot.value
        if (line.lastNodeId > 0) {
            // Update registry nếu thiếu
            if (nodeRegistry.get(line.lastNodeId) == null) {
                val pose = slamEngine.getPose()
                nodeRegistry.addFromPose(line.lastNodeId, pose.x, pose.y, pose.theta)
            }
        }
    }

    /**
     * Bật line tracking: ESP32 sẽ tự bám line + turn ở junction.
     * Mode này chính xác hơn SLAM cho indoor robot dò line.
     */
    fun enableLineMode(enable: Boolean = true) {
        lineTrackingEnabled = enable
        if (enable) {
            motorLink.setMode(3)  // MODE_LINE = 3
            Log.i(TAG, "Line mode enabled — ESP32 will follow line + stop at nodes")
        } else {
            motorLink.setMode(0)
            Log.i(TAG, "Line mode disabled — back to manual")
        }
    }

    private fun handleGoalReached() {
        motorLink.stop()
        onVelocityCommand?.invoke(0f, 0f)
        setState(STATE_COMPLETED)
        Log.i(TAG, "Target reached!")
        onTargetReached?.invoke()
    }

    private fun setState(s: Int) {
        if (navState != s) {
            navState = s
            onStateChanged?.invoke(s)
        }
    }
}