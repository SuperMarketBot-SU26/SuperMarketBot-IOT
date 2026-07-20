package com.smartmarketbot.hub.navigation

import android.util.Log
import kotlin.math.*

/**
 * Pure Pursuit Controller
 * 
 * Follows a path by targeting a lookahead point on the path.
 * The robot steers towards the lookahead point while driving forward.
 * 
 * Reference: 
 * - "Implementation of the Pure Pursuit Path Tracking Algorithm" 
 *   by R. Craig Coulter
 */
class PurePursuitController(
    private val wheelBase: Float = 0.3f,  // meters (distance between wheels)
    private val maxSpeed: Float = 0.5f,   // m/s
    private val minSpeed: Float = 0.1f,   // m/s
    private val maxAngularVel: Float = 2.0f // rad/s
) {
    companion object {
        private const val TAG = "PurePursuit"
        
        // Default lookahead distance based on speed
        private const val MIN_LOOKAHEAD = 0.2f  // meters
        private const val MAX_LOOKAHEAD = 1.0f  // meters
        private const val LOOKAHEAD_FACTOR = 0.5f // multiplier of speed
        
        // Acceptance thresholds
        private const val GOAL_THRESHOLD = 0.1f  // meters
        private const val WAYPOINT_THRESHOLD = 0.05f // meters
    }

    // Robot state
    data class RobotState(
        var x: Float = 0f,
        var y: Float = 0f,
        var theta: Float = 0f  // radians
    )

    // Control output
    data class ControlOutput(
        val linearVel: Float,    // m/s
        val angularVel: Float,   // rad/s
        val targetIndex: Int,    // Current waypoint index
        val isGoalReached: Boolean
    )

    // Current path
    private var path: List<AStarPlanner.Waypoint> = emptyList()
    private var currentWaypointIndex = 0
    private var lookaheadDistance = 0.3f  // meters
    
    // Velocity control
    private var targetSpeed = 0.3f  // m/s
    
    init {
        setLookaheadBySpeed(targetSpeed)
    }

    /**
     * Set path to follow
     */
    fun setPath(waypoints: List<AStarPlanner.Waypoint>) {
        if (waypoints.isEmpty()) {
            Log.w(TAG, "Empty path provided")
            return
        }
        this.path = waypoints
        this.currentWaypointIndex = 0
        Log.i(TAG, "Path set with ${waypoints.size} waypoints")
    }

    /**
     * Set target speed
     */
    fun setSpeed(speed: Float) {
        targetSpeed = speed.coerceIn(minSpeed, maxSpeed)
        setLookaheadBySpeed(targetSpeed)
    }

    /**
     * Update lookahead distance based on speed
     * Higher speed = longer lookahead for smoother curves
     */
    private fun setLookaheadBySpeed(speed: Float) {
        lookaheadDistance = (MIN_LOOKAHEAD + speed * LOOKAHEAD_FACTOR)
            .coerceIn(MIN_LOOKAHEAD, MAX_LOOKAHEAD)
    }

    /**
     * Compute control output for current state
     * 
     * @param state Current robot state (x, y, theta)
     * @return ControlOutput with velocity commands
     */
    fun compute(state: RobotState): ControlOutput {
        if (path.isEmpty()) {
            return ControlOutput(0f, 0f, 0, true)
        }
        
        // Find target point on path
        val targetPoint = findTargetPoint(state)
        
        if (targetPoint == null) {
            Log.w(TAG, "No valid target point found")
            return ControlOutput(0f, 0f, currentWaypointIndex, true)
        }
        
        // Calculate lookahead point index
        val targetIdx = findLookaheadIndex(state, targetPoint)
        
        // Calculate lookahead distance to target
        val ld = distance(state.x, state.y, targetPoint.x, targetPoint.y)
        
        // Transform target to vehicle frame
        val dx = targetPoint.x - state.x
        val dy = targetPoint.y - state.y
        
        // Rotate to vehicle frame
        val localX = dx * cos(state.theta) + dy * sin(state.theta)
        val localY = -dx * sin(state.theta) + dy * cos(state.theta)
        
        // Calculate curvature (κ = 2y / L²)
        // Using non-holonomic kinematic model
        val curvature = if (ld > 0.01f) {
            2f * localY / (ld * ld)
        } else {
            0f
        }
        
        // Calculate angular velocity
        // v_angular = κ * v_linear
        val speed = calculateSpeed(targetIdx)
        var angularVel = curvature * speed
        
        // Clamp angular velocity
        angularVel = angularVel.coerceIn(-maxAngularVel, maxAngularVel)
        
        // Calculate linear velocity
        // Slow down for sharp turns
        val turnFactor = 1f - abs(angularVel) / maxAngularVel
        val linearVel = speed * (0.5f + 0.5f * turnFactor)
        
        // Check if goal reached
        val isGoalReached = isGoalReached(state, targetIdx)
        
        if (isGoalReached) {
            Log.i(TAG, "Goal reached!")
            return ControlOutput(0f, 0f, path.size - 1, true)
        }
        
        return ControlOutput(linearVel, angularVel, targetIdx, false)
    }

    /**
     * Find the target point on the path at lookahead distance
     */
    private fun findTargetPoint(state: RobotState): AStarPlanner.Waypoint? {
        if (path.isEmpty()) return null
        
        // Start from current waypoint
        for (i in currentWaypointIndex until path.size) {
            val wp = path[i]
            val dist = distance(state.x, state.y, wp.x, wp.y)
            
            if (dist >= lookaheadDistance) {
                return wp
            }
        }
        
        // Return last waypoint if no lookahead point found
        return path.lastOrNull()
    }

    /**
     * Find index of lookahead point
     */
    private fun findLookaheadIndex(state: RobotState, targetPoint: AStarPlanner.Waypoint): Int {
        for (i in currentWaypointIndex until path.size) {
            if (path[i] === targetPoint) {
                // Advance current waypoint if we're close
                val wp = path[i]
                val dist = distance(state.x, state.y, wp.x, wp.y)
                if (dist < WAYPOINT_THRESHOLD * 2) {
                    currentWaypointIndex = (currentWaypointIndex + 1).coerceAtMost(path.size - 1)
                }
                return i
            }
        }
        return currentWaypointIndex
    }

    /**
     * Calculate speed based on remaining path
     */
    private fun calculateSpeed(targetIdx: Int): Float {
        val remaining = path.size - targetIdx
        
        // Reduce speed as we approach goal
        return when {
            remaining <= 2 -> minSpeed
            remaining <= 5 -> targetSpeed * 0.7f
            else -> targetSpeed
        }
    }

    /**
     * Check if goal is reached
     */
    private fun isGoalReached(state: RobotState, targetIdx: Int): Boolean {
        if (path.isEmpty()) return true
        
        val goal = path.last()
        val distToGoal = distance(state.x, state.y, goal.x, goal.y)
        
        // Goal reached if close enough to final waypoint
        if (distToGoal < GOAL_THRESHOLD) {
            return true
        }
        
        // Also check if we've processed all waypoints
        return targetIdx >= path.size - 1 && distToGoal < GOAL_THRESHOLD * 2
    }

    /**
     * Calculate distance between two points
     */
    private fun distance(x1: Float, y1: Float, x2: Float, y2: Float): Float {
        val dx = x2 - x1
        val dy = y2 - y1
        return sqrt(dx * dx + dy * dy)
    }

    /**
     * Reset controller state
     */
    fun reset() {
        currentWaypointIndex = 0
        path = emptyList()
    }

    /**
     * Get progress on path (0.0 to 1.0)
     */
    fun getProgress(): Float {
        if (path.isEmpty()) return 1f
        return currentWaypointIndex.toFloat() / (path.size - 1).coerceAtLeast(1)
    }

    /**
     * Get remaining distance to goal
     */
    fun getRemainingDistance(state: RobotState): Float {
        if (path.isEmpty()) return 0f
        
        var total = distance(state.x, state.y, path[currentWaypointIndex].x, path[currentWaypointIndex].y)
        
        for (i in currentWaypointIndex until path.size - 1) {
            total += distance(
                path[i].x, path[i].y,
                path[i + 1].x, path[i + 1].y
            )
        }
        
        return total
    }

    /**
     * Get remaining path as waypoints
     */
    fun getRemainingPath(): List<AStarPlanner.Waypoint> {
        if (path.isEmpty() || currentWaypointIndex >= path.size) {
            return emptyList()
        }
        return path.subList(currentWaypointIndex, path.size)
    }

    /**
     * Skip to specific waypoint index
     */
    fun skipTo(index: Int) {
        currentWaypointIndex = index.coerceIn(0, (path.size - 1).coerceAtLeast(0))
    }
}
