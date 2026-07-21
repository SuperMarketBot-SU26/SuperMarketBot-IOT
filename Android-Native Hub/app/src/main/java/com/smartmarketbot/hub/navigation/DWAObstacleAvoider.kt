package com.smartmarketbot.hub.navigation

import android.util.Log
import com.smartmarketbot.hub.slam.SLAMEngine
import kotlin.math.*

/**
 * Dynamic Window Approach (DWA) Obstacle Avoidance
 * 
 * DWA generates safe velocity commands by:
 * 1. Discretizing the velocity space
 * 2. Simulating trajectories for each (v, ω) pair
 * 3. Scoring trajectories based on:
 *    - Progress towards goal
 *    - Clearance from obstacles
 *    - Forward velocity
 * 4. Selecting the best velocity command
 * 
 * Reference:
 * - "The Dynamic Window Approach to Collision Avoidance" 
 *   by Dieter Fox, Wolfram Burgard, and Sebastian Thrun
 */
class DWAObstacleAvoider(
    private val slamEngine: SLAMEngine,
    private val wheelBase: Float = 0.3f
) {
    companion object {
        private const val TAG = "DWA"
        
        // Velocity limits
        const val MAX_LINEAR_VEL = 0.5f      // m/s
        const val MIN_LINEAR_VEL = 0f       // m/s
        const val MAX_ANGULAR_VEL = 2.5f    // rad/s
        
        // Acceleration limits
        const val MAX_LINEAR_ACCEL = 0.8f   // m/s²
        const val MAX_ANGULAR_ACCEL = 3.0f   // rad/s²
        
        // Simulation parameters
        const val SIM_TIME = 1.0f           // seconds to simulate
        const val DT = 0.1f                 // simulation timestep
        
        // Scoring weights
        const val WEIGHT_HEADING = 0.2f     // progress towards goal
        const val WEIGHT_CLEARANCE = 0.4f   // distance to obstacles
        const val WEIGHT_VELOCITY = 0.1f    // forward velocity
        const val WEIGHT_PATH = 0.3f        // follow path
        
        // Safety
        const val ROBOT_RADIUS = 0.15f      // meters
        const val SAFE_DISTANCE = 0.3f       // meters
    }

    // Velocity command
    data class VelocityCmd(
        val linearVel: Float,    // m/s
        val angularVel: Float,   // rad/s
        val score: Float = 0f
    )

    // Robot trajectory
    data class Trajectory(
        val points: List<TrajectoryPoint>,
        val linearVel: Float,
        val angularVel: Float,
        val clearance: Float  // minimum distance to obstacle
    )

    data class TrajectoryPoint(
        val x: Float,
        val y: Float,
        val theta: Float
    )

    // Target info for heading calculation
    private var targetX = 0f
    private var targetY = 0f
    private var hasTarget = false

    /**
     * Set target position for heading calculation
     */
    fun setTarget(x: Float, y: Float) {
        targetX = x
        targetY = y
        hasTarget = true
    }

    /**
     * Clear target
     */
    fun clearTarget() {
        hasTarget = false
    }

    /**
     * Compute safe velocity command avoiding obstacles
     * 
     * @param robotX Current X position (meters)
     * @param robotY Current Y position (meters)
     * @param robotTheta Current heading (radians)
     * @param currentLinearVel Current linear velocity (m/s)
     * @param currentAngularVel Current angular velocity (rad/s)
     * @return VelocityCmd with safe velocities
     */
    fun computeSafeVelocity(
        robotX: Float,
        robotY: Float,
        robotTheta: Float,
        currentLinearVel: Float,
        currentAngularVel: Float
    ): VelocityCmd {
        
        // Step 1: Generate dynamic window of admissible velocities
        val window = generateDynamicWindow(currentLinearVel, currentAngularVel)
        
        // Step 2: Discretize velocity space
        val vSamples = 5
        val wSamples = 7
        val vStep = (window.maxLinearVel - window.minLinearVel) / vSamples
        val wStep = (window.maxAngularVel - window.minAngularVel) / wSamples
        
        var bestVelocity = VelocityCmd(0f, 0f, Float.MIN_VALUE)
        val trajectories = mutableListOf<Trajectory>()
        
        // Step 3: Evaluate each velocity option
        for (i in 0..vSamples) {
            val v = window.minLinearVel + i * vStep
            
            for (j in 0..wSamples) {
                val w = window.minAngularVel + j * wStep
                
                // Skip if velocity is zero
                if (abs(v) < 0.01f && abs(w) < 0.01f) continue
                
                // Simulate trajectory
                val trajectory = simulateTrajectory(robotX, robotY, robotTheta, v, w)
                trajectories.add(trajectory)
                
                // Score trajectory
                val score = evaluateTrajectory(
                    trajectory,
                    robotX, robotY, robotTheta,
                    v, w
                )
                
                if (score > bestVelocity.score) {
                    bestVelocity = VelocityCmd(v, w, score)
                }
            }
        }
        
        // Step 4: Return best velocity, or zero if no safe option
        return if (bestVelocity.score > 0f) {
            bestVelocity
        } else {
            // Emergency stop
            Log.w(TAG, "No safe velocity found, stopping!")
            VelocityCmd(0f, 0f, 0f)
        }
    }

    /**
     * Generate dynamic window based on current velocity and acceleration limits
     */
    private fun generateDynamicWindow(
        currentLinearVel: Float,
        currentAngularVel: Float
    ): VelocityWindow {
        // Minimum velocities (can be negative for reversing)
        val minV = maxOf(
            MIN_LINEAR_VEL,
            currentLinearVel - MAX_LINEAR_ACCEL * DT
        )
        val maxV = minOf(
            MAX_LINEAR_VEL,
            currentLinearVel + MAX_LINEAR_ACCEL * DT
        )
        
        val minW = maxOf(
            -MAX_ANGULAR_VEL,
            currentAngularVel - MAX_ANGULAR_ACCEL * DT
        )
        val maxW = minOf(
            MAX_ANGULAR_VEL,
            currentAngularVel + MAX_ANGULAR_ACCEL * DT
        )
        
        return VelocityWindow(minV, maxV, minW, maxW)
    }

    data class VelocityWindow(
        val minLinearVel: Float,
        val maxLinearVel: Float,
        val minAngularVel: Float,
        val maxAngularVel: Float
    )

    /**
     * Simulate trajectory for given velocities
     */
    private fun simulateTrajectory(
        x0: Float,
        y0: Float,
        theta0: Float,
        v: Float,
        w: Float
    ): Trajectory {
        val points = mutableListOf<TrajectoryPoint>()
        
        var x = x0
        var y = y0
        var theta = theta0
        
        var minClearance = Float.MAX_VALUE
        
        // Simulate forward in time
        val steps = (SIM_TIME / DT).toInt()
        for (i in 0..steps) {
            // Record point
            points.add(TrajectoryPoint(x, y, theta))
            
            // Check clearance
            val clearance = getClearance(x, y)
            minClearance = minOf(minClearance, clearance)
            
            // If too close to obstacle, stop simulation
            if (clearance < ROBOT_RADIUS) {
                break
            }
            
            // Kinematic model
            // x' = v * cos(θ)
            // y' = v * sin(θ)
            // θ' = w
            x += v * cos(theta) * DT
            y += v * sin(theta) * DT
            theta += w * DT
            
            // Normalize theta
            while (theta > PI.toFloat()) theta -= (2 * PI).toFloat()
            while (theta < -PI.toFloat()) theta += (2 * PI).toFloat()
        }
        
        return Trajectory(points, v, w, minClearance)
    }

    /**
     * Get clearance (distance to nearest obstacle)
     */
    private fun getClearance(x: Float, y: Float): Float {
        // Sample points around robot
        val numSamples = 16
        var minDist = Float.MAX_VALUE
        
        for (i in 0 until numSamples) {
            val angle = (2 * PI * i / numSamples).toFloat()
            val px = x + ROBOT_RADIUS * cos(angle)
            val py = y + ROBOT_RADIUS * sin(angle)
            
            // Check clearance in multiple directions
            var r = 0.1f
            while (r <= SAFE_DISTANCE) {
                val checkX = x + r * cos(angle)
                val checkY = y + r * sin(angle)
                
                if (slamEngine.isOccupied(checkX, checkY, 0.6f)) {
                    minDist = minOf(minDist, r)
                    break
                }
                r += 0.05f
            }
        }
        
        // Also check center
        if (slamEngine.isOccupied(x, y, 0.6f)) {
            minDist = 0f
        }
        
        return if (minDist == Float.MAX_VALUE) SAFE_DISTANCE else minDist
    }

    /**
     * Evaluate trajectory and return score
     */
    private fun evaluateTrajectory(
        trajectory: Trajectory,
        robotX: Float,
        robotY: Float,
        robotTheta: Float,
        v: Float,
        w: Float
    ): Float {
        var score = 0f
        
        // Check if trajectory is safe
        if (trajectory.clearance < ROBOT_RADIUS) {
            return 0f
        }
        
        // Weight 1: Heading to target (progress towards goal)
        if (hasTarget) {
            val finalPoint = trajectory.points.lastOrNull() ?: return 0f
            val headingScore = evaluateHeading(finalPoint.x, finalPoint.y, robotTheta)
            score += WEIGHT_HEADING * headingScore
        }
        
        // Weight 2: Clearance (distance to obstacles)
        val clearanceScore = trajectory.clearance / SAFE_DISTANCE
        score += WEIGHT_CLEARANCE * clearanceScore
        
        // Weight 3: Velocity (prefer faster)
        val velocityScore = abs(v) / MAX_LINEAR_VEL
        score += WEIGHT_VELOCITY * velocityScore
        
        // Weight 4: Path following
        if (hasTarget) {
            val pathScore = evaluatePathProgress(trajectory)
            score += WEIGHT_PATH * pathScore
        }
        
        // Penalty for turning too sharp when moving fast
        if (abs(v) > 0.2f && abs(w) > 1.5f) {
            score *= 0.5f
        }
        
        return score
    }

    /**
     * Evaluate heading towards target
     * Returns 1.0 if facing target, 0.0 if opposite
     *
     * @param fromX trajectory endpoint X (không phải target)
     * @param fromY trajectory endpoint Y
     * @param robotTheta current robot heading (radians)
     */
    private fun evaluateHeading(fromX: Float, fromY: Float, robotTheta: Float): Float {
        if (!hasTarget) return 0.5f

        // Angle từ trajectory endpoint tới target
        val toTargetAngle = atan2(
            targetY - fromY,
            targetX - fromX
        )

        // Angular difference giữa heading hiện tại và hướng tới target
        var angleDiff = toTargetAngle - robotTheta

        // Normalize về [-π, π]
        while (angleDiff > PI.toFloat()) angleDiff -= (2 * PI).toFloat()
        while (angleDiff < -PI.toFloat()) angleDiff += (2 * PI).toFloat()

        // Convert về 0..1 score (1 = chĩa thẳng target, 0 = quay lưng)
        return 1f - abs(angleDiff) / PI.toFloat()
    }

    /**
     * Evaluate path progress (how much closer to target)
     */
    private fun evaluatePathProgress(trajectory: Trajectory): Float {
        if (!hasTarget || trajectory.points.isEmpty()) return 0.5f
        
        val finalPoint = trajectory.points.last()
        val startPoint = trajectory.points.first()
        
        val distStart = hypot(targetX - startPoint.x, targetY - startPoint.y)
        val distEnd = hypot(targetX - finalPoint.x, targetY - finalPoint.y)
        
        // Score = improvement in distance
        val improvement = distStart - distEnd
        return (improvement / SAFE_DISTANCE).coerceIn(0f, 1f)
    }

    /**
     * Check if path is clear between two points
     */
    fun isPathClear(fromX: Float, fromY: Float, toX: Float, toY: Float): Boolean {
        val dist = hypot(toX - fromX, toY - fromY)
        val steps = (dist / 0.05f).toInt()
        
        for (i in 0..steps) {
            val t = i.toFloat() / steps
            val x = fromX + (toX - fromX) * t
            val y = fromY + (toY - fromY) * t
            
            if (slamEngine.isOccupied(x, y, 0.6f)) {
                return false
            }
        }
        return true
    }

    /**
     * Find nearest safe velocity that makes progress
     */
    fun findSafeVelocity(
        robotX: Float,
        robotY: Float,
        robotTheta: Float,
        desiredLinearVel: Float,
        desiredAngularVel: Float
    ): VelocityCmd {
        // First try desired velocity
        val trajectory = simulateTrajectory(robotX, robotY, robotTheta, desiredLinearVel, desiredAngularVel)

        if (trajectory.clearance > ROBOT_RADIUS * 1.5f) {
            return VelocityCmd(desiredLinearVel, desiredAngularVel, 1f)
        }

        // Gradually reduce speed until safe
        val vStep = 0.1f
        val wStep = 0.5f

        for (reduction in 0..5) {
            val v = (desiredLinearVel * (1f - reduction * vStep)).coerceAtLeast(0f)

            for (dw in listOf(-wStep, 0f, wStep)) {
                val w = desiredAngularVel + dw
                val traj = simulateTrajectory(robotX, robotY, robotTheta, v, w)

                if (traj.clearance > ROBOT_RADIUS) {
                    return VelocityCmd(v, w, 1f)
                }
            }
        }

        // No safe velocity found
        return VelocityCmd(0f, 0f, 0f)
    }

    /**
     * Get clearance at specific angle direction
     * @param x Robot X position
     * @param y Robot Y position
     * @param angle Direction to check (radians)
     * @return Distance to nearest obstacle
     */
    fun getClearanceAtAngle(x: Float, y: Float, angle: Float): Float {
        val step = 0.05f
        val maxRange = 2.0f

        var r = step
        while (r <= maxRange) {
            val checkX = x + r * cos(angle)
            val checkY = y + r * sin(angle)

            if (slamEngine.isOccupied(checkX, checkY, 0.6f)) {
                return r
            }
            r += step
        }

        return maxRange
    }

    /**
     * Get SLAM engine reference
     */
    fun getSLAMEngine(): SLAMEngine = slamEngine
}
