package com.smartmarketbot.hub.navigation

import android.util.Log
import com.smartmarketbot.hub.slam.SLAMEngine
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.pow

/**
 * A* Path Planner
 * 
 * Finds optimal path on occupancy grid using A* algorithm
 * Optimized for real-time navigation on Android
 */
class AStarPlanner(
    private val gridResolution: Float = 0.05f  // 5cm per cell (same as SLAM)
) {
    companion object {
        private const val TAG = "AStarPlanner"
        
        // Cost weights
        private const val COST_TURN = 10        // Cost for turning
        private const val COST_REVERSE = 5     // Cost for reversing
        private const val COST_OBSTACLE = 100  // Cost multiplier near obstacles
        private const val GOAL_TOLERANCE = 2   // cells
    }

    // Node for A* search
    data class Node(
        val x: Int,
        val y: Int,
        val g: Float,      // Cost from start
        val h: Float,      // Heuristic cost to goal
        val f: Float,      // Total cost (g + h)
        val parent: Node?,
        val direction: Int // 0=N, 1=E, 2=S, 3=W, 4=NE, 5=SE, 6=SW, 7=NW
    ) : Comparable<Node> {
        override fun compareTo(other: Node): Int = f.compareTo(other.f)
    }

    // Direction vectors (8-directional movement)
    private val dirVectors = arrayOf(
        intArrayOf(0, -1),   // N
        intArrayOf(1, 0),    // E
        intArrayOf(0, 1),    // S
        intArrayOf(-1, 0),   // W
        intArrayOf(1, -1),   // NE
        intArrayOf(1, 1),    // SE
        intArrayOf(-1, 1),   // SW
        intArrayOf(-1, -1)   // NW
    )
    
    // Diagonal movement cost
    private val dirCost = floatArrayOf(1f, 1f, 1f, 1f, 1.414f, 1.414f, 1.414f, 1.414f)

    // Path result
    data class PathResult(
        val waypoints: List<Waypoint>,    // Path waypoints
        val success: Boolean,
        val pathLength: Float,           // meters
        val computationTimeMs: Long,
        val message: String
    )

    // Waypoint in world coordinates
    data class Waypoint(
        val x: Float,    // meters
        val y: Float,    // meters
        val angle: Float  // radians (optional heading)
    )

    /**
     * Find path from start to goal using A*
     * 
     * @param slamEngine SLAM engine with occupancy grid
     * @param startX Start X in world coordinates (meters)
     * @param startY Start Y in world coordinates (meters)
     * @param goalX Goal X in world coordinates (meters)
     * @param goalY Goal Y in world coordinates (meters)
     * @param maxIterations Maximum iterations to prevent infinite loops
     * @return PathResult with waypoints or error message
     */
    fun findPath(
        slamEngine: SLAMEngine,
        startX: Float,
        startY: Float,
        goalX: Float,
        goalY: Float,
        maxIterations: Int = 10000
    ): PathResult {
        val startTime = System.currentTimeMillis()
        
        // Get grid dimensions
        val grid = slamEngine.getGrid()
        val gridHeight = grid.size
        val gridWidth = if (grid.isNotEmpty()) grid[0].size else 0
        
        if (gridWidth == 0 || gridHeight == 0) {
            return PathResult(
                emptyList(), false, 0f, 
                System.currentTimeMillis() - startTime,
                "Invalid grid"
            )
        }
        
        // Convert world to grid coordinates
        val mapCenterX = gridWidth / 2
        val mapCenterY = gridHeight / 2
        
        val startGridX = mapCenterX + (startX / gridResolution).toInt()
        val startGridY = mapCenterY + (startY / gridResolution).toInt()
        val goalGridX = mapCenterX + (goalX / gridResolution).toInt()
        val goalGridY = mapCenterY + (goalY / gridResolution).toInt()
        
        // Validate coordinates
        if (!isValidCell(startGridX, startGridY, gridWidth, gridHeight) ||
            !isValidCell(goalGridX, goalGridY, gridWidth, gridHeight)) {
            return PathResult(
                emptyList(), false, 0f,
                System.currentTimeMillis() - startTime,
                "Start or goal outside map bounds"
            )
        }
        
        // Check if goal is accessible
        if (slamEngine.isOccupied(goalX, goalY, 0.5f)) {
            // Try to find nearest free cell
            val adjustedGoal = findNearestFreeCell(slamEngine, goalX, goalY)
            if (adjustedGoal == null) {
                return PathResult(
                    emptyList(), false, 0f,
                    System.currentTimeMillis() - startTime,
                    "Goal is in obstacle"
                )
            }
            return findPath(slamEngine, startX, startY, adjustedGoal.first, adjustedGoal.second, maxIterations)
        }
        
        // A* search
        val openSet = java.util.PriorityQueue<Node>()
        val closedSet = mutableSetOf<Pair<Int, Int>>()
        val gScores = mutableMapOf<Pair<Int, Int>, Float>()
        
        val startNode = Node(
            x = startGridX,
            y = startGridY,
            g = 0f,
            h = heuristic(startGridX, startGridY, goalGridX, goalGridY),
            f = heuristic(startGridX, startGridY, goalGridX, goalGridY),
            parent = null,
            direction = -1
        )
        
        openSet.add(startNode)
        gScores[startGridX to startGridY] = 0f
        
        var iterations = 0
        var current: Node? = null
        
        while (openSet.isNotEmpty() && iterations < maxIterations) {
            iterations++
            current = openSet.poll()
            
            // Check if reached goal
            if (abs(current.x - goalGridX) <= GOAL_TOLERANCE && 
                abs(current.y - goalGridY) <= GOAL_TOLERANCE) {
                break
            }
            
            // Skip if already processed
            val pos = current.x to current.y
            if (pos in closedSet) continue
            closedSet.add(pos)
            
            // Explore neighbors
            for (dir in dirVectors.indices) {
                val nx = current.x + dirVectors[dir][0]
                val ny = current.y + dirVectors[dir][1]
                val nPos = nx to ny
                
                // Skip if out of bounds or in closed set
                if (!isValidCell(nx, ny, gridWidth, gridHeight)) continue
                if (nPos in closedSet) continue
                
                // Check if cell is traversable
                val worldX = (nx - mapCenterX) * gridResolution
                val worldY = (ny - mapCenterY) * gridResolution
                
                if (slamEngine.isOccupied(worldX, worldY, 0.6f)) continue
                
                // Calculate cost with turn penalty
                var moveCost = dirCost[dir]
                if (current.direction != -1 && current.direction != dir) {
                    // Turning costs extra
                    val turnAngle = abs(current.direction - dir) * 45
                    moveCost += COST_TURN * (turnAngle / 90f)
                }
                
                // Add obstacle proximity cost
                val obstacleCost = getObstacleProximityCost(slamEngine, worldX, worldY)
                moveCost += obstacleCost
                
                val newG = current.g + moveCost
                
                // Update if better path found
                val existingG = gScores[nPos]
                if (existingG == null || newG < existingG) {
                    gScores[nPos] = newG
                    val h = heuristic(nx, ny, goalGridX, goalGridY)
                    val newNode = Node(
                        x = nx,
                        y = ny,
                        g = newG,
                        h = h,
                        f = newG + h,
                        parent = current,
                        direction = dir
                    )
                    openSet.add(newNode)
                }
            }
        }
        
        // Reconstruct path
        if (current == null || abs(current.x - goalGridX) > GOAL_TOLERANCE || 
            abs(current.y - goalGridY) > GOAL_TOLERANCE) {
            return PathResult(
                emptyList(), false, 0f,
                System.currentTimeMillis() - startTime,
                "No path found (explored $iterations nodes)"
            )
        }
        
        val path = reconstructPath(current, mapCenterX, mapCenterY)
        val pathLength = calculatePathLength(path)
        
        Log.i(TAG, "Path found: ${path.size} waypoints, ${"%.2f".format(pathLength)}m, ${iterations} iterations")
        
        return PathResult(
            waypoints = path,
            success = true,
            pathLength = pathLength,
            computationTimeMs = System.currentTimeMillis() - startTime,
            message = "Path found with ${path.size} waypoints"
        )
    }

    /**
     * Heuristic: Euclidean distance
     */
    private fun heuristic(x1: Int, y1: Int, x2: Int, y2: Int): Float {
        return sqrt((x2 - x1).toFloat().pow(2) + (y2 - y1).toFloat().pow(2))
    }

    /**
     * Check if cell is within grid bounds
     */
    private fun isValidCell(x: Int, y: Int, width: Int, height: Int): Boolean {
        return x in 0 until width && y in 0 until height
    }

    /**
     * Get cost based on distance to nearest obstacle
     */
    private fun getObstacleProximityCost(slamEngine: SLAMEngine, worldX: Float, worldY: Float): Float {
        var minDist = Float.MAX_VALUE
        val checkRadius = 3 // cells
        
        for (dx in -checkRadius..checkRadius) {
            for (dy in -checkRadius..checkRadius) {
                val wx = worldX + dx * gridResolution
                val wy = worldY + dy * gridResolution
                if (slamEngine.isOccupied(wx, wy, 0.5f)) {
                    val dist = sqrt(dx.toFloat().pow(2) + dy.toFloat().pow(2)) * gridResolution
                    minDist = minOf(minDist, dist)
                }
            }
        }
        
        return if (minDist < 0.3f) {
            COST_OBSTACLE * (1f - minDist / 0.3f)
        } else 0f
    }

    /**
     * Find nearest free cell to given position
     */
    private fun findNearestFreeCell(
        slamEngine: SLAMEngine,
        x: Float,
        y: Float
    ): Pair<Float, Float>? {
        val searchRadius = 10 // cells
        val step = gridResolution
        
        for (r in 1..searchRadius) {
            for (angle in 0..360 step 30) {
                val rad = Math.toRadians(angle.toDouble())
                val nx = x + r * step * kotlin.math.cos(rad).toFloat()
                val ny = y + r * step * kotlin.math.sin(rad).toFloat()
                
                if (!slamEngine.isOccupied(nx, ny, 0.5f)) {
                    return nx to ny
                }
            }
        }
        return null
    }

    /**
     * Reconstruct path from goal node to start
     */
    private fun reconstructPath(node: Node, mapCenterX: Int, mapCenterY: Int): List<Waypoint> {
        val path = mutableListOf<Waypoint>()
        var current: Node? = node
        
        while (current != null) {
            val worldX = (current.x - mapCenterX) * gridResolution
            val worldY = (current.y - mapCenterY) * gridResolution
            path.add(0, Waypoint(worldX, worldY, 0f))
            current = current.parent
        }
        
        // Add angle information based on direction
        for (i in 1 until path.size) {
            val prev = path[i - 1]
            val curr = path[i]
            val angle = kotlin.math.atan2(curr.y - prev.y, curr.x - prev.x).toFloat()
            path[i] = Waypoint(curr.x, curr.y, angle)
        }
        
        // Smooth path
        return smoothPath(path)
    }

    /**
     * Smooth path by removing unnecessary waypoints
     */
    private fun smoothPath(path: List<Waypoint>): List<Waypoint> {
        if (path.size < 3) return path
        
        val smoothed = mutableListOf(path.first())
        var i = 0
        
        while (i < path.size - 1) {
            var j = path.size - 1
            while (j > i + 1) {
                if (isLineOfSight(path[i], path[j])) {
                    break
                }
                j--
            }
            smoothed.add(path[j])
            i = j
        }
        
        return smoothed
    }

    /**
     * Check if there's line of sight between two waypoints
     */
    private fun isLineOfSight(from: Waypoint, to: Waypoint): Boolean {
        val dx = to.x - from.x
        val dy = to.y - from.y
        val dist = sqrt(dx * dx + dy * dy)
        val steps = (dist / gridResolution).toInt()
        
        for (i in 0..steps) {
            val ratio = i.toFloat() / steps
            val x = from.x + dx * ratio
            val y = from.y + dy * ratio
            if (slamEngine?.isOccupied(x, y, 0.5f) == true) {
                return false
            }
        }
        return true
    }

    // Reference to SLAM engine for line of sight check
    private var slamEngine: SLAMEngine? = null
    
    fun setSLAMEngine(engine: SLAMEngine) {
        slamEngine = engine
    }

    /**
     * Calculate total path length
     */
    private fun calculatePathLength(path: List<Waypoint>): Float {
        if (path.size < 2) return 0f
        
        var length = 0f
        for (i in 1 until path.size) {
            val dx = path[i].x - path[i - 1].x
            val dy = path[i].y - path[i - 1].y
            length += sqrt(dx * dx + dy * dy)
        }
        return length
    }
}
