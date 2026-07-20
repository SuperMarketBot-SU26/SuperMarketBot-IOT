package com.smartmarketbot.hub.slam.ui

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.View
import com.smartmarketbot.hub.slam.SLAMEngine
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

/**
 * SLAM Map View - Custom View for rendering SLAM visualization
 * 
 * Renders:
 * - Occupancy grid
 * - Robot pose (arrow)
 * - Scan points
 * - Waypoints and path
 */
class SLAMMapView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    // Paints
    private val gridPaint = Paint().apply {
        style = Paint.Style.FILL
    }
    
    private val gridLinePaint = Paint().apply {
        color = Color.argb(30, 255, 255, 255)
        strokeWidth = 1f
        style = Paint.Style.STROKE
    }
    
    private val scanPaint = Paint().apply {
        color = Color.argb(150, 128, 128, 128)
        strokeWidth = 2f
        style = Paint.Style.FILL
        isAntiAlias = true
    }
    
    private val robotPaint = Paint().apply {
        color = Color.argb(255, 33, 150, 243)
        strokeWidth = 3f
        style = Paint.Style.FILL
        isAntiAlias = true
    }
    
    private val robotOutlinePaint = Paint().apply {
        color = Color.argb(255, 21, 101, 192)
        strokeWidth = 2f
        style = Paint.Style.STROKE
        isAntiAlias = true
    }
    
    private val pathPaint = Paint().apply {
        color = Color.argb(200, 76, 175, 80)
        strokeWidth = 4f
        style = Paint.Style.STROKE
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
        isAntiAlias = true
    }
    
    private val waypointPaint = Paint().apply {
        color = Color.argb(255, 255, 152, 0)
        strokeWidth = 2f
        style = Paint.Style.FILL
        isAntiAlias = true
    }
    
    private val waypointTextPaint = Paint().apply {
        color = Color.WHITE
        textSize = 28f
        textAlign = Paint.Align.CENTER
        isAntiAlias = true
    }
    
    private val obstaclePaint = Paint().apply {
        color = Color.argb(100, 244, 67, 54)
        strokeWidth = 3f
        style = Paint.Style.STROKE
        isAntiAlias = true
    }

    // SLAM data
    private var occupancyGrid: Array<FloatArray>? = null
    private var scanPoints: List<PointF> = emptyList()
    private var robotPose: SLAMEngine.Pose = SLAMEngine.Pose()
    private var pathPoints: List<PointF> = emptyList()
    private var waypoints: List<WaypointData> = emptyList()

    // View settings
    private var viewScale = 50f  // pixels per meter
    private var showGrid = true
    private var showScanPoints = true
    private var showPath = true
    private var showWaypoints = true
    
    // Grid colors
    private val freeColor = Color.rgb(40, 40, 40)
    private val occupiedColor = Color.rgb(20, 20, 20)
    private val unknownColor = Color.rgb(60, 60, 60)
    
    // Robot path history
    private val pathHistory = mutableListOf<PointF>()
    private val maxPathHistory = 500

    // Data class for waypoint
    data class WaypointData(
        val x: Float,
        val y: Float,
        val index: Int,
        val isTarget: Boolean = false
    )

    /**
     * Update SLAM data
     */
    fun updateData(
        grid: Array<FloatArray>?,
        scan: List<PointF>,
        pose: SLAMEngine.Pose,
        path: List<PointF>? = null,
        waypoints: List<WaypointData>? = null
    ) {
        this.occupancyGrid = grid
        this.scanPoints = scan
        this.robotPose = pose
        
        // Add to path history
        pathHistory.add(PointF(pose.x, pose.y))
        if (pathHistory.size > maxPathHistory) {
            pathHistory.removeAt(0)
        }
        
        path?.let { this.pathPoints = it }
        waypoints?.let { this.waypoints = it }
        
        invalidate()
    }

    /**
     * Update navigation path
     */
    fun updateNavigationPath(path: List<Pair<Float, Float>>) {
        this.pathPoints = path.map { PointF(it.first, it.second) }
        invalidate()
    }

    /**
     * Update only pose (for frequent updates)
     */
    fun updatePose(pose: SLAMEngine.Pose) {
        this.robotPose = pose
        
        // Add to path history
        pathHistory.add(PointF(pose.x, pose.y))
        if (pathHistory.size > maxPathHistory) {
            pathHistory.removeAt(0)
        }
        
        invalidate()
    }

    /**
     * Update scan points only
     */
    fun updateScanPoints(scan: List<PointF>) {
        this.scanPoints = scan
        invalidate()
    }

    /**
     * Update grid only (less frequent)
     */
    fun updateGrid(grid: Array<FloatArray>) {
        this.occupancyGrid = grid
        invalidate()
    }

    /**
     * Clear all data
     */
    fun clear() {
        occupancyGrid = null
        scanPoints = emptyList()
        pathPoints = emptyList()
        waypoints = emptyList()
        pathHistory.clear()
        invalidate()
    }

    /**
     * Reset path history
     */
    fun resetPath() {
        pathHistory.clear()
        invalidate()
    }

    /**
     * Set view scale (pixels per meter)
     */
    fun setScale(scale: Float) {
        viewScale = scale
        invalidate()
    }

    /**
     * Toggle grid visibility
     */
    fun setShowGrid(show: Boolean) {
        showGrid = show
        invalidate()
    }

    /**
     * Toggle scan points visibility
     */
    fun setShowScanPoints(show: Boolean) {
        showScanPoints = show
        invalidate()
    }

    /**
     * Toggle path visibility
     */
    fun setShowPath(show: Boolean) {
        showPath = show
        invalidate()
    }

    /**
     * Toggle waypoint visibility
     */
    fun setShowWaypoints(show: Boolean) {
        showWaypoints = show
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        
        val centerX = width / 2f
        val centerY = height / 2f
        
        // Draw grid
        if (showGrid && occupancyGrid != null) {
            drawOccupancyGrid(canvas, centerX, centerY)
        }
        
        // Draw grid lines
        if (showGrid) {
            drawGridLines(canvas, centerX, centerY)
        }
        
        // Draw scan points
        if (showScanPoints) {
            drawScanPoints(canvas, centerX, centerY)
        }
        
        // Draw path history
        if (showPath && pathHistory.size > 1) {
            drawPath(canvas, centerX, centerY)
        }
        
        // Draw waypoints
        if (showWaypoints) {
            drawWaypoints(canvas, centerX, centerY)
        }
        
        // Draw robot
        drawRobot(canvas, centerX, centerY)
    }

    private fun drawOccupancyGrid(canvas: Canvas, centerX: Float, centerY: Float) {
        val grid = occupancyGrid ?: return
        
        val cellSize = viewScale * 0.05f  // 5cm per cell
        
        val gridWidth = grid.getOrNull(0)?.size ?: return
        val gridHeight = grid.size
        
        val startX = centerX - (gridWidth * cellSize / 2)
        val startY = centerY - (gridHeight * cellSize / 2)
        
        for (y in 0 until gridHeight) {
            val row = grid[y]
            for (x in 0 until row.size) {
                val prob = row[x]
                
                gridPaint.color = when {
                    prob > 0.7f -> occupiedColor
                    prob < 0.3f -> freeColor
                    else -> unknownColor
                }
                
                canvas.drawRect(
                    startX + x * cellSize,
                    startY + y * cellSize,
                    startX + (x + 1) * cellSize,
                    startY + (y + 1) * cellSize,
                    gridPaint
                )
            }
        }
    }

    private fun drawGridLines(canvas: Canvas, centerX: Float, centerY: Float) {
        val gridSpacing = viewScale  // 1 meter
        
        // Vertical lines
        var x = centerX % gridSpacing
        while (x < width) {
            canvas.drawLine(x, 0f, x, height.toFloat(), gridLinePaint)
            x += gridSpacing
        }
        
        // Horizontal lines
        var y = centerY % gridSpacing
        while (y < height) {
            canvas.drawLine(0f, y, width.toFloat(), y, gridLinePaint)
            y += gridSpacing
        }
    }

    private fun drawScanPoints(canvas: Canvas, centerX: Float, centerY: Float) {
        for (point in scanPoints) {
            // Transform world to screen
            val screenX = centerX + (point.x - robotPose.x) * viewScale
            val screenY = centerY - (point.y - robotPose.y) * viewScale
            
            // Only draw if in view
            if (screenX >= 0 && screenX < width && screenY >= 0 && screenY < height) {
                canvas.drawCircle(screenX, screenY, 3f, scanPaint)
            }
        }
    }

    private fun drawPath(canvas: Canvas, centerX: Float, centerY: Float) {
        if (pathHistory.size < 2) return
        
        val path = Path()
        var started = false
        
        for (point in pathHistory) {
            val screenX = centerX + (point.x - robotPose.x) * viewScale
            val screenY = centerY - (point.y - robotPose.y) * viewScale
            
            if (!started) {
                path.moveTo(screenX, screenY)
                started = true
            } else {
                path.lineTo(screenX, screenY)
            }
        }
        
        canvas.drawPath(path, pathPaint)
    }

    private fun drawWaypoints(canvas: Canvas, centerX: Float, centerY: Float) {
        for (wp in waypoints) {
            val screenX = centerX + (wp.x - robotPose.x) * viewScale
            val screenY = centerY - (wp.y - robotPose.y) * viewScale
            
            // Draw circle
            val color = if (wp.isTarget) Color.rgb(255, 87, 34) else Color.rgb(255, 152, 0)
            waypointPaint.color = color
            
            canvas.drawCircle(screenX, screenY, 20f, waypointPaint)
            
            // Draw index
            canvas.drawText(wp.index.toString(), screenX, screenY + 10f, waypointTextPaint)
        }
    }

    private fun drawRobot(canvas: Canvas, centerX: Float, centerY: Float) {
        val robotRadius = 15f
        
        // Draw direction indicator (arrow)
        val arrowLength = 30f
        val arrowAngle = robotPose.theta
        
        val endX = centerX + cos(arrowAngle) * arrowLength
        val endY = centerY - sin(arrowAngle) * arrowLength
        
        // Draw arrow line
        canvas.drawLine(centerX, centerY, endX, endY, robotOutlinePaint)
        
        // Draw robot body
        canvas.drawCircle(centerX, centerY, robotRadius, robotPaint)
        canvas.drawCircle(centerX, centerY, robotRadius, robotOutlinePaint)
        
        // Draw direction arrow
        val arrowPath = Path()
        arrowPath.moveTo(endX, endY)
        arrowPath.lineTo(
            endX - 10 * cos(arrowAngle - 0.5f),
            endY + 10 * sin(arrowAngle - 0.5f)
        )
        arrowPath.lineTo(
            endX - 10 * cos(arrowAngle + 0.5f),
            endY + 10 * sin(arrowAngle + 0.5f)
        )
        arrowPath.close()
        canvas.drawPath(arrowPath, robotPaint)
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val desiredSize = 800
        
        val widthMode = MeasureSpec.getMode(widthMeasureSpec)
        val widthSize = MeasureSpec.getSize(widthMeasureSpec)
        val heightMode = MeasureSpec.getMode(heightMeasureSpec)
        val heightSize = MeasureSpec.getSize(heightMeasureSpec)
        
        val width = when (widthMode) {
            MeasureSpec.EXACTLY -> widthSize
            MeasureSpec.AT_MOST -> min(desiredSize, widthSize)
            else -> desiredSize
        }
        
        val height = when (heightMode) {
            MeasureSpec.EXACTLY -> heightSize
            MeasureSpec.AT_MOST -> min(desiredSize, heightSize)
            else -> desiredSize
        }
        
        setMeasuredDimension(width, height)
    }
}
