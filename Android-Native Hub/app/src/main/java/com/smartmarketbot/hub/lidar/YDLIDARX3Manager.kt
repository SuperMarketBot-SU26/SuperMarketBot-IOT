package com.smartmarketbot.hub.lidar

import android.util.Log
import com.hoho.android.usbserial.driver.CdcAcmSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import java.nio.ByteBuffer
import java.util.concurrent.ConcurrentLinkedQueue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * YDLIDAR X3 Protocol Parser
 *
 * Protocol: YDLIDAR X3 uses a simple binary protocol
 * Header: 0xAA 0x55
 * Each scan point: 2 bytes distance + 1 byte quality + 1 byte angle offset
 *
 * YDLIDAR X3 Specifications:
 * - Scan rate: 5-12 Hz (configurable)
 * - Sample rate: 3000 samples/sec
 * - Range: 0.1-8m
 * - Baudrate: 115200
 */
class YDLIDARX3Manager(
    private val onScanReady: (List<LidarScanPoint>) -> Unit,
    private val onError: (String) -> Unit
) {
    companion object {
        private const val TAG = "YDLIDAR_X3"
        const val BAUD_RATE = 115200

        // Protocol constants
        private const val HEADER_BYTE_1 = 0xAA.toByte()
        private const val HEADER_BYTE_2 = 0x55.toByte()

        // X3 packet types
        private const val CMD_STOP = 0x65
        private const val CMD_SCAN = 0x60
        private const val CMD_FORCE_SCAN = 0x61
        private const val CMD_GET_INFO = 0x01
        private const val CMD_GET_HEALTH = 0x02
    }

    data class LidarScanPoint(
        val angleRad: Float,    // radians (0 = front)
        val distanceMm: Int,    // millimeters
        val quality: Int        // signal quality (0-255)
    )

    data class LidarScan(
        val points: List<LidarScanPoint>,
        val timestampMs: Long,
        val scanRateHz: Float
    )

    private var serialPort: UsbSerialPort? = null
    private var isRunning = false
    private var readThread: Thread? = null

    // Parse state machine
    private var state = ParseState.SYNC1
    private var packetLength = 0
    private val dataBuffer = ByteArray(1024)
    private var dataIndex = 0

    // Scan data queue
    private val scanQueue = ConcurrentLinkedQueue<LidarScan>()

    // Performance metrics
    var totalBytesReceived = 0L
        private set
    var lastScanTimestamp = 0L
        private set
    var scanCount = 0
        private set

    enum class ParseState {
        SYNC1,      // Looking for 0xAA
        SYNC2,      // Looking for 0x55
        LENGTH,     // Got header, reading length
        DATA,       // Reading data payload
        CHECKSUM    // Reading checksum (optional)
    }

    /**
     * Connect to YDLIDAR X3 via USB serial port
     */
    fun connect(port: UsbSerialPort): Boolean {
        return try {
            serialPort = port
            port.open(port.driver.connect(port.device, BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE))
            port.setDTR(true)
            port.setParameters(BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)

            // Send start scan command
            val startCmd = byteArrayOf(HEADER_BYTE_1, CMD_SCAN)
            port.write(startCmd, 1000)

            isRunning = true
            startReadLoop()

            Log.i(TAG, "YDLIDAR X3 connected successfully")
            true
        } catch (e: Exception) {
            Log.e(TAG, "Failed to connect YDLIDAR X3: ${e.message}")
            onError("Connection failed: ${e.message}")
            false
        }
    }

    /**
     * Start continuous scan
     */
    fun startScan(): Boolean {
        return try {
            val cmd = byteArrayOf(HEADER_BYTE_1, CMD_SCAN)
            serialPort?.write(cmd, 1000)
            Log.i(TAG, "Start scan command sent")
            true
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start scan: ${e.message}")
            false
        }
    }

    /**
     * Stop scan
     */
    fun stopScan(): Boolean {
        return try {
            val cmd = byteArrayOf(HEADER_BYTE_1, CMD_STOP)
            serialPort?.write(cmd, 1000)
            Log.i(TAG, "Stop scan command sent")
            true
        } catch (e: Exception) {
            false
        }
    }

    /**
     * Read loop running in background thread
     */
    private fun startReadLoop() {
        readThread = Thread {
            val buffer = ByteArray(2048)
            while (isRunning) {
                try {
                    val bytesRead = serialPort?.read(buffer, 100) ?: 0
                    if (bytesRead > 0) {
                        totalBytesReceived += bytesRead
                        for (i in 0 until bytesRead) {
                            processByte(buffer[i])
                        }
                    }
                } catch (e: Exception) {
                    if (isRunning) {
                        Log.e(TAG, "Read error: ${e.message}")
                        onError("Read error: ${e.message}")
                    }
                    break
                }
            }
        }.apply {
            name = "YDLIDAR_ReadLoop"
            priority = Thread.MAX_PRIORITY
            start()
        }
    }

    /**
     * Protocol state machine for parsing YDLIDAR X3 data
     */
    private fun processByte(b: Byte) {
        when (state) {
            ParseState.SYNC1 -> {
                if (b == HEADER_BYTE_1) {
                    state = ParseState.SYNC2
                }
            }
            ParseState.SYNC2 -> {
                if (b == HEADER_BYTE_2) {
                    state = ParseState.LENGTH
                    dataIndex = 0
                } else if (b != HEADER_BYTE_1) {
                    state = ParseState.SYNC1
                }
            }
            ParseState.LENGTH -> {
                // X3 sends length as single byte
                packetLength = (b.toInt() and 0xFF)
                if (packetLength > dataBuffer.size) {
                    Log.w(TAG, "Packet too large: $packetLength")
                    state = ParseState.SYNC1
                    return
                }
                state = ParseState.DATA
            }
            ParseState.DATA -> {
                dataBuffer[dataIndex++] = b
                if (dataIndex >= packetLength) {
                    parsePacket()
                    state = ParseState.SYNC1
                }
            }
            ParseState.CHECKSUM -> {
                state = ParseState.SYNC1
            }
        }
    }

    /**
     * Parse complete packet into scan points
     *
     * YDLIDAR X3 packet format:
     * - Header: 0xAA 0x55 (handled)
     * - Length: 1 byte
     * - Sample count: 1 byte
     * - Start angle: 2 bytes (Q15.7 fixed point, little endian)
     * - Points: [distance_low, distance_high, quality, angle_offset] x N
     * - End angle: 2 bytes
     * - Checksum: 1 byte
     */
    private fun parsePacket() {
        try {
            if (dataIndex < 5) return

            val sampleCount = dataBuffer[0].toInt() and 0xFF
            if (sampleCount == 0 || sampleCount > 200) return

            // Start angle (Q15.7 format: angle_rad = value / 64.0)
            val startAngleRaw = (dataBuffer[2].toInt() and 0xFF) or
                    ((dataBuffer[3].toInt() and 0xFF) shl 8)
            val startAngleDeg = startAngleRaw / 64.0

            val points = mutableListOf<LidarScanPoint>()

            var offsetIndex = 4
            for (i in 0 until sampleCount) {
                if (offsetIndex + 3 >= dataIndex) break

                // Distance (little endian, 2 bytes)
                val distLow = dataBuffer[offsetIndex++].toInt() and 0xFF
                val distHigh = dataBuffer[offsetIndex++].toInt() and 0xFF
                val distanceMm = distLow or (distHigh shl 8)

                // Quality (signal strength)
                val quality = dataBuffer[offsetIndex++].toInt() and 0xFF

                // Angle offset from start (1 byte, value / 64.0)
                val angleOffsetRaw = dataBuffer[offsetIndex++].toInt() and 0xFF
                val angleOffsetDeg = angleOffsetRaw / 64.0

                // Calculate absolute angle
                val angleDeg = startAngleDeg + (i * angleOffsetDeg)
                val angleRad = Math.toRadians(angleDeg)

                // Filter invalid points
                if (distanceMm > 100 && distanceMm < 8000 && quality > 10) {
                    points.add(LidarScanPoint(
                        angleRad = angleRad.toFloat(),
                        distanceMm = distanceMm,
                        quality = quality
                    ))
                }
            }

            if (points.isNotEmpty()) {
                val now = System.currentTimeMillis()
                val scan = LidarScan(
                    points = points,
                    timestampMs = now,
                    scanRateHz = if (lastScanTimestamp > 0) {
                        1000f / (now - lastScanTimestamp)
                    } else 10f
                )
                scanQueue.offer(scan)
                lastScanTimestamp = now
                scanCount++
                onScanReady(points)
            }

        } catch (e: Exception) {
            Log.e(TAG, "Parse error: ${e.message}")
        }
    }

    /**
     * Get latest scan from queue (non-blocking)
     */
    fun getLatestScan(): LidarScan? = scanQueue.poll()

    /**
     * Get scan points for SLAM processing
     * Returns polar coordinates suitable for SLAM algorithms
     */
    fun getScanForSLAM(): Array<FloatArray>? {
        val scan = scanQueue.poll() ?: return null

        // Convert to Cartesian coordinates for SLAM
        val slamData = Array(scan.points.size) { i ->
            val point = scan.points[i]
            val x = (point.distanceMm / 1000f) * cos(point.angleRad)
            val y = (point.distanceMm / 1000f) * sin(point.angleRad)
            floatArrayOf(x, y, point.angleRad)
        }

        return slamData
    }

    /**
     * Disconnect and cleanup
     */
    fun disconnect() {
        isRunning = false
        readThread?.interrupt()
        readThread = null

        try {
            stopScan()
            serialPort?.close()
        } catch (e: Exception) {
            Log.e(TAG, "Disconnect error: ${e.message}")
        }
        serialPort = null
        Log.i(TAG, "YDLIDAR X3 disconnected")
    }

    /**
     * Get performance statistics
     */
    fun getStats(): String {
        return buildString {
            appendLine("=== YDLIDAR X3 Statistics ===")
            appendLine("Status: ${if (isRunning) "Running" else "Stopped"}")
            appendLine("Total bytes received: $totalBytesReceived")
            appendLine("Total scans: $scanCount")
            appendLine("Last scan: ${lastScanTimestamp}ms ago")
            appendLine("Queue size: ${scanQueue.size}")
        }
    }
}

/**
 * Simple scan matching for pose correction
 * Uses ICP (Iterative Closest Point) algorithm
 */
class ScanMatcher {
    // Previous scan points
    private var prevPoints = floatArrayOf()

    /**
     * Match current scan against previous scan to estimate motion
     * Returns: [deltaX, deltaY, deltaTheta]
     */
    fun match(currentPoints: List<YDLIDARX3Manager.LidarScanPoint>): FloatArray {
        if (currentPoints.isEmpty()) {
            return floatArrayOf(0f, 0f, 0f)
        }

        // Convert to Cartesian
        val curr = currentPoints.map { p ->
            floatArrayOf(
                (p.distanceMm / 1000f) * cos(p.angleRad),
                (p.distanceMm / 1000f) * sin(p.angleRad)
            )
        }

        // Simple scan matching using closest point association
        // For full ICP, see OpenCV or PCL implementations
        return floatArrayOf(0f, 0f, 0f) // Placeholder
    }
}
