package com.smartmarketbot.hub.navigation

import android.util.Log
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import org.json.JSONArray
import org.json.JSONObject

/**
 * LineSensorBridge — Phase 9: parse line sensor telemetry từ ESP32 WebSocket.
 *
 * ESP32 gửi mỗi frame (50Hz):
 *   {
 *     "lineR":      [int8 x 8],   // raw ADC 0..4095
 *     "lineOff":    int,          // -100..+100
 *     "lineMask":   int,          // bitmask 8 sensor
 *     "lineStab":   int,          // stable frames
 *     "linePat":    "track"|"node"|"junc"|"lost"|"unknown",
 *     "linePatRaw": int,
 *     "lastNode":   int           // node ID cuối cùng đã pass
 *   }
 *
 * Expose:
 *   - StateFlow<LineSnapshot> — UI realtime
 *   - SharedFlow<NodeEvent>    — phát event khi đi qua node (cho NavigatorCore)
 *   - SharedFlow<LineEvent>    — pattern change events
 */
class LineSensorBridge {

    enum class Pattern { UNKNOWN, LOST, TRACKING, JUNCTION, NODE }

    data class LineSnapshot(
        val raw: IntArray = IntArray(8),
        val offset: Int = 0,
        val activeMask: Int = 0,
        val stableFrames: Int = 0,
        val pattern: Pattern = Pattern.UNKNOWN,
        val lastNodeId: Int = 0,
        val timestampMs: Long = 0L
    ) {
        val activeCount: Int get() = Integer.bitCount(activeMask)
        val isOnTrack: Boolean get() = pattern == Pattern.TRACKING || pattern == Pattern.JUNCTION
        val isAtNode: Boolean get() = pattern == Pattern.NODE
        val isLost: Boolean get() = pattern == Pattern.LOST

        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (other !is LineSnapshot) return false
            return offset == other.offset &&
                activeMask == other.activeMask &&
                stableFrames == other.stableFrames &&
                pattern == other.pattern &&
                lastNodeId == other.lastNodeId &&
                raw.contentEquals(other.raw)
        }
        override fun hashCode(): Int {
            var r = offset
            r = 31 * r + activeMask
            r = 31 * r + stableFrames
            r = 31 * r + pattern.hashCode()
            r = 31 * r + lastNodeId
            return r
        }
    }

    /** Event phát khi robot vừa đi qua 1 node — NavigatorCore sẽ snap pose. */
    data class NodeEvent(val nodeId: Int, val timestampMs: Long)

    /** Event khi pattern thay đổi. */
    data class PatternEvent(val from: Pattern, val to: Pattern, val timestampMs: Long)

    companion object { private const val TAG = "LineBridge" }

    private val _snapshot = MutableStateFlow(LineSnapshot())
    val snapshot: StateFlow<LineSnapshot> = _snapshot.asStateFlow()

    private val _nodeEvents = MutableSharedFlow<NodeEvent>(extraBufferCapacity = 16)
    val nodeEvents: SharedFlow<NodeEvent> = _nodeEvents.asSharedFlow()

    private val _patternEvents = MutableSharedFlow<PatternEvent>(extraBufferCapacity = 16)
    val patternEvents: SharedFlow<PatternEvent> = _patternEvents.asSharedFlow()

    /**
     * Parse 1 frame telemetry (JSON object đã có "lineR" key).
     * Idempotent — gọi mỗi khi nhận WebSocket message.
     */
    fun onTelemetry(t: JSONObject) {
        try {
            // Parse lineR array
            val rawArr = t.optJSONArray("lineR")
            val raw = IntArray(8)
            if (rawArr != null) {
                for (i in 0 until minOf(8, rawArr.length())) {
                    raw[i] = rawArr.optInt(i, 0)
                }
            }
            val offset = t.optInt("lineOff", 0)
            val mask = t.optInt("lineMask", 0)
            val stable = t.optInt("lineStab", 0)
            val patRaw = t.optInt("linePatRaw", 0)
            val pattern = when (patRaw) {
                1 -> Pattern.LOST
                2 -> Pattern.TRACKING
                3 -> Pattern.JUNCTION
                4 -> Pattern.NODE
                else -> Pattern.UNKNOWN
            }
            val nodeId = t.optInt("lastNode", 0)

            // Detect node event: nodeId tăng → robot vừa cross 1 node mới
            val prev = _snapshot.value
            val newSnap = LineSnapshot(raw, offset, mask, stable, pattern, nodeId, System.currentTimeMillis())
            _snapshot.value = newSnap

            if (nodeId > prev.lastNodeId && nodeId > 0) {
                _nodeEvents.tryEmit(NodeEvent(nodeId, newSnap.timestampMs))
            }
            if (pattern != prev.pattern && pattern != Pattern.UNKNOWN) {
                _patternEvents.tryEmit(PatternEvent(prev.pattern, pattern, newSnap.timestampMs))
            }
        } catch (e: Exception) {
            Log.w(TAG, "parse error: ${e.message}")
        }
    }

    /** Reset khi map reset / connection restart. */
    fun reset() {
        _snapshot.value = LineSnapshot()
    }

    /** True khi line sensor đang tracking đúng line (≥2 sensor active). */
    fun isHealthy(): Boolean {
        val s = _snapshot.value
        return s.activeCount >= 2 && s.pattern != Pattern.LOST
    }
}
