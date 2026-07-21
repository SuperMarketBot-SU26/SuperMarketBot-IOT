package com.smartmarketbot.hub.navigation

import android.os.Handler
import android.os.Looper
import android.util.Log
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import java.util.concurrent.TimeUnit

/**
 * MotorLink — Phase 8: WebSocket JSON bridge tới ESP32.
 *
 * ESP32 firmware dùng WebSocket Server (port 81, library [WebSocketsServer]).
 * Protocol JSON (xem [CtrlJson.h] / [Config.h] của ESP32):
 *   - `{t:"joy", x:turn, y:fwd, s:strafe}`   — joystick/manual drive
 *   - `{t:"spd", v:pct}`                    — baseSpeed percent (0..100)
 *   - `{t:"mode", m:N}`                     — chuyển mode (0=manual, 1=auto, 2=waypoint)
 *   - `{t:"estop"}`                          — emergency stop
 *   - `{t:"waypoint", x, y}`                 — goal target (Android → ESP32, optional)
 *
 * Drive convention cho 4WD differential (bánh thường):
 *   - `y` (forward): ±100  (âm = lùi, dương = tiến)
 *   - `x` (turn):    ±100  (âm = CCW, dương = CW)
 *   - `s` (strafe):  ±100  — KHÔNG dùng cho 4WD thường, luôn = 0
 *
 * 4WD differential → ESP32 gọi [botDrive(turn, fwd, base)]:
 *   left  = fwd + turn
 *   right = fwd - turn
 *
 * Đơn vị Android gửi: m/s, rad/s. Conversion ở đây:
 *   - Pure Pursuit ra (v, ω) m/s, rad/s
 *   - Linear m/s → % PWM (0..100) của maxSpeed
 *   - Angular rad/s → % turn dựa trên maxYawRate (rad/s)
 */
class MotorLink(
    private val esp32Host: String = "192.168.4.1",
    private val esp32WsPort: Int = 81,
    private val maxLinearMps: Float = 0.5f,   // m/s max ứng với 100% PWM
    private val maxAngularRps: Float = 2.0f    // rad/s max ứng với 100% turn
) {
    companion object {
        private const val TAG = "MotorLink"
    }

    @Volatile private var connected = false
    @Volatile private var webSocket: WebSocket? = null
    private val client: OkHttpClient = OkHttpClient.Builder()
        .pingInterval(20, TimeUnit.SECONDS)
        .readTimeout(0, TimeUnit.MILLISECONDS)  // WebSocket forever
        .build()

    /** Subscribers nhận telemetry frames (parsed JSON). Thread-safe. */
    private val telemetryListeners = java.util.concurrent.CopyOnWriteArrayList<(org.json.JSONObject) -> Unit>()

    /** Add subscriber. */
    fun addTelemetryListener(l: (org.json.JSONObject) -> Unit) {
        telemetryListeners.add(l)
    }

    /** Remove subscriber. */
    fun removeTelemetryListener(l: (org.json.JSONObject) -> Unit) {
        telemetryListeners.remove(l)
    }

    private val mainHandler = Handler(Looper.getMainLooper())

    /** Mở WebSocket connection. Idempotent. */
    @Synchronized
    fun open() {
        if (connected) return
        val url = "ws://$esp32Host:$esp32WsPort/"
        val request = Request.Builder().url(url).build()
        webSocket = client.newWebSocket(request, listener)
        Log.i(TAG, "MotorLink connecting → $url")
    }

    @Synchronized
    fun close() {
        webSocket?.close(1000, "shutdown")
        webSocket = null
        connected = false
    }

    /** Gửi raw JSON message. */
    fun sendJson(json: String): Boolean {
        val ws = webSocket ?: return false
        if (!connected) return false
        return ws.send(json)
    }

    /**
     * Gửi differential drive command (4WD thường).
     *
     * @param linearMps forward velocity (m/s) — dương = tiến
     * @param angularRps angular velocity (rad/s) — dương = CW
     * @return true nếu gửi thành công
     */
    fun sendVelocity(linearMps: Float, angularRps: Float): Boolean {
        // Map m/s → % PWM (0..100)
        val fwdPct = ((linearMps / maxLinearMps) * 100f).coerceIn(-100f, 100f).toInt()
        // Map rad/s → % turn (0..100)
        val turnPct = ((angularRps / maxAngularRps) * 100f).coerceIn(-100f, 100f).toInt()

        val msg = """{"t":"joy","x":$turnPct,"y":$fwdPct,"s":0}"""
        return sendJson(msg)
    }

    /** Emergency stop — gửi joystick (0,0,0) + estop flag. */
    fun stop(): Boolean {
        sendJson("""{"t":"joy","x":0,"y":0,"s":0}""")
        return sendJson("""{"t":"estop"}""")
    }

    /**
     * Set baseSpeed percent (ESP32 → PWM_MAX).
     */
    fun setSpeed(pct: Int): Boolean {
        val p = pct.coerceIn(0, 100)
        return sendJson("""{"t":"spd","v":$p}""")
    }

    /**
     * Chuyển ESP32 mode.
     * @param mode 0 = MANUAL (Android điều khiển), 1 = AUTO, 2 = WAYPOINT
     */
    fun setMode(mode: Int): Boolean {
        val m = mode.coerceIn(0, 2)
        return sendJson("""{"t":"mode","m":$m}""")
    }

    /**
     * Gửi waypoint goal cho ESP32 nếu muốn ESP32 tự navigate (mode 2).
     * @param x target X (m)
     * @param y target Y (m)
     */
    fun sendWaypoint(x: Float, y: Float): Boolean {
        return sendJson("""{"t":"waypoint","x":$x,"y":$y}""")
    }

    fun isConnected(): Boolean = connected

    // ----- Listener -----
    private val listener = object : WebSocketListener() {
        override fun onOpen(webSocket: WebSocket, response: Response) {
            connected = true
            Log.i(TAG, "WebSocket OPEN to ESP32")
            // Khi connect, đảm bảo mode = MANUAL (0) để nhận joy cmd
            sendJson("""{"t":"mode","m":0}""")
        }

        override fun onMessage(webSocket: WebSocket, text: String) {
            // Telemetry frames from ESP32 (50Hz). Parse + dispatch to subscribers.
            // Telemetry KHÔNG có `t` key (only `t:"joy"`, `t:"estop"` for inbound cmd).
            try {
                if (text.contains("\"t\"")) return  // ignore echo of our own cmd
                val obj = org.json.JSONObject(text)
                telemetryListeners.forEach { it(obj) }
            } catch (e: Exception) {
                // Truncated buffer / non-JSON → ignore
            }
        }

        override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
            connected = false
            Log.w(TAG, "WebSocket closing: $code $reason")
            webSocket.close(1000, null)
        }

        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
            connected = false
            Log.e(TAG, "WebSocket failure: ${t.message}")
        }
    }
}