package com.smartmarketbot.hub

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.util.Log
import androidx.core.app.NotificationCompat
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import java.io.IOException
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener

class UsbSerialService : Service() {

    companion object {
        const val TAG = "UsbSerialService"
        const val CHANNEL_ID = "SmartMarketBotHubChannel"
        const val NOTIFICATION_ID = 2026
        
        // Broadcast Actions
        const val ACTION_LOG = "com.smartmarketbot.hub.ACTION_LOG"
        const val ACTION_STATUS_CHANGE = "com.smartmarketbot.hub.ACTION_STATUS_CHANGE"
        
        // Broadcast Extras
        const val EXTRA_LOG_MSG = "EXTRA_LOG_MSG"
        const val EXTRA_ESP_CONNECTED = "EXTRA_ESP_CONNECTED"
        const val EXTRA_LIDAR_CONNECTED = "EXTRA_LIDAR_CONNECTED"
        
        // USB VIDs and PIDs
        const val ESP32_VID = 12565 // 0x303A
        const val ESP32_PID = 4102  // 0x1001 (Native JTAG/Serial)
        const val CP210X_VID = 4292 // 0x10C4 (YDLidar X3 default)
    }

    private var usbManager: UsbManager? = null
    private var espPort: UsbSerialPort? = null
    private var lidarPort: UsbSerialPort? = null
    
    private var isEspConnected = false
    private var isLidarConnected = false
    
    private val executorService: ExecutorService = Executors.newFixedThreadPool(2)
    private var isRunning = false

    // WiFi WebSocket properties
    private var currentMode = "USB" // "USB" hoặc "WIFI"
    private var robotIp = "192.168.1.100"
    private var okHttpClient: OkHttpClient? = null
    private var webSocket: WebSocket? = null
    private val mainHandler = Handler(Looper.getMainLooper())
    private val reconnectRunnable = Runnable { connectWebSocket() }

    override fun onCreate() {
        super.onCreate()
        usbManager = getSystemService(Context.USB_SERVICE) as UsbManager
        createNotificationChannel()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            startForeground(NOTIFICATION_ID, createNotification(), android.content.pm.ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE)
        } else {
            startForeground(NOTIFICATION_ID, createNotification())
        }
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val newMode = intent?.getStringExtra("MODE") ?: "USB"
        val newIp = intent?.getStringExtra("ROBOT_IP") ?: "192.168.1.100"

        logToActivity("[System] Khởi chạy dịch vụ nền. Chế độ: $newMode")

        // Nếu gọi từ nút nhấn của App (intent != null) hoặc chưa chạy, ép buộc kết nối lại
        if (intent != null || !isRunning) {
            disconnectAll()
            currentMode = newMode
            robotIp = newIp
            isRunning = true

            if (currentMode == "WIFI") {
                okHttpClient = OkHttpClient.Builder().build()
                connectWebSocket()
            } else {
                discoverAndConnectDevices()
            }
        }
        return START_STICKY
    }

    // --- KẾT NỐI KHÔNG DÂY WIFI (WEBSOCKET CLIENT) ---
    private fun connectWebSocket() {
        if (!isRunning || currentMode != "WIFI") return

        val wsUrl = "ws://$robotIp:81"
        logToActivity("[WiFi] Đang mở kết nối WebSocket tới: $wsUrl ...")
        
        val request = Request.Builder().url(wsUrl).build()
        webSocket = okHttpClient?.newWebSocket(request, object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                isEspConnected = true
                isLidarConnected = false // LiDAR không hỗ trợ truyền qua WiFi trong firmware hiện tại
                notifyStatusChanged()
                logToActivity("[WiFi] Đã kết nối thành công tới ESP32 Robot qua WebSocket!")
                mainHandler.removeCallbacks(reconnectRunnable)
            }

            override fun onMessage(webSocket: WebSocket, text: String) {
                // Log dữ liệu telemetry nhận được từ ESP32
                Log.d(TAG, "WS Recv: $text")
                logToActivity("[ESP32 Recv] $text")
            }

            override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
                logToActivity("[WiFi] Cảnh báo: Robot đang ngắt kết nối WebSocket...")
            }

            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                isEspConnected = false
                notifyStatusChanged()
                logToActivity("[WiFi] Đã đóng kết nối WebSocket.")
                scheduleReconnect()
            }

            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                isEspConnected = false
                notifyStatusChanged()
                logToActivity("[WiFi ERROR] Kết nối thất bại: ${t.message}")
                scheduleReconnect()
            }
        })
    }

    private fun scheduleReconnect() {
        if (!isRunning || currentMode != "WIFI") return
        logToActivity("[WiFi] Sẽ tự động kết nối lại sau 3 giây...")
        mainHandler.removeCallbacks(reconnectRunnable)
        mainHandler.postDelayed(reconnectRunnable, 3000)
    }

    // --- KẾT NỐI CÓ DÂY USB SERIAL ---
    private fun discoverAndConnectDevices() {
        val availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
        if (availableDrivers.isEmpty()) {
            logToActivity("[USB] Không tìm thấy thiết bị phần cứng USB Serial nào.")
            logToActivity("[Simulator] Tự động kích hoạt chế độ GIẢ LẬP DỮ LIỆU để chạy thử...")
            startMockDataSimulation()
            return
        }

        for (driver in availableDrivers) {
            val device = driver.device
            
            // 1. ESP32-S3 Board Detection
            if (device.vendorId == ESP32_VID || (device.vendorId == CP210X_VID && !isLidarConnected)) {
                if (!isEspConnected) {
                    try {
                        val connection = usbManager!!.openDevice(device)
                        if (connection != null) {
                            espPort = driver.ports[0]
                            espPort!!.open(connection)
                            espPort!!.setParameters(115200, 8, UsbSerialPort.DATABITS_8, UsbSerialPort.PARITY_NONE)
                            isEspConnected = true
                            logToActivity("[ESP32] Kết nối thành công Serial-over-USB ở baudrate 115200.")
                            startEspReadLoop()
                        } else {
                            logToActivity("[ESP32 ERROR] Không thể xin quyền/mở thiết bị USB.")
                        }
                    } catch (e: Exception) {
                        logToActivity("[ESP32 ERROR] Lỗi mở cổng: ${e.message}")
                    }
                }
            } 
            // 2. YDLidar X3 Detection
            else if (device.vendorId == CP210X_VID) {
                if (!isLidarConnected) {
                    try {
                        val connection = usbManager!!.openDevice(device)
                        if (connection != null) {
                            lidarPort = driver.ports[0]
                            lidarPort!!.open(connection)
                            lidarPort!!.setParameters(115200, 8, UsbSerialPort.DATABITS_8, UsbSerialPort.PARITY_NONE)
                            isLidarConnected = true
                            logToActivity("[LiDAR] Kết nối thành công YDLIDAR X3 ở baudrate 115200.")
                            startLidarReadLoop()
                        } else {
                            logToActivity("[LiDAR ERROR] Không thể mở thiết bị USB.")
                        }
                    } catch (e: Exception) {
                        logToActivity("[LiDAR ERROR] Lỗi mở cổng: ${e.message}")
                    }
                }
            }
        }
        notifyStatusChanged()
    }

    private fun startEspReadLoop() {
        executorService.submit {
            val buffer = ByteArray(1024)
            logToActivity("[ESP32] Bắt đầu luồng đọc dữ liệu từ Robot Motor Controller...")
            while (isRunning && isEspConnected && currentMode == "USB") {
                try {
                    val numBytesRead = espPort?.read(buffer, 100) ?: 0
                    if (numBytesRead > 0) {
                        val receivedString = String(buffer, 0, numBytesRead)
                        Log.d(TAG, "ESP32 Recv: $receivedString")
                    }
                } catch (e: IOException) {
                    logToActivity("[ESP32] Mất kết nối Serial USB.")
                    isEspConnected = false
                    notifyStatusChanged()
                    break
                }
            }
        }
    }

    private fun startLidarReadLoop() {
        executorService.submit {
            val buffer = ByteArray(2048)
            logToActivity("[LiDAR] Bắt đầu luồng phân tích dữ liệu quét mây điểm (Scan)...")
            while (isRunning && isLidarConnected && currentMode == "USB") {
                try {
                    val numBytesRead = lidarPort?.read(buffer, 100) ?: 0
                    if (numBytesRead > 0) {
                        Log.d(TAG, "Lidar Recv bytes: $numBytesRead")
                    }
                } catch (e: IOException) {
                    logToActivity("[LiDAR] Mất kết nối LiDAR USB.")
                    isLidarConnected = false
                    notifyStatusChanged()
                    break
                }
            }
        }
    }

    // Luồng sinh dữ liệu giả lập
    private fun startMockDataSimulation() {
        isEspConnected = true
        isLidarConnected = true
        notifyStatusChanged()
        
        executorService.submit {
            logToActivity("[ESP32-Sim] Đã kết nối giả lập. Đang gửi dữ liệu phản hồi động cơ...")
            var seq = 0
            while (isRunning && isEspConnected && currentMode == "USB") {
                try {
                    val mockMsg = "{\"t\":\"telemetry\",\"x\":${String.format("%.2f", seq * 0.15)},\"y\":${String.format("%.2f", seq * 0.08)},\"yaw\":${String.format("%.2f", (seq % 360) * 0.017)},\"bat\":95}"
                    logToActivity("[ESP32 Recv] $mockMsg")
                    seq++
                    Thread.sleep(1000)
                } catch (e: InterruptedException) {
                    break
                }
            }
        }

        executorService.submit {
            logToActivity("[LiDAR-Sim] Đã kết nối giả lập. Đang gửi dữ liệu mây điểm...")
            var deg = 0
            while (isRunning && isLidarConnected && currentMode == "USB") {
                try {
                    val mockScan = "[LiDAR Scan] Angle: $deg°, Dist: ${120 + (deg % 50)}cm (Clearance: OK)"
                    logToActivity("[LiDAR Recv] $mockScan")
                    deg = (deg + 45) % 360
                    Thread.sleep(1500)
                } catch (e: InterruptedException) {
                    break
                }
            }
        }
    }

    private fun logToActivity(msg: String) {
        val intent = Intent(ACTION_LOG).apply {
            putExtra(EXTRA_LOG_MSG, msg)
        }
        sendBroadcast(intent)
    }

    private fun notifyStatusChanged() {
        val intent = Intent(ACTION_STATUS_CHANGE).apply {
            putExtra(EXTRA_ESP_CONNECTED, isEspConnected)
            putExtra(EXTRA_LIDAR_CONNECTED, isLidarConnected)
        }
        sendBroadcast(intent)
    }

    private fun disconnectAll() {
        // Hủy luồng WebSocket
        mainHandler.removeCallbacks(reconnectRunnable)
        webSocket?.close(1000, "Service mode changed")
        webSocket = null
        okHttpClient = null

        // Hủy cổng USB
        try {
            espPort?.close()
            lidarPort?.close()
        } catch (e: Exception) {
            // Ignore
        }
        espPort = null
        lidarPort = null
        
        isEspConnected = false
        isLidarConnected = false
        notifyStatusChanged()
    }

    override fun onDestroy() {
        isRunning = false
        disconnectAll()
        executorService.shutdownNow()
        logToActivity("[System] Dịch vụ UsbSerialService đã dừng.")
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null

    private fun createNotification(): Notification {
        val pendingIntent: PendingIntent = Intent(this, MainActivity::class.java).let { notificationIntent ->
            PendingIntent.getActivity(this, 0, notificationIntent, PendingIntent.FLAG_IMMUTABLE)
        }

        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("SmartMarketBot Hub")
            .setContentText("Đang chạy ngầm xử lý giao tiếp USB Serial / WiFi WebSocket")
            .setSmallIcon(android.R.drawable.ic_menu_info_details)
            .setContentIntent(pendingIntent)
            .build()
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val serviceChannel = NotificationChannel(
                CHANNEL_ID,
                "SmartMarketBot Hub Control Service",
                NotificationManager.IMPORTANCE_LOW
            )
            val manager = getSystemService(NotificationManager::class.java)
            manager.createNotificationChannel(serviceChannel)
        }
    }
}
