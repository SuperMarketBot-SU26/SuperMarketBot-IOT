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

    override fun onCreate() {
        super.onCreate()
        usbManager = getSystemService(Context.USB_SERVICE) as UsbManager
        createNotificationChannel()
        startForeground(NOTIFICATION_ID, createNotification())
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        logToActivity("[System] Khởi tạo dịch vụ nền UsbSerialService...")
        if (!isRunning) {
            isRunning = true
            discoverAndConnectDevices()
        }
        return START_STICKY
    }

    private fun discoverAndConnectDevices() {
        val availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
        if (availableDrivers.isEmpty()) {
            logToActivity("[USB] Không tìm thấy thiết bị Driver Serial nào.")
            return
        }

        for (driver in availableDrivers) {
            val device = driver.device
            val deviceId = "VID: ${device.vendorId}, PID: ${device.productId}"
            
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
                            // YDLidar X3 thường hoạt động ở tốc độ 115200 bps
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
            while (isRunning && isEspConnected) {
                try {
                    val numBytesRead = espPort?.read(buffer, 100) ?: 0
                    if (numBytesRead > 0) {
                        val receivedString = String(buffer, 0, numBytesRead)
                        // Xử lý gói tin Protobuf/Nhị phân ở đây
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
            while (isRunning && isLidarConnected) {
                try {
                    val numBytesRead = lidarPort?.read(buffer, 100) ?: 0
                    if (numBytesRead > 0) {
                        // Driver phân tích gói tin LiDAR ở đây
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

    override fun onDestroy() {
        isRunning = false
        try {
            espPort?.close()
            lidarPort?.close()
        } catch (e: Exception) {
            // Ignore closing exception
        }
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
            .setContentText("Đang chạy ngầm xử lý giao tiếp USB Serial (LiDAR + ESP32)")
            .setSmallIcon(android.R.drawable.stat_sys_data_bluetooth) // Dùng tạm icon hệ thống
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
