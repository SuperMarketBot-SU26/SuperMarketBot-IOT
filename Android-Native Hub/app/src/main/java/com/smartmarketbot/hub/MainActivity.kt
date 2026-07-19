package com.smartmarketbot.hub

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.Build
import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.ScrollView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class MainActivity : AppCompatActivity() {

    private lateinit var txtServiceStatus: TextView
    private lateinit var txtEspStatus: TextView
    private lateinit var txtLidarStatus: TextView
    private lateinit var txtConsole: TextView
    private lateinit var scrollConsole: ScrollView
    private lateinit var btnStart: Button
    private lateinit var btnStop: Button

    private val timeFormat = SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault())

    // Bộ nhận tin tức từ UsbSerialService gửi lên
    private val serviceReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            when (intent?.action) {
                UsbSerialService.ACTION_LOG -> {
                    val logMsg = intent.getStringExtra(UsbSerialService.EXTRA_LOG_MSG) ?: ""
                    appendLog(logMsg)
                }
                UsbSerialService.ACTION_STATUS_CHANGE -> {
                    val espOk = intent.getBooleanExtra(UsbSerialService.EXTRA_ESP_CONNECTED, false)
                    val lidarOk = intent.getBooleanExtra(UsbSerialService.EXTRA_LIDAR_CONNECTED, false)
                    updateConnectionUI(espOk, lidarOk)
                }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Ánh xạ View
        txtServiceStatus = findViewById(R.id.txtServiceStatus)
        txtEspStatus = findViewById(R.id.txtEspStatus)
        txtLidarStatus = findViewById(R.id.txtLidarStatus)
        txtConsole = findViewById(R.id.txtConsole)
        scrollConsole = findViewById(R.id.scrollConsole)
        btnStart = findViewById(R.id.btnStartService)
        btnStop = findViewById(R.id.btnStopService)

        // Thiết lập sự kiện Button
        btnStart.setOnClickListener {
            startUsbService()
        }

        btnStop.setOnClickListener {
            stopUsbService()
        }

        // Tự động khởi động dịch vụ khi mở app
        startUsbService()
    }

    override fun onResume() {
        super.onResume()
        // Đăng ký bộ nhận Broadcast
        val filter = IntentFilter().apply {
            addAction(UsbSerialService.ACTION_LOG)
            addAction(UsbSerialService.ACTION_STATUS_CHANGE)
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            registerReceiver(serviceReceiver, filter, Context.RECEIVER_NOT_EXPORTED)
        } else {
            registerReceiver(serviceReceiver, filter)
        }
    }

    override fun onPause() {
        super.onPause()
        // Hủy đăng ký để tránh rò rỉ bộ nhớ
        unregisterReceiver(serviceReceiver)
    }

    private fun startUsbService() {
        appendLog("[System] Đang khởi chạy UsbSerialService...")
        val intent = Intent(this, UsbSerialService::class.java)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(intent)
        } else {
            startService(intent)
        }
        txtServiceStatus.text = "SERVICE ACTIVE"
        txtServiceStatus.setTextColor(0xFF00E676.toInt()) // Màu xanh lá cây sáng
        txtServiceStatus.setBackgroundColor(0xFF1B2E1E.toInt()) // Nền xanh tối
    }

    private fun stopUsbService() {
        appendLog("[System] Đang dừng UsbSerialService...")
        val intent = Intent(this, UsbSerialService::class.java)
        stopService(intent)
        txtServiceStatus.text = "SERVICE INACTIVE"
        txtServiceStatus.setTextColor(0xFFFF1744.toInt()) // Màu đỏ sáng
        txtServiceStatus.setBackgroundColor(0xFF2E1B1B.toInt()) // Nền đỏ tối
        updateConnectionUI(false, false)
    }

    private fun updateConnectionUI(espConnected: Boolean, lidarConnected: Boolean) {
        if (espConnected) {
            txtEspStatus.text = "CONNECTED"
            txtEspStatus.setTextColor(0xFF00E676.toInt())
        } else {
            txtEspStatus.text = "DISCONNECTED"
            txtEspStatus.setTextColor(0xFFFF1744.toInt())
        }

        if (lidarConnected) {
            txtLidarStatus.text = "CONNECTED"
            txtLidarStatus.setTextColor(0xFF00E676.toInt())
        } else {
            txtLidarStatus.text = "DISCONNECTED"
            txtLidarStatus.setTextColor(0xFFFF1744.toInt())
        }
    }

    private fun appendLog(msg: String) {
        runOnUiThread {
            val timestamp = timeFormat.format(Date())
            txtConsole.append("[$timestamp] $msg\n")
            
            // Tự động cuộn ScrollView xuống cuối cùng khi có dòng log mới
            scrollConsole.post {
                scrollConsole.fullScroll(View.FOCUS_DOWN)
            }
        }
    }
}
