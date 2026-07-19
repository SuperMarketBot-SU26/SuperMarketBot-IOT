package com.smartmarketbot.hub

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.Build
import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.RadioGroup
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
    
    // UI components mới cho WiFi
    private lateinit var radioGroupMode: RadioGroup
    private lateinit var layoutWifiConfig: LinearLayout
    private lateinit var editRobotIp: EditText

    private val timeFormat = SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault())
    private val PREFS_NAME = "SmartMarketBotHubPrefs"
    private val KEY_ROBOT_IP = "KEY_ROBOT_IP"
    private val KEY_CONN_MODE = "KEY_CONN_MODE" // 0: USB, 1: WiFi

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
        
        radioGroupMode = findViewById(R.id.radioGroupMode)
        layoutWifiConfig = findViewById(R.id.layoutWifiConfig)
        editRobotIp = findViewById(R.id.editRobotIp)

        // Phục hồi cấu hình cũ từ SharedPreferences
        val prefs = getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
        val savedIp = prefs.getString(KEY_ROBOT_IP, "192.168.1.100")
        val savedMode = prefs.getInt(KEY_CONN_MODE, 0) // Mặc định là USB (0)
        
        editRobotIp.setText(savedIp)
        if (savedMode == 1) {
            radioGroupMode.check(R.id.radioWifi)
            layoutWifiConfig.visibility = View.VISIBLE
        } else {
            radioGroupMode.check(R.id.radioUsb)
            layoutWifiConfig.visibility = View.GONE
        }

        // Sự kiện chuyển đổi chế độ USB / WiFi
        radioGroupMode.setOnCheckedChangeListener { _, checkedId ->
            if (checkedId == R.id.radioWifi) {
                layoutWifiConfig.visibility = View.VISIBLE
                appendLog("[HMI] Đã chọn chế độ không dây WiFi. Nhập IP của Robot.")
            } else {
                layoutWifiConfig.visibility = View.GONE
                appendLog("[HMI] Đã chọn chế độ có dây USB Serial.")
            }
            // Lưu lựa chọn chế độ
            prefs.edit().putInt(KEY_CONN_MODE, if (checkedId == R.id.radioWifi) 1 else 0).apply()
        }

        // Thiết lập sự kiện Button
        btnStart.setOnClickListener {
            // Lưu IP hiện tại khi bấm Start
            val currentIp = editRobotIp.text.toString().trim()
            prefs.edit().putString(KEY_ROBOT_IP, currentIp).apply()
            
            startUsbService(currentIp, radioGroupMode.checkedRadioButtonId == R.id.radioWifi)
        }

        btnStop.setOnClickListener {
            stopUsbService()
        }

        // Tự động khởi động dịch vụ lần đầu tiên khi mở app
        val currentIp = editRobotIp.text.toString().trim()
        startUsbService(currentIp, radioGroupMode.checkedRadioButtonId == R.id.radioWifi)
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
        unregisterReceiver(serviceReceiver)
    }

    private fun startUsbService(robotIp: String, isWifiMode: Boolean) {
        val intent = Intent(this, UsbSerialService::class.java).apply {
            putExtra("MODE", if (isWifiMode) "WIFI" else "USB")
            putExtra("ROBOT_IP", robotIp)
        }
        
        appendLog("[System] Đang khởi chạy UsbSerialService chế độ: " + (if (isWifiMode) "WiFi ($robotIp)" else "USB Serial"))
        
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(intent)
        } else {
            startService(intent)
        }
        
        txtServiceStatus.text = "SERVICE ACTIVE"
        txtServiceStatus.setTextColor(0xFF00E676.toInt())
        txtServiceStatus.setBackgroundColor(0xFF1B2E1E.toInt())
    }

    private fun stopUsbService() {
        appendLog("[System] Đang dừng UsbSerialService...")
        val intent = Intent(this, UsbSerialService::class.java)
        stopService(intent)
        txtServiceStatus.text = "SERVICE INACTIVE"
        txtServiceStatus.setTextColor(0xFFFF1744.toInt())
        txtServiceStatus.setBackgroundColor(0xFF2E1B1B.toInt())
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
            scrollConsole.post {
                scrollConsole.fullScroll(View.FOCUS_DOWN)
            }
        }
    }
}
