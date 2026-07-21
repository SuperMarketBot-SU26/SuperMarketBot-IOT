package com.smartmarketbot.hub

import android.content.BroadcastReceiver
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.ServiceConnection
import android.os.Build
import android.os.Bundle
import android.os.IBinder
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.RadioGroup
import android.widget.ScrollView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.smartmarketbot.hub.navigation.WaypointNavigatorService
import com.smartmarketbot.hub.slam.MapPersistence
import com.smartmarketbot.hub.slam.SLAMEngine
import com.smartmarketbot.hub.slam.SLAMService
import com.smartmarketbot.hub.slam.ui.SLAMMapView
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class MainActivity : AppCompatActivity() {

    private lateinit var txtServiceStatus: TextView
    private lateinit var txtEspStatus: TextView
    private lateinit var txtLidarStatus: TextView
    private lateinit var txtSlamStatus: TextView
    private lateinit var txtConsole: TextView
    private lateinit var scrollConsole: ScrollView
    private lateinit var btnStart: Button
    private lateinit var btnStop: Button

    // SLAM UI components
    private var slamMapView: SLAMMapView? = null
    private var layoutSlamControls: LinearLayout? = null
    private var btnStartSlam: Button? = null
    private var btnStopSlam: Button? = null
    private var btnResetMap: Button? = null
    private var btnSaveMap: Button? = null
    private var btnLoadMap: Button? = null
    private var btnNavigate: Button? = null
    private var btnEStop: Button? = null
    private var editNavX: EditText? = null
    private var editNavY: EditText? = null

    // UI components mới cho WiFi
    private lateinit var radioGroupMode: RadioGroup
    private lateinit var layoutWifiConfig: LinearLayout
    private lateinit var editRobotIp: EditText

    private val timeFormat = SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault())
    private val PREFS_NAME = "SmartMarketBotHubPrefs"
    private val KEY_ROBOT_IP = "KEY_ROBOT_IP"
    private val KEY_CONN_MODE = "KEY_CONN_MODE" // 0: USB, 1: WiFi

    // SLAM Service binding
    private var slamService: SLAMService? = null
    private var slamServiceBound = false

    private val slamServiceConnection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, service: IBinder?) {
            val binder = service as SLAMService.SLAMServiceBinder
            slamService = binder.getService()
            slamServiceBound = true
            appendLog("[SLAM] Service connected. Opening LiDAR USB...")
            slamService?.connectLidar()
            updateSlamUI()
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            slamService = null
            slamServiceBound = false
            appendLog("[SLAM] Service disconnected")
        }
    }

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
                // SLAM service broadcasts
                SLAMService.ACTION_LIDAR_CONNECTED -> {
                    txtSlamStatus.text = "LiDAR: CONNECTED"
                    txtSlamStatus.setTextColor(0xFF00E676.toInt())
                    txtLidarStatus.text = "CONNECTED"
                    txtLidarStatus.setTextColor(0xFF00E676.toInt())
                    appendLog("[SLAM] LiDAR connected successfully")
                }
                SLAMService.ACTION_LIDAR_DISCONNECTED -> {
                    txtSlamStatus.text = "LiDAR: DISCONNECTED"
                    txtSlamStatus.setTextColor(0xFFFF1744.toInt())
                    txtLidarStatus.text = "DISCONNECTED"
                    txtLidarStatus.setTextColor(0xFFFF1744.toInt())
                }
                SLAMService.ACTION_SCAN_READY -> {
                    val engine = slamService?.getSLAMEngine()
                    if (engine != null && slamMapView != null) {
                        slamMapView?.updateGrid(engine.getRawGrid())
                        slamMapView?.updateScanPoints(engine.getLatestScanPoints())
                    }
                }
                SLAMService.ACTION_LIDAR_ERROR -> {
                    val error = intent.getStringExtra(SLAMService.EXTRA_ERROR_MSG) ?: "Unknown"
                    txtSlamStatus.text = "LiDAR: ERROR"
                    txtSlamStatus.setTextColor(0xFFFF1744.toInt())
                    appendLog("[SLAM ERROR] $error")
                }
                SLAMService.ACTION_POSE_UPDATE -> {
                    val x = intent.getFloatExtra(SLAMService.EXTRA_POSE_X, 0f)
                    val y = intent.getFloatExtra(SLAMService.EXTRA_POSE_Y, 0f)
                    val theta = intent.getFloatExtra(SLAMService.EXTRA_POSE_THETA, 0f)
                    updateSLAMPose(x, y, theta)
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

        // SLAM UI components (optional, may not exist in basic layout)
        try {
            slamMapView = findViewById(R.id.slamMapView)
            layoutSlamControls = findViewById(R.id.layoutSlamControls)
            btnStartSlam = findViewById(R.id.btnStartSlam)
            btnStopSlam = findViewById(R.id.btnStopSlam)
            btnResetMap = findViewById(R.id.btnResetMap)
            btnSaveMap = findViewById(R.id.btnSaveMap)
            btnLoadMap = findViewById(R.id.btnLoadMap)
            txtSlamStatus = findViewById(R.id.txtSlamStatus)
            btnNavigate = findViewById(R.id.btnNavigate)
            btnEStop = findViewById(R.id.btnEStop)
            editNavX = findViewById(R.id.editNavX)
            editNavY = findViewById(R.id.editNavY)
            setupSLAMControls()
        } catch (e: Exception) {
            // SLAM views not available in basic layout
            appendLog("[SLAM] SLAM UI not available in current layout")
        }

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
            startSLAMService()
        }

        btnStop.setOnClickListener {
            stopUsbService()
            stopSLAMService()
        }

        // Tự động khởi động dịch vụ lần đầu tiên khi mở app
        val currentIp = editRobotIp.text.toString().trim()
        startUsbService(currentIp, radioGroupMode.checkedRadioButtonId == R.id.radioWifi)
        startSLAMService()
    }

    /**
     * Setup SLAM control buttons
     */
    private fun setupSLAMControls() {
        try {
            btnStartSlam?.setOnClickListener {
                startSLAMService()
                slamService?.connectLidar()
            }

            btnStopSlam?.setOnClickListener {
                stopSLAMService()
            }

            btnResetMap?.setOnClickListener {
                slamService?.resetMap()
                slamMapView?.clear()
                appendLog("[SLAM] Map reset")
            }

            // [Phase 8] Save/Load map
            btnSaveMap?.setOnClickListener {
                saveMap()
            }

            btnLoadMap?.setOnClickListener {
                loadMap()
            }

            // [Phase 7] Navigation controls
            btnNavigate?.setOnClickListener {
                startNavigation()
            }

            btnEStop?.setOnClickListener {
                emergencyStop()
            }

            // [Phase 9] Line sensor telemetry → LineSensorBridge
            val navigator = WaypointNavigatorService.getInstance()?.navigator
            if (navigator != null) {
                WaypointNavigatorService.getInstance()?.motorLink?.let { ml ->
                    ml.addTelemetryListener { t ->
                        // Parse line data
                        navigator.lineBridge.onTelemetry(t)
                        // Auto-register node nếu thiếu
                        navigator.syncBridgeToSLAMPose()
                    }
                }

                // Subscribe node events để snap pose
                val coroutineScope = kotlinx.coroutines.CoroutineScope(
                    kotlinx.coroutines.Dispatchers.Main + kotlinx.coroutines.Job())
                coroutineScope.launch {
                    navigator.lineBridge.nodeEvents.collect { ev ->
                        navigator.onNodeCrossed(ev.nodeId)
                        appendLog("[NODE] Crossed node #${ev.nodeId}")
                        slamMapView?.invalidate()
                    }
                }

                // [Phase 9] Initial render of nodes when map loaded
                coroutineScope.launch {
                    navigator.lineBridge.snapshot.collect {
                        // refresh node list trên UI mỗi frame change
                        val list = navigator.nodeRegistry.all().map { n ->
                            SLAMMapView.NodeRenderData(n.id, n.x, n.y, n.label)
                        }
                        slamMapView?.setNodes(list)
                    }
                }
            }

            // Set initial status
            txtSlamStatus.text = "SLAM: IDLE"

        } catch (e: Exception) {
            // SLAM views not available
            appendLog("[SLAM] SLAM controls not available")
        }
    }

    /**
     * Start navigation to (x, y) entered in UI.
     * Gửi intent broadcast tới WaypointNavigatorService.
     */
    private fun startNavigation() {
        val x = editNavX?.text?.toString()?.toFloatOrNull()
        val y = editNavY?.text?.toString()?.toFloatOrNull()
        if (x == null || y == null) {
            appendLog("[NAV] Invalid x/y")
            return
        }

        // Khởi động WaypointNavigatorService nếu chưa có
        val navIntent = Intent(this, WaypointNavigatorService::class.java).apply {
            action = WaypointNavigatorService.ACTION_NAVIGATE_TO
            putExtra(WaypointNavigatorService.EXTRA_TARGET_X, x)
            putExtra(WaypointNavigatorService.EXTRA_TARGET_Y, y)
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(navIntent)
        } else {
            startService(navIntent)
        }

        appendLog("[NAV] Navigate to ($x, $y)")
        txtSlamStatus.text = "NAV: ${x.toInt()},${y.toInt()}"
    }

    /**
     * Emergency stop — gửi broadcast cho WaypointNavigatorService và motorLink.
     */
    private fun emergencyStop() {
        val stopIntent = Intent(this, WaypointNavigatorService::class.java).apply {
            action = WaypointNavigatorService.ACTION_EMERGENCY_STOP
        }
        startService(stopIntent)
        appendLog("[NAV] EMERGENCY STOP")
        txtSlamStatus.text = "SLAM: ESTOP"
    }

    /**
     * [Phase 8] Save current SLAM map tới internal storage.
     */
    private fun saveMap() {
        val engine = slamService?.getSLAMEngine() ?: run {
            appendLog("[SAVE] SLAM engine not available")
            return
        }
        val persistence = MapPersistence(this)
        val navigator = WaypointNavigatorService.getInstance()?.navigator
        val path = persistence.saveMap(engine, "default", navigator?.nodeRegistry)
        appendLog(if (path != null) "[SAVE] Map → $path" else "[SAVE] Failed")
        txtSlamStatus.text = if (path != null) "SAVED" else "SAVE FAIL"
    }

    /**
     * [Phase 8] Load SLAM map mới nhất từ internal storage.
     */
    private fun loadMap() {
        val engine = slamService?.getSLAMEngine() ?: run {
            appendLog("[LOAD] SLAM engine not available")
            return
        }
        val persistence = MapPersistence(this)
        val maps = persistence.listMaps()
        if (maps.isEmpty()) {
            appendLog("[LOAD] No saved maps found")
            txtSlamStatus.text = "NO MAP"
            return
        }
        val latest = maps.last()  // sorted alphabetically, last = newest
        val navigator = WaypointNavigatorService.getInstance()?.navigator
        val ok = persistence.loadMap(engine, latest, navigator?.nodeRegistry)
        appendLog(if (ok) "[LOAD] Map ← $latest" else "[LOAD] Failed")
        txtSlamStatus.text = if (ok) "LOADED" else "LOAD FAIL"

        // Refresh SLAMMapView
        slamMapView?.updateGrid(engine.getRawGrid())
        val nodes = navigator?.nodeRegistry?.all() ?: emptyList()
        slamMapView?.setNodes(nodes.map { SLAMMapView.NodeRenderData(it.id, it.x, it.y, it.label) })
    }

    /**
     * Start SLAM service and connect to LiDAR
     */
    private fun startSLAMService() {
        try {
            val intent = Intent(this, SLAMService::class.java)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                startForegroundService(intent)
            } else {
                startService(intent)
            }
            
            // Bind to service
            bindService(intent, slamServiceConnection, Context.BIND_AUTO_CREATE)
            
            appendLog("[SLAM] Starting SLAM service...")
            txtSlamStatus.text = "SLAM: STARTING"
            
        } catch (e: Exception) {
            appendLog("[SLAM ERROR] ${e.message}")
        }
    }

    /**
     * Stop SLAM service
     */
    private fun stopSLAMService() {
        try {
            slamService?.disconnectLidar()
            if (slamServiceBound) {
                unbindService(slamServiceConnection)
                slamServiceBound = false
            }
            stopService(Intent(this, SLAMService::class.java))
            
            appendLog("[SLAM] SLAM service stopped")
            txtSlamStatus.text = "SLAM: IDLE"
            slamMapView?.clear()
            
        } catch (e: Exception) {
            appendLog("[SLAM ERROR] ${e.message}")
        }
    }

    /**
     * Update SLAM pose on map
     */
    private fun updateSLAMPose(x: Float, y: Float, theta: Float) {
        try {
            val pose = SLAMEngine.Pose().apply {
                this.x = x
                this.y = y
                this.theta = theta
            }
            slamMapView?.updatePose(pose)
            
        } catch (e: Exception) {
            // View not available
        }
    }

    /**
     * Update SLAM UI state
     */
    private fun updateSlamUI() {
        val connected = slamService?.isLidarConnected() ?: false
        if (connected) {
            txtSlamStatus.text = "LiDAR: CONNECTED"
            txtSlamStatus.setTextColor(0xFF00E676.toInt())
            btnStartSlam?.isEnabled = false
            btnStopSlam?.isEnabled = true
        } else {
            txtSlamStatus.text = "LiDAR: DISCONNECTED"
            txtSlamStatus.setTextColor(0xFFFF1744.toInt())
            btnStartSlam?.isEnabled = true
            btnStopSlam?.isEnabled = false
        }
    }

    override fun onResume() {
        super.onResume()
        // Đăng ký bộ nhận Broadcast
        val filter = IntentFilter().apply {
            addAction(UsbSerialService.ACTION_LOG)
            addAction(UsbSerialService.ACTION_STATUS_CHANGE)
            addAction(SLAMService.ACTION_LIDAR_CONNECTED)
            addAction(SLAMService.ACTION_LIDAR_DISCONNECTED)
            addAction(SLAMService.ACTION_LIDAR_ERROR)
            addAction(SLAMService.ACTION_POSE_UPDATE)
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

    override fun onDestroy() {
        super.onDestroy()
        if (slamServiceBound) {
            unbindService(slamServiceConnection)
            slamServiceBound = false
        }
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
        txtServiceStatus.setBackgroundColor(0xFF2E1B1E.toInt())
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
