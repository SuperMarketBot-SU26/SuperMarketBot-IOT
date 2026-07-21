package com.smartmarketbot.hub

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.hardware.usb.UsbManager
import android.os.Build
import android.util.Log
import com.smartmarketbot.hub.slam.SLAMService

/**
 * BootReceiver - Tự động kích hoạt Dịch vụ Robot & LiDAR SLAM ngay khi cắm nguồn hoặc khởi động Android
 */
class BootReceiver : BroadcastReceiver() {

    companion object {
        private const val TAG = "BootReceiver"
    }

    override fun onReceive(context: Context, intent: Intent) {
        val action = intent.action
        Log.i(TAG, "Nhận tín hiệu sự kiện hệ thống: $action")

        if (Intent.ACTION_BOOT_COMPLETED == action ||
            UsbManager.ACTION_USB_DEVICE_ATTACHED == action ||
            "android.intent.action.QUICKBOOT_POWERON" == action
        ) {
            Log.i(TAG, "Tự động kích hoạt Dịch vụ ngầm Robot & LiDAR SLAM...")
            try {
                // 1. Start UsbSerialService
                val usbIntent = Intent(context, UsbSerialService::class.java).apply {
                    putExtra("MODE", "USB")
                }
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                    context.startForegroundService(usbIntent)
                } else {
                    context.startService(usbIntent)
                }

                // 2. Start SLAMService (Tự kết nối YDLIDAR X3 & Quét mây điểm)
                val slamIntent = Intent(context, SLAMService::class.java)
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                    context.startForegroundService(slamIntent)
                } else {
                    context.startService(slamIntent)
                }

                // 3. Tự động mở MainActivity nếu là sự kiện Boot hoặc cắm USB
                val mainIntent = Intent(context, MainActivity::class.java).apply {
                    addFlags(Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_SINGLE_TOP)
                }
                context.startActivity(mainIntent)

            } catch (e: Exception) {
                Log.e(TAG, "Lỗi khi tự động chạy dịch vụ ngầm: ${e.message}")
            }
        }
    }
}
