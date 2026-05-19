# SmartMarketBot — ESP32-CAM (vision node)

Module firmware **độc lập** với robot ESP32-S3. Chỉ làm:

- Kết nối WiFi **STA** (không tạo SoftAP)
- Stream MJPEG, chụp JPEG, upload ảnh lên backend ASP.NET
- **Không** chạy AI / object detection trên chip

```
Router / Hotspot chung
├── ESP32-S3   → motor, LiDAR, navigation (folder ../ESP32-S3)
├── ESP32-CAM  → camera (folder này)
├── Tablet HMI
└── ASP.NET Backend
```

---

## Phần cứng

- **Board:** AI-Thinker ESP32-CAM + shield USB (như ảnh dự án)
- **Nguồn:** 5 V ổn định ≥ 500 mA, **GND chung** với robot
- Tránh cấp nguồn yếu / chung rail nhiễu với motor khi quay — nên buck riêng hoặc LC filter cho CAM

---

## Arduino IDE — cài đặt

1. **Board:** `esp32` by Espressif **≥ 3.0**
2. **Board chọn:** `AI Thinker ESP32-CAM`
3. **Partition:** `Huge APP` hoặc `16M flash` tùy module
4. **PSRAM:** `Enabled` (bắt buộc cho VGA/SVGA ổn định)
5. Mở sketch: `ESP32-CAM/ESP32-CAM.ino` (cả folder là sketch)

**Nạp firmware:**

- Giữ nút **IO0** trên shield → bấm **RST** → thả IO0 sau khi bắt đầu upload
- Cổng COM từ shield USB

---

## Cấu hình WiFi & backend

Sửa `Config.h`:

```cpp
#define WIFI_SSID "SmartMarketBot"   // SSID router hoặc hotspot tablet
#define WIFI_PASS "12345678"

#define BACKEND_UPLOAD_ENABLE  1
#define BACKEND_UPLOAD_URL     "http://192.168.1.100:5000/api/vision/upload"
#define BACKEND_UPLOAD_FIELD   "file"   // khớp IFormFile trên API
```

Tham khảo `Config.example.h`.

Sau boot, Serial Monitor **115200** in IP dạng `192.168.x.x`.

---

## HTTP API

| Endpoint | Mô tả |
|----------|--------|
| `GET /` | JSON: device, IP, WiFi, camera, danh sách endpoint |
| `GET /status` | JSON: `ip`, `wifi` (RSSI), `heap`, `uptime`, `camera` |
| `GET /stream` | MJPEG realtime (VGA mặc định) |
| `GET /capture` | Một ảnh JPEG (AI / kệ hàng) |
| `GET` hoặc `POST /upload` | Chụp + POST multipart tới backend (khi `BACKEND_UPLOAD_ENABLE=1`) |

**Ví dụ:**

```text
http://192.168.1.51/
http://192.168.1.51/stream
http://192.168.1.51/capture
http://192.168.1.51/status
http://192.168.1.51/upload
```

Tablet/backend gọi theo **IP LAN** — không dùng AP riêng của CAM.

---

## Camera (mặc định)

| Tham số | Giá trị |
|---------|---------|
| Độ phân giải | VGA (`CAM_USE_SVGA=0`) hoặc SVGA |
| JPEG quality | 12 (`CAM_JPEG_QUALITY` 10–15) |
| AWB, LENC, WPC, DCW | Bật |
| Gain ceiling | 8x (`CAM_GAIN_CEILING_16X=0`) hoặc 16x |
| Stream throttle | `STREAM_MIN_FRAME_MS` 80 ms |

Không dùng UXGA / FPS cao.

---

## Cấu trúc mã

```text
ESP32-CAM/
  ESP32-CAM.ino      — setup / loop, watchdog-safe yield
  Config.h           — WiFi, camera, backend, HTTP
  CamDriver.h        — esp_camera init (AI-Thinker pins)
  WifiSta.h          — STA + reconnect
  HttpCamServer.h    — WebServer routes
  BackendUpload.h    — multipart POST
  Config.example.h
  README.md
```

**Không sửa** firmware trong `../ESP32-S3/`.

---

## ASP.NET — nhận upload

API ví dụ:

```csharp
[HttpPost("api/vision/upload")]
public async Task<IActionResult> Upload(IFormFile file)
{
    if (file == null || file.Length == 0) return BadRequest();
    // lưu / đẩy sang AI service
    return Ok(new { bytes = file.Length });
}
```

ESP32 gửi `multipart/form-data`, field name = `BACKEND_UPLOAD_FIELD` (`file`).

---

## Xử lý sự cố

| Triệu chứng | Gợi ý |
|-------------|--------|
| Camera init fail | Kiểm tra ribbon, 5 V, bật PSRAM |
| WiFi không vào | SSID/pass, 2.4 GHz, CAM xa router |
| Stream đứng | Giảm `CAM_USE_SVGA`, tăng `STREAM_MIN_FRAME_MS` |
| Upload 502 | URL backend, firewall, API field name |
| Nhiễu / reset | Nguồn riêng CAM, tụ 100 µF gần module |

---

## Giấy phép

Cùng dự án SmartMarketBot — chỉ dùng cho đồ án / triển khai nội bộ nhóm.
