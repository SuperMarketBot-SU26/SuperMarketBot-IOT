/* =====================================================================
 *  StatusRGB.h — Chỉ LED RGB zin trên bo ESP32-S3-DevKit (GPIO 38), không LED ngoài
 *  Thư viện: Adafruit NeoPixel.
 *  ENC_RR=48, Neo=38 → không trùng chân; Echo US 10–13, ENC_FL=39.
 * =====================================================================*/
#ifndef STATUS_RGB_H
#define STATUS_RGB_H

#include "Config.h"
#include <math.h>

#if SMB_ONBOARD_RGB
#include <Adafruit_NeoPixel.h>

// Màu thương hiệu HMI (#2dd4bf → RGB)
static const uint8_t SMB_COL_R = 45;
static const uint8_t SMB_COL_G = 212;
static const uint8_t SMB_COL_B = 191;

static Adafruit_NeoPixel s_px(SMB_NEOPIXEL_COUNT, SMB_NEOPIXEL_PIN,
                              NEO_GRB + NEO_KHZ800);

inline void statusRgbInit() {
  s_px.begin();
  s_px.setBrightness(SMB_RGB_BRIGHTNESS);
  s_px.clear();
  s_px.show();
}

/**
 * Cập nhật hiệu ứng theo trạng thái (gọi ~20–30 Hz).
 */
inline void statusRgbUpdate() {
  const uint32_t t = millis();
  uint8_t r, g, b;

  if (g_state.estop) {
    // Nháy đỏ cảnh báo
    const bool on = ((t / 100) & 1u) != 0;
    r = on ? 220 : 0;
    g = b = 0;
  } else if (g_state.mode == MODE_AUTO) {
    // Tự hành: hơi sáng hơn, nhịp nhanh hơn, thêm ánh xanh
    const float br = 0.35f + 0.65f * (0.5f + 0.5f * sinf(t * 0.01f));
    r = (uint8_t)(SMB_COL_R * br);
    g = (uint8_t)((SMB_COL_G + 15) * br);
    {
      int bi = (int)((SMB_COL_B + 40) * br);
      b = (uint8_t)(bi > 255 ? 255 : bi);
    }
  } else {
    // Lái tay: thở chậm (breathing) teal
    const float br = 0.3f + 0.7f * (0.5f + 0.5f * sinf(t * 0.0045f));
    r = (uint8_t)(SMB_COL_R * br);
    g = (uint8_t)(SMB_COL_G * br);
    b = (uint8_t)(SMB_COL_B * br);
  }

  s_px.setPixelColor(0, s_px.Color(r, g, b));
  s_px.show();
}

#else
inline void statusRgbInit() {}
inline void statusRgbUpdate() {}
#endif

#endif
