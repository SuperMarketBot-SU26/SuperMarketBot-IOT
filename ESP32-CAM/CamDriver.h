/* =====================================================================
 *  CamDriver.h — Khởi tạo OV2640 (AI-Thinker ESP32-CAM), cấu hình ổn định
 * =====================================================================*/
#ifndef ESP32_CAM_CAM_DRIVER_H
#define ESP32_CAM_CAM_DRIVER_H

#include "Config.h"
#include "esp_camera.h"

/** AI-Thinker ESP32-CAM — chân chuẩn (không đổi nếu không phải board khác). */
#define CAM_PWDN_GPIO_NUM     32
#define CAM_RESET_GPIO_NUM    -1
#define CAM_XCLK_GPIO_NUM      0
#define CAM_SIOD_GPIO_NUM     26
#define CAM_SIOC_GPIO_NUM     27
#define CAM_Y9_GPIO_NUM       35
#define CAM_Y8_GPIO_NUM       34
#define CAM_Y7_GPIO_NUM       39
#define CAM_Y6_GPIO_NUM       36
#define CAM_Y5_GPIO_NUM       21
#define CAM_Y4_GPIO_NUM       19
#define CAM_Y3_GPIO_NUM       18
#define CAM_Y2_GPIO_NUM        5
#define CAM_VSYNC_GPIO_NUM    25
#define CAM_HREF_GPIO_NUM     23
#define CAM_PCLK_GPIO_NUM     22

inline bool camInit() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = CAM_Y2_GPIO_NUM;
  cfg.pin_d1       = CAM_Y3_GPIO_NUM;
  cfg.pin_d2       = CAM_Y4_GPIO_NUM;
  cfg.pin_d3       = CAM_Y5_GPIO_NUM;
  cfg.pin_d4       = CAM_Y6_GPIO_NUM;
  cfg.pin_d5       = CAM_Y7_GPIO_NUM;
  cfg.pin_d6       = CAM_Y8_GPIO_NUM;
  cfg.pin_d7       = CAM_Y9_GPIO_NUM;
  cfg.pin_xclk     = CAM_XCLK_GPIO_NUM;
  cfg.pin_pclk     = CAM_PCLK_GPIO_NUM;
  cfg.pin_vsync    = CAM_VSYNC_GPIO_NUM;
  cfg.pin_href     = CAM_HREF_GPIO_NUM;
  cfg.pin_sccb_sda = CAM_SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = CAM_SIOC_GPIO_NUM;
  cfg.pin_pwdn     = CAM_PWDN_GPIO_NUM;
  cfg.pin_reset    = CAM_RESET_GPIO_NUM;
  cfg.xclk_freq_hz = CAM_XCLK_FREQ_HZ;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = CAM_USE_SVGA ? FRAMESIZE_SVGA : FRAMESIZE_VGA;
  cfg.jpeg_quality = CAM_JPEG_QUALITY;
  cfg.fb_count     = CAM_FB_COUNT;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
    cfg.jpeg_quality = CAM_JPEG_QUALITY;
  } else {
    cfg.fb_location = CAMERA_FB_IN_DRAM;
    cfg.frame_size  = FRAMESIZE_CIF;
    cfg.fb_count    = 1;
    Serial.println(F("[CAM] No PSRAM — fallback CIF, fb_count=1"));
  }

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] esp_camera_init failed: 0x%x\n", (unsigned)err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println(F("[CAM] sensor_get failed"));
    return false;
  }

  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_special_effect(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 0);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, CAM_GAIN_CEILING_16X ? GAINCEILING_16X : GAINCEILING_8X);
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);

  Serial.printf("[CAM] OK — %s JPEG q=%d, AWB/LENC/WPC/DCW on\n",
                CAM_USE_SVGA ? "SVGA" : "VGA", CAM_JPEG_QUALITY);
  return true;
}

/** Chụp một frame JPEG; caller phải esp_camera_fb_return(fb). */
inline camera_fb_t *camCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println(F("[CAM] fb_get failed"));
    return nullptr;
  }
  if (fb->format != PIXFORMAT_JPEG) {
    Serial.println(F("[CAM] not JPEG"));
    esp_camera_fb_return(fb);
    return nullptr;
  }
  return fb;
}

inline const char *camStatusStr() {
  return esp_camera_sensor_get() ? "ok" : "error";
}

#endif /* ESP32_CAM_CAM_DRIVER_H */
