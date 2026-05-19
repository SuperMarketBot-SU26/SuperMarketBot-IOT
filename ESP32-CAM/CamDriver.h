/* =====================================================================
 *  CamDriver.h — QVGA live, VGA capture, lật ảnh, profile chất lượng
 * =====================================================================*/
#ifndef ESP32_CAM_CAM_DRIVER_H
#define ESP32_CAM_CAM_DRIVER_H

#include "Config.h"
#include "esp_camera.h"

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

enum CamProfile : uint8_t {
  CAM_PROF_SMOOTH = 0,
  CAM_PROF_BALANCED = 1,
  CAM_PROF_CLEAR = 2
};

static uint8_t g_cam_hmirror = CAM_HMIRROR_DEFAULT;
static uint8_t g_cam_vflip = CAM_VFLIP_DEFAULT;
static uint8_t g_cam_profile = CAM_PROF_SMOOTH;

inline framesize_t camHdFrameSize() {
#if CAM_HD_FRAMESIZE_VGA
  return FRAMESIZE_VGA;
#else
  return FRAMESIZE_SVGA;
#endif
}

inline void camApplyOrientation() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  s->set_hmirror(s, g_cam_hmirror ? 1 : 0);
  s->set_vflip(s, g_cam_vflip ? 1 : 0);
}

inline void camSetProfile(uint8_t profile) {
  if (profile > CAM_PROF_CLEAR) profile = CAM_PROF_CLEAR;
  g_cam_profile = profile;
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  switch (profile) {
    case CAM_PROF_CLEAR:
      s->set_framesize(s, FRAMESIZE_VGA);
      s->set_quality(s, 16);
      break;
    case CAM_PROF_BALANCED:
      s->set_framesize(s, FRAMESIZE_HVGA);
      s->set_quality(s, 22);
      break;
    default:
      s->set_framesize(s, FRAMESIZE_QVGA);
      s->set_quality(s, CAM_PREVIEW_QUALITY);
      break;
  }
  camApplyOrientation();
}

inline void camSetOrientation(int hmirror, int vflip) {
  g_cam_hmirror = hmirror ? 1 : 0;
  g_cam_vflip = vflip ? 1 : 0;
  camApplyOrientation();
}

inline uint8_t camGetHMirror() { return g_cam_hmirror; }
inline uint8_t camGetVFlip() { return g_cam_vflip; }
inline uint8_t camGetProfile() { return g_cam_profile; }

inline const char *camProfileName() {
  switch (g_cam_profile) {
    case CAM_PROF_CLEAR: return "clear";
    case CAM_PROF_BALANCED: return "balanced";
    default: return "smooth";
  }
}

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
  cfg.frame_size   = FRAMESIZE_QVGA;
  cfg.jpeg_quality = CAM_PREVIEW_QUALITY;
  cfg.fb_count     = CAM_FB_COUNT;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    cfg.fb_location = CAMERA_FB_IN_DRAM;
    cfg.frame_size  = FRAMESIZE_QQVGA;
    cfg.fb_count    = 1;
  }

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println(F("[CAM] init failed"));
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (!s) return false;

  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_gain_ctrl(s, 1);
  s->set_gainceiling(s, CAM_GAIN_CEILING_16X ? GAINCEILING_16X : GAINCEILING_8X);
  s->set_wpc(s, 1);
  s->set_lenc(s, 1);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);

  camSetProfile(CAM_PROF_SMOOTH);
  camSetOrientation(CAM_HMIRROR_DEFAULT, CAM_VFLIP_DEFAULT);

  Serial.printf("[CAM] profile=smooth QVGA | flip H=%d V=%d\n",
                (int)g_cam_hmirror, (int)g_cam_vflip);
  return true;
}

inline camera_fb_t *camCapturePreview() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb || fb->format != PIXFORMAT_JPEG) {
    if (fb) esp_camera_fb_return(fb);
    return nullptr;
  }
  return fb;
}

inline camera_fb_t *camCapture() {
  const uint8_t savedProf = g_cam_profile;
  camSetProfile(CAM_PROF_CLEAR);
  sensor_t *s = esp_camera_sensor_get();
  if (s) s->set_quality(s, CAM_JPEG_QUALITY);

  for (int i = 0; i < 2; i++) {
    camera_fb_t *old = esp_camera_fb_get();
    if (old) esp_camera_fb_return(old);
  }
  camera_fb_t *fb = esp_camera_fb_get();
  camSetProfile(savedProf);
  if (!fb || fb->format != PIXFORMAT_JPEG) {
    if (fb) esp_camera_fb_return(fb);
    return nullptr;
  }
  return fb;
}

inline const char *camStatusStr() {
  return esp_camera_sensor_get() ? "ok" : "error";
}

#endif
