/**
 * @file camera_driver.c
 * @brief ESP32-CAM camera driver implementation
 */

#include "camera_driver.h"
#include "app_config.h"
#include <esp_log.h>

static const char *TAG = "camera_driver";

/* Camera Configuration */
#if ESP_CAMERA_SUPPORTED
static camera_config_t camera_config = {
    /* Pin configuration */
    .pin_pwdn       = CAM_PIN_PWDN,
    .pin_reset      = CAM_PIN_RESET,
    .pin_xclk       = CAM_PIN_XCLK,
    .pin_sccb_sda   = CAM_PIN_SIOD,
    .pin_sccb_scl   = CAM_PIN_SIOC,
    
    /* Data pins */
    .pin_d7         = CAM_PIN_D7,
    .pin_d6         = CAM_PIN_D6,
    .pin_d5         = CAM_PIN_D5,
    .pin_d4         = CAM_PIN_D4,
    .pin_d3         = CAM_PIN_D3,
    .pin_d2         = CAM_PIN_D2,
    .pin_d1         = CAM_PIN_D1,
    .pin_d0         = CAM_PIN_D0,
    .pin_vsync      = CAM_PIN_VSYNC,
    .pin_href       = CAM_PIN_HREF,
    .pin_pclk       = CAM_PIN_PCLK,

    /* Clock configuration */
    .xclk_freq_hz   = 20000000,    // XCLK 20MHz for OV2640
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,

    /* Image configuration */
    .pixel_format   = PIXFORMAT_GRAYSCALE,
    .frame_size     = FRAMESIZE_UXGA,
    .jpeg_quality   = 8,                // Only used for JPEG modes
    .fb_count       = 1,
    .fb_location    = CAMERA_FB_IN_PSRAM,
    .grab_mode      = CAMERA_GRAB_WHEN_EMPTY,
};
#endif

esp_err_t camera_init(void)
{
#if ESP_CAMERA_SUPPORTED
    ESP_LOGI(TAG, "Initializing camera...");
    
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
#else
    ESP_LOGW(TAG, "Camera not supported on this platform");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

camera_fb_t* camera_capture_photo(void)
{
#if ESP_CAMERA_SUPPORTED
    ESP_LOGI(TAG, "Capturing photo...");
    
    camera_fb_t *frame_buffer = esp_camera_fb_get();
    if (frame_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to capture photo");
        return NULL;
    }
    
    ESP_LOGI(TAG, "Photo captured successfully (%zu bytes)", frame_buffer->len);
    return frame_buffer;
#else
    ESP_LOGW(TAG, "Camera not supported on this platform");
    return NULL;
#endif
}

void camera_return_frame_buffer(camera_fb_t* fb)
{
#if ESP_CAMERA_SUPPORTED
    if (fb) {
        esp_camera_fb_return(fb);
    }
#endif
}

bool camera_is_supported(void)
{
#if ESP_CAMERA_SUPPORTED
    return true;
#else
    return false;
#endif
}
