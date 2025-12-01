/**
 * ESP32-CAM Digital Signal Processing Application
 *
 * This application demonstrates advanced image processing on ESP32-CAM:
 * - Captures grayscale images (VGA 640x480) from OV2640 camera sensor
 * - Applies multiple DSP operations using ESP-DSP library:
 *   * 2D convolution filtering (configurable NxN kernels: edge detection, blur, sharpen)
 *   * Contrast stretching (linear intensity mapping with configurable range)
 *   * Histogram equalization (automatic contrast enhancement via CDF redistribution)
 * - Saves original and processed images as PGM files to SD card
 * - Generates detailed processing logs with timing metrics for each operation
 * - All outputs share a common UUID for easy correlation
 *
 * Hardware Requirements:
 * - ESP32-CAM module (AI-Thinker or compatible) with PSRAM
 * - SD card (FAT32 formatted, long filename support enabled)
 * - Adequate lighting for camera sensor
 *
 * Output Files (per capture session):
 * - {UUID}_original.pgm  : Original grayscale image
 * - {UUID}_filtered.pgm  : Convolution filtered result
 * - {UUID}_stretched.pgm : Contrast-stretched image
 * - {UUID}_equalized.pgm : Histogram-equalized image
 * - {UUID}_log.txt       : Processing log with timestamps and performance metrics
 *
 * Configuration:
 * - Convolution kernel: Modify KERNEL_SIZE and CONV_KERNEL array
 * - Contrast range: Adjust CONTRAST_BOTTOM and CONTRAST_TOP defines
 * - Equalization: Set EQUALIZATION_MAX_LEVEL (typically 255)
 * - Camera resolution: Edit camera_driver.c FRAMESIZE setting
 */

#include <stdio.h>
/* Standard library includes */
#include <stdarg.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

/* ESP-IDF includes */
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <esp_err.h>
#include <esp_timer.h>

/* FreeRTOS includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Application modules */
#include "app_config.h"
#include "camera_driver.h"
#include "sd_card_driver.h"
#include "file_operations.h"

#include <esp_dsp.h>
#include "uuid4/uuid4.h"
#include <malloc.h>

static const char *TAG = "dps_final_project";

/* Global log file for capture session */
static FILE *g_log_file = NULL;

/**
 * @brief Log to both console and file
 */
static void log_to_file(esp_log_level_t level, const char *format, ...)
{
    va_list args;
    
    // Log to console using standard ESP_LOG
    va_start(args, format);
    esp_log_writev(level, TAG, format, args);
    va_end(args);
    
    // Also write to file if open
    if (g_log_file != NULL) {
        const char *level_str;
        switch(level) {
            case ESP_LOG_ERROR: level_str = "E"; break;
            case ESP_LOG_WARN:  level_str = "W"; break;
            case ESP_LOG_INFO:  level_str = "I"; break;
            case ESP_LOG_DEBUG: level_str = "D"; break;
            default:            level_str = "V"; break;
        }
        
        fprintf(g_log_file, "(%lld) %s: ", esp_timer_get_time() / 1000, level_str);
        va_start(args, format);
        vfprintf(g_log_file, format, args);
        va_end(args);
        fprintf(g_log_file, "\n");
        fflush(g_log_file); // Ensure data is written immediately
    }
}

/* Convenience macros for file logging */
#define LOG_FILE_I(fmt, ...) log_to_file(ESP_LOG_INFO, fmt, ##__VA_ARGS__)
#define LOG_FILE_W(fmt, ...) log_to_file(ESP_LOG_WARN, fmt, ##__VA_ARGS__)
#define LOG_FILE_E(fmt, ...) log_to_file(ESP_LOG_ERROR, fmt, ##__VA_ARGS__)

/* ===== KERNEL DEFINITION ===== */
/* Define your N×N convolution kernel here */
#define KERNEL_SIZE 3
static const float CONV_KERNEL[KERNEL_SIZE * KERNEL_SIZE] = {
    // Edge detection (Sobel X):
   -1.0f, 0.0f, 1.0f,
   -2.0f, 0.0f, 2.0f,
   -1.0f, 0.0f, 1.0f
};
/* Alternative kernels you can try:

   // 3x3 Sharpen kernel example
    0.0f, -1.0f,  0.0f,
   -1.0f,  5.0f, -1.0f,
    0.0f, -1.0f,  0.0f
   
   // Gaussian blur:
   1.0f/16, 2.0f/16, 1.0f/16,
   2.0f/16, 4.0f/16, 2.0f/16,
   1.0f/16, 2.0f/16, 1.0f/16
   
   // Box blur:
   1.0f/9, 1.0f/9, 1.0f/9,
   1.0f/9, 1.0f/9, 1.0f/9,
   1.0f/9, 1.0f/9, 1.0f/9
*/
/* ============================= */

/* ===== CONTRAST STRETCHING PARAMETERS ===== */
/* Adjust these values to control the output intensity range */
#define CONTRAST_BOTTOM 50      // Minimum pixel level in adjusted image (0-255)
#define CONTRAST_TOP 200       // Maximum pixel level in adjusted image (0-255)
/* ========================================== */

/* ===== HISTOGRAM EQUALIZATION PARAMETERS ===== */
/* Maximum scale level for equalized output (typically 255) */
#define EQUALIZATION_MAX_LEVEL 255
/* ============================================= */

/**
 * @brief Apply 2D convolution to grayscale image
 * @param input Input image buffer (grayscale)
 * @param output Output image buffer (must be pre-allocated)
 * @param width Image width
 * @param height Image height
 * @param kernel Convolution kernel
 * @param kernel_size Kernel dimension (N for NxN kernel)
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t apply_conv2d(const uint8_t *input, float *output, size_t width, size_t height, 
                               const float *kernel, int kernel_size)
{
    // Allocate memory for input as floats (ESP-DSP needs float arrays)
    ESP_LOGI(TAG, "Allocating %.1f KB for input conversion", (width * height * sizeof(float)) / 1024.0f);
    
    // Try regular malloc first, then SPIRAM
    float *input_f = (float *)malloc(width * height * sizeof(float));
    if (input_f == NULL) {
        input_f = (float *)heap_caps_malloc(width * height * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (input_f == NULL) {
        ESP_LOGE(TAG, "Failed to allocate input float buffer");
        return ESP_ERR_NO_MEM;
    }

    // Allocate kernel copy (aligned)
    float *kernel_f = (float *)memalign(16, kernel_size * kernel_size * sizeof(float));
    if (kernel_f == NULL) {
        free(input_f);
        ESP_LOGE(TAG, "Failed to allocate kernel buffer");
        return ESP_ERR_NO_MEM;
    }

    // Convert uint8 to float
    for (size_t i = 0; i < width * height; i++) {
        input_f[i] = (float)input[i];
    }

    // Copy kernel
    memcpy(kernel_f, kernel, kernel_size * kernel_size * sizeof(float));

    // Setup image structures for ESP-DSP
    image2d_t img_input = {
        .data = input_f,
        .step_x = 1,
        .step_y = 1,
        .stride_x = (int)width,
        .stride_y = (int)height,
        .size_x = (int)width,
        .size_y = (int)height
    };

    image2d_t img_kernel = {
        .data = kernel_f,
        .step_x = 1,
        .step_y = 1,
        .stride_x = kernel_size,
        .stride_y = kernel_size,
        .size_x = kernel_size,
        .size_y = kernel_size
    };

    // Output will be smaller due to valid convolution
    int out_width = (int)width - kernel_size + 1;
    int out_height = (int)height - kernel_size + 1;
    
    image2d_t img_output = {
        .data = output,
        .step_x = 1,
        .step_y = 1,
        .stride_x = out_width,
        .stride_y = out_height,
        .size_x = out_width,
        .size_y = out_height
    };

    // Perform 2D convolution
    ESP_LOGI(TAG, "Applying %dx%d convolution to %zux%zu image...", kernel_size, kernel_size, width, height);
    esp_err_t ret = dspi_conv_f32(&img_input, &img_kernel, &img_output);

    free(input_f);
    free(kernel_f);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Convolution failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Convolution complete. Output size: %dx%d", out_width, out_height);
    }

    return ret;
}

/**
 * @brief Apply contrast stretching to grayscale image
 * 
 * Applies the formula: p_adjust(m,n) = Bottom + ((p(m,n) - L) / (H - L)) * (Top - Bottom)
 * where:
 *   - p(m,n) is the original pixel value
 *   - p_adjust(m,n) is the adjusted pixel value
 *   - H is the maximum pixel level in the original image
 *   - L is the minimum pixel level in the original image
 *   - Top is the maximum pixel level in the new image (CONTRAST_TOP)
 *   - Bottom is the minimum pixel level in the new image (CONTRAST_BOTTOM)
 * 
 * @param input Input image buffer (grayscale)
 * @param output Output image buffer (must be pre-allocated, same size as input)
 * @param width Image width
 * @param height Image height
 * @param bottom Minimum output intensity level
 * @param top Maximum output intensity level
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t apply_contrast_stretch(const uint8_t *input, uint8_t *output, 
                                         size_t width, size_t height,
                                         uint8_t bottom, uint8_t top)
{
    if (input == NULL || output == NULL || width == 0 || height == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total_pixels = width * height;
    
    // Find min (L) and max (H) pixel values in the original image
    uint8_t L = 255, H = 0;
    for (size_t i = 0; i < total_pixels; i++) {
        if (input[i] < L) L = input[i];
        if (input[i] > H) H = input[i];
    }

    ESP_LOGI(TAG, "Original image range: L=%u, H=%u", L, H);
    ESP_LOGI(TAG, "Target range: Bottom=%u, Top=%u", bottom, top);

    // Handle edge case where all pixels have the same value
    if (H == L) {
        ESP_LOGW(TAG, "Image has uniform intensity, setting all pixels to midpoint");
        uint8_t mid_value = (bottom + top) / 2;
        for (size_t i = 0; i < total_pixels; i++) {
            output[i] = mid_value;
        }
        return ESP_OK;
    }

    // Apply contrast stretching formula to each pixel
    float range_ratio = (float)(top - bottom) / (float)(H - L);
    for (size_t i = 0; i < total_pixels; i++) {
        // p_adjust = Bottom + ((p - L) / (H - L)) * (Top - Bottom)
        float adjusted = bottom + (input[i] - L) * range_ratio;
        
        // Clamp to valid range (should not be necessary, but safety check)
        if (adjusted < 0.0f) adjusted = 0.0f;
        if (adjusted > 255.0f) adjusted = 255.0f;
        
        output[i] = (uint8_t)(adjusted + 0.5f); // Round to nearest integer
    }

    ESP_LOGI(TAG, "Contrast stretching complete");
    return ESP_OK;
}

/**
 * @brief Apply histogram equalization to grayscale image
 * 
 * Applies the formula: p_eq(m,n) = [CDF(p(m,n)) / total_pixels] * max_level
 * where CDF(p) is the cumulative count of pixels with intensity <= p
 * 
 * This redistributes pixel intensities to use the full dynamic range,
 * improving contrast in images with poor intensity distribution.
 * 
 * @param input Input image buffer (grayscale)
 * @param output Output image buffer (must be pre-allocated, same size as input)
 * @param width Image width
 * @param height Image height
 * @param max_level Maximum scale level for output (typically 255)
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t apply_histogram_equalization(const uint8_t *input, uint8_t *output,
                                                size_t width, size_t height,
                                                uint8_t max_level)
{
    if (input == NULL || output == NULL || width == 0 || height == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total_pixels = width * height;
    
    uint32_t *histogram = (uint32_t *)calloc(256, sizeof(uint32_t));
    uint32_t *cdf = (uint32_t *)malloc(256 * sizeof(uint32_t));
    
    if (histogram == NULL || cdf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate histogram/CDF buffers");
        free(histogram);
        free(cdf);
        return ESP_ERR_NO_MEM;
    }
    
    // Build histogram: count occurrences of each intensity level (0-255)
    for (size_t i = 0; i < total_pixels; i++) {
        histogram[input[i]]++;
    }

    // Calculate cumulative distribution function (CDF)
    // CDF[i] = number of pixels with intensity <= i
    cdf[0] = histogram[0];
    for (int i = 1; i < 256; i++) {
        cdf[i] = cdf[i - 1] + histogram[i];
    }

    // Log histogram statistics
    int min_intensity = -1, max_intensity = -1;
    for (int i = 0; i < 256; i++) {
        if (histogram[i] > 0) {
            if (min_intensity == -1) min_intensity = i;
            max_intensity = i;
        }
    }
    ESP_LOGI(TAG, "Histogram range: %d to %d", min_intensity, max_intensity);

    // Apply equalization formula to each pixel
    // p_eq = (CDF[p] / total_pixels) * max_level
    float scale_factor = (float)max_level / (float)total_pixels;
    for (size_t i = 0; i < total_pixels; i++) {
        uint8_t original_value = input[i];
        float equalized = cdf[original_value] * scale_factor;
        
        // Clamp to valid range
        if (equalized < 0.0f) equalized = 0.0f;
        if (equalized > (float)max_level) equalized = (float)max_level;
        
        output[i] = (uint8_t)(equalized + 0.5f); // Round to nearest
    }

    free(histogram);
    free(cdf);
    
    ESP_LOGI(TAG, "Histogram equalization complete");
    return ESP_OK;
}

/**
 * @brief Capture and save a photo to SD card
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t capture_and_save_photo(void)
{
    if (!camera_is_supported())
    {
        ESP_LOGW(TAG, "Camera not supported on this platform");
        return ESP_ERR_NOT_SUPPORTED;
    }

    camera_fb_t *frame_buffer = camera_capture_photo();
    if (frame_buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to capture photo");
        return ESP_FAIL;
    }

    if (frame_buffer->format != PIXFORMAT_GRAYSCALE)
    {
        ESP_LOGE(TAG, "Unexpected pixel format: %d", frame_buffer->format);
        camera_return_frame_buffer(frame_buffer);
        return ESP_ERR_INVALID_STATE;
    }

    /* Generate UUID for this capture session */
    char uuid_buf[UUID4_LEN];
    uuid4_generate(uuid_buf);
    
    /* Open log file for this capture session */
    char log_path[sizeof(MOUNT_POINT) + UUID4_LEN + sizeof("/_log.txt")];
    snprintf(log_path, sizeof(log_path), MOUNT_POINT "/%s_log.txt", uuid_buf);
    g_log_file = fopen(log_path, "w");
    if (g_log_file == NULL) {
        ESP_LOGW(TAG, "Failed to open log file: %s", log_path);
    } else {
        fprintf(g_log_file, "=== Image Processing Log ===\n");
        fprintf(g_log_file, "UUID: %s\n", uuid_buf);
        fprintf(g_log_file, "Timestamp: %lld ms\n\n", esp_timer_get_time() / 1000);
        fflush(g_log_file);
    }

    /* Save original photo to SD card */
    char original_path[sizeof(MOUNT_POINT) + UUID4_LEN + sizeof("/_original.pgm")];
    snprintf(original_path, sizeof(original_path), MOUNT_POINT "/%s_original.pgm", uuid_buf);
    esp_err_t ret = file_write_pgm(original_path, frame_buffer->buf, frame_buffer->len, 
                                    frame_buffer->width, frame_buffer->height);
    if (ret != ESP_OK) {
        LOG_FILE_E("Failed to save original image");
        if (g_log_file) { fclose(g_log_file); g_log_file = NULL; }
        camera_return_frame_buffer(frame_buffer);
        return ret;
    }
    LOG_FILE_I("Original image saved: %s (%zux%zu pixels)", original_path, frame_buffer->width, frame_buffer->height);

    // Copy frame data to working buffer and free camera frame early
    size_t frame_size = frame_buffer->len;
    size_t frame_width = frame_buffer->width;
    size_t frame_height = frame_buffer->height;
    
    uint8_t *working_buffer = (uint8_t *)malloc(frame_size);
    if (working_buffer == NULL) {
        LOG_FILE_E("Failed to allocate working buffer");
        if (g_log_file) { fclose(g_log_file); g_log_file = NULL; }
        camera_return_frame_buffer(frame_buffer);
        return ESP_ERR_NO_MEM;
    }
    memcpy(working_buffer, frame_buffer->buf, frame_size);
    camera_return_frame_buffer(frame_buffer); // Free camera buffer early
    frame_buffer = NULL;

    /* Apply convolution */
    LOG_FILE_I("Processing convolution...");
    int64_t conv_start = esp_timer_get_time();
    int out_width = frame_width - KERNEL_SIZE + 1;
    int out_height = frame_height - KERNEL_SIZE + 1;
    size_t output_size = out_width * out_height;

    float *conv_output = (float *)malloc(output_size * sizeof(float));
    if (conv_output == NULL) {
        conv_output = (float *)heap_caps_malloc(output_size * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (conv_output != NULL) {
        ret = apply_conv2d(working_buffer, conv_output, frame_width, frame_height,
                           CONV_KERNEL, KERNEL_SIZE);
        
        if (ret == ESP_OK) {
            uint8_t *output_u8 = (uint8_t *)malloc(output_size);
            if (output_u8 != NULL) {
                for (size_t i = 0; i < output_size; i++) {
                    float val = conv_output[i];
                    if (val < 0.0f) val = 0.0f;
                    if (val > 255.0f) val = 255.0f;
                    output_u8[i] = (uint8_t)val;
                }

                char filtered_path[sizeof(MOUNT_POINT) + UUID4_LEN + sizeof("/_filtered.pgm")];
                snprintf(filtered_path, sizeof(filtered_path), MOUNT_POINT "/%s_filtered.pgm", uuid_buf);
                ret = file_write_pgm(filtered_path, output_u8, output_size, out_width, out_height);
                
                if (ret == ESP_OK) {
                    LOG_FILE_I("Filtered image saved: %s", filtered_path);
                } else {
                    LOG_FILE_E("Failed to save filtered image");
                }
                
                free(output_u8);
            }
        }
        free(conv_output);
        int64_t conv_end = esp_timer_get_time();
        LOG_FILE_I("Convolution completed in %.2f ms", (conv_end - conv_start) / 1000.0);
    } else {
        LOG_FILE_W("Skipping convolution - insufficient memory");
    }

    /* Apply contrast stretching - reuse working_buffer for output */
    LOG_FILE_I("Processing contrast stretching...");
    int64_t stretch_start = esp_timer_get_time();
    uint8_t *stretched_output = (uint8_t *)malloc(frame_size);
    if (stretched_output != NULL) {
        esp_err_t stretch_ret = apply_contrast_stretch(working_buffer, stretched_output,
                                                        frame_width, frame_height,
                                                        CONTRAST_BOTTOM, CONTRAST_TOP);
        
        if (stretch_ret == ESP_OK) {
            char stretched_path[sizeof(MOUNT_POINT) + UUID4_LEN + sizeof("/_stretched.pgm")];
            snprintf(stretched_path, sizeof(stretched_path), MOUNT_POINT "/%s_stretched.pgm", uuid_buf);
            stretch_ret = file_write_pgm(stretched_path, stretched_output, frame_size,
                                         frame_width, frame_height);
            
            if (stretch_ret == ESP_OK) {
                LOG_FILE_I("Contrast-stretched image saved: %s", stretched_path);
            } else {
                LOG_FILE_E("Failed to save contrast-stretched image");
            }
        }
        free(stretched_output);
        int64_t stretch_end = esp_timer_get_time();
        LOG_FILE_I("Contrast stretching completed in %.2f ms", (stretch_end - stretch_start) / 1000.0);
    } else {
        LOG_FILE_W("Skipping contrast stretching - insufficient memory");
    }

    /* Apply histogram equalization - reuse working_buffer for output */
    LOG_FILE_I("Processing histogram equalization...");
    int64_t eq_start = esp_timer_get_time();
    uint8_t *equalized_output = (uint8_t *)malloc(frame_size);
    if (equalized_output != NULL) {
        esp_err_t eq_ret = apply_histogram_equalization(working_buffer, equalized_output,
                                                         frame_width, frame_height,
                                                         EQUALIZATION_MAX_LEVEL);
        
        if (eq_ret == ESP_OK) {
            char equalized_path[sizeof(MOUNT_POINT) + UUID4_LEN + sizeof("/_equalized.pgm")];
            snprintf(equalized_path, sizeof(equalized_path), MOUNT_POINT "/%s_equalized.pgm", uuid_buf);
            eq_ret = file_write_pgm(equalized_path, equalized_output, frame_size,
                                    frame_width, frame_height);
            
            if (eq_ret == ESP_OK) {
                LOG_FILE_I("Equalized image saved: %s", equalized_path);
            } else {
                LOG_FILE_E("Failed to save equalized image");
            }
        }
        free(equalized_output);
        int64_t eq_end = esp_timer_get_time();
        LOG_FILE_I("Histogram equalization completed in %.2f ms", (eq_end - eq_start) / 1000.0);
    } else {
        LOG_FILE_W("Skipping histogram equalization - insufficient memory");
    }

    free(working_buffer);
    
    /* Close log file */
    if (g_log_file) {
        fprintf(g_log_file, "\n=== Processing Complete ===\n");
        fclose(g_log_file);
        g_log_file = NULL;
        ESP_LOGI(TAG, "Processing log saved: %s", log_path);
    }

    return ret;
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{

    uuid4_init();

    ESP_LOGI(TAG, "Starting Camera SD Card Example");

    /* Initialize camera */
    if (camera_is_supported())
    {
        if (camera_init() != ESP_OK)
        {
            ESP_LOGE(TAG, "Camera initialization failed, exiting");
            return;
        }
    }
    else
    {
        ESP_LOGW(TAG, "Camera not supported, continuing with SD card only");
    }

    /* Initialize SD card */
    if (sd_card_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "SD card initialization failed, exiting");
        return;
    }

#ifdef CONFIG_EXAMPLE_FORMAT_SD_CARD
    /* Format SD card if requested */
    if (sd_card_format() != ESP_OK)
    {
        ESP_LOGE(TAG, "SD card formatting failed, exiting");
        sd_card_cleanup();
        return;
    }
#endif

    ESP_LOGI(TAG, "Initiating camera warm-up delay (3 seconds)...");
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    /* Warm-up loop to discard first few frames */
    for (int i = 0; i < 10; i++)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Failed to get frame buffer during warm-up");
            continue;
        }
        esp_camera_fb_return(fb);
    }

    /* Capture a single photo */
    ESP_LOGI(TAG, "Capturing a single photo...");
    esp_err_t ret = capture_and_save_photo();
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Photo captured and saved successfully!");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to capture/save photo: %s", esp_err_to_name(ret));
    }

    /* Cleanup and exit */
    sd_card_cleanup();
    ESP_LOGI(TAG, "Application completed, entering idle state");
}
