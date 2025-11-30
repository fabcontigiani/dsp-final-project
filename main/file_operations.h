/**
 * @file file_operations.h
 * @brief File operation utilities for SD card
 */

#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write data to a file on the SD card
 * @param path File path to write to
 * @param data Data buffer to write
 * @param size Size of data to write
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t file_write_binary(const char *path, const uint8_t *data, size_t size);

/**
 * @brief Write an 8-bit grayscale image as a binary PGM file
 * @param path Destination file path
 * @param data Pointer to grayscale pixel buffer
 * @param size Number of bytes in the pixel buffer
 * @param width Image width in pixels
 * @param height Image height in pixels
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t file_write_pgm(const char *path, const uint8_t *data, size_t size, size_t width, size_t height);

/**
 * @brief Read and display content from a text file
 * @param path File path to read from
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t file_read_text(const char *path);

/**
 * @brief Write text data to a file
 * @param path File path to write to
 * @param text Text string to write
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t file_write_text(const char *path, const char *text);

#ifdef __cplusplus
}
#endif
