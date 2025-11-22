#ifndef FILESYSTEM_MANAGER_H
#define FILESYSTEM_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mount point for LittleFS
#define LFS_MOUNT_POINT "/lfs"

// Maximum path length
#define LFS_MAX_PATH_LEN 128

/**
 * @brief Initialize and mount the LittleFS filesystem
 * 
 * This function initializes the LittleFS partition and mounts it.
 * If the filesystem is corrupted, it will attempt to format it.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t filesystem_init(void);

/**
 * @brief Unmount and deinitialize the filesystem
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t filesystem_deinit(void);

/**
 * @brief Format the LittleFS filesystem
 * 
 * WARNING: This will erase all data on the filesystem!
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t filesystem_format(void);

/**
 * @brief Get filesystem usage information
 * 
 * @param total_bytes Pointer to store total filesystem size in bytes
 * @param used_bytes Pointer to store used space in bytes
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t filesystem_get_usage(size_t *total_bytes, size_t *used_bytes);

/**
 * @brief Check if filesystem is mounted
 * 
 * @return true if mounted, false otherwise
 */
bool filesystem_is_mounted(void);

/**
 * @brief Test function: Write and read a test file
 * 
 * This function performs a simple write/read test to verify
 * the filesystem is working correctly.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t filesystem_test(void);

/**
 * @brief List all files in a directory
 * 
 * @param path Directory path (e.g., "/lfs/" or "/lfs/config/")
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t filesystem_list_dir(const char *path);

/**
 * @brief List all files recursively with full paths
 * 
 * @param path Directory path to start from
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t filesystem_list_dir_recursive(const char *path);

/**
 * @brief Read entire file contents into a buffer
 * 
 * @param path Full file path
 * @param buffer Pointer to store allocated buffer (caller must free)
 * @param size Pointer to store file size
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t filesystem_read_file(const char *path, char **buffer, size_t *size);

/**
 * @brief Delete a file
 * 
 * @param path Full file path (e.g., "/lfs/test.txt")
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t filesystem_delete_file(const char *path);

/**
 * @brief Create a directory
 * 
 * @param path Directory path (e.g., "/lfs/config")
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t filesystem_create_dir(const char *path);

#ifdef __cplusplus
}
#endif

#endif // FILESYSTEM_MANAGER_H
