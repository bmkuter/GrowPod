#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// Configuration file paths
#define CONFIG_DIR              "/lfs/config"
#define MOTOR_CONFIG_FILE       "/lfs/config/motors.json"
#define POD_CONFIG_FILE         "/lfs/config/pod.json"
#define SYSTEM_CONFIG_FILE      "/lfs/config/system.json"
#define SCHEDULES_CONFIG_FILE   "/lfs/config/schedules.json"
#define PLANT_CONFIG_FILE       "/lfs/config/plant.json"

// Plant information structure
typedef struct {
    char plant_name[64];      // Name of the plant
    char start_date[16];      // Start date in format "YYYY-MM-DD"
    int32_t start_timestamp;  // Unix timestamp when planting started
} plant_info_t;

/**
 * @brief Initialize the configuration manager
 * 
 * Creates the config directory if it doesn't exist and performs
 * one-time migration from NVS to JSON if needed.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t config_manager_init(void);

/**
 * @brief Save a JSON object to a file
 * 
 * @param file_path Full path to the config file
 * @param json JSON object to save
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t config_save_json(const char *file_path, cJSON *json);

/**
 * @brief Load a JSON object from a file
 * 
 * @param file_path Full path to the config file
 * @return cJSON* Parsed JSON object (must be freed with cJSON_Delete), or NULL on error
 */
cJSON* config_load_json(const char *file_path);

/**
 * @brief Check if a config file exists
 * 
 * @param file_path Full path to the config file
 * @return true if file exists, false otherwise
 */
bool config_file_exists(const char *file_path);

// Motor configuration functions
/**
 * @brief Save motor direction configuration to JSON
 * 
 * @param motor1_inverted Motor 1 (Planter) direction inverted
 * @param motor2_inverted Motor 2 (Food) direction inverted
 * @param motor3_inverted Motor 3 (Source) direction inverted
 * @param motor4_inverted Motor 4 (LED) direction inverted
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t config_save_motor_directions(bool motor1_inverted, bool motor2_inverted, 
                                        bool motor3_inverted, bool motor4_inverted);

/**
 * @brief Load motor direction configuration from JSON
 * 
 * @param motor1_inverted Pointer to store Motor 1 direction
 * @param motor2_inverted Pointer to store Motor 2 direction
 * @param motor3_inverted Pointer to store Motor 3 direction
 * @param motor4_inverted Pointer to store Motor 4 direction
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if file doesn't exist
 */
esp_err_t config_load_motor_directions(bool *motor1_inverted, bool *motor2_inverted,
                                        bool *motor3_inverted, bool *motor4_inverted);

// Pod configuration functions
/**
 * @brief Save pod calibration settings to JSON
 * 
 * @param empty_height_mm Empty tank height in mm
 * @param full_height_mm Full tank height in mm
 * @param headspace_mm Headspace distance in mm
 * @param calibrated Whether pod is calibrated
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t config_save_pod_calibration(int32_t empty_height_mm, int32_t full_height_mm,
                                       int32_t headspace_mm, bool calibrated);

/**
 * @brief Load pod calibration settings from JSON
 * 
 * @param empty_height_mm Pointer to store empty height
 * @param full_height_mm Pointer to store full height
 * @param headspace_mm Pointer to store headspace
 * @param calibrated Pointer to store calibration status
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if file doesn't exist
 */
esp_err_t config_load_pod_calibration(int32_t *empty_height_mm, int32_t *full_height_mm,
                                       int32_t *headspace_mm, bool *calibrated);

// System configuration functions
/**
 * @brief Save mDNS suffix to system config
 * 
 * @param mdns_suffix mDNS suffix string
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t config_save_mdns_suffix(const char *mdns_suffix);

/**
 * @brief Load mDNS suffix from system config
 * 
 * @param mdns_suffix Buffer to store suffix (must be at least 64 bytes)
 * @param max_len Maximum length of buffer
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if not set
 */
esp_err_t config_load_mdns_suffix(char *mdns_suffix, size_t max_len);

// Schedule configuration functions
/**
 * @brief Save a schedule to JSON
 * 
 * @param schedule_type "light", "planter", or "air"
 * @param schedule Array of 24 uint8_t values (0-100)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t config_save_schedule(const char *schedule_type, const uint8_t *schedule);

/**
 * @brief Load a schedule from JSON
 * 
 * @param schedule_type "light", "planter", or "air"
 * @param schedule Buffer to store 24 uint8_t values
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if not set
 */
esp_err_t config_load_schedule(const char *schedule_type, uint8_t *schedule);

// Plant information functions
/**
 * @brief Save plant information to JSON
 * 
 * @param info Pointer to plant_info_t structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t config_save_plant_info(const plant_info_t *info);

/**
 * @brief Load plant information from JSON
 * 
 * @param info Pointer to plant_info_t structure to fill
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if not set
 */
esp_err_t config_load_plant_info(plant_info_t *info);

/**
 * @brief Calculate days since planting based on current time
 * 
 * @param info Pointer to plant_info_t structure
 * @return int32_t Number of days since planting, or -1 if not set/invalid
 */
int32_t config_get_days_growing(const plant_info_t *info);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_MANAGER_H
