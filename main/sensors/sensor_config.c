#include "sensor_config.h"
#include "filesystem/filesystem_manager.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "SENSOR_CFG";

esp_err_t sensor_config_save_water_calibration(const fdc1004_calibration_t *calibration)
{
    if (calibration == NULL) {
        ESP_LOGE(TAG, "Null calibration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (!calibration->is_calibrated) {
        ESP_LOGW(TAG, "Attempting to save uncalibrated  sensor data");
        return ESP_ERR_INVALID_STATE;
    }

    if (!filesystem_is_mounted()) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Saving water level sensor calibration...");

    // Create JSON object
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return ESP_ERR_NO_MEM;
    }

    // Create water_level section
    cJSON *water_level = cJSON_CreateObject();
    if (water_level == NULL) {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Failed to create water_level JSON object");
        return ESP_ERR_NO_MEM;
    }

    // Add calibration fields
    cJSON_AddNumberToObject(water_level, "capdac", calibration->capdac);
    cJSON_AddNumberToObject(water_level, "cap_empty_pf", calibration->cap_empty_pf);
    cJSON_AddNumberToObject(water_level, "cap_full_pf", calibration->cap_full_pf);
    cJSON_AddNumberToObject(water_level, "height_mm", calibration->height_mm);
    cJSON_AddNumberToObject(water_level, "cap_per_mm", calibration->cap_per_mm);
    cJSON_AddBoolToObject(water_level, "is_calibrated", calibration->is_calibrated);

    // Add timestamp (for reference)
    cJSON_AddNumberToObject(water_level, "timestamp", (double)time(NULL));

    // Add to root
    cJSON_AddItemToObject(root, "water_level", water_level);

    // Convert to string
    char *json_string = cJSON_Print(root);
    if (json_string == NULL) {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Failed to serialize JSON");
        return ESP_ERR_NO_MEM;
    }

    // Ensure config directory exists
    struct stat st;
    if (stat(LFS_MOUNT_POINT "/config", &st) != 0) {
        ESP_LOGI(TAG, "Creating config directory");
        if (mkdir(LFS_MOUNT_POINT "/config", 0755) != 0) {
            ESP_LOGW(TAG, "Failed to create config directory (may already exist)");
        }
    }

    // Write to file
    FILE *f = fopen(SENSOR_CONFIG_FILE, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open %s for writing", SENSOR_CONFIG_FILE);
        cJSON_Delete(root);
        free(json_string);
        return ESP_FAIL;
    }

    size_t len = strlen(json_string);
    size_t written = fwrite(json_string, 1, len, f);
    fclose(f);

    bool success = (written == len);

    // Cleanup
    cJSON_Delete(root);
    free(json_string);

    if (!success) {
        ESP_LOGE(TAG, "Failed to write complete calibration data");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "✓ Water level calibration saved to %s", SENSOR_CONFIG_FILE);
    ESP_LOGI(TAG, "  CAPDAC=%d, empty=%.2f pF, full=%.2f pF, height=%.1f mm",
             calibration->capdac, calibration->cap_empty_pf,
             calibration->cap_full_pf, calibration->height_mm);

    return ESP_OK;
}

esp_err_t sensor_config_load_water_calibration(fdc1004_calibration_t *calibration)
{
    if (calibration == NULL) {
        ESP_LOGE(TAG, "Null calibration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (!filesystem_is_mounted()) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Loading water level sensor calibration from %s", SENSOR_CONFIG_FILE);

    // Check if file exists
    struct stat st;
    if (stat(SENSOR_CONFIG_FILE, &st) != 0) {
        ESP_LOGW(TAG, "Calibration file not found - sensor needs calibration");
        return ESP_ERR_NOT_FOUND;
    }

    // Read file
    FILE *f = fopen(SENSOR_CONFIG_FILE, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open %s for reading", SENSOR_CONFIG_FILE);
        return ESP_FAIL;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (file_size <= 0 || file_size > 4096) {
        ESP_LOGE(TAG, "Invalid file size: %ld", file_size);
        fclose(f);
        return ESP_FAIL;
    }

    // Allocate buffer
    char *buffer = malloc(file_size + 1);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for config file");
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    // Read file content
    size_t read_size = fread(buffer, 1, file_size, f);
    fclose(f);
    buffer[read_size] = '\0';

    // Parse JSON
    cJSON *root = cJSON_Parse(buffer);
    free(buffer);

    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON: %s", cJSON_GetErrorPtr());
        return ESP_FAIL;
    }

    // Get water_level object
    cJSON *water_level = cJSON_GetObjectItem(root, "water_level");
    if (water_level == NULL) {
        ESP_LOGE(TAG, "Missing 'water_level' object in config file");
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    // Extract calibration fields
    cJSON *capdac = cJSON_GetObjectItem(water_level, "capdac");
    cJSON *cap_empty = cJSON_GetObjectItem(water_level, "cap_empty_pf");
    cJSON *cap_full = cJSON_GetObjectItem(water_level, "cap_full_pf");
    cJSON *height = cJSON_GetObjectItem(water_level, "height_mm");
    cJSON *cap_per_mm = cJSON_GetObjectItem(water_level, "cap_per_mm");
    cJSON *is_cal = cJSON_GetObjectItem(water_level, "is_calibrated");

    // Validate all fields present
    if (!cJSON_IsNumber(capdac) || !cJSON_IsNumber(cap_empty) ||
        !cJSON_IsNumber(cap_full) || !cJSON_IsNumber(height) ||
        !cJSON_IsNumber(cap_per_mm) || !cJSON_IsBool(is_cal)) {
        ESP_LOGE(TAG, "Missing or invalid calibration fields in JSON");
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    // Fill calibration structure
    calibration->capdac = (uint8_t)capdac->valueint;
    calibration->cap_empty_pf = (float)cap_empty->valuedouble;
    calibration->cap_full_pf = (float)cap_full->valuedouble;
    calibration->height_mm = (float)height->valuedouble;
    calibration->cap_per_mm = (float)cap_per_mm->valuedouble;
    calibration->is_calibrated = cJSON_IsTrue(is_cal);

    cJSON_Delete(root);

    ESP_LOGI(TAG, "✓ Water level calibration loaded successfully");
    ESP_LOGI(TAG, "  CAPDAC=%d, empty=%.2f pF, full=%.2f pF, height=%.1f mm",
             calibration->capdac, calibration->cap_empty_pf,
             calibration->cap_full_pf, calibration->height_mm);

    // Check if sensor behavior is inverted
    if (calibration->cap_per_mm < 0) {
        ESP_LOGI(TAG, "  Sensor type: INVERTED (capacitance decreases with water)");
    } else {
        ESP_LOGI(TAG, "  Sensor type: NORMAL (capacitance increases with water)");
    }

    return ESP_OK;
}

esp_err_t sensor_config_init(void)
{
    ESP_LOGI(TAG, "Initializing sensor configuration...");

    if (!filesystem_is_mounted()) {
        ESP_LOGE(TAG, "Filesystem not mounted - cannot load sensor config");
        return ESP_ERR_INVALID_STATE;
    }

    // Ensure config directory exists
    struct stat st;
    if (stat(LFS_MOUNT_POINT "/config", &st) != 0) {
        ESP_LOGI(TAG, "Creating config directory");
        if (mkdir(LFS_MOUNT_POINT "/config", 0755) != 0) {
            ESP_LOGW(TAG, "Failed to create config directory (may already exist)");
        }
    }

    // Try to load water level calibration
    fdc1004_calibration_t cal;
    esp_err_t ret = sensor_config_load_water_calibration(&cal);
    
    if (ret == ESP_OK) {
        // Apply calibration to sensor driver
        ret = fdc1004_set_calibration(&cal);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Loaded and applied water level calibration");
        } else {
            ESP_LOGW(TAG, "Loaded calibration but failed to apply: %s", esp_err_to_name(ret));
        }
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved calibration found - sensor needs calibration");
    } else {
        ESP_LOGW(TAG, "Failed to load calibration: %s", esp_err_to_name(ret));
    }

    return ESP_OK;
}
