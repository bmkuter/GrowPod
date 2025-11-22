#include "config_manager.h"
#include "filesystem_manager.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "CONFIG_MGR";

esp_err_t config_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing configuration manager...");
    
    // Create config directory if it doesn't exist
    esp_err_t ret = filesystem_create_dir(CONFIG_DIR);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create config directory");
        return ret;
    }
    
    ESP_LOGI(TAG, "Configuration manager initialized");
    return ESP_OK;
}

esp_err_t config_save_json(const char *file_path, cJSON *json)
{
    if (json == NULL) {
        ESP_LOGE(TAG, "NULL JSON object");
        return ESP_ERR_INVALID_ARG;
    }
    
    char *json_str = cJSON_PrintUnformatted(json);
    if (json_str == NULL) {
        ESP_LOGE(TAG, "Failed to serialize JSON");
        return ESP_FAIL;
    }
    
    FILE *f = fopen(file_path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", file_path);
        free(json_str);
        return ESP_FAIL;
    }
    
    size_t len = strlen(json_str);
    size_t written = fwrite(json_str, 1, len, f);
    fclose(f);
    free(json_str);
    
    if (written != len) {
        ESP_LOGE(TAG, "Failed to write complete JSON to file");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Saved config: %s (%d bytes)", file_path, written);
    return ESP_OK;
}

cJSON* config_load_json(const char *file_path)
{
    FILE *f = fopen(file_path, "r");
    if (f == NULL) {
        ESP_LOGD(TAG, "Config file not found: %s", file_path);
        return NULL;
    }
    
    // Get file size
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    if (file_size <= 0 || file_size > 10240) {  // Max 10KB per config file
        ESP_LOGE(TAG, "Invalid file size: %ld bytes", file_size);
        fclose(f);
        return NULL;
    }
    
    char *buffer = malloc(file_size + 1);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON");
        fclose(f);
        return NULL;
    }
    
    size_t read_size = fread(buffer, 1, file_size, f);
    fclose(f);
    buffer[read_size] = '\0';
    
    cJSON *json = cJSON_Parse(buffer);
    free(buffer);
    
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON from: %s", file_path);
        return NULL;
    }
    
    ESP_LOGI(TAG, "Loaded config: %s", file_path);
    return json;
}

bool config_file_exists(const char *file_path)
{
    struct stat st;
    return (stat(file_path, &st) == 0);
}

// Motor configuration functions
esp_err_t config_save_motor_directions(bool motor1_inverted, bool motor2_inverted,
                                        bool motor3_inverted, bool motor4_inverted)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    cJSON_AddBoolToObject(root, "motor1_inverted", motor1_inverted);
    cJSON_AddBoolToObject(root, "motor2_inverted", motor2_inverted);
    cJSON_AddBoolToObject(root, "motor3_inverted", motor3_inverted);
    cJSON_AddBoolToObject(root, "motor4_inverted", motor4_inverted);
    
    esp_err_t ret = config_save_json(MOTOR_CONFIG_FILE, root);
    cJSON_Delete(root);
    
    return ret;
}

esp_err_t config_load_motor_directions(bool *motor1_inverted, bool *motor2_inverted,
                                        bool *motor3_inverted, bool *motor4_inverted)
{
    cJSON *root = config_load_json(MOTOR_CONFIG_FILE);
    if (root == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    
    cJSON *item;
    
    item = cJSON_GetObjectItem(root, "motor1_inverted");
    if (item && cJSON_IsBool(item)) *motor1_inverted = cJSON_IsTrue(item);
    
    item = cJSON_GetObjectItem(root, "motor2_inverted");
    if (item && cJSON_IsBool(item)) *motor2_inverted = cJSON_IsTrue(item);
    
    item = cJSON_GetObjectItem(root, "motor3_inverted");
    if (item && cJSON_IsBool(item)) *motor3_inverted = cJSON_IsTrue(item);
    
    item = cJSON_GetObjectItem(root, "motor4_inverted");
    if (item && cJSON_IsBool(item)) *motor4_inverted = cJSON_IsTrue(item);
    
    cJSON_Delete(root);
    return ESP_OK;
}

// Pod configuration functions
esp_err_t config_save_pod_calibration(int32_t empty_height_mm, int32_t full_height_mm,
                                       int32_t headspace_mm, bool calibrated)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    cJSON_AddNumberToObject(root, "empty_height_mm", empty_height_mm);
    cJSON_AddNumberToObject(root, "full_height_mm", full_height_mm);
    cJSON_AddNumberToObject(root, "headspace_mm", headspace_mm);
    cJSON_AddBoolToObject(root, "calibrated", calibrated);
    
    esp_err_t ret = config_save_json(POD_CONFIG_FILE, root);
    cJSON_Delete(root);
    
    return ret;
}

esp_err_t config_load_pod_calibration(int32_t *empty_height_mm, int32_t *full_height_mm,
                                       int32_t *headspace_mm, bool *calibrated)
{
    cJSON *root = config_load_json(POD_CONFIG_FILE);
    if (root == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    
    cJSON *item;
    
    item = cJSON_GetObjectItem(root, "empty_height_mm");
    if (item && cJSON_IsNumber(item)) *empty_height_mm = item->valueint;
    
    item = cJSON_GetObjectItem(root, "full_height_mm");
    if (item && cJSON_IsNumber(item)) *full_height_mm = item->valueint;
    
    item = cJSON_GetObjectItem(root, "headspace_mm");
    if (item && cJSON_IsNumber(item)) *headspace_mm = item->valueint;
    
    item = cJSON_GetObjectItem(root, "calibrated");
    if (item && cJSON_IsBool(item)) *calibrated = cJSON_IsTrue(item);
    
    cJSON_Delete(root);
    return ESP_OK;
}

// System configuration functions
esp_err_t config_save_mdns_suffix(const char *mdns_suffix)
{
    cJSON *root = config_load_json(SYSTEM_CONFIG_FILE);
    if (root == NULL) {
        root = cJSON_CreateObject();
        if (root == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Update or add the mdns_suffix field
    cJSON *existing = cJSON_GetObjectItem(root, "mdns_suffix");
    if (existing) {
        cJSON_SetValuestring(existing, mdns_suffix);
    } else {
        cJSON_AddStringToObject(root, "mdns_suffix", mdns_suffix);
    }
    
    esp_err_t ret = config_save_json(SYSTEM_CONFIG_FILE, root);
    cJSON_Delete(root);
    
    return ret;
}

esp_err_t config_load_mdns_suffix(char *mdns_suffix, size_t max_len)
{
    cJSON *root = config_load_json(SYSTEM_CONFIG_FILE);
    if (root == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    
    cJSON *item = cJSON_GetObjectItem(root, "mdns_suffix");
    if (item && cJSON_IsString(item)) {
        strncpy(mdns_suffix, item->valuestring, max_len - 1);
        mdns_suffix[max_len - 1] = '\0';
        cJSON_Delete(root);
        return ESP_OK;
    }
    
    cJSON_Delete(root);
    return ESP_ERR_NOT_FOUND;
}

// Schedule configuration functions
esp_err_t config_save_schedule(const char *schedule_type, const uint8_t *schedule)
{
    cJSON *root = config_load_json(SCHEDULES_CONFIG_FILE);
    if (root == NULL) {
        root = cJSON_CreateObject();
        if (root == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Create array for schedule values
    cJSON *schedule_array = cJSON_CreateArray();
    if (schedule_array == NULL) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    
    for (int i = 0; i < 24; i++) {
        cJSON_AddItemToArray(schedule_array, cJSON_CreateNumber(schedule[i]));
    }
    
    // Replace or add schedule
    cJSON *existing = cJSON_GetObjectItem(root, schedule_type);
    if (existing) {
        cJSON_ReplaceItemInObject(root, schedule_type, schedule_array);
    } else {
        cJSON_AddItemToObject(root, schedule_type, schedule_array);
    }
    
    esp_err_t ret = config_save_json(SCHEDULES_CONFIG_FILE, root);
    cJSON_Delete(root);
    
    return ret;
}

esp_err_t config_load_schedule(const char *schedule_type, uint8_t *schedule)
{
    cJSON *root = config_load_json(SCHEDULES_CONFIG_FILE);
    if (root == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    
    cJSON *schedule_array = cJSON_GetObjectItem(root, schedule_type);
    if (schedule_array == NULL || !cJSON_IsArray(schedule_array)) {
        cJSON_Delete(root);
        return ESP_ERR_NOT_FOUND;
    }
    
    int array_size = cJSON_GetArraySize(schedule_array);
    if (array_size != 24) {
        ESP_LOGW(TAG, "Schedule array size mismatch: %d (expected 24)", array_size);
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    
    for (int i = 0; i < 24; i++) {
        cJSON *item = cJSON_GetArrayItem(schedule_array, i);
        if (item && cJSON_IsNumber(item)) {
            schedule[i] = (uint8_t)item->valueint;
        } else {
            schedule[i] = 0;
        }
    }
    
    cJSON_Delete(root);
    return ESP_OK;
}
