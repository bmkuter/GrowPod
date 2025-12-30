/**
 * @file sensor_logger.c
 * @brief Implementation of sensor data logging to persistent storage
 */

#include "sensor_logger.h"
#include "sensor_manager.h"
#include "esp_log.h"
#include "esp_system.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>

static const char *TAG = "SENSOR_LOG";

// Configuration
#define HISTORY_FILE_PATH "/lfs/data/sensor_history.json"
#define MAX_HISTORY_ENTRIES 120  // 2 hours * 60 minutes
#define DATA_DIR "/lfs/data"

// Logger state
typedef struct {
    sensor_history_entry_t *entries;  // Circular buffer
    uint32_t head;                    // Write position
    uint32_t count;                   // Current number of entries
    bool initialized;
    
    // Statistics
    uint32_t total_entries;
    uint32_t write_errors;
    uint32_t buffer_wraps;
} sensor_logger_state_t;

static sensor_logger_state_t g_logger = {0};

// Forward declarations
static esp_err_t load_history_from_file(void);
static esp_err_t save_history_to_file(void);
static int64_t get_current_timestamp(void);

/**
 * @brief Get current Unix timestamp
 */
static int64_t get_current_timestamp(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec;
}

/**
 * @brief Initialize sensor logger
 */
esp_err_t sensor_logger_init(void) {
    if (g_logger.initialized) {
        ESP_LOGW(TAG, "Sensor logger already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing sensor logger");
    
    // Allocate circular buffer
    g_logger.entries = calloc(MAX_HISTORY_ENTRIES, sizeof(sensor_history_entry_t));
    if (g_logger.entries == NULL) {
        ESP_LOGE(TAG, "Failed to allocate history buffer");
        return ESP_ERR_NO_MEM;
    }
    
    g_logger.head = 0;
    g_logger.count = 0;
    g_logger.total_entries = 0;
    g_logger.write_errors = 0;
    g_logger.buffer_wraps = 0;
    
    // Create data directory if it doesn't exist
    struct stat st = {0};
    if (stat(DATA_DIR, &st) == -1) {
        ESP_LOGI(TAG, "Creating data directory: %s", DATA_DIR);
        if (mkdir(DATA_DIR, 0755) != 0) {
            ESP_LOGW(TAG, "Failed to create data directory (may already exist)");
        }
    }
    
    // Try to load existing history
    esp_err_t ret = load_history_from_file();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded %lu existing history entries", g_logger.count);
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "No existing history file, starting fresh");
    } else {
        ESP_LOGW(TAG, "Failed to load history file: %s", esp_err_to_name(ret));
    }
    
    g_logger.initialized = true;
    ESP_LOGI(TAG, "Sensor logger initialized (buffer size: %d entries)", MAX_HISTORY_ENTRIES);
    
    return ESP_OK;
}

/**
 * @brief Log current sensor snapshot
 */
esp_err_t sensor_logger_log_snapshot(void) {
    if (!g_logger.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get current timestamp
    int64_t timestamp = get_current_timestamp();
    
    // Create entry from sensor manager cache
    sensor_history_entry_t entry = {
        .timestamp = timestamp,
        .temperature_c = -999.0f,
        .humidity_rh = -999.0f,
        .light_lux = -999.0f,
        .light_visible = 0,
        .light_infrared = 0,
        .power_mw = 0.0f,
        .current_ma = 0.0f,
        .voltage_mv = 0.0f,
        .water_level_mm = -1.0f
    };
    
    // Read environment sensor (temperature & humidity)
    float temp, humid;
    if (sensor_manager_get_environment_cached(&temp, &humid, NULL) == ESP_OK) {
        entry.temperature_c = temp;
        entry.humidity_rh = humid;
    }
    
    // Read light sensor
    sensor_data_t light_data;
    if (sensor_manager_get_data_cached(SENSOR_TYPE_LIGHT, &light_data, NULL) == ESP_OK) {
        entry.light_lux = light_data.light.lux;
        entry.light_visible = light_data.light.visible;
        entry.light_infrared = light_data.light.infrared;
    }
    
    // Read power sensors
    float value;
    if (sensor_manager_get_cached(SENSOR_TYPE_POWER_POWER, &value, NULL) == ESP_OK) {
        entry.power_mw = value;
    }
    if (sensor_manager_get_cached(SENSOR_TYPE_POWER_CURRENT, &value, NULL) == ESP_OK) {
        entry.current_ma = value;
    }
    if (sensor_manager_get_cached(SENSOR_TYPE_POWER_VOLTAGE, &value, NULL) == ESP_OK) {
        entry.voltage_mv = value;
    }
    
    // Read water level
    if (sensor_manager_get_cached(SENSOR_TYPE_WATER_LEVEL, &value, NULL) == ESP_OK) {
        entry.water_level_mm = value;
    }
    
    // Add to circular buffer
    g_logger.entries[g_logger.head] = entry;
    g_logger.head = (g_logger.head + 1) % MAX_HISTORY_ENTRIES;
    
    if (g_logger.count < MAX_HISTORY_ENTRIES) {
        g_logger.count++;
    } else {
        g_logger.buffer_wraps++;
    }
    
    g_logger.total_entries++;
    
    ESP_LOGD(TAG, "Logged snapshot: ts=%lld, temp=%.1f°C, humid=%.1f%%, lux=%.1f, power=%.1fmW",
             timestamp, entry.temperature_c, entry.humidity_rh, entry.light_lux, entry.power_mw);
    
    // Save to file every 5 entries (every 5 minutes) for testing
    // TODO: Change to 10 or 20 for production to reduce flash wear
    if (g_logger.total_entries % 5 == 0) {
        esp_err_t ret = save_history_to_file();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save history to file: %s", esp_err_to_name(ret));
            g_logger.write_errors++;
            return ret;
        }
        ESP_LOGI(TAG, "History saved to file (%lu entries)", g_logger.count);
    }
    
    return ESP_OK;
}

/**
 * @brief Get historical data within time range
 */
esp_err_t sensor_logger_get_history(int64_t start_timestamp,
                                     int64_t end_timestamp,
                                     sensor_history_entry_t *entries,
                                     uint32_t max_entries,
                                     uint32_t *count_out) {
    if (!g_logger.initialized || entries == NULL || count_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (end_timestamp == 0) {
        end_timestamp = get_current_timestamp();
    }
    
    *count_out = 0;
    
    // Iterate through circular buffer
    uint32_t start_idx = (g_logger.head >= g_logger.count) ? 
                         0 : (g_logger.head - g_logger.count + MAX_HISTORY_ENTRIES) % MAX_HISTORY_ENTRIES;
    
    for (uint32_t i = 0; i < g_logger.count && *count_out < max_entries; i++) {
        uint32_t idx = (start_idx + i) % MAX_HISTORY_ENTRIES;
        sensor_history_entry_t *entry = &g_logger.entries[idx];
        
        // Check if entry is within time range
        if ((start_timestamp == 0 || entry->timestamp >= start_timestamp) &&
            (entry->timestamp <= end_timestamp)) {
            entries[*count_out] = *entry;
            (*count_out)++;
        }
    }
    
    ESP_LOGI(TAG, "Retrieved %lu entries (range: %lld to %lld)", 
             *count_out, start_timestamp, end_timestamp);
    
    return ESP_OK;
}

/**
 * @brief Load history from JSON file
 */
static esp_err_t load_history_from_file(void) {
    // Check if file exists
    struct stat st;
    if (stat(HISTORY_FILE_PATH, &st) != 0) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Open file
    FILE *file = fopen(HISTORY_FILE_PATH, "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open history file for reading");
        return ESP_FAIL;
    }
    
    // Get file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if (file_size <= 0 || file_size > 1024 * 1024) {  // Sanity check: max 1MB
        ESP_LOGW(TAG, "Invalid file size: %ld", file_size);
        fclose(file);
        return ESP_FAIL;
    }
    
    // Read file content
    char *json_str = malloc(file_size + 1);
    if (json_str == NULL) {
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    
    size_t read_size = fread(json_str, 1, file_size, file);
    fclose(file);
    json_str[read_size] = '\0';
    
    // Parse JSON
    cJSON *root = cJSON_Parse(json_str);
    free(json_str);
    
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse history JSON");
        return ESP_FAIL;
    }
    
    // Extract metadata
    cJSON *count_json = cJSON_GetObjectItem(root, "count");
    cJSON *head_json = cJSON_GetObjectItem(root, "head");
    cJSON *readings_json = cJSON_GetObjectItem(root, "readings");
    
    if (!cJSON_IsNumber(count_json) || !cJSON_IsNumber(head_json) || 
        !cJSON_IsArray(readings_json)) {
        ESP_LOGE(TAG, "Invalid history file format");
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    
    g_logger.count = count_json->valueint;
    g_logger.head = head_json->valueint;
    
    // Load entries
    cJSON *entry_json = NULL;
    uint32_t idx = 0;
    cJSON_ArrayForEach(entry_json, readings_json) {
        if (idx >= MAX_HISTORY_ENTRIES) break;
        
        cJSON *ts = cJSON_GetObjectItem(entry_json, "ts");
        cJSON *temp = cJSON_GetObjectItem(entry_json, "temp_c");
        cJSON *humid = cJSON_GetObjectItem(entry_json, "humid_rh");
        cJSON *lux = cJSON_GetObjectItem(entry_json, "lux");
        cJSON *visible = cJSON_GetObjectItem(entry_json, "visible");
        cJSON *infrared = cJSON_GetObjectItem(entry_json, "infrared");
        cJSON *power = cJSON_GetObjectItem(entry_json, "power_mw");
        cJSON *current = cJSON_GetObjectItem(entry_json, "current_ma");
        cJSON *voltage = cJSON_GetObjectItem(entry_json, "voltage_mv");
        cJSON *water = cJSON_GetObjectItem(entry_json, "water_mm");
        
        if (cJSON_IsNumber(ts)) {
            g_logger.entries[idx].timestamp = (int64_t)ts->valuedouble;
            g_logger.entries[idx].temperature_c = cJSON_IsNumber(temp) ? temp->valuedouble : -999.0f;
            g_logger.entries[idx].humidity_rh = cJSON_IsNumber(humid) ? humid->valuedouble : -999.0f;
            g_logger.entries[idx].light_lux = cJSON_IsNumber(lux) ? lux->valuedouble : -999.0f;
            g_logger.entries[idx].light_visible = cJSON_IsNumber(visible) ? visible->valueint : 0;
            g_logger.entries[idx].light_infrared = cJSON_IsNumber(infrared) ? infrared->valueint : 0;
            g_logger.entries[idx].power_mw = cJSON_IsNumber(power) ? power->valuedouble : 0.0f;
            g_logger.entries[idx].current_ma = cJSON_IsNumber(current) ? current->valuedouble : 0.0f;
            g_logger.entries[idx].voltage_mv = cJSON_IsNumber(voltage) ? voltage->valuedouble : 0.0f;
            g_logger.entries[idx].water_level_mm = cJSON_IsNumber(water) ? water->valuedouble : -1.0f;
            idx++;
        }
    }
    
    cJSON_Delete(root);
    
    ESP_LOGI(TAG, "Loaded %lu entries from history file", idx);
    return ESP_OK;
}

/**
 * @brief Save history to JSON file
 */
static esp_err_t save_history_to_file(void) {
    ESP_LOGD(TAG, "Saving history to file (%lu entries)", g_logger.count);
    
    // Create JSON root
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    // Add metadata
    cJSON_AddNumberToObject(root, "version", 1);
    cJSON_AddNumberToObject(root, "count", g_logger.count);
    cJSON_AddNumberToObject(root, "head", g_logger.head);
    cJSON_AddNumberToObject(root, "max_entries", MAX_HISTORY_ENTRIES);
    cJSON_AddNumberToObject(root, "saved_at", get_current_timestamp());
    
    // Create readings array
    cJSON *readings = cJSON_CreateArray();
    if (readings == NULL) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    
    // Add entries in chronological order (oldest first)
    uint32_t start_idx = (g_logger.head >= g_logger.count) ? 
                         0 : (g_logger.head - g_logger.count + MAX_HISTORY_ENTRIES) % MAX_HISTORY_ENTRIES;
    
    for (uint32_t i = 0; i < g_logger.count; i++) {
        uint32_t idx = (start_idx + i) % MAX_HISTORY_ENTRIES;
        sensor_history_entry_t *entry = &g_logger.entries[idx];
        
        cJSON *entry_json = cJSON_CreateObject();
        if (entry_json == NULL) continue;
        
        cJSON_AddNumberToObject(entry_json, "ts", entry->timestamp);
        cJSON_AddNumberToObject(entry_json, "temp_c", entry->temperature_c);
        cJSON_AddNumberToObject(entry_json, "humid_rh", entry->humidity_rh);
        cJSON_AddNumberToObject(entry_json, "lux", entry->light_lux);
        cJSON_AddNumberToObject(entry_json, "visible", entry->light_visible);
        cJSON_AddNumberToObject(entry_json, "infrared", entry->light_infrared);
        cJSON_AddNumberToObject(entry_json, "power_mw", entry->power_mw);
        cJSON_AddNumberToObject(entry_json, "current_ma", entry->current_ma);
        cJSON_AddNumberToObject(entry_json, "voltage_mv", entry->voltage_mv);
        cJSON_AddNumberToObject(entry_json, "water_mm", entry->water_level_mm);
        
        cJSON_AddItemToArray(readings, entry_json);
    }
    
    cJSON_AddItemToObject(root, "readings", readings);
    
    // Convert to string
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    // Write to file
    FILE *file = fopen(HISTORY_FILE_PATH, "w");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open history file for writing");
        free(json_str);
        return ESP_FAIL;
    }
    
    size_t len = strlen(json_str);
    size_t written = fwrite(json_str, 1, len, file);
    fclose(file);
    free(json_str);
    
    if (written != len) {
        ESP_LOGE(TAG, "Failed to write complete history file");
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "History saved successfully (%zu bytes)", written);
    return ESP_OK;
}

/**
 * @brief Get logger statistics
 */
esp_err_t sensor_logger_get_stats(sensor_logger_stats_t *stats) {
    if (!g_logger.initialized || stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    stats->total_entries = g_logger.total_entries;
    stats->write_errors = g_logger.write_errors;
    stats->current_count = g_logger.count;
    stats->buffer_wraps = g_logger.buffer_wraps;
    
    // Find oldest and newest timestamps
    if (g_logger.count > 0) {
        uint32_t oldest_idx = (g_logger.head >= g_logger.count) ? 
                              0 : (g_logger.head - g_logger.count + MAX_HISTORY_ENTRIES) % MAX_HISTORY_ENTRIES;
        uint32_t newest_idx = (g_logger.head == 0) ? MAX_HISTORY_ENTRIES - 1 : g_logger.head - 1;
        
        stats->oldest_timestamp = g_logger.entries[oldest_idx].timestamp;
        stats->newest_timestamp = g_logger.entries[newest_idx].timestamp;
    } else {
        stats->oldest_timestamp = 0;
        stats->newest_timestamp = 0;
    }
    
    return ESP_OK;
}

/**
 * @brief Clear all historical data
 */
esp_err_t sensor_logger_clear_history(void) {
    if (!g_logger.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Clearing history");
    
    g_logger.head = 0;
    g_logger.count = 0;
    memset(g_logger.entries, 0, MAX_HISTORY_ENTRIES * sizeof(sensor_history_entry_t));
    
    // Delete file
    remove(HISTORY_FILE_PATH);
    
    return ESP_OK;
}

/**
 * @brief Print debug information
 */
void sensor_logger_print_debug_info(void) {
    if (!g_logger.initialized) {
        ESP_LOGI(TAG, "Sensor logger not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "=== Sensor Logger Debug Info ===");
    ESP_LOGI(TAG, "Total entries logged: %lu", g_logger.total_entries);
    ESP_LOGI(TAG, "Current buffer count: %lu / %d", g_logger.count, MAX_HISTORY_ENTRIES);
    ESP_LOGI(TAG, "Buffer head position: %lu", g_logger.head);
    ESP_LOGI(TAG, "Buffer wraps: %lu", g_logger.buffer_wraps);
    ESP_LOGI(TAG, "Write errors: %lu", g_logger.write_errors);
    
    if (g_logger.count > 0) {
        uint32_t oldest_idx = (g_logger.head >= g_logger.count) ? 
                              0 : (g_logger.head - g_logger.count + MAX_HISTORY_ENTRIES) % MAX_HISTORY_ENTRIES;
        uint32_t newest_idx = (g_logger.head == 0) ? MAX_HISTORY_ENTRIES - 1 : g_logger.head - 1;
        
        ESP_LOGI(TAG, "Oldest entry: timestamp=%lld", g_logger.entries[oldest_idx].timestamp);
        ESP_LOGI(TAG, "Newest entry: timestamp=%lld", g_logger.entries[newest_idx].timestamp);
        ESP_LOGI(TAG, "Time span: %.1f hours", 
                 (double)(g_logger.entries[newest_idx].timestamp - g_logger.entries[oldest_idx].timestamp) / 3600.0);
    }
}
