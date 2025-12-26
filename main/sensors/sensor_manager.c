/**
 * @file sensor_manager.c
 * @brief Implementation of centralized sensor management system
 */

#include "sensor_manager.h"
#include "sht45_sensor.h"
#include "../ina_power_monitor/power_monitor_HAL.h"
#include "distance_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include <string.h>

static const char *TAG = "SENSOR_MGR";

// I2C Configuration - matches board configuration (ina219.h)
// Note: Reduced to 100kHz for better sensor compatibility (was 400kHz)
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       42    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_SCL_IO       41    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_FREQ_HZ      100000 /*!< I2C master clock frequency */

// Sensor manager state
typedef struct {
    TaskHandle_t task_handle;
    QueueHandle_t request_queue;
    SemaphoreHandle_t cache_mutex;
    SemaphoreHandle_t i2c_mutex;
    sensor_cache_entry_t cache[SENSOR_TYPE_MAX];
    sensor_priority_t priorities[SENSOR_TYPE_MAX];
    bool enabled[SENSOR_TYPE_MAX];
    bool running;
    sensor_manager_config_t config;
    
    // Statistics
    uint32_t total_reads;
    uint32_t cache_hits;
    uint32_t cache_misses;
    uint32_t errors;
    uint32_t poll_cycles;
} sensor_manager_state_t;

static sensor_manager_state_t g_sensor_mgr = {0};

// Forward declarations
static void sensor_manager_task(void *arg);
static esp_err_t read_power_sensor(sensor_type_t type, float *value);
static esp_err_t read_sht45_sensor(sensor_type_t type, environment_data_t *env_data);
static esp_err_t read_water_sensor(float *value);
static esp_err_t update_sensor_cache(sensor_type_t type);
static bool should_poll_sensor(sensor_type_t type, uint32_t cycle_count);
static const char* sensor_type_to_string(sensor_type_t type);
static const char* priority_to_string(sensor_priority_t priority);

/**
 * @brief Get default configuration
 */
sensor_manager_config_t sensor_manager_get_default_config(void) {
    sensor_manager_config_t config = {
        .task_stack_size = 4096,
        .task_priority = 5,
        .poll_period_ms = 1000,
        .cache_timeout_ms = 5000,
        .request_queue_size = 10
    };
    return config;
}

/**
 * @brief Initialize I2C bus
 */
static esp_err_t init_i2c_bus(void) {
    static bool initialized = false;
    if (initialized) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing I2C bus (SDA=%d, SCL=%d, freq=%dHz)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "I2C driver already installed");
        initialized = true;
        return ESP_OK;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    initialized = true;
    ESP_LOGI(TAG, "I2C bus initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize sensor manager
 */
esp_err_t sensor_manager_init(const sensor_manager_config_t *config) {
    ESP_LOGI(TAG, "Initializing sensor manager");
    
    // Use default config if none provided
    if (config == NULL) {
        g_sensor_mgr.config = sensor_manager_get_default_config();
    } else {
        g_sensor_mgr.config = *config;
    }
    
    // Initialize I2C bus
    esp_err_t ret = init_i2c_bus();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Create mutexes
    g_sensor_mgr.cache_mutex = xSemaphoreCreateMutex();
    g_sensor_mgr.i2c_mutex = xSemaphoreCreateMutex();
    
    if (g_sensor_mgr.cache_mutex == NULL || g_sensor_mgr.i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return ESP_ERR_NO_MEM;
    }
    
    // Create request queue
    g_sensor_mgr.request_queue = xQueueCreate(
        g_sensor_mgr.config.request_queue_size,
        sizeof(sensor_request_t)
    );
    
    if (g_sensor_mgr.request_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create request queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize cache
    memset(g_sensor_mgr.cache, 0, sizeof(g_sensor_mgr.cache));
    
    // Set default priorities
    g_sensor_mgr.priorities[SENSOR_TYPE_POWER_CURRENT] = SENSOR_PRIORITY_HIGH;
    g_sensor_mgr.priorities[SENSOR_TYPE_POWER_VOLTAGE] = SENSOR_PRIORITY_HIGH;
    g_sensor_mgr.priorities[SENSOR_TYPE_POWER_POWER] = SENSOR_PRIORITY_HIGH;
    g_sensor_mgr.priorities[SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY] = SENSOR_PRIORITY_CRITICAL;
    g_sensor_mgr.priorities[SENSOR_TYPE_WATER_LEVEL] = SENSOR_PRIORITY_MEDIUM;
    
    // Enable all sensors by default
    for (int i = 0; i < SENSOR_TYPE_MAX; i++) {
        g_sensor_mgr.enabled[i] = true;
    }
    
    // Initialize sensors
    ESP_LOGI(TAG, "Initializing power monitor");
    ret = power_monitor_init(POWER_MONITOR_CHIP_INA219, 0x40);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Power monitor init failed (may not be connected): %s", esp_err_to_name(ret));
    }
    
    // Initialize SHT45 (may not be connected yet during development)
    ESP_LOGI(TAG, "Initializing SHT45 temperature/humidity sensor");
    ret = sht45_init(I2C_MASTER_NUM, SHT45_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SHT45 init failed (may not be connected): %s", esp_err_to_name(ret));
        // Disable environment sensor if init fails
        g_sensor_mgr.enabled[SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY] = false;
    }
    
    ESP_LOGI(TAG, "Sensor manager initialized successfully");
    return ESP_OK;
}

/**
 * @brief Start sensor manager task
 */
esp_err_t sensor_manager_start(void) {
    if (g_sensor_mgr.running) {
        ESP_LOGW(TAG, "Sensor manager already running");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting sensor manager task");
    
    // Set running flag BEFORE creating task to avoid race condition
    // (new task may run immediately and check this flag)
    g_sensor_mgr.running = true;
    
    BaseType_t ret = xTaskCreate(
        sensor_manager_task,
        "sensor_mgr",
        g_sensor_mgr.config.task_stack_size,
        NULL,
        g_sensor_mgr.config.task_priority,
        &g_sensor_mgr.task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor manager task");
        g_sensor_mgr.running = false;  // Reset flag on failure
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Sensor manager task started");
    return ESP_OK;
}

/**
 * @brief Stop sensor manager task
 */
esp_err_t sensor_manager_stop(void) {
    if (!g_sensor_mgr.running) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping sensor manager task");
    g_sensor_mgr.running = false;
    
    if (g_sensor_mgr.task_handle != NULL) {
        vTaskDelete(g_sensor_mgr.task_handle);
        g_sensor_mgr.task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Sensor manager task stopped");
    return ESP_OK;
}

/**
 * @brief Check if sensor should be polled this cycle
 */
static bool should_poll_sensor(sensor_type_t type, uint32_t cycle_count) {
    if (!g_sensor_mgr.enabled[type]) {
        return false;
    }
    
    sensor_priority_t priority = g_sensor_mgr.priorities[type];
    
    switch (priority) {
        case SENSOR_PRIORITY_CRITICAL:
            return true;  // Every cycle
        case SENSOR_PRIORITY_HIGH:
            return (cycle_count % 5) == 0;  // Every 5 cycles
        case SENSOR_PRIORITY_MEDIUM:
            return (cycle_count % 10) == 0;  // Every 10 cycles
        case SENSOR_PRIORITY_LOW:
            return (cycle_count % 20) == 0;  // Every 20 cycles
        default:
            return false;
    }
}

/**
 * @brief Read power monitor sensor
 */
static esp_err_t read_power_sensor(sensor_type_t type, float *value) {
    ESP_LOGD(TAG, "[I2C] Reading power sensor: %s", sensor_type_to_string(type));
    
    esp_err_t ret;
    switch (type) {
        case SENSOR_TYPE_POWER_CURRENT:
            ret = power_monitor_read_current(value);
            break;
        case SENSOR_TYPE_POWER_VOLTAGE:
            ret = power_monitor_read_voltage(value);
            break;
        case SENSOR_TYPE_POWER_POWER:
            ret = power_monitor_read_power(value);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "[I2C] Power sensor read OK: %.2f", *value);
    } else {
        ESP_LOGW(TAG, "[I2C] Power sensor read failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Read SHT45 sensor (returns both temperature and humidity)
 */
static esp_err_t read_sht45_sensor(sensor_type_t type, environment_data_t *env_data) {
    ESP_LOGD(TAG, "[I2C] Reading SHT45 sensor: %s", sensor_type_to_string(type));
    
    if (type != SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY) {
        return ESP_ERR_INVALID_ARG;
    }
    
    sht45_data_t data;
    esp_err_t ret = sht45_read_data(I2C_MASTER_NUM, SHT45_I2C_ADDR, 
                                    SHT45_PRECISION_HIGH, &data);
    
    if (ret == ESP_OK && data.valid) {
        env_data->temperature_c = data.temperature_c;
        env_data->humidity_rh = data.humidity_rh;
        ESP_LOGD(TAG, "[I2C] SHT45 read OK: %.2f°C, %.2f%%RH", 
                 env_data->temperature_c, env_data->humidity_rh);
    } else {
        ESP_LOGW(TAG, "[I2C] SHT45 read failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Read water level sensor
 */
static esp_err_t read_water_sensor(float *value) {
    ESP_LOGD(TAG, "[I2C] Reading water level sensor");
    
    // TODO: Implement water sensor reading when ready
    // For now, return a placeholder value
    *value = 0.0f;
    
    ESP_LOGD(TAG, "[I2C] Water level (placeholder): %.2f mm", *value);
    return ESP_OK;
}

/**
 * @brief Update sensor cache with fresh reading
 */
static esp_err_t update_sensor_cache(sensor_type_t type) {
    esp_err_t ret = ESP_FAIL;
    sensor_data_t sensor_data = {0};
    
    // Take I2C mutex for hardware access
    if (xSemaphoreTake(g_sensor_mgr.i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take I2C mutex for %s", sensor_type_to_string(type));
        g_sensor_mgr.errors++;
        return ESP_ERR_TIMEOUT;
    }
    
    // Read from appropriate sensor
    switch (type) {
        case SENSOR_TYPE_POWER_CURRENT:
        case SENSOR_TYPE_POWER_VOLTAGE:
        case SENSOR_TYPE_POWER_POWER:
            ret = read_power_sensor(type, &sensor_data.power.value);
            break;
            
        case SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY:
            ret = read_sht45_sensor(type, &sensor_data.environment);
            break;
            
        case SENSOR_TYPE_WATER_LEVEL:
            ret = read_water_sensor(&sensor_data.water_level.level_mm);
            break;
            
        default:
            ESP_LOGE(TAG, "Unknown sensor type: %d", type);
            ret = ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreGive(g_sensor_mgr.i2c_mutex);
    
    // Update cache with result
    if (xSemaphoreTake(g_sensor_mgr.cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_sensor_mgr.cache[type].data = sensor_data;
        g_sensor_mgr.cache[type].type = type;
        g_sensor_mgr.cache[type].timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        g_sensor_mgr.cache[type].valid = (ret == ESP_OK);
        g_sensor_mgr.cache[type].last_error = ret;
        xSemaphoreGive(g_sensor_mgr.cache_mutex);
    }
    
    g_sensor_mgr.total_reads++;
    if (ret != ESP_OK) {
        g_sensor_mgr.errors++;
    }
    
    return ret;
}

/**
 * @brief Main sensor manager task
 */
static void sensor_manager_task(void *arg) {
    ESP_LOGI(TAG, "Sensor manager task running");
    
    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Initial delay to allow system stabilization

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t poll_period = pdMS_TO_TICKS(g_sensor_mgr.config.poll_period_ms);
    
    while (g_sensor_mgr.running) {
        g_sensor_mgr.poll_cycles++;
        uint32_t cycle = g_sensor_mgr.poll_cycles;
        
        ESP_LOGD(TAG, "=== Poll cycle %lu ===", cycle);
        
        // Poll sensors based on priority
        for (int type = 0; type < SENSOR_TYPE_MAX; type++) {
            if (should_poll_sensor(type, cycle)) {
                ESP_LOGD(TAG, "Polling %s (priority=%s, cycle=%lu)",
                         sensor_type_to_string(type),
                         priority_to_string(g_sensor_mgr.priorities[type]),
                         cycle);
                update_sensor_cache(type);
                
                // Add a small delay between sensor reads to prevent I2C bus contention
                // This is especially important when multiple sensors are polled in the same cycle
                vTaskDelay(pdMS_TO_TICKS(25));
            }
        }
        
        // Process any pending requests (non-blocking check)
        sensor_request_t request;
        while (xQueueReceive(g_sensor_mgr.request_queue, &request, 0) == pdTRUE) {
            ESP_LOGD(TAG, "Processing request for %s", sensor_type_to_string(request.sensor_type));
            
            // Get cached value
            esp_err_t ret = sensor_manager_get_cached(
                request.sensor_type,
                request.value_out,
                NULL
            );
            
            if (request.error_out != NULL) {
                *request.error_out = ret;
            }
            
            // Signal completion
            if (request.done_sem != NULL) {
                xSemaphoreGive(request.done_sem);
            }
        }
        
        // Sleep until next poll period
        vTaskDelayUntil(&last_wake_time, poll_period);
    }
    
    ESP_LOGI(TAG, "Sensor manager task exiting");
    vTaskDelete(NULL);
}

/**
 * @brief Request sensor reading
 */
esp_err_t sensor_manager_read(sensor_type_t sensor_type, float *value, uint32_t timeout_ms) {
    if (value == NULL || sensor_type >= SENSOR_TYPE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_sensor_mgr.running) {
        ESP_LOGW(TAG, "Sensor manager not running");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGD(TAG, "API request for %s (timeout=%lums)", 
             sensor_type_to_string(sensor_type), timeout_ms);
    
    // Check if cached data is fresh enough
    uint32_t timestamp_ms;
    esp_err_t ret = sensor_manager_get_cached(sensor_type, value, &timestamp_ms);
    
    if (ret == ESP_OK) {
        uint32_t age_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp_ms;
        if (age_ms < g_sensor_mgr.config.cache_timeout_ms) {
            ESP_LOGD(TAG, "Cache hit for %s (age=%lums)", sensor_type_to_string(sensor_type), age_ms);
            g_sensor_mgr.cache_hits++;
            return ESP_OK;
        }
        // Cache exists but is stale
        ESP_LOGW(TAG, "Cache miss for %s: data stale (age=%lums > timeout=%lums)", 
                 sensor_type_to_string(sensor_type), age_ms, g_sensor_mgr.config.cache_timeout_ms);
    } else {
        // No valid cache entry
        const char* reason = (ret == ESP_ERR_NOT_FOUND) ? "no valid data" : "cache access failed";
        ESP_LOGW(TAG, "Cache miss for %s: %s (%s)", 
                 sensor_type_to_string(sensor_type), reason, esp_err_to_name(ret));
    }
    
    g_sensor_mgr.cache_misses++;
    ESP_LOGW(TAG, "Queuing request for fresh data from %s (total misses: %lu)", 
             sensor_type_to_string(sensor_type), g_sensor_mgr.cache_misses);
    
    // Create request
    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (done_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    sensor_request_t request = {
        .sensor_type = sensor_type,
        .value_out = value,
        .error_out = &ret,
        .done_sem = done_sem
    };
    
    // Send request to sensor task
    if (xQueueSend(g_sensor_mgr.request_queue, &request, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        vSemaphoreDelete(done_sem);
        ESP_LOGW(TAG, "Request queue full for %s", sensor_type_to_string(sensor_type));
        return ESP_ERR_TIMEOUT;
    }
    
    // Wait for completion
    if (xSemaphoreTake(done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        vSemaphoreDelete(done_sem);
        ESP_LOGW(TAG, "Request timeout for %s", sensor_type_to_string(sensor_type));
        return ESP_ERR_TIMEOUT;
    }
    
    vSemaphoreDelete(done_sem);
    
    ESP_LOGW(TAG, "Request completed for %s: value=%.2f, error=%s",
             sensor_type_to_string(sensor_type), *value, esp_err_to_name(ret));
    
    return ret;
}

/**
 * @brief Get cached sensor reading
 */
esp_err_t sensor_manager_get_cached(sensor_type_t sensor_type, float *value, uint32_t *timestamp_ms) {
    if (value == NULL || sensor_type >= SENSOR_TYPE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(g_sensor_mgr.cache_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    sensor_cache_entry_t *entry = &g_sensor_mgr.cache[sensor_type];
    
    if (!entry->valid) {
        xSemaphoreGive(g_sensor_mgr.cache_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Extract single value from appropriate union member
    switch (sensor_type) {
        case SENSOR_TYPE_POWER_CURRENT:
        case SENSOR_TYPE_POWER_VOLTAGE:
        case SENSOR_TYPE_POWER_POWER:
            *value = entry->data.power.value;
            break;
            
        case SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY:
            // For combined sensors, return temperature as primary value
            *value = entry->data.environment.temperature_c;
            break;
            
        case SENSOR_TYPE_WATER_LEVEL:
            *value = entry->data.water_level.level_mm;
            break;
            
        default:
            xSemaphoreGive(g_sensor_mgr.cache_mutex);
            return ESP_ERR_INVALID_ARG;
    }
    
    if (timestamp_ms != NULL) {
        *timestamp_ms = entry->timestamp_ms;
    }
    
    xSemaphoreGive(g_sensor_mgr.cache_mutex);
    return ESP_OK;
}

/**
 * @brief Get cached environment sensor reading (temperature and humidity)
 */
esp_err_t sensor_manager_get_environment_cached(float *temperature_c, float *humidity_rh, uint32_t *timestamp_ms) {
    if (xSemaphoreTake(g_sensor_mgr.cache_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    sensor_cache_entry_t *entry = &g_sensor_mgr.cache[SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY];
    
    if (!entry->valid) {
        xSemaphoreGive(g_sensor_mgr.cache_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    
    if (temperature_c != NULL) {
        *temperature_c = entry->data.environment.temperature_c;
    }
    if (humidity_rh != NULL) {
        *humidity_rh = entry->data.environment.humidity_rh;
    }
    if (timestamp_ms != NULL) {
        *timestamp_ms = entry->timestamp_ms;
    }
    
    xSemaphoreGive(g_sensor_mgr.cache_mutex);
    return ESP_OK;
}

/**
 * @brief Get full cached sensor data structure
 */
esp_err_t sensor_manager_get_data_cached(sensor_type_t sensor_type, sensor_data_t *data, uint32_t *timestamp_ms) {
    if (data == NULL || sensor_type >= SENSOR_TYPE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(g_sensor_mgr.cache_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    sensor_cache_entry_t *entry = &g_sensor_mgr.cache[sensor_type];
    
    if (!entry->valid) {
        xSemaphoreGive(g_sensor_mgr.cache_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    
    *data = entry->data;
    if (timestamp_ms != NULL) {
        *timestamp_ms = entry->timestamp_ms;
    }
    
    xSemaphoreGive(g_sensor_mgr.cache_mutex);
    return ESP_OK;
}

/**
 * @brief Force immediate reading
 */
esp_err_t sensor_manager_read_forced(sensor_type_t sensor_type, float *value, uint32_t timeout_ms) {
    if (value == NULL || sensor_type >= SENSOR_TYPE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Forced read request for %s", sensor_type_to_string(sensor_type));
    
    // Directly update cache (bypasses normal polling schedule)
    esp_err_t ret = update_sensor_cache(sensor_type);
    
    if (ret == ESP_OK) {
        return sensor_manager_get_cached(sensor_type, value, NULL);
    }
    
    return ret;
}

/**
 * @brief Get statistics
 */
void sensor_manager_get_stats(uint32_t *total_reads, uint32_t *cache_hits,
                              uint32_t *cache_misses, uint32_t *errors) {
    if (total_reads) *total_reads = g_sensor_mgr.total_reads;
    if (cache_hits) *cache_hits = g_sensor_mgr.cache_hits;
    if (cache_misses) *cache_misses = g_sensor_mgr.cache_misses;
    if (errors) *errors = g_sensor_mgr.errors;
}

/**
 * @brief Print debug information
 */
void sensor_manager_print_debug_info(void) {
    ESP_LOGI(TAG, "=== Sensor Manager Debug Info ===");
    ESP_LOGI(TAG, "Running: %s", g_sensor_mgr.running ? "YES" : "NO");
    ESP_LOGI(TAG, "Poll cycles: %lu", g_sensor_mgr.poll_cycles);
    ESP_LOGI(TAG, "Total reads: %lu", g_sensor_mgr.total_reads);
    ESP_LOGI(TAG, "Cache hits: %lu", g_sensor_mgr.cache_hits);
    ESP_LOGI(TAG, "Cache misses: %lu", g_sensor_mgr.cache_misses);
    ESP_LOGI(TAG, "Errors: %lu", g_sensor_mgr.errors);
    
    ESP_LOGI(TAG, "\nSensor Status:");
    for (int i = 0; i < SENSOR_TYPE_MAX; i++) {
        if (xSemaphoreTake(g_sensor_mgr.cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            sensor_cache_entry_t *entry = &g_sensor_mgr.cache[i];
            
            // Format sensor-specific data
            char value_str[100];
            switch (i) {
                case SENSOR_TYPE_POWER_CURRENT:
                case SENSOR_TYPE_POWER_VOLTAGE:
                case SENSOR_TYPE_POWER_POWER:
                    snprintf(value_str, sizeof(value_str), "%.2f", entry->data.power.value);
                    break;
                    
                case SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY:
                    {
                        float temp_c = entry->data.environment.temperature_c;
                        float temp_f = (temp_c * 9.0f / 5.0f) + 32.0f;
                        snprintf(value_str, sizeof(value_str), "%.2f°C (%.2f°F), %.2f%%RH", 
                                 temp_c, temp_f, entry->data.environment.humidity_rh);
                    }
                    break;
                    
                case SENSOR_TYPE_WATER_LEVEL:
                    snprintf(value_str, sizeof(value_str), "%.2f mm", entry->data.water_level.level_mm);
                    break;
                    
                default:
                    snprintf(value_str, sizeof(value_str), "N/A");
                    break;
            }
            
            ESP_LOGI(TAG, "  %s: %s, priority=%s, value=%s, age=%lums",
                     sensor_type_to_string(i),
                     g_sensor_mgr.enabled[i] ? (entry->valid ? "VALID" : "INVALID") : "DISABLED",
                     priority_to_string(g_sensor_mgr.priorities[i]),
                     value_str,
                     (xTaskGetTickCount() * portTICK_PERIOD_MS) - entry->timestamp_ms);
            xSemaphoreGive(g_sensor_mgr.cache_mutex);
        }
    }
}

/**
 * @brief Set sensor priority
 */
esp_err_t sensor_manager_set_priority(sensor_type_t sensor_type, sensor_priority_t priority) {
    if (sensor_type >= SENSOR_TYPE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_sensor_mgr.priorities[sensor_type] = priority;
    ESP_LOGI(TAG, "Set priority for %s to %s",
             sensor_type_to_string(sensor_type),
             priority_to_string(priority));
    return ESP_OK;
}

/**
 * @brief Enable/disable sensor
 */
esp_err_t sensor_manager_set_enabled(sensor_type_t sensor_type, bool enabled) {
    if (sensor_type >= SENSOR_TYPE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_sensor_mgr.enabled[sensor_type] = enabled;
    ESP_LOGI(TAG, "%s sensor %s",
             enabled ? "Enabled" : "Disabled",
             sensor_type_to_string(sensor_type));
    return ESP_OK;
}

/**
 * @brief Check if running
 */
bool sensor_manager_is_running(void) {
    return g_sensor_mgr.running;
}

/**
 * @brief Helper: Sensor type to string
 */
static const char* sensor_type_to_string(sensor_type_t type) {
    switch (type) {
        case SENSOR_TYPE_POWER_CURRENT: return "POWER_CURRENT";
        case SENSOR_TYPE_POWER_VOLTAGE: return "POWER_VOLTAGE";
        case SENSOR_TYPE_POWER_POWER: return "POWER_POWER";
        case SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY: return "TEMPERATURE_AND_HUMIDITY";
        case SENSOR_TYPE_WATER_LEVEL: return "WATER_LEVEL";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Helper: Priority to string
 */
static const char* priority_to_string(sensor_priority_t priority) {
    switch (priority) {
        case SENSOR_PRIORITY_CRITICAL: return "CRITICAL(1s)";
        case SENSOR_PRIORITY_HIGH: return "HIGH(5s)";
        case SENSOR_PRIORITY_MEDIUM: return "MEDIUM(10s)";
        case SENSOR_PRIORITY_LOW: return "LOW(20s)";
        default: return "UNKNOWN";
    }
}
