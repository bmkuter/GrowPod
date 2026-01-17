/**
 * @file sensor_manager.c
 * @brief Implementation of centralized sensor management system
 */

#include "sensor_manager.h"
#include "sensor_logger.h"
#include "sensor_config.h"
#include "sht45_sensor.h"
#include "tsl2591_sensor.h"
#include "i2c_scanner.h"
#include "../ina_power_monitor/power_monitor_HAL.h"
#include "distance_sensor.h"
#include "fdc1004_distance_sensor.h"
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
    uint8_t poll_offsets[SENSOR_TYPE_MAX];  // Stagger offset for each sensor
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
static esp_err_t read_tsl2591_sensor(sensor_type_t type, light_data_t *light_data);
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
        .poll_period_ms = 100,  // Base period 100ms for faster polling
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
    g_sensor_mgr.priorities[SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY] = SENSOR_PRIORITY_LOW;
    g_sensor_mgr.priorities[SENSOR_TYPE_LIGHT] = SENSOR_PRIORITY_MEDIUM;
    g_sensor_mgr.priorities[SENSOR_TYPE_WATER_LEVEL] = SENSOR_PRIORITY_MEDIUM;
    
    // Calculate staggered poll offsets to spread I2C load
    // Group sensors by priority and assign sequential offsets within each group
    uint8_t offset_counters[4] = {0};  // One counter per priority level
    for (int i = 0; i < SENSOR_TYPE_MAX; i++) {
        sensor_priority_t prio = g_sensor_mgr.priorities[i];
        g_sensor_mgr.poll_offsets[i] = offset_counters[prio]++;
    }
    
    // Enable all sensors by default
    for (int i = 0; i < SENSOR_TYPE_MAX; i++) {
        g_sensor_mgr.enabled[i] = true;
    }
    
    // Log staggered schedule
    ESP_LOGI(TAG, "Staggered polling schedule:");
    for (int i = 0; i < SENSOR_TYPE_MAX; i++) {
        ESP_LOGI(TAG, "  %s: priority=%s, offset=%d",
                 sensor_type_to_string(i),
                 priority_to_string(g_sensor_mgr.priorities[i]),
                 g_sensor_mgr.poll_offsets[i]);
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
    
    // Initialize TSL2591 light sensor
    ESP_LOGI(TAG, "Initializing TSL2591 light sensor");
    ret = tsl2591_init(I2C_MASTER_NUM, TSL2591_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TSL2591 init failed (may not be connected): %s", esp_err_to_name(ret));
        // Disable light sensor if init fails
        g_sensor_mgr.enabled[SENSOR_TYPE_LIGHT] = false;
    }
    
    // Initialize FDC1004 capacitive distance sensor
    ESP_LOGI(TAG, "Initializing FDC1004 capacitive distance sensor");
    ret = fdc1004_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "FDC1004 init failed (may not be connected): %s", esp_err_to_name(ret));
        // Disable water level sensor if init fails
        g_sensor_mgr.enabled[SENSOR_TYPE_WATER_LEVEL] = false;
    } else {
        // Try to load saved calibration from filesystem
        ESP_LOGI(TAG, "Loading FDC1004 calibration from filesystem...");
        sensor_config_init();
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
 * @brief Check if sensor should be polled this cycle (with staggered scheduling)
 * 
 * This function implements a staggered polling schedule to avoid I2C bus congestion.
 * Instead of all sensors with the same priority polling on the same cycles,
 * each sensor is assigned a unique offset to spread the load evenly.
 * 
 * Example: HIGH priority (every 5 cycles) with 3 sensors:
 *   - Sensor A: cycles 0, 5, 10, 15... (offset 0)
 *   - Sensor B: cycles 1, 6, 11, 16... (offset 1)
 *   - Sensor C: cycles 2, 7, 12, 17... (offset 2)
 */
static bool should_poll_sensor(sensor_type_t type, uint32_t cycle_count) {
    if (!g_sensor_mgr.enabled[type]) {
        return false;
    }
    
    sensor_priority_t priority = g_sensor_mgr.priorities[type];
    uint8_t offset = g_sensor_mgr.poll_offsets[type];
    uint32_t interval;
    
    switch (priority) {
        case SENSOR_PRIORITY_CRITICAL:
            interval = 5;   // Every 5 cycles (~500ms)
            break;
        case SENSOR_PRIORITY_HIGH:
            interval = 10;  // Every 10 cycles (~1s)
            break;
        case SENSOR_PRIORITY_MEDIUM:
            interval = 25;  // Every 25 cycles (~2.5s)
            break;
        case SENSOR_PRIORITY_LOW:
            interval = 50;  // Every 50 cycles (~5s)
            break;
        default:
            return false;
    }
    
    // Check if (cycle_count - offset) is divisible by interval
    // This staggers sensors across different cycles
    return ((cycle_count >= offset) && ((cycle_count - offset) % interval) == 0);
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
 * @brief Read TSL2591 light sensor
 */
static esp_err_t read_tsl2591_sensor(sensor_type_t type, light_data_t *light_data) {
    ESP_LOGD(TAG, "[I2C] Reading TSL2591 sensor: %s", sensor_type_to_string(type));
    
    if (type != SENSOR_TYPE_LIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    tsl2591_data_t data;
    esp_err_t ret = tsl2591_read_data(I2C_MASTER_NUM, TSL2591_I2C_ADDR, &data);
    
    if (ret == ESP_OK && data.valid) {
        light_data->lux = data.lux;
        light_data->visible = data.visible;
        light_data->infrared = data.ch1;
        ESP_LOGD(TAG, "[I2C] TSL2591 read OK: %.2f lux (V:%u IR:%u)", 
                 light_data->lux, light_data->visible, light_data->infrared);
    } else {
        ESP_LOGW(TAG, "[I2C] TSL2591 read failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Read water level sensor
 */
static esp_err_t read_water_sensor(float *value) {
    ESP_LOGD(TAG, "[I2C] Reading water level sensor (FDC1004)");
    
    // Read fill percentage from FDC1004 capacitive sensor (preferred method)
    float fill_percent = fdc1004_read_fill_percent();
    
    if (fill_percent < -50.0f) {
        ESP_LOGW(TAG, "[I2C] Failed to read FDC1004 water level sensor");
        return ESP_FAIL;
    }
    
    *value = fill_percent;
    ESP_LOGD(TAG, "[I2C] Water level: %.1f%%", *value);
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
            
        case SENSOR_TYPE_LIGHT:
            ret = read_tsl2591_sensor(type, &sensor_data.light);
            break;
            
        case SENSOR_TYPE_WATER_LEVEL:
            // Read fill percentage (primary method, drift-resistant)
            ret = read_water_sensor(&sensor_data.water_level.fill_percent);
            // Also read mm for legacy compatibility (may drift with temperature)
            if (ret == ESP_OK) {
                int dist_mm = fdc1004_read_distance_mm();
                sensor_data.water_level.level_mm = (dist_mm >= 0) ? (float)dist_mm : 0.0f;
            }
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
    
    // Initialize sensor logger for historical data storage
    esp_err_t logger_ret = sensor_logger_init();
    if (logger_ret == ESP_OK) {
        ESP_LOGI(TAG, "Sensor logger initialized");
    } else {
        ESP_LOGW(TAG, "Failed to initialize sensor logger: %s", esp_err_to_name(logger_ret));
    }
    
    // Calculate cycles per minute for logging (poll_period_ms is typically 100ms)
    // 60 seconds * 1000ms / poll_period_ms = cycles per minute
    // Example: 60000 / 100 = 600 cycles per minute
    uint32_t cycles_per_minute = 60000 / g_sensor_mgr.config.poll_period_ms;
    uint32_t last_log_cycle = 0;
    ESP_LOGI(TAG, "Sensor logging interval: 1 minute (%lu poll cycles)", cycles_per_minute);
    
    while (g_sensor_mgr.running) {
        g_sensor_mgr.poll_cycles++;
        uint32_t cycle = g_sensor_mgr.poll_cycles;
        
        ESP_LOGD(TAG, "=== Poll cycle %lu ===", cycle);
        
        // Poll sensors based on priority with staggered scheduling
        // Staggering ensures at most one sensor per priority level polls each cycle,
        // preventing I2C bus congestion
        for (int type = 0; type < SENSOR_TYPE_MAX; type++) {
            if (should_poll_sensor(type, cycle)) {
                ESP_LOGD(TAG, "Polling %s (priority=%s, offset=%d, cycle=%lu)",
                         sensor_type_to_string(type),
                         priority_to_string(g_sensor_mgr.priorities[type]),
                         g_sensor_mgr.poll_offsets[type],
                         cycle);
                update_sensor_cache(type);
                
                // Small delay to allow I2C bus to settle between transactions
                // This is much smaller than before since staggering prevents multiple
                // sensors from polling in the same cycle
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        
        // Process any pending requests (non-blocking check)
        sensor_request_t request;
        while (xQueueReceive(g_sensor_mgr.request_queue, &request, 0) == pdTRUE) {
            ESP_LOGD(TAG, "Processing request for %s", sensor_type_to_string(request.sensor_type));
            
            // Check if request was abandoned (timeout)
            if (request.abandoned != NULL && *request.abandoned) {
                ESP_LOGD(TAG, "Skipping abandoned request for %s", sensor_type_to_string(request.sensor_type));
                continue;
            }
            
            // Get cached value
            esp_err_t ret = sensor_manager_get_cached(
                request.sensor_type,
                request.value_out,
                NULL
            );
            
            if (request.error_out != NULL) {
                *request.error_out = ret;
            }
            
            // Signal completion only if not abandoned
            if (request.done_sem != NULL && 
                (request.abandoned == NULL || !*request.abandoned)) {
                xSemaphoreGive(request.done_sem);
            }
        }
        
        // Log sensor snapshot to persistent storage every minute
        if (logger_ret == ESP_OK && (cycle - last_log_cycle) >= cycles_per_minute) {
            ESP_LOGI(TAG, "Logging sensor snapshot (cycle=%lu)", cycle);
            esp_err_t log_ret = sensor_logger_log_snapshot();
            if (log_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to log sensor snapshot: %s", esp_err_to_name(log_ret));
            }
            last_log_cycle = cycle;
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
        ESP_LOGD(TAG, "Cache miss for %s: data stale (age=%lums > timeout=%lums)", 
                 sensor_type_to_string(sensor_type), age_ms, g_sensor_mgr.config.cache_timeout_ms);
    } else {
        // No valid cache entry
        const char* reason = (ret == ESP_ERR_NOT_FOUND) ? "no valid data" : "cache access failed";
        ESP_LOGD(TAG, "Cache miss for %s: %s (%s)", 
                 sensor_type_to_string(sensor_type), reason, esp_err_to_name(ret));
    }
    
    g_sensor_mgr.cache_misses++;
    ESP_LOGD(TAG, "Queuing request for fresh data from %s (total misses: %lu)", 
             sensor_type_to_string(sensor_type), g_sensor_mgr.cache_misses);
    
    // Create request with abandoned flag
    volatile bool abandoned = false;  // Stack variable, valid during our wait
    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (done_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    sensor_request_t request = {
        .sensor_type = sensor_type,
        .value_out = value,
        .error_out = &ret,
        .done_sem = done_sem,
        .abandoned = &abandoned
    };
    
    // Send request to sensor task
    if (xQueueSend(g_sensor_mgr.request_queue, &request, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        vSemaphoreDelete(done_sem);
        ESP_LOGW(TAG, "Request queue full for %s", sensor_type_to_string(sensor_type));
        return ESP_ERR_TIMEOUT;
    }
    
    // Wait for completion
    if (xSemaphoreTake(done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        // Mark request as abandoned so sensor task won't try to signal the semaphore
        abandoned = true;
        // Small delay to ensure sensor task sees the flag if it's processing now
        vTaskDelay(pdMS_TO_TICKS(10));
        vSemaphoreDelete(done_sem);
        ESP_LOGW(TAG, "Request timeout for %s", sensor_type_to_string(sensor_type));
        return ESP_ERR_TIMEOUT;
    }
    
    vSemaphoreDelete(done_sem);
    
    ESP_LOGD(TAG, "Request completed for %s: value=%.2f, error=%s",
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
            
        case SENSOR_TYPE_LIGHT:
            *value = entry->data.light.lux;
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
                    
                case SENSOR_TYPE_LIGHT:
                    snprintf(value_str, sizeof(value_str), "%.2f lux (V:%u IR:%u)", 
                             entry->data.light.lux, entry->data.light.visible, entry->data.light.infrared);
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
        case SENSOR_TYPE_LIGHT: return "LIGHT";
        case SENSOR_TYPE_WATER_LEVEL: return "WATER_LEVEL";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Helper: Priority to string
 */
static const char* priority_to_string(sensor_priority_t priority) {
    switch (priority) {
        case SENSOR_PRIORITY_CRITICAL: return "CRITICAL(500ms)";
        case SENSOR_PRIORITY_HIGH: return "HIGH(1s)";
        case SENSOR_PRIORITY_MEDIUM: return "MEDIUM(2.5s)";
        case SENSOR_PRIORITY_LOW: return "LOW(5s)";
        default: return "UNKNOWN";
    }
}
