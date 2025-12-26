/**
 * @file sensor_api.c
 * @brief Implementation of external sensor API
 */

#include "sensor_api.h"
#include "sensor_manager.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "SENSOR_API";

/**
 * @brief Read current from power monitor
 */
esp_err_t sensor_api_read_power_current(float *current_ma) {
    if (current_ma == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "API call: read_power_current");
    return sensor_manager_read(SENSOR_TYPE_POWER_CURRENT, current_ma, SENSOR_API_DEFAULT_TIMEOUT_MS);
}

/**
 * @brief Read voltage from power monitor
 */
esp_err_t sensor_api_read_power_voltage(float *voltage_mv) {
    if (voltage_mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "API call: read_power_voltage");
    return sensor_manager_read(SENSOR_TYPE_POWER_VOLTAGE, voltage_mv, SENSOR_API_DEFAULT_TIMEOUT_MS);
}

/**
 * @brief Read power from power monitor
 */
esp_err_t sensor_api_read_power_power(float *power_mw) {
    if (power_mw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "API call: read_power_power");
    return sensor_manager_read(SENSOR_TYPE_POWER_POWER, power_mw, SENSOR_API_DEFAULT_TIMEOUT_MS);
}

/**
 * @brief Read temperature from SHT45
 */
esp_err_t sensor_api_read_temperature(float *temperature_c) {
    if (temperature_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "API call: read_temperature");
    return sensor_manager_read(SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY, temperature_c, SENSOR_API_DEFAULT_TIMEOUT_MS);
}

/**
 * @brief Read humidity from SHT45
 */
esp_err_t sensor_api_read_humidity(float *humidity_rh) {
    if (humidity_rh == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "API call: read_humidity");
    
    // Humidity is stored in value2 of the ENVIRONMENT sensor cache
    // Use the specialized function to get both values from the combined sensor
    return sensor_manager_get_environment_cached(NULL, humidity_rh, NULL);
}

/**
 * @brief Read water level
 */
esp_err_t sensor_api_read_water_level(float *level_mm) {
    if (level_mm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "API call: read_water_level");
    return sensor_manager_read(SENSOR_TYPE_WATER_LEVEL, level_mm, SENSOR_API_DEFAULT_TIMEOUT_MS);
}

/**
 * @brief Read all power metrics at once
 */
esp_err_t sensor_api_read_power_all(float *current_ma, float *voltage_mv, float *power_mw) {
    esp_err_t ret = ESP_OK;
    esp_err_t err;
    
    ESP_LOGD(TAG, "API call: read_power_all");
    
    if (current_ma != NULL) {
        err = sensor_manager_read(SENSOR_TYPE_POWER_CURRENT, current_ma, SENSOR_API_DEFAULT_TIMEOUT_MS);
        if (err != ESP_OK) ret = err;
    }
    
    if (voltage_mv != NULL) {
        err = sensor_manager_read(SENSOR_TYPE_POWER_VOLTAGE, voltage_mv, SENSOR_API_DEFAULT_TIMEOUT_MS);
        if (err != ESP_OK) ret = err;
    }
    
    if (power_mw != NULL) {
        err = sensor_manager_read(SENSOR_TYPE_POWER_POWER, power_mw, SENSOR_API_DEFAULT_TIMEOUT_MS);
        if (err != ESP_OK) ret = err;
    }
    
    return ret;
}

/**
 * @brief Read all environment metrics at once
 */
esp_err_t sensor_api_read_environment_all(float *temperature_c, float *humidity_rh) {
    ESP_LOGD(TAG, "API call: read_environment_all");
    
    // Read from TEMPERATURE_AND_HUMIDITY sensor once to trigger update if needed
    float temp;
    esp_err_t ret = sensor_manager_read(SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY, &temp, SENSOR_API_DEFAULT_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        // Get both values from cache
        ret = sensor_manager_get_environment_cached(temperature_c, humidity_rh, NULL);
    }
    
    return ret;
}

/**
 * @brief Helper: Convert sensor name string to sensor type enum
 */
static sensor_type_t sensor_name_to_type(const char *name) {
    if (strcmp(name, "current") == 0) return SENSOR_TYPE_POWER_CURRENT;
    if (strcmp(name, "voltage") == 0) return SENSOR_TYPE_POWER_VOLTAGE;
    if (strcmp(name, "power") == 0) return SENSOR_TYPE_POWER_POWER;
    if (strcmp(name, "temperature") == 0) return SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY;
    if (strcmp(name, "humidity") == 0) return SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY;
    if (strcmp(name, "water_level") == 0) return SENSOR_TYPE_WATER_LEVEL;
    return SENSOR_TYPE_MAX;  // Invalid
}

/**
 * @brief Get cached sensor value
 */
esp_err_t sensor_api_get_cached(const char *sensor_name, float *value, uint32_t *age_ms) {
    if (sensor_name == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    sensor_type_t type = sensor_name_to_type(sensor_name);
    if (type == SENSOR_TYPE_MAX) {
        ESP_LOGE(TAG, "Unknown sensor name: %s", sensor_name);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t timestamp_ms;
    esp_err_t ret = sensor_manager_get_cached(type, value, &timestamp_ms);
    
    if (ret == ESP_OK && age_ms != NULL) {
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        *age_ms = now_ms - timestamp_ms;
    }
    
    return ret;
}

/**
 * @brief Force immediate sensor reading
 */
esp_err_t sensor_api_force_read(const char *sensor_name, float *value) {
    if (sensor_name == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    sensor_type_t type = sensor_name_to_type(sensor_name);
    if (type == SENSOR_TYPE_MAX) {
        ESP_LOGE(TAG, "Unknown sensor name: %s", sensor_name);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "API call: force_read(%s)", sensor_name);
    return sensor_manager_read_forced(type, value, SENSOR_API_DEFAULT_TIMEOUT_MS);
}

/**
 * @brief Print all current sensor values
 */
void sensor_api_print_all(void) {
    float current_ma, voltage_mv, power_mw;
    float temperature_c, humidity_rh;
    float water_level_mm;
    
    ESP_LOGI(TAG, "=== Current Sensor Values ===");
    
    // Power metrics
    if (sensor_api_read_power_all(&current_ma, &voltage_mv, &power_mw) == ESP_OK) {
        ESP_LOGI(TAG, "Power Monitor:");
        ESP_LOGI(TAG, "  Current: %.2f mA", current_ma);
        ESP_LOGI(TAG, "  Voltage: %.2f mV", voltage_mv);
        ESP_LOGI(TAG, "  Power: %.2f mW", power_mw);
    } else {
        ESP_LOGW(TAG, "Power Monitor: Failed to read");
    }
    
    // Environment metrics
    if (sensor_api_read_environment_all(&temperature_c, &humidity_rh) == ESP_OK) {
        ESP_LOGI(TAG, "SHT45 Environment:");
        ESP_LOGI(TAG, "  Temperature: %.2f °C", temperature_c);
        ESP_LOGI(TAG, "  Humidity: %.2f %%RH", humidity_rh);
    } else {
        ESP_LOGW(TAG, "SHT45 Environment: Failed to read");
    }
    
    // Light sensor
    sensor_data_t light_data;
    if (sensor_manager_get_data_cached(SENSOR_TYPE_LIGHT, &light_data, NULL) == ESP_OK) {
        ESP_LOGI(TAG, "TSL2591 Light Sensor:");
        ESP_LOGI(TAG, "  Lux: %.2f", light_data.light.lux);
        ESP_LOGI(TAG, "  Visible: %u", light_data.light.visible);
        ESP_LOGI(TAG, "  Infrared: %u", light_data.light.infrared);
    } else {
        ESP_LOGW(TAG, "TSL2591 Light Sensor: Failed to read");
    }
    
    // Water level
    if (sensor_api_read_water_level(&water_level_mm) == ESP_OK) {
        ESP_LOGI(TAG, "Water Level: %.2f mm", water_level_mm);
    } else {
        ESP_LOGW(TAG, "Water Level: Failed to read");
    }
    
    // Print sensor manager statistics
    uint32_t total_reads, cache_hits, cache_misses, errors;
    sensor_manager_get_stats(&total_reads, &cache_hits, &cache_misses, &errors);
    ESP_LOGI(TAG, "\nStatistics:");
    ESP_LOGI(TAG, "  Total reads: %lu", total_reads);
    ESP_LOGI(TAG, "  Cache hits: %lu", cache_hits);
    ESP_LOGI(TAG, "  Cache misses: %lu", cache_misses);
    ESP_LOGI(TAG, "  Errors: %lu", errors);
}
