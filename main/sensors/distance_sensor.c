#include "distance_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

// Define to use laser sensor or ultrasonic
#define USE_LASER_SENSOR 1
// #define USE_ULTRASONIC_SENSOR 1

static const char *TAG = "DIST_SENSOR";

// Mutex to protect access to the distance sensor
static SemaphoreHandle_t sensor_mutex = NULL;

// Last valid measurement and timestamp (for caching)
static int last_valid_distance_mm = -1;
static uint32_t last_measurement_time = 0;

//===========================================================
// If using laser sensor (I2C)
#if defined(USE_LASER_SENSOR)

#include "driver/i2c.h"
#include "driver/i2c_master.h"

// Laser sensor I2C address
#define LASER_I2C_ADDRESS   0x74

// Write register helper
static esp_err_t laser_write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    // We need to send [reg][data...]
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    // Write to the device in one shot:
    return i2c_master_write_to_device(
        I2C_NUM_0,
        LASER_I2C_ADDRESS, 
        buf, 
        len + 1, 
        pdMS_TO_TICKS(50)
    );
}

// Read register helper
static esp_err_t laser_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    // 1) Write the register address
    // 2) Then read 'len' bytes into 'data'

    // The new API does this in one line:
    return i2c_master_write_read_device(
        I2C_NUM_0,
        LASER_I2C_ADDRESS,
        &reg,  // pointer to register address
        1,     // size of register address
        data, 
        len, 
        pdMS_TO_TICKS(50)
    );
}


static int laser_get_distance_mm(void)
{
    // Command the laser to start measurement
    uint8_t dat = 0xB0;
    esp_err_t ret = laser_write_reg(0x10, &dat, 1);
    if (ret != ESP_OK) {
        return -1; 
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for measurement to complete

    // Read 2 bytes from register 0x02
    uint8_t buf[2];
    ret = laser_read_reg(0x02, buf, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read distance from laser sensor: %s", esp_err_to_name(ret));
        return -1;
    }

    int distance_mm = (((int)buf[0] << 8) | buf[1]) - 10; // Subtract 10 for calibration
    // int distance_mm = ((int)buf[0] * 0x100) + buf[1] - 10;
    // ESP_LOGI(TAG, "Distance reading: %d mm", distance_mm);
    return distance_mm;
}


static esp_err_t laser_init_hw(void) {
    // Assume i2c_master_init() is already called in app_main
    return ESP_OK;
}

#endif // USE_LASER_SENSOR

//===========================================================
// If using ultrasonic sensor (UART)
#if defined(USE_ULTRASONIC_SENSOR)

#include "driver/uart.h"
#include "driver/gpio.h"

#define ULTRASONIC_UART_NUM  UART_NUM_1
#define ULTRASONIC_UART_RX   5 // sensor TX -> ESP32 RX
#define ULTRASONIC_UART_TX   4 // sensor RX -> ESP32 TX

static int ultrasonic_get_distance_mm(void)
{
    uint8_t frame[4];
    int length = uart_read_bytes(ULTRASONIC_UART_NUM, frame, 4, pdMS_TO_TICKS(1000));
    if (length == 4) {
        if (frame[0] == 0xFF) {
            uint8_t csum = (frame[0] + frame[1] + frame[2]) & 0x00FF;
            if (csum != frame[3]) {
                return -1;
            }
            // distance in mm
            uint16_t distance_mm = (frame[1]<<8) + frame[2];  //((uint16_t)frame[1] << 8) | frame[2];
            ESP_LOGE(TAG, "mm inside ultrasonic: %u mm", distance_mm);
            if (distance_mm < 30) {
                return -2;
            }
            return distance_mm;
        }
    }
    return -1;
}

static esp_err_t ultrasonic_init_hw(void)
{
    uart_config_t cfg = {
        .baud_rate  = 9600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(ULTRASONIC_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(ULTRASONIC_UART_NUM,
                                 ULTRASONIC_UART_TX,
                                 ULTRASONIC_UART_RX,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(ULTRASONIC_UART_NUM, 1024, 0, 0, NULL, 0));

    // Some sensors require driving TX pin high
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << ULTRASONIC_UART_TX,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(ULTRASONIC_UART_TX, 1);

    return ESP_OK;
}

#endif // USE_ULTRASONIC_SENSOR

//===========================================================
// Common entry points
//===========================================================
esp_err_t distance_sensor_init(void)
{
    uint32_t err = ESP_OK;
    // Create mutex for thread-safe access to sensor
    if (sensor_mutex == NULL) {
        sensor_mutex = xSemaphoreCreateMutex();
        if (sensor_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create sensor mutex");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Sensor access mutex created");
    }
    
    // Initialize hardware
#if defined(USE_LASER_SENSOR)
    ESP_LOGI(TAG, "Distance sensor: LASER mode, I2C=0x74");
    err = laser_init_hw();
#elif defined(USE_ULTRASONIC_SENSOR)
    ESP_LOGI(TAG, "Distance sensor: ULTRASONIC mode, UART pins %d/%d",
             ULTRASONIC_UART_TX, ULTRASONIC_UART_RX);
    err = ultrasonic_init_hw();
#else
    ESP_LOGE(TAG, "No sensor mode defined!");
    return ESP_FAIL;
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize distance sensor hardware: %s", esp_err_to_name(err));
        return err;
    }

    // Perform a quick test reading to verify everything is working
    int test_reading = distance_sensor_read_mm();

    if (test_reading < 0) {
        ESP_LOGE(TAG, "Failed to take initial distance reading: %d", test_reading);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Take an actual reading from the distance sensor hardware
 * 
 * This is an internal function that accesses the sensor hardware directly.
 * It should only be called when the mutex is held to prevent concurrent access.
 * 
 * @note This function takes about 400ms to complete (due to averaging multiple readings)
 * @return Distance in mm, or negative value if error
 */
static int distance_sensor_read_mm_actual(void)
{
#if defined(USE_LASER_SENSOR)
    const int avg_count = 8;
    int sum = 0;
    int valid_readings = 0;
    
    // Take multiple readings and average them for better accuracy
    for (int i = 0; i < avg_count; i++) {
        int tmp = laser_get_distance_mm();
        if (tmp > 0) {  // Valid reading
            sum += tmp;
            valid_readings++;
        } else {
            ESP_LOGW(TAG, "Invalid laser reading attempt (%d) %d/%d", tmp, i+1, avg_count);
        }
        // Small delay between readings
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // Return average if we got at least half of the readings
    if (valid_readings >= avg_count/2) {
        return sum / valid_readings;
    }
    
    ESP_LOGE(TAG, "Too many invalid laser readings (%d/%d failed)", 
             avg_count - valid_readings, avg_count);
    return -1;
    
#elif defined(USE_ULTRASONIC_SENSOR)
    return ultrasonic_get_distance_mm();   
#endif
}

int distance_sensor_read_mm(void)
{
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t reading_age = current_time - last_measurement_time;
    
    // First, check if we have a recent valid reading (within last 500ms)
    // Use the cached value if it's fresh enough
    if (last_valid_distance_mm > 0 && reading_age < 500) {
        ESP_LOGI(TAG, "Using fresh cached reading: %d mm (age: %lu ms)", 
                 last_valid_distance_mm, reading_age);
        return last_valid_distance_mm;
    }
    
    int result = -1;
    
    // Try to acquire mutex with timeout (1 second max wait)
    if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // Critical section - only one task can access the sensor at a time
        ESP_LOGI(TAG, "Acquired sensor mutex, taking new reading");
        
        // Check again if another task updated the reading while we were waiting
        // This prevents redundant readings if multiple tasks requested at the same time
        uint32_t new_reading_age = xTaskGetTickCount() * portTICK_PERIOD_MS - last_measurement_time;
        if (last_valid_distance_mm > 0 && new_reading_age < 200) {
            // Another task just updated the reading, use that instead
            result = last_valid_distance_mm;
            ESP_LOGI(TAG, "Another task updated reading while waiting, using that: %d mm", result);
        } else {
            // Take a new sensor reading (this takes ~400ms)
            uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            result = distance_sensor_read_mm_actual();
            uint32_t elapsed = xTaskGetTickCount() * portTICK_PERIOD_MS - start_time;
            
            ESP_LOGI(TAG, "Sensor reading took %lu ms, result: %d mm", elapsed, result);
            
            // If reading is valid, update the cache
            if (result > 0) {
                last_valid_distance_mm = result;
                last_measurement_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }
        }
        
        // Release mutex
        xSemaphoreGive(sensor_mutex);
        ESP_LOGI(TAG, "Released sensor mutex");
    } else {
        // Failed to acquire mutex after timeout
        ESP_LOGW(TAG, "Failed to acquire sensor mutex after 1s timeout");
        
        // If we couldn't get mutex access but have a previous valid reading,
        // return that instead (even if it's older than 500ms)
        if (last_valid_distance_mm > 0) {
            ESP_LOGW(TAG, "Using stale cached reading: %d mm (age: %lu ms)", 
                     last_valid_distance_mm, reading_age);
            return last_valid_distance_mm;
        } else {
            ESP_LOGE(TAG, "No valid reading available and sensor is busy");
        }
    }
    
    return result;
}

/**
 * @brief Get the last valid reading without trying to access the sensor
 * 
 * This function is completely non-blocking and returns immediately with the
 * most recent cached distance value. It's safe to call from any task or ISR.
 * 
 * @return Last valid distance in mm, or -1 if no valid reading available
 */
int distance_sensor_get_last_reading_mm(void)
{
    // Just return the cached value (atomic read for 32-bit value)
    uint32_t reading_age = xTaskGetTickCount() * portTICK_PERIOD_MS - last_measurement_time;
    
    return last_valid_distance_mm;
}

/**
 * @brief Background task that periodically updates the distance sensor reading
 * 
 * This task runs continuously in the background and:
 * 1. Updates the cached distance measurement at a regular interval
 * 2. Logs distance information at appropriate levels
 * 
 * @param pvParameters FreeRTOS task parameters (not used)
 */
void distance_sensor_task(void *pvParameters)
{
    // Poll at 1 Hz (every 1000ms)
    const TickType_t delay_ticks = pdMS_TO_TICKS(200);
    
    // Wait a bit before starting to let other initializations complete
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "Distance sensor background task started");
    
    // Counter for retry attempts when reading fails
    int retry_count = 0;
    // Counter for logging (to reduce spam)
    int log_counter = 0;
    
    while (1) {
        // Try to acquire mutex with a shorter timeout since this is a background task
        if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            // Take a direct sensor reading
            int dist_mm = distance_sensor_read_mm_actual();
            
            if (dist_mm > 0) {
                // Valid reading - update cached values
                last_valid_distance_mm = dist_mm;
                last_measurement_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                retry_count = 0; // Reset retry counter
            } else {
                // Invalid reading
                retry_count++;
                ESP_LOGW(TAG, "Invalid distance reading (%d consecutive failures)", retry_count);
                
                // If we've had too many consecutive failures, log an error
                if (retry_count >= 5) {
                    ESP_LOGE(TAG, "Distance sensor may be disconnected or faulty");
                }
            }
            
            // Release the mutex
            xSemaphoreGive(sensor_mutex);
            
        } else {
            // Couldn't acquire mutex - sensor is busy
            ESP_LOGW(TAG, "Background task couldn't acquire sensor mutex - sensor busy");
        }
        
        // Wait before next reading
        vTaskDelay(delay_ticks);
    }
}
