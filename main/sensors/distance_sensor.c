#include "distance_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define to use laser sensor or ultrasonic
// #define USE_LASER_SENSOR 1
#define USE_ULTRASONIC_SENSOR 1

static const char *TAG = "DIST_SENSOR";

//===========================================================
// If using laser sensor (I2C)
#if defined(USE_LASER_SENSOR)

#include "driver/i2c.h"

// Laser sensor I2C address
#define LASER_I2C_ADDRESS   0x74

// Write register helper
static esp_err_t laser_write_reg(uint8_t reg, const uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LASER_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    if (len > 0) {
        i2c_master_write(cmd, (uint8_t*)data, len, true);
    }

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

// Read register helper
static esp_err_t laser_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    // Write the register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LASER_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    // Now read 'len' bytes from that register
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LASER_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// One-shot measurement
static int laser_get_distance_mm(void) {
    // Similar logic from your example: writeReg(0x10, 0xB0), then readReg(0x02, 2 bytes)
    uint8_t dat = 0xB0;
    esp_err_t ret = laser_write_reg(0x10, &dat, 1);
    if (ret != ESP_OK) {
        return -1;
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for measurement

    uint8_t buf[2] = {0};
    ret = laser_read_reg(0x02, buf, 2);
    if (ret != ESP_OK) {
        return -1;
    }

    int distance_mm = ((int)buf[0] << 8) | buf[1];
    distance_mm += 10; // offset from your example
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

static int ultrasonic_get_distance_cm(void)
{
    uint8_t frame[4];
    int length = uart_read_bytes(ULTRASONIC_UART_NUM, frame, 4, pdMS_TO_TICKS(100));
    if (length == 4) {
        if (frame[0] == 0xFF) {
            uint8_t csum = (frame[0] + frame[1] + frame[2]) & 0xFF;
            if (csum != frame[3]) {
                return -1;
            }
            // distance in mm
            uint16_t distance_mm = ((uint16_t)frame[1] << 8) | frame[2];
            if (distance_mm < 30) {
                return -1;
            }
            return distance_mm / 10; // cm
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
#if defined(USE_LASER_SENSOR)
    ESP_LOGI(TAG, "Distance sensor: LASER mode, I2C=0x74");
    return laser_init_hw();
#elif defined(USE_ULTRASONIC_SENSOR)
    ESP_LOGI(TAG, "Distance sensor: ULTRASONIC mode, UART pins %d/%d",
             ULTRASONIC_UART_TX, ULTRASONIC_UART_RX);
    return ultrasonic_init_hw();
#else
    ESP_LOGE(TAG, "No sensor mode defined!");
    return ESP_FAIL;
#endif
}

int distance_sensor_read_mm()
{
#if defined(USE_LASER_SENSOR)
    return laser_get_distance_mm();
#elif defined(USE_ULTRASONIC_SENSOR)
    return ultrasonic_get_distance_cm() * 10;    
#endif
}

void distance_sensor_task(void *pvParameters)
{
    // We want 10 Hz => 100 ms delay
    const TickType_t delay_ticks = pdMS_TO_TICKS(50);

    while (1) {
        vTaskDelay(delay_ticks);

#if defined(USE_LASER_SENSOR)
        int dist_mm = laser_get_distance_mm();
        if (dist_mm > 0) {
            // Instead of printing the distance, we do an ASCII bar
            // For example: 30 mm => 1 'X', 300 mm => 10 'X'
            // Let's do 1 'X' per 30 mm
            int bar_element_size = 10;
            int count = dist_mm / bar_element_size;
            if (count < 1) count = 1;      // at least 1
            if (count > 80) count = 80;   // clamp so we don't spam too many X's

            // Print line: "<distance_mm>mm [XXXXX...]"
            printf("%4d mm ", dist_mm);
            for (int i = 0; i < count; i++) {
                printf("X");
            }
            printf("\n");
        } else {
            printf("---- mm\n");
        }

#elif defined(USE_ULTRASONIC_SENSOR)
        int dist_cm = ultrasonic_get_distance_cm();
        if (dist_cm > 0) {
            // e.g. 3 cm => 1 'X', 30 cm => 10 'X'
            // We'll do 1 'X' per 3 cm.
            int count = dist_cm / 3;
            if (count < 1) count = 1;
            if (count > 80) count = 80;

            printf("%3d cm ", dist_cm);
            for (int i = 0; i < count; i++) {
                printf("X");
            }
            printf("\n");
        } else {
            printf("--- cm\n");
        }
#endif
    }
}
