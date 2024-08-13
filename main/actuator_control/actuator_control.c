#include "actuator_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "ACTUATOR_CONTROL";
static QueueHandle_t actuator_queue = NULL; // Queue handle

// Maximum duty cycle based on the resolution of the PWM signal (13 bits -> 8191 max)
#define LEDC_MAX_DUTY       ((1 << LEDC_TIMER_13_BIT) - 1)  // 8191 for 13-bit resolution

void actuator_control_init(void) {
    ESP_LOGI(TAG, "Initializing actuators");

    // Create the queue for commands
    actuator_queue = xQueueCreate(10, sizeof(actuator_command_t)); // Queue can hold 10 commands
    if (actuator_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create actuator command queue");
        return;
    }

    // Configure PWM for Air Pump
    ledc_timer_config_t air_pump_timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&air_pump_timer);

    ledc_channel_config_t air_pump_channel = {
        .speed_mode = PWM_MODE,
        .channel = AIR_PUMP_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = AIR_PUMP_GPIO,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&air_pump_channel);

    // Configure PWM for Water Pump
    ledc_channel_config_t water_pump_channel = {
        .speed_mode = PWM_MODE,
        .channel = WATER_PUMP_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = WATER_PUMP_GPIO,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&water_pump_channel);

    // Configure GPIO for Solenoid Valve
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SOLENOID_VALVE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(SOLENOID_VALVE_GPIO, 0); // Initialize Solenoid Valve to OFF

    // Configure GPIO for LED Array (Binary Control)
    io_conf.pin_bit_mask = (1ULL << LED_ARRAY_GPIO);
    gpio_config(&io_conf);
    gpio_set_level(LED_ARRAY_GPIO, 0); // Initialize LED Array to OFF

    ESP_LOGI(TAG, "Actuators initialized to OFF states");
}

void set_air_pump_pwm(uint32_t duty_percentage) {
    if (duty_percentage > 100) {
        duty_percentage = 100;  // Clamp the value to 100%
    }
    uint32_t duty = (duty_percentage * LEDC_MAX_DUTY) / 100; // Convert percentage to duty cycle
    ESP_LOGI(TAG, "Setting air pump PWM to %lu%% (duty: %lu)", duty_percentage, (unsigned long)duty);
    ledc_set_duty(PWM_MODE, AIR_PUMP_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, AIR_PUMP_CHANNEL);
}

void set_water_pump_pwm(uint32_t duty_percentage) {
    if (duty_percentage > 100) {
        duty_percentage = 100;  // Clamp the value to 100%
    }
    uint32_t duty = (duty_percentage * LEDC_MAX_DUTY) / 100; // Convert percentage to duty cycle
    ESP_LOGI(TAG, "Setting water pump PWM to %lu%% (duty: %lu)", duty_percentage, (unsigned long)duty);
    ledc_set_duty(PWM_MODE, WATER_PUMP_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, WATER_PUMP_CHANNEL);
}

void set_solenoid_valve(bool state) {
    ESP_LOGI(TAG, "Setting solenoid valve to %s", state ? "ON" : "OFF");
    gpio_set_level(SOLENOID_VALVE_GPIO, state ? 1 : 0);
}

void set_led_array_binary(bool state) {
    ESP_LOGI(TAG, "Setting LED array to %s", state ? "ON" : "OFF");
    gpio_set_level(LED_ARRAY_GPIO, state ? 1 : 0);
}

void actuator_control_task(void *pvParameters) {
    actuator_command_t command;

    while (1) {
        if (xQueueReceive(actuator_queue, &command, portMAX_DELAY)) {
            switch (command.cmd_type) {
                case ACTUATOR_CMD_AIR_PUMP_PWM:
                    set_air_pump_pwm(command.value);
                    break;
                case ACTUATOR_CMD_WATER_PUMP_PWM:
                    set_water_pump_pwm(command.value);
                    break;
                case ACTUATOR_CMD_SOLENOID_VALVE:
                    set_solenoid_valve(command.value);
                    break;
                case ACTUATOR_CMD_LED_ARRAY_BINARY:
                    set_led_array_binary(command.value);
                    break;
                default:
                    ESP_LOGW(TAG, "Received unknown command");
                    break;
            }
        }
    }
}

QueueHandle_t get_actuator_queue(void) {
    return actuator_queue;
}
