#include "actuator_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "ACTUATOR_CONTROL";

// -----------------------------------------------------------
// 1) Global data
// -----------------------------------------------------------

// Queue for commands
static QueueHandle_t actuator_queue = NULL;

// An array of actuator info. We'll maintain one entry per device.
static actuator_info_t s_actuators[ACTUATOR_IDX_MAX];

// 8-bit maximum duty for LEDC
#define LEDC_MAX_DUTY_8BIT ((1 << LEDC_TIMER_8_BIT) - 1)  // 255

// Placeholder maximum power (in mW) at 100% duty
// for each actuator. Adjust these to match reality.
static float get_max_power_mW_for_actuator(actuator_index_t idx)
{
    switch (idx) {
    case ACTUATOR_IDX_AIR_PUMP:     return 1000.0f;  // e.g. 1 W
    case ACTUATOR_IDX_SOURCE_PUMP:  return 1500.0f;  // e.g. 1.5 W
    case ACTUATOR_IDX_DRAIN_PUMP:   return 2000.0f;  // e.g. 2.0 W
    case ACTUATOR_IDX_PLANTER_PUMP: return 1200.0f;  // e.g. 1.2 W
    case ACTUATOR_IDX_LED_ARRAY:    return 5000.0f;  // e.g. 5 W
    default:                        return 1000.0f;  // fallback
    }
}

// A helper to update the global info array whenever we set duty
static void update_actuator_info(actuator_index_t idx, float duty_percentage)
{
    // clamp 0..100
    if (duty_percentage < 0.0f) duty_percentage = 0.0f;
    if (duty_percentage > 100.0f) duty_percentage = 100.0f;

    s_actuators[idx].duty_percentage = duty_percentage;
    s_actuators[idx].is_on = (duty_percentage > 0.0f);

    // Rough estimate: scale max power by duty %
    float max_power = get_max_power_mW_for_actuator(idx);
    s_actuators[idx].estimated_power_mW = max_power * (duty_percentage / 100.0f);
}

// -----------------------------------------------------------
// 2) Initialization
// -----------------------------------------------------------
void actuator_control_init(void)
{
    ESP_LOGI(TAG, "Initializing actuators...");

    // Create the queue for commands
    actuator_queue = xQueueCreate(10, sizeof(actuator_command_t));
    if (actuator_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create actuator command queue");
        return;
    }

    // Initialize the s_actuators array
    for (int i = 0; i < ACTUATOR_IDX_MAX; i++) {
        s_actuators[i].is_on               = false;
        s_actuators[i].duty_percentage     = 0.0f;
        s_actuators[i].estimated_power_mW  = 0.0f;
    }

    // -----------------------------------------------------------
    // 1) Initialize I2C motor driver
    // -----------------------------------------------------------
    // Note: We're using the shared I2C initialization from power_monitor_HAL
    // No need to specify GPIO pins or clock speed as they're defined there
    esp_err_t ret = i2c_motor_driver_init(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C motor driver: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize all motors to stopped state
    i2c_motor_stop_all();

    // NOTE: LED is now controlled through I2C motor driver on channel 4 (previously air pump)
    // No GPIO configuration needed

    ESP_LOGI(TAG, "Actuators initialized.");
}

// -----------------------------------------------------------
// 3) Individual set functions
// -----------------------------------------------------------
void set_air_pump_pwm(uint32_t duty_percentage)
{
    // Air pump functionality has been replaced by LED control
    // This function is kept for backward compatibility
    ESP_LOGW(TAG, "Air pump has been replaced by LED control. Use set_led_array_pwm() instead.");
    
    // Update our global struct to maintain compatibility
    update_actuator_info(ACTUATOR_IDX_AIR_PUMP, 0.0f);
    
    // Redirect to LED control if needed
    // set_led_array_pwm(duty_percentage);
}

void set_source_pump_pwm(uint32_t duty_percentage)
{
    if (duty_percentage > 100) {
        duty_percentage = 100;
    }

    ESP_LOGI(TAG, "Setting source pump PWM to %lu%%", (unsigned long)duty_percentage);

    esp_err_t ret;
    if (duty_percentage == 0) {
        ret = i2c_motor_set(MOTOR_SOURCE_PUMP, 0, MOTOR_RELEASE);
    } else {
        ret = i2c_motor_set(MOTOR_SOURCE_PUMP, duty_percentage, MOTOR_FORWARD);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set source pump: %s", esp_err_to_name(ret));
    }

    update_actuator_info(ACTUATOR_IDX_SOURCE_PUMP, (float)duty_percentage);
}

void set_drain_pump_pwm(uint32_t duty_percentage)
{
    if (duty_percentage > 100) {
        duty_percentage = 100;
    }
    ESP_LOGI(TAG, "Setting drain pump PWM to %lu%%", (unsigned long)duty_percentage);

    esp_err_t ret;
    if (duty_percentage == 0) {
        ret = i2c_motor_set(MOTOR_DRAIN_PUMP, 0, MOTOR_RELEASE);
    } else {
        ret = i2c_motor_set(MOTOR_DRAIN_PUMP, duty_percentage, MOTOR_FORWARD);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set drain pump: %s", esp_err_to_name(ret));
    }

    update_actuator_info(ACTUATOR_IDX_DRAIN_PUMP, (float)duty_percentage);
}

void set_planter_pump_pwm(uint32_t duty_percentage)
{
    if (duty_percentage > 100) {
        duty_percentage = 100;
    }

    // ESP_LOGI(TAG, "Setting planter pump PWM to %lu%%", (unsigned long)duty_percentage);

    esp_err_t ret;
    if (duty_percentage == 0) {
        ret = i2c_motor_set(MOTOR_PLANTER_PUMP, 0, MOTOR_RELEASE);
    } else {
        ret = i2c_motor_set(MOTOR_PLANTER_PUMP, duty_percentage, MOTOR_FORWARD);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set planter pump: %s", esp_err_to_name(ret));
    }

    update_actuator_info(ACTUATOR_IDX_PLANTER_PUMP, (float)duty_percentage);
}

void set_led_array_pwm(uint32_t duty_percentage)
{
    if (duty_percentage > 100) {
        duty_percentage = 100;
    }
    
    ESP_LOGI(TAG, "Setting LED array PWM to %lu%%", (unsigned long)duty_percentage);

    // Use the I2C motor driver to control the LED
    esp_err_t ret = i2c_led_set_brightness(duty_percentage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LED brightness: %s", esp_err_to_name(ret));
    }

    // Update actuator info
    update_actuator_info(ACTUATOR_IDX_LED_ARRAY, (float)duty_percentage);
}

void set_led_array_binary(bool state)
{
    ESP_LOGI(TAG, "Setting LED array to %s", state ? "ON" : "OFF");
    
    // Use the I2C motor driver to control the LED (binary on/off)
    esp_err_t ret = i2c_led_set_brightness(state ? 100 : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LED state: %s", esp_err_to_name(ret));
    }

    // For LED array, we treat on=100%, off=0%
    update_actuator_info(ACTUATOR_IDX_LED_ARRAY, state ? 100.0f : 0.0f);
}

// -----------------------------------------------------------
// 4) The main task that handles incoming commands
// -----------------------------------------------------------
void actuator_control_task(void *pvParameters)
{
    actuator_command_t command;

    while (1) {
        if (xQueueReceive(actuator_queue, &command, portMAX_DELAY)) {
            switch (command.cmd_type) {

                case ACTUATOR_CMD_AIR_PUMP_PWM:
                    set_air_pump_pwm(command.value);
                    break;

                case ACTUATOR_CMD_SOURCE_PUMP_PWM:
                    set_source_pump_pwm(command.value);
                    break;

                case ACTUATOR_CMD_DRAIN_PUMP_PWM:
                    set_drain_pump_pwm(command.value);
                    break;

                case ACTUATOR_CMD_PLANTER_PUMP_PWM:
                    set_planter_pump_pwm(command.value);
                    break;

                case ACTUATOR_CMD_LED_ARRAY_PWM:
                    set_led_array_pwm(command.value);
                    break;

                default:
                    ESP_LOGW(TAG, "Received unknown command type: %d", command.cmd_type);
                    break;
            }
        }
    }
}

// -----------------------------------------------------------
// 5) Public getter for the queue + actuator info
// -----------------------------------------------------------
QueueHandle_t get_actuator_queue(void)
{
    return actuator_queue;
}

const actuator_info_t* actuator_control_get_info(void)
{
    // Return a pointer to our static array so other code
    // can read each actuator’s is_on, duty_percentage, etc.
    // If multiple tasks may read concurrently, consider a mutex.
    return s_actuators;
}
