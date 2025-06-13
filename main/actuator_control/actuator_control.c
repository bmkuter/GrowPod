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
    // 1) Configure a 25 kHz, 8-bit PWM timer for all pumps
    // -----------------------------------------------------------
    ledc_timer_config_t pump_timer = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,         // LEDC_TIMER_0
        .duty_resolution = LEDC_TIMER_8_BIT,  // 8-bit resolution
        .freq_hz         = 25000,             // 25 kHz
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pump_timer);

    // Air Pump channel (Channel 0)
    ledc_channel_config_t air_pump_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = AIR_PUMP_CHANNEL,    // LEDC_CHANNEL_0
        .timer_sel      = PWM_TIMER,           // LEDC_TIMER_0
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = AIR_PUMP_GPIO,       // GPIO 0
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&air_pump_channel);

    // Source Pump channel (Channel 1)
    ledc_channel_config_t source_pump_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = SOURCE_PUMP_CHANNEL, // LEDC_CHANNEL_1
        .timer_sel      = PWM_TIMER,           // LEDC_TIMER_0
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SOURCE_PUMP_GPIO,    // GPIO 1
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&source_pump_channel);

    // Drain Pump channel (Channel 2)
    ledc_channel_config_t drain_pump_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = DRAIN_PUMP_CHANNEL,  // LEDC_CHANNEL_2
        .timer_sel      = PWM_TIMER,           // LEDC_TIMER_0
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = DRAIN_PUMP_GPIO,     // GPIO 6
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&drain_pump_channel);

    // Planter Pump channel (Channel 3)
    ledc_channel_config_t planter_pump_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = PLANTER_PUMP_CHANNEL, // LEDC_CHANNEL_3
        .timer_sel      = PWM_TIMER,            // LEDC_TIMER_0
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PLANTER_PUMP_GPIO,    // GPIO 7
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&planter_pump_channel);

    // // -----------------------------------------------------------
    // // 2) Configure GPIO for the LED Array (Binary ON/OFF)
    // // -----------------------------------------------------------
    // gpio_config_t io_conf = {
    //     .pin_bit_mask = (1ULL << LED_ARRAY_GPIO),
    //     .mode         = GPIO_MODE_OUTPUT,
    //     .pull_up_en   = GPIO_PULLUP_DISABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type    = GPIO_INTR_DISABLE
    // };
    // gpio_config(&io_conf);
    // gpio_set_level(LED_ARRAY_GPIO, 0); // OFF by default
    // -----------------------------------------------------------
    // 2) Configure LED Array using PWM (instead of simple GPIO)
    // -----------------------------------------------------------
    ledc_channel_config_t led_array_channel = {
        .speed_mode = PWM_MODE,
        .channel    = LED_ARRAY_CHANNEL,   // e.g., LEDC_CHANNEL_4
        .timer_sel  = PWM_TIMER,           // same timer as used for pumps (LEDC_TIMER_0)
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LED_ARRAY_GPIO,       // your LED GPIO
        .duty       = 0,                    // off by default (0% duty cycle)
        .hpoint     = 0
    };
    ledc_channel_config(&led_array_channel);

    ESP_LOGI(TAG, "Actuators initialized.");
}

// -----------------------------------------------------------
// 3) Individual set functions
// -----------------------------------------------------------
void set_air_pump_pwm(uint32_t duty_percentage)
{
    if (duty_percentage > 100) {
        duty_percentage = 100;
    }
    uint32_t duty = (duty_percentage * LEDC_MAX_DUTY_8BIT) / 100;  // 0..255

    ESP_LOGI(TAG, "Setting air pump PWM to %lu%% (duty: %lu)",
             (unsigned long)duty_percentage, (unsigned long)duty);

    ledc_set_duty(PWM_MODE, AIR_PUMP_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, AIR_PUMP_CHANNEL);

    // Update our global struct
    update_actuator_info(ACTUATOR_IDX_AIR_PUMP, (float)duty_percentage);
}

void set_source_pump_pwm(uint32_t duty_percentage)
{
    if (duty_percentage > 100) {
        duty_percentage = 100;
    }
    uint32_t duty = (duty_percentage * LEDC_MAX_DUTY_8BIT) / 100;  // 0..255

    ESP_LOGI(TAG, "Setting source pump PWM to %lu%% (duty: %lu)",
             (unsigned long)duty_percentage, (unsigned long)duty);

    ledc_set_duty(PWM_MODE, SOURCE_PUMP_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, SOURCE_PUMP_CHANNEL);

    update_actuator_info(ACTUATOR_IDX_SOURCE_PUMP, (float)duty_percentage);
}

void set_drain_pump_pwm(uint32_t duty_percentage)
{
    if (duty_percentage > 100) {
        duty_percentage = 100;
    }
    uint32_t duty = (duty_percentage * LEDC_MAX_DUTY_8BIT) / 100;  // 0..255

    ESP_LOGI(TAG, "Setting drain pump PWM to %lu%% (duty: %lu)",
             (unsigned long)duty_percentage, (unsigned long)duty);

    ledc_set_duty(PWM_MODE, DRAIN_PUMP_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, DRAIN_PUMP_CHANNEL);

    update_actuator_info(ACTUATOR_IDX_DRAIN_PUMP, (float)duty_percentage);
}

void set_planter_pump_pwm(uint32_t duty_percentage)
{
    if (duty_percentage > 100) {
        duty_percentage = 100;
    }
    uint32_t duty = (duty_percentage * LEDC_MAX_DUTY_8BIT) / 100; // 0..255

    // ESP_LOGI(TAG, "Setting planter pump PWM to %lu%% (duty: %lu)",
    //          (unsigned long)duty_percentage, (unsigned long)duty);

    ledc_set_duty(PWM_MODE, PLANTER_PUMP_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PLANTER_PUMP_CHANNEL);

    update_actuator_info(ACTUATOR_IDX_PLANTER_PUMP, (float)duty_percentage);
}

void set_led_array_pwm(uint32_t duty_percentage)
{
    if (duty_percentage > 100) {
        duty_percentage = 100;
    }
    // Calculate the duty value (0..255 for 8-bit resolution)
    uint32_t duty = (duty_percentage * LEDC_MAX_DUTY_8BIT) / 100;

    ESP_LOGI(TAG, "Setting LED array PWM to %lu%% (duty: %lu)",
             (unsigned long)duty_percentage, (unsigned long)duty);

    // Set the duty cycle for the LED channel
    ledc_set_duty(PWM_MODE, LED_ARRAY_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, LED_ARRAY_CHANNEL);

    // Update actuator info: for the LED array, 0% (off) or 100% (fully on) 
    // can be treated as before, or you can show the actual percentage
    update_actuator_info(ACTUATOR_IDX_LED_ARRAY, (float)duty_percentage);
}

void set_led_array_binary(bool state)
{
    ESP_LOGI(TAG, "Setting LED array to %s", state ? "ON" : "OFF");
    gpio_set_level(LED_ARRAY_GPIO, state ? 1 : 0);

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
