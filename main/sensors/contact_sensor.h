#ifndef CONTACT_SENSOR_H
#define CONTACT_SENSOR_H

#include "esp_err.h"

// I2C address of the MCP23017 expander (change if AD0 is set high)
#ifndef CONTACT_SENSOR_I2C_ADDR
#define CONTACT_SENSOR_I2C_ADDR 0x20
#endif

// Measurement configuration
#ifndef CONTACT_SENSOR_STEP_MM
#define CONTACT_SENSOR_STEP_MM        5    // mm between pads
#endif 

#ifndef CONTACT_SENSOR_MEASURE_HEIGHT_MM
#define CONTACT_SENSOR_MEASURE_HEIGHT_MM 70 // total height to measure in mm
#endif

// Number of contact pads needed (height divided by step)
#define CONTACT_SENSOR_PAD_COUNT (CONTACT_SENSOR_MEASURE_HEIGHT_MM / CONTACT_SENSOR_STEP_MM)
// Ensure we do not exceed available GPIO pins
_Static_assert(CONTACT_SENSOR_PAD_COUNT <= 16, "Contact sensor pad count exceeds 16 available MCP23017 pins");

// Maximum measurable height is the configured measurement height
#define CONTACT_SENSOR_MAX_HEIGHT_MM CONTACT_SENSOR_MEASURE_HEIGHT_MM

// Default debounce threshold (number of consecutive identical readings required)
#ifndef CONTACT_SENSOR_DEBOUNCE_COUNT
#define CONTACT_SENSOR_DEBOUNCE_COUNT 3
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the MCP23017 for contact sensor input
 *
 * Configures all GPIO A and B pins as inputs with internal pull-ups enabled
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t contact_sensor_init_hw(void);

/**
 * @brief Read the current water level via contact pads
 *
 * Reads the GPIO registers of MCP23017 and finds the lowest pad that is grounded
 * (logic low), mapping it to a height in mm based on pad spacing.
 *
 * This function debounces brief false contacts using CONTACT_SENSOR_DEBOUNCE_COUNT.
 *
 * @return Water level in mm (0 to CONTACT_SENSOR_MEASURE_HEIGHT_MM) or negative on error/no stable contact
 */
int contact_sensor_read_mm_actual(void);

#ifdef __cplusplus
}
#endif

#endif // CONTACT_SENSOR_H
