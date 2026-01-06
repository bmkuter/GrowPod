#include "fdc1004_distance_sensor.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "power_monitor_HAL.h"  // For shared I2C bus configuration
#include <math.h>  // For fabsf

/*
 * FDC1004 Capacitive Distance Sensor Driver
 * 
 * Measurement Architecture:
 * - FDC1004 operates in CONTINUOUS REPEAT mode (REPEAT=1)
 * - Hardware measures at 400 S/s continuously in background
 * - Each measurement is internally averaged by the sigma-delta modulator
 * - We read the latest result whenever needed (no triggering required)
 * - Additional software moving average filter for extra stability
 * 
 * Advantages of this approach:
 * - Hardware does the heavy lifting (400 measurements/sec)
 * - No polling/trigger overhead - just read latest value
 * - Can poll infrequently (every 1-2 seconds) and still get fresh data
 * - Better noise rejection from continuous hardware averaging
 * - Lower CPU usage compared to software-triggered measurements
 * 
 * Register 0x0C (FDC_CONF) bit layout (verified against Arduino library):
 *   [15:12] Reserved
 *   [11:10] RATE (sample rate: 01=100S/s, 10=200S/s, 11=400S/s)
 *   [9:8]   REPEAT (00=single-shot, 01=continuous repeat) ← Set to 01
 *   [7:4]   MEAS_x enable bits (bit7=MEAS1 enable, bit6=MEAS2, etc)
 *   [3:0]   DONE_x status bits (read 1 when complete: bit3=DONE1, bit2=DONE2, etc)
 * 
 * Note: Measurement numbers are 0-indexed in registers but 1-indexed in API
 *       (measurement 1 = MEAS1 = bit 7 enable, bit 3 done)
 */

static const char *TAG = "FDC1004";

// FDC1004 uses the shared I2C bus
static i2c_port_t i2c_port = I2C_NUM_0;

// Calibration parameters (to be determined during calibration)
static float s_cap_to_mm_scale = FDC1004_CAP_TO_MM_SCALE;
static float s_cap_to_mm_offset = FDC1004_CAP_TO_MM_OFFSET;

// Calibration data storage
static fdc1004_calibration_t s_calibration = {
    .capdac = 3,              // Default starting value
    .cap_empty_pf = 0.0f,
    .cap_full_pf = 0.0f,
    .height_mm = 0.0f,
    .cap_per_mm = 0.0f,
    .is_calibrated = false
};

// Moving average filter for stability
#define FDC1004_FILTER_SIZE 1
static float s_cap_readings[FDC1004_FILTER_SIZE] = {0};
static int s_filter_index = 0;
static bool s_filter_full = false;

// Helper function to write to FDC1004 register (16-bit)
static esp_err_t fdc1004_write_register(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {
        reg,
        (uint8_t)(value >> 8),   // MSB
        (uint8_t)(value & 0xFF)  // LSB
    };
    
    esp_err_t ret = i2c_master_write_to_device(i2c_port, FDC1004_I2C_ADDR, 
                                                data, sizeof(data), 
                                                1000 / portTICK_PERIOD_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

// Helper function to read from FDC1004 register (16-bit)
static esp_err_t fdc1004_read_register(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    
    esp_err_t ret = i2c_master_write_read_device(i2c_port, FDC1004_I2C_ADDR,
                                                   &reg, 1,
                                                   data, sizeof(data),
                                                   1000 / portTICK_PERIOD_MS);
    
    if (ret == ESP_OK) {
        *value = ((uint16_t)data[0] << 8) | data[1];
    } else {
        ESP_LOGE(TAG, "Failed to read register 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t fdc1004_init(void)
{
    ESP_LOGI(TAG, "Initializing FDC1004 capacitive distance sensor...");
    
    // The I2C bus is already initialized by power_monitor_HAL
    // We just need to verify the device is present and configure it
    
    // Read device and manufacturer IDs to verify presence
    uint16_t manufacturer_id, device_id;
    esp_err_t ret = fdc1004_read_device_id(&manufacturer_id, &device_id);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FDC1004 not found on I2C bus");
        return ret;
    }
    
    ESP_LOGI(TAG, "FDC1004 found - Manufacturer ID: 0x%04x, Device ID: 0x%04x", 
             manufacturer_id, device_id);
    
    // Configure measurement on channel 1 in CONTINUOUS REPEAT mode
    // Configuration:
    // - Channel 1 (CIN1) to CAPDAC (single-ended)
    // - CAPDAC = 3 (9.375 pF offset to center measurement range)
    //   Empty baseline ~9-10 pF, so CAPDAC=3 centers around 0 pF
    //   This gives best resolution: ±15 pF range around the operating point
    // - Sample rate = 400 S/s (400 measurements per second)
    // - REPEAT mode = ENABLED (continuous background measurement)
    //   FDC1004 continuously measures and updates result register
    //   We just read latest value when needed (no trigger required)
    ret = fdc1004_configure_measurement(1, 3, FDC1004_SAMPLE_RATE_400);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "FDC1004 initialized: 400 S/s continuous mode (hardware averaging)");
        ESP_LOGI(TAG, "Sensor will measure continuously in background");
    } else {
        ESP_LOGE(TAG, "Failed to configure FDC1004");
    }
    
    return ret;
}

esp_err_t fdc1004_configure_measurement(uint8_t channel, uint8_t capdac, uint8_t sample_rate)
{
    if (channel < 1 || channel > 4) {
        ESP_LOGE(TAG, "Invalid channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (capdac > FDC1004_CAPDAC_MAX) {
        ESP_LOGE(TAG, "Invalid CAPDAC value: %d", capdac);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate configuration register address
    uint8_t conf_reg = FDC1004_REG_CONF_MEAS1 + (channel - 1);
    
    // Configuration format (CONF_MEASx register):
    // [15:13] CHA (positive input channel, e.g., CIN1 = 0b000)
    // [12:10] CHB (negative input channel, CAPDAC = 0b100 for single-ended)
    // [9:5]   CAPDAC value (offset capacitance if >15pF)
    // [4:0]   Reserved
    
    uint16_t conf_value = (0b000 << 13) |  // CHA = CIN1 (adjust for other channels)
                          (0b100 << 10) |  // CHB = CAPDAC (single-ended mode)
                          ((uint16_t)capdac << 5);
    
    esp_err_t ret = fdc1004_write_register(conf_reg, conf_value);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure FDC_CONF register for REPEAT mode (continuous measurement)
    // Per Arduino library: RATE is bits [11:10], not [7:6]!
    // [15:12] Reserved
    // [11:10] RATE (sample rate: 01=100S/s, 10=200S/s, 11=400S/s)
    // [9:8]   REPEAT mode (01 = repeat, 00 = single shot)
    // [7:4]   MEAS_x enable bits (set which channels to measure continuously)
    // [3:0]   DONE_x status bits (set by hardware after each measurement)
    
    uint8_t meas_bit_position = 8 - channel;  // Channel 1 → bit 7
    
    uint16_t fdc_conf = (sample_rate << 10) |   // RATE bits [11:10]
                        (0b01 << 8) |            // REPEAT = 1 (continuous mode)
                        (1 << meas_bit_position); // Enable MEAS_x for this channel
    
    ESP_LOGI(TAG, "Configuring FDC_CONF = 0x%04x (REPEAT=1 continuous, RATE=%d, MEAS_%d enabled)", 
             fdc_conf, sample_rate, channel);
    
    ret = fdc1004_write_register(FDC1004_REG_FDC_CONF, fdc_conf);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read back to verify the write took effect
    uint16_t fdc_conf_readback;
    ret = fdc1004_read_register(FDC1004_REG_FDC_CONF, &fdc_conf_readback);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "FDC_CONF readback = 0x%04x (wrote 0x%04x)", fdc_conf_readback, fdc_conf);
        if (fdc_conf_readback != fdc_conf) {
            ESP_LOGW(TAG, "FDC_CONF readback mismatch! Expected 0x%04x, got 0x%04x", 
                     fdc_conf, fdc_conf_readback);
        }
    }
    
    return ESP_OK;
}

esp_err_t fdc1004_trigger_measurement(uint8_t channel)
{
    if (channel < 1 || channel > 4) {
        ESP_LOGE(TAG, "Invalid channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Per Arduino library (github.com/beshaya/FDC1004):
    // trigger_data = ((uint16_t)rate) << 10;  // RATE at bits [11:10]!
    // trigger_data |= 0 << 8;                 // REPEAT disabled
    // trigger_data |= (1 << (7-measurement)); // MEAS bit
    //
    // For measurement 1 at 100S/s: 0x0480
    // Breaking down: 0x0480 = 0b 0000 0100 1000 0000
    //   Bits [11:10] RATE = 0b01 (100 S/s)
    //   Bits [9:8] REPEAT = 0b00 (single)
    //   Bit 7 (MEAS_1) = 1
    
    // Read current to preserve RATE setting
    uint16_t fdc_conf;
    esp_err_t ret = fdc1004_read_register(FDC1004_REG_FDC_CONF, &fdc_conf);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGD(TAG, "FDC_CONF before trigger: 0x%04x", fdc_conf);
    
    // Build new value: REPEAT=0, preserve RATE, set MEAS bit, clear DONE bits
    // MEAS_1=bit7, MEAS_2=bit6, MEAS_3=bit5, MEAS_4=bit4
    uint8_t meas_bit_position = 8 - channel;     // Channel 1→bit7, 2→bit6, etc.
    uint16_t rate = (fdc_conf >> 10) & 0x03;     // Extract RATE from bits [11:10] - FIXED!
    
    ESP_LOGD(TAG, "  Extracted RATE = 0x%x (bits 11-10)", rate);
    ESP_LOGD(TAG, "  Will set MEAS_%d at bit position %d", channel, meas_bit_position);
    
    fdc_conf = (rate << 10) |                    // RATE at bits [11:10] - FIXED!
               (0 << 8) |                         // REPEAT = 0 (single measurement)
               (1 << meas_bit_position);          // Set MEAS_x bit
    
    ESP_LOGD(TAG, "Triggering single measurement: FDC_CONF = 0x%04x (REPEAT=0, MEAS_%d bit %d)", 
             fdc_conf, channel, meas_bit_position);
    ESP_LOGD(TAG, "  Breakdown: RATE<<10=0x%04x, MEAS_bit<<7=0x%04x",
             rate << 10, 1 << meas_bit_position);
    
    return fdc1004_write_register(FDC1004_REG_FDC_CONF, fdc_conf);
}

esp_err_t fdc1004_is_measurement_ready(uint8_t channel, bool *is_ready)
{
    if (channel < 1 || channel > 4 || is_ready == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read FDC_CONF register
    uint16_t fdc_conf;
    esp_err_t ret = fdc1004_read_register(FDC1004_REG_FDC_CONF, &fdc_conf);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Check DONE_x bit (bits [3:0]) - set to 1 when measurement complete
    // Per Arduino library: bit position = (3 - measurement)
    // For measurement 0 (channel 0 in lib): bit 3 = DONE_1
    // For measurement 1 (channel 1 in lib): bit 2 = DONE_2
    // Our API is 1-indexed, so: channel 1 -> bit (4 - 1) = bit 3
    *is_ready = ((fdc_conf >> (4 - channel)) & 0x01) != 0;
    
    ESP_LOGD(TAG, "Channel %d DONE bit = %d, ready = %s", 
             channel, (*is_ready ? 1 : 0), *is_ready ? "YES" : "NO");
    
    return ESP_OK;
}

esp_err_t fdc1004_read_capacitance(float *capacitance)
{
    if (capacitance == NULL) {
        ESP_LOGE(TAG, "Null capacitance pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "=== Reading latest FDC1004 measurement (continuous mode) ===");
    
    // Read current CAPDAC setting from CONF_MEAS1 register
    uint16_t conf_meas1;
    esp_err_t ret = fdc1004_read_register(FDC1004_REG_CONF_MEAS1, &conf_meas1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CONF_MEAS1");
        return ret;
    }
    
    uint8_t capdac = (conf_meas1 >> 5) & 0x1F;  // Extract CAPDAC from bits [9:5]
    ESP_LOGD(TAG, "Current CAPDAC = %d (%.3f pF offset)", capdac, capdac * 3.125f);
    
    // In REPEAT mode, FDC1004 is continuously measuring
    // We just read the latest result directly - no trigger needed!
    // The DONE bit will be set every time a new measurement completes (at 400 S/s)
    
    // Optional: Check DONE bit to ensure fresh data (not strictly necessary)
    bool measurement_done = false;
    ret = fdc1004_is_measurement_ready(1, &measurement_done);
    if (ret == ESP_OK && !measurement_done) {
        // Wait briefly for next measurement to complete
        vTaskDelay(pdMS_TO_TICKS(3));  // Max 2.5ms at 400 S/s, give it 3ms
        ret = fdc1004_is_measurement_ready(1, &measurement_done);
    }
    
    // Read MSB and LSB (latest measurement result)
    uint16_t msb, lsb;
    ret = fdc1004_read_register(FDC1004_REG_MEAS1_MSB, &msb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MEAS1_MSB");
        return ret;
    }
    
    ret = fdc1004_read_register(FDC1004_REG_MEAS1_LSB, &lsb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MEAS1_LSB");
        return ret;
    }
    
    // Combine MSB and LSB to get 24-bit signed value
    int32_t raw_value_24 = ((int32_t)msb << 8) | (lsb >> 8);
    
    // Sign extend from 24-bit to 32-bit
    if (raw_value_24 & 0x800000) {
        raw_value_24 |= 0xFF000000;
    }
    
    // Convert to capacitance in pF
    // FDC1004 LSB = 3.125 fF (femtofarads) = 3.125e-6 pF
    // Total capacitance = (raw_value * 3.125e-6) + (CAPDAC * 3.125)
    float cap_from_raw = (float)raw_value_24 * 3.125e-6f;
    float cap_from_capdac = (float)capdac * 3.125f;
    *capacitance = cap_from_raw + cap_from_capdac;
    
    ESP_LOGD(TAG, "=== FDC1004 reading complete ===");
    ESP_LOGD(TAG, "MSB = 0x%04x (%d), LSB = 0x%04x", msb, (int16_t)msb, lsb);
    ESP_LOGD(TAG, "Raw value (24-bit) = 0x%06lx (%ld)", 
             (unsigned long)(raw_value_24 & 0xFFFFFF), (long)raw_value_24);
    ESP_LOGD(TAG, "Capacitance from measurement = %.4f pF", cap_from_raw);
    ESP_LOGD(TAG, "Capacitance from CAPDAC = %.4f pF", cap_from_capdac);
    ESP_LOGD(TAG, "Total capacitance = %.4f pF", *capacitance);
    
    // Diagnostic info (check based on total capacitance, not raw MSB)
    if (msb == 0x7FFF) {
        ESP_LOGW(TAG, "⚠️  POSITIVE SATURATION (0x7FFF) - Capacitance > measurement range!");
        ESP_LOGW(TAG, "    Increase CAPDAC to shift measurement range higher");
    } else if (msb == 0x8000) {
        ESP_LOGW(TAG, "⚠️  NEGATIVE SATURATION (0x8000) - Capacitance < measurement range!");
        ESP_LOGW(TAG, "    Decrease CAPDAC to shift measurement range lower");
    } else if (*capacitance > 100.0f) {
        ESP_LOGW(TAG, "⚠️  Very high capacitance (>100pF). Check sensor configuration");
    } else if (*capacitance < -10.0f) {
        ESP_LOGW(TAG, "⚠️  Negative capacitance. Check CAPDAC setting or sensor connections");
    } else {
        ESP_LOGD(TAG, "✓ Reading in valid range");
    }
    
    return ESP_OK;
}

esp_err_t fdc1004_calibrate_water_min(float min_height_mm)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "FDC1004 Calibration - Step 1: MIN LEVEL");
    ESP_LOGI(TAG, "Minimum height: %.1f mm", min_height_mm);
    ESP_LOGI(TAG, "========================================");
    
    // Step 1: Measure baseline with CAPDAC=0
    ESP_LOGI(TAG, "Measuring baseline capacitance (CAPDAC=0)...");
    esp_err_t ret = fdc1004_configure_measurement(1, 0, FDC1004_SAMPLE_RATE_400);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure for baseline measurement");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Take multiple readings and average
    const int num_samples = 10;
    float cap_sum = 0.0f;
    int valid_samples = 0;
    
    ESP_LOGI(TAG, "Taking %d samples...", num_samples);
    for (int i = 0; i < num_samples; i++) {
        float cap;
        ret = fdc1004_read_capacitance(&cap);
        if (ret == ESP_OK) {
            cap_sum += cap;
            valid_samples++;
            ESP_LOGI(TAG, "  Sample %d: %.4f pF", i + 1, cap);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (valid_samples == 0) {
        ESP_LOGE(TAG, "Failed to read baseline capacitance");
        return ESP_FAIL;
    }
    
    float cap_baseline = cap_sum / valid_samples;
    ESP_LOGI(TAG, "✓ Baseline (CAPDAC=0): %.4f pF (avg of %d samples)", cap_baseline, valid_samples);
    
    // Step 2: Calculate optimal CAPDAC
    uint8_t optimal_capdac = (uint8_t)(cap_baseline / 3.125f + 0.5f);
    if (optimal_capdac > FDC1004_CAPDAC_MAX) {
        optimal_capdac = FDC1004_CAPDAC_MAX;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Calculated optimal CAPDAC = %d (%.3f pF offset)", 
             optimal_capdac, optimal_capdac * 3.125f);
    ESP_LOGI(TAG, "This centers the measurement range for best resolution");
    
    // Step 3: Reconfigure with optimal CAPDAC
    ret = fdc1004_configure_measurement(1, optimal_capdac, FDC1004_SAMPLE_RATE_400);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure with optimal CAPDAC");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Step 4: Re-measure with CAPDAC applied
    cap_sum = 0.0f;
    valid_samples = 0;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Re-measuring with CAPDAC=%d...", optimal_capdac);
    for (int i = 0; i < num_samples; i++) {
        float cap;
        ret = fdc1004_read_capacitance(&cap);
        if (ret == ESP_OK) {
            cap_sum += cap;
            valid_samples++;
            ESP_LOGI(TAG, "  Sample %d: %.4f pF (with CAPDAC offset)", i + 1, cap);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    float cap_min = cap_sum / valid_samples;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "MIN LEVEL CALIBRATION COMPLETE");
    ESP_LOGI(TAG, "  Height:            %.1f mm", min_height_mm);
    ESP_LOGI(TAG, "  Raw capacitance:   %.4f pF (CAPDAC=0)", cap_baseline);
    ESP_LOGI(TAG, "  CAPDAC:            %d (%.3f pF offset)", optimal_capdac, optimal_capdac * 3.125f);
    ESP_LOGI(TAG, "  Final reading:     %.4f pF", cap_min);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Next step: Fill to MAX level and run:");
    ESP_LOGI(TAG, "  calibrate_water_max <max_height_mm>");
    ESP_LOGI(TAG, "");
    
    // Store partial calibration
    s_calibration.capdac = optimal_capdac;
    s_calibration.cap_empty_pf = cap_min;
    s_calibration.height_mm = min_height_mm;  // Store min height temporarily
    s_calibration.is_calibrated = false;  // Not complete yet
    
    return ESP_OK;
}

esp_err_t fdc1004_calibrate_water_max(float max_height_mm)
{
    // Check if min was calibrated first
    if (s_calibration.capdac == 0 && s_calibration.cap_empty_pf == 0.0f) {
        ESP_LOGE(TAG, "Must run calibrate_water_min first!");
        return ESP_ERR_INVALID_STATE;
    }
    
    float min_height_mm = s_calibration.height_mm;  // Retrieve stored min height
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "FDC1004 Calibration - Step 2: MAX LEVEL");
    ESP_LOGI(TAG, "Maximum height: %.1f mm", max_height_mm);
    ESP_LOGI(TAG, "Current CAPDAC: %d (%.3f pF offset)", 
             s_calibration.capdac, s_calibration.capdac * 3.125f);
    ESP_LOGI(TAG, "========================================");
    
    if (max_height_mm <= min_height_mm) {
        ESP_LOGE(TAG, "Max height (%.1f mm) must be greater than min height (%.1f mm)", 
                 max_height_mm, min_height_mm);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Measure full level
    const int num_samples = 10;
    float cap_sum = 0.0f;
    int valid_samples = 0;
    
    ESP_LOGI(TAG, "Measuring MAX level capacitance...");
    ESP_LOGI(TAG, "Taking %d samples...", num_samples);
    
    for (int i = 0; i < num_samples; i++) {
        float cap;
        esp_err_t ret = fdc1004_read_capacitance(&cap);
        if (ret == ESP_OK) {
            cap_sum += cap;
            valid_samples++;
            ESP_LOGI(TAG, "  Sample %d: %.4f pF", i + 1, cap);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (valid_samples == 0) {
        ESP_LOGE(TAG, "Failed to read full capacitance");
        return ESP_FAIL;
    }
    
    float cap_max = cap_sum / valid_samples;
    
    // Calculate calibration parameters
    float total_height = max_height_mm - min_height_mm;
    float cap_range = cap_max - s_calibration.cap_empty_pf;
    
    // Detect if sensor is inverted (capacitance decreases with water level)
    bool inverted = (cap_range < 0);
    if (inverted) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "ℹ️  Detected INVERTED sensor behavior");
        ESP_LOGI(TAG, "    (Capacitance decreases as water level increases)");
        ESP_LOGI(TAG, "    This is normal with floating ground plane configuration");
        cap_range = -cap_range;  // Make positive for calculations
    }
    
    float cap_per_mm = cap_range / total_height;
    
    // Store sign for inverted operation
    if (inverted) {
        cap_per_mm = -cap_per_mm;  // Negative slope
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "TWO-POINT CALIBRATION COMPLETE");
    ESP_LOGI(TAG, "  CAPDAC:            %d (%.3f pF offset)", 
             s_calibration.capdac, s_calibration.capdac * 3.125f);
    ESP_LOGI(TAG, "  Min level:");
    ESP_LOGI(TAG, "    Height:          %.1f mm", min_height_mm);
    ESP_LOGI(TAG, "    Capacitance:     %.4f pF", s_calibration.cap_empty_pf);
    ESP_LOGI(TAG, "  Max level:");
    ESP_LOGI(TAG, "    Height:          %.1f mm", max_height_mm);
    ESP_LOGI(TAG, "    Capacitance:     %.4f pF", cap_max);
    ESP_LOGI(TAG, "  Range:");
    ESP_LOGI(TAG, "    Height span:     %.1f mm", total_height);
    ESP_LOGI(TAG, "    Cap change:      %.4f pF%s", fabsf(cap_range), inverted ? " (inverted)" : "");
    ESP_LOGI(TAG, "    Sensitivity:     %.4f pF/mm%s", fabsf(cap_per_mm), inverted ? " (inverted)" : "");
    ESP_LOGI(TAG, "========================================");
    
    // Validate calibration
    if (fabsf(cap_range) < 0.5f) {
        ESP_LOGW(TAG, "⚠️  Very small capacitance change (%.4f pF)", fabsf(cap_range));
        ESP_LOGW(TAG, "    Calibration may be inaccurate. Check sensor positioning.");
    } else if (fabsf(cap_range) > 25.0f) {
        ESP_LOGW(TAG, "⚠️  Very large capacitance change (%.4f pF)", fabsf(cap_range));
        ESP_LOGW(TAG, "    May exceed measurement range at extremes.");
    } else {
        ESP_LOGI(TAG, "✓ Calibration looks good!");
    }
    
    // Complete calibration
    s_calibration.cap_full_pf = cap_max;
    s_calibration.height_mm = max_height_mm;  // Store max height
    s_calibration.cap_per_mm = cap_per_mm;
    s_calibration.is_calibrated = true;
    
    // Update conversion parameters
    s_cap_to_mm_scale = 1.0f / cap_per_mm;
    s_cap_to_mm_offset = s_calibration.cap_empty_pf;
    
    ESP_LOGI(TAG, "✓ Calibration applied and ready to use");
    
    // Auto-save calibration to filesystem
    esp_err_t save_ret = sensor_config_save_water_calibration(&s_calibration);
    if (save_ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Calibration saved to filesystem (persists across reboots)");
    } else {
        ESP_LOGW(TAG, "⚠️  Failed to save calibration to filesystem: %s", esp_err_to_name(save_ret));
        ESP_LOGW(TAG, "   Calibration is active but will be lost on reboot");
    }
    
    ESP_LOGI(TAG, "");
    
    return ESP_OK;
}

esp_err_t fdc1004_calibrate_water_level(float height_mm, fdc1004_calibration_t *calibration)
{
    if (calibration == NULL || height_mm <= 0) {
        ESP_LOGE(TAG, "Invalid calibration parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "FDC1004 Water Level Calibration");
    ESP_LOGI(TAG, "Calibration height: %.1f mm", height_mm);
    ESP_LOGI(TAG, "========================================");
    
    // Step 1: Measure empty capacitance with CAPDAC=0
    ESP_LOGI(TAG, "Step 1: Measuring EMPTY container baseline (CAPDAC=0)...");
    esp_err_t ret = fdc1004_configure_measurement(1, 0, FDC1004_SAMPLE_RATE_400);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure for empty measurement");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));  // Let configuration settle
    
    // Take multiple readings and average for stability
    const int num_samples = 10;
    float cap_empty_sum = 0.0f;
    int valid_samples = 0;
    
    for (int i = 0; i < num_samples; i++) {
        float cap;
        ret = fdc1004_read_capacitance(&cap);
        if (ret == ESP_OK) {
            cap_empty_sum += cap;
            valid_samples++;
            ESP_LOGD(TAG, "  Empty sample %d: %.4f pF", i + 1, cap);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (valid_samples == 0) {
        ESP_LOGE(TAG, "Failed to read empty capacitance");
        return ESP_FAIL;
    }
    
    float cap_empty = cap_empty_sum / valid_samples;
    ESP_LOGI(TAG, "✓ Empty baseline: %.4f pF (average of %d samples)", cap_empty, valid_samples);
    
    // Step 2: Calculate optimal CAPDAC to center the measurement range
    // We want the midpoint of empty and full to be near 0 pF after CAPDAC offset
    // Strategy: Set CAPDAC to approximately the empty baseline, giving us
    // headroom in both positive and negative directions
    
    uint8_t optimal_capdac = (uint8_t)(cap_empty / 3.125f + 0.5f);  // Round to nearest CAPDAC value
    if (optimal_capdac > FDC1004_CAPDAC_MAX) {
        optimal_capdac = FDC1004_CAPDAC_MAX;
    }
    
    ESP_LOGI(TAG, "Optimal CAPDAC = %d (%.3f pF offset)", optimal_capdac, optimal_capdac * 3.125f);
    ESP_LOGI(TAG, "This centers the measurement range for best resolution");
    
    // Reconfigure with optimal CAPDAC
    ret = fdc1004_configure_measurement(1, optimal_capdac, FDC1004_SAMPLE_RATE_400);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure with optimal CAPDAC");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Re-measure empty with new CAPDAC
    cap_empty_sum = 0.0f;
    valid_samples = 0;
    
    for (int i = 0; i < num_samples; i++) {
        float cap;
        ret = fdc1004_read_capacitance(&cap);
        if (ret == ESP_OK) {
            cap_empty_sum += cap;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    cap_empty = cap_empty_sum / valid_samples;
    ESP_LOGI(TAG, "✓ Empty with CAPDAC=%d: %.4f pF", optimal_capdac, cap_empty);
    
    // Step 3: Wait for user to fill container
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ACTION REQUIRED:");
    ESP_LOGI(TAG, "Fill water to the MAX FILL line (%.1f mm)", height_mm);
    ESP_LOGI(TAG, "Waiting 10 seconds for you to fill...");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    
    for (int i = 10; i > 0; i--) {
        ESP_LOGI(TAG, "  %d...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Step 4: Measure full capacitance
    ESP_LOGI(TAG, "Step 2: Measuring FULL container...");
    
    float cap_full_sum = 0.0f;
    valid_samples = 0;
    
    for (int i = 0; i < num_samples; i++) {
        float cap;
        ret = fdc1004_read_capacitance(&cap);
        if (ret == ESP_OK) {
            cap_full_sum += cap;
            valid_samples++;
            ESP_LOGD(TAG, "  Full sample %d: %.4f pF", i + 1, cap);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (valid_samples == 0) {
        ESP_LOGE(TAG, "Failed to read full capacitance");
        return ESP_FAIL;
    }
    
    float cap_full = cap_full_sum / valid_samples;
    ESP_LOGI(TAG, "✓ Full reading: %.4f pF (average of %d samples)", cap_full, valid_samples);
    
    // Step 5: Calculate calibration parameters
    float cap_range = cap_full - cap_empty;
    float cap_per_mm = cap_range / height_mm;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CALIBRATION RESULTS:");
    ESP_LOGI(TAG, "  CAPDAC:          %d (%.3f pF offset)", optimal_capdac, optimal_capdac * 3.125f);
    ESP_LOGI(TAG, "  Empty:           %.4f pF", cap_empty);
    ESP_LOGI(TAG, "  Full:            %.4f pF", cap_full);
    ESP_LOGI(TAG, "  Range:           %.4f pF", cap_range);
    ESP_LOGI(TAG, "  Height:          %.1f mm", height_mm);
    ESP_LOGI(TAG, "  Sensitivity:     %.4f pF/mm", cap_per_mm);
    ESP_LOGI(TAG, "========================================");
    
    // Validate calibration
    if (cap_range < 0.5f) {
        ESP_LOGW(TAG, "⚠️  Very small capacitance change (%.4f pF)", cap_range);
        ESP_LOGW(TAG, "    Calibration may be inaccurate. Check sensor positioning.");
    } else if (cap_range > 25.0f) {
        ESP_LOGW(TAG, "⚠️  Very large capacitance change (%.4f pF)", cap_range);
        ESP_LOGW(TAG, "    May exceed measurement range. Consider adjusting CAPDAC.");
    } else {
        ESP_LOGI(TAG, "✓ Calibration looks good!");
    }
    
    // Store calibration
    calibration->capdac = optimal_capdac;
    calibration->cap_empty_pf = cap_empty;
    calibration->cap_full_pf = cap_full;
    calibration->height_mm = height_mm;
    calibration->cap_per_mm = cap_per_mm;
    calibration->is_calibrated = true;
    
    // Apply to global state
    s_calibration = *calibration;
    s_cap_to_mm_scale = 1.0f / cap_per_mm;  // mm per pF
    s_cap_to_mm_offset = cap_empty;
    
    ESP_LOGI(TAG, "✓ Calibration applied and ready to use");
    
    // Auto-save calibration to filesystem
    esp_err_t save_ret = sensor_config_save_water_calibration(&s_calibration);
    if (save_ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Calibration saved to filesystem (persists across reboots)");
    } else {
        ESP_LOGW(TAG, "⚠️  Failed to save calibration to filesystem: %s", esp_err_to_name(save_ret));
        ESP_LOGW(TAG, "   Calibration is active but will be lost on reboot");
    }
    
    return ESP_OK;
}

esp_err_t fdc1004_get_calibration(fdc1004_calibration_t *calibration)
{
    if (calibration == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_calibration.is_calibrated) {
        ESP_LOGW(TAG, "Sensor not calibrated yet");
        return ESP_ERR_INVALID_STATE;
    }
    
    *calibration = s_calibration;
    return ESP_OK;
}

esp_err_t fdc1004_set_calibration(const fdc1004_calibration_t *calibration)
{
    if (calibration == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!calibration->is_calibrated) {
        ESP_LOGW(TAG, "Attempting to set invalid calibration");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Apply calibration
    s_calibration = *calibration;
    s_cap_to_mm_scale = 1.0f / calibration->cap_per_mm;
    s_cap_to_mm_offset = calibration->cap_empty_pf;
    
    // Reconfigure sensor with calibrated CAPDAC
    esp_err_t ret = fdc1004_configure_measurement(1, calibration->capdac, FDC1004_SAMPLE_RATE_400);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply calibration CAPDAC");
        return ret;
    }
    
    ESP_LOGI(TAG, "Calibration loaded:");
    ESP_LOGI(TAG, "  CAPDAC: %d, Range: %.4f-%.4f pF, Height: %.1f mm",
             calibration->capdac, calibration->cap_empty_pf, 
             calibration->cap_full_pf, calibration->height_mm);
    
    return ESP_OK;
}

int fdc1004_read_distance_mm(void)
{
    float capacitance;
    esp_err_t ret = fdc1004_read_capacitance(&capacitance);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read capacitance");
        return -1;
    }
    
    // Apply moving average filter for stability
    s_cap_readings[s_filter_index] = capacitance;
    s_filter_index = (s_filter_index + 1) % FDC1004_FILTER_SIZE;
    if (s_filter_index == 0) {
        s_filter_full = true;
    }
    
    // Calculate filtered capacitance
    float filtered_cap = 0.0f;
    int samples_to_use = s_filter_full ? FDC1004_FILTER_SIZE : (s_filter_index > 0 ? s_filter_index : 1);
    for (int i = 0; i < samples_to_use; i++) {
        filtered_cap += s_cap_readings[i];
    }
    filtered_cap /= samples_to_use;
    
    // Check if calibrated
    if (!s_calibration.is_calibrated) {
        // ESP_LOGW(TAG, "Sensor not calibrated - returning raw capacitance as mm");
        return (int)filtered_cap;
    }
    
    // Convert capacitance to water level height using calibration
    // water_level_mm = (current_cap - empty_cap) / (cap_per_mm)
    // Note: cap_per_mm can be negative for inverted sensors
    float cap_delta = filtered_cap - s_calibration.cap_empty_pf;
    float water_level_mm = cap_delta / s_calibration.cap_per_mm;
    
    // Clamp to valid range [0, height_mm]
    if (water_level_mm < 0.0f) {
        water_level_mm = 0.0f;
    } else if (water_level_mm > s_calibration.height_mm) {
        water_level_mm = s_calibration.height_mm;
    }
    
    // Single clean log line with water level and raw capacitance
    ESP_LOGI(TAG, "Water level: %d mm (%.2f pF raw, %.2f pF filtered)", 
             (int)water_level_mm, capacitance, filtered_cap);
    
    return (int)water_level_mm;
}

esp_err_t fdc1004_read_device_id(uint16_t *manufacturer_id, uint16_t *device_id)
{
    if (manufacturer_id == NULL || device_id == NULL) {
        ESP_LOGE(TAG, "Null pointer for device ID");
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = fdc1004_read_register(FDC1004_REG_MANUFACTURER_ID, manufacturer_id);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return fdc1004_read_register(FDC1004_REG_DEVICE_ID, device_id);
}
