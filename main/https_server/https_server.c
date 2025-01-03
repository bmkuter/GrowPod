// https_server/https_server.c

#include "https_server.h"
#include "esp_https_server.h"
#include "esp_log.h"
#include "cJSON.h"

// Your includes
#include "actuator_control.h"   // For set_air_pump_pwm, set_water_pump_pwm, set_servo_angle, set_led_array_binary
#include "ina260.h"             // For sensor reading
#include "flowmeter_control.h"  // For flow rate reading
#include "control_logic.h"      // For system_state_t, get_system_state, etc.

static const char *TAG = "HTTPS_SERVER";

// Forward declarations of request handlers
static esp_err_t airpump_post_handler(httpd_req_t *req);
static esp_err_t waterpump_post_handler(httpd_req_t *req);
static esp_err_t servo_post_handler(httpd_req_t *req);   // NEW servo endpoint
static esp_err_t led_post_handler(httpd_req_t *req);
static esp_err_t sensors_get_handler(httpd_req_t *req);

// HTTPS server task
void start_https_server(void) {
    httpd_ssl_config_t config = HTTPD_SSL_CONFIG_DEFAULT();

    // Access embedded server certificate and key
    extern const unsigned char server_cert_pem_start[] asm("_binary_server_crt_start");
    extern const unsigned char server_cert_pem_end[]   asm("_binary_server_crt_end");
    extern const unsigned char server_key_pem_start[]  asm("_binary_server_key_start");
    extern const unsigned char server_key_pem_end[]    asm("_binary_server_key_end");

    config.servercert     = server_cert_pem_start;
    config.servercert_len = server_cert_pem_end - server_cert_pem_start;
    config.prvtkey_pem    = server_key_pem_start;
    config.prvtkey_len    = server_key_pem_end - server_key_pem_start;

    // If using mutual TLS (mTLS), supply a CA cert to verify client cert
    extern const unsigned char ca_cert_pem_start[] asm("_binary_root_ca_crt_start");
    extern const unsigned char ca_cert_pem_end[]   asm("_binary_root_ca_crt_end");
    config.cacert_pem     = ca_cert_pem_start;
    config.cacert_len     = ca_cert_pem_end - ca_cert_pem_start;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_ssl_start(&server, &config);
    if (ret == ESP_OK) {
        // Register URI handler for air pump
        httpd_uri_t airpump_post_uri = {
            .uri      = "/api/actuators/airpump",
            .method   = HTTP_POST,
            .handler  = airpump_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &airpump_post_uri);

        // Register URI handler for water pump
        httpd_uri_t waterpump_post_uri = {
            .uri      = "/api/actuators/waterpump",
            .method   = HTTP_POST,
            .handler  = waterpump_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &waterpump_post_uri);

        // Register URI handler for the NEW servo endpoint
        httpd_uri_t servo_post_uri = {
            .uri      = "/api/actuators/servo",
            .method   = HTTP_POST,
            .handler  = servo_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &servo_post_uri);

        // Register URI handler for LED array
        httpd_uri_t led_post_uri = {
            .uri      = "/api/actuators/led",
            .method   = HTTP_POST,
            .handler  = led_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &led_post_uri);

        // Register URI handler for fetching sensor data
        httpd_uri_t sensors_get_uri = {
            .uri      = "/api/sensors",
            .method   = HTTP_GET,
            .handler  = sensors_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &sensors_get_uri);

        ESP_LOGI(TAG, "HTTPS server started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTPS server");
    }
}

//-----------------------------------------------------------------------------------
// Handler for air pump POST request (JSON: {"value": <0..100>})
//-----------------------------------------------------------------------------------
static esp_err_t airpump_post_handler(httpd_req_t *req)
{
    char content[100];
    int ret = httpd_req_recv(req, content, req->content_len);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive request data");
        return ESP_FAIL;
    }
    content[ret] = '\0';

    // Parse JSON
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON received");
        return ESP_FAIL;
    }

    cJSON *value = cJSON_GetObjectItem(json, "value");
    if (!value || !cJSON_IsNumber(value)) {
        ESP_LOGE(TAG, "Invalid or missing 'value' in JSON");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    uint32_t duty = value->valueint;
    set_air_pump_pwm(duty);

    cJSON_Delete(json);

    httpd_resp_sendstr(req, "Air pump updated");
    return ESP_OK;
}

//-----------------------------------------------------------------------------------
// Handler for water pump POST request (JSON: {"value": <0..100>})
//-----------------------------------------------------------------------------------
static esp_err_t waterpump_post_handler(httpd_req_t *req)
{
    char content[100];
    int ret = httpd_req_recv(req, content, req->content_len);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive request data");
        return ESP_FAIL;
    }
    content[ret] = '\0';

    // Parse JSON
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON received");
        return ESP_FAIL;
    }

    cJSON *value = cJSON_GetObjectItem(json, "value");
    if (!value || !cJSON_IsNumber(value)) {
        ESP_LOGE(TAG, "Invalid or missing 'value' in JSON");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    uint32_t duty = value->valueint;
    set_water_pump_pwm(duty);

    cJSON_Delete(json);

    httpd_resp_sendstr(req, "Water pump updated");
    return ESP_OK;
}

//-----------------------------------------------------------------------------------
// NEW Handler for servo POST request (JSON: {"angle": <0..180>})
//-----------------------------------------------------------------------------------
static esp_err_t servo_post_handler(httpd_req_t *req)
{
    char content[100];
    int ret = httpd_req_recv(req, content, req->content_len);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive request data");
        return ESP_FAIL;
    }
    content[ret] = '\0';

    // Parse JSON
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON received");
        return ESP_FAIL;
    }

    cJSON *angle_json = cJSON_GetObjectItem(json, "angle");
    if (!angle_json || !cJSON_IsNumber(angle_json)) {
        ESP_LOGE(TAG, "Invalid or missing 'angle' in JSON");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    uint32_t angle_val = angle_json->valueint;
    set_servo_angle(angle_val);

    cJSON_Delete(json);

    httpd_resp_sendstr(req, "Servo angle updated");
    return ESP_OK;
}

//-----------------------------------------------------------------------------------
// Handler for LED POST request (JSON: {"state": "on"|"off"})
//-----------------------------------------------------------------------------------
static esp_err_t led_post_handler(httpd_req_t *req)
{
    char content[100];
    int ret = httpd_req_recv(req, content, req->content_len);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive request data");
        return ESP_FAIL;
    }
    content[ret] = '\0';  // Null-terminate the content

    ESP_LOGI(TAG, "Received content: %s", content);

    // Parse JSON
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON received");
        return ESP_FAIL;
    }

    cJSON *state = cJSON_GetObjectItem(json, "state");
    if (!state || !cJSON_IsString(state)) {
        ESP_LOGE(TAG, "Invalid or missing 'state' in JSON");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    // Set LED array state: "on" -> 1, "off" -> 0
    if (strcmp(state->valuestring, "on") == 0) {
        set_led_array_binary(true);
        httpd_resp_sendstr(req, "LED array turned ON");
    } else if (strcmp(state->valuestring, "off") == 0) {
        set_led_array_binary(false);
        httpd_resp_sendstr(req, "LED array turned OFF");
    } else {
        ESP_LOGE(TAG, "Invalid 'state' value, must be 'on' or 'off'");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    cJSON_Delete(json);
    return ESP_OK;
}

//-----------------------------------------------------------------------------------
// Handler for sensors GET request
//-----------------------------------------------------------------------------------
static esp_err_t sensors_get_handler(httpd_req_t *req)
{
    // Create JSON response
    cJSON *json = cJSON_CreateObject();
    cJSON *table = cJSON_CreateArray();  // JSON array for the table

    // Variables to hold sensor data
    float current_led = 0.0f, voltage_led = 0.0f, power_led = 0.0f;
    float current_drain = 0.0f, voltage_drain = 0.0f, power_drain = 0.0f;
    float current_source = 0.0f, voltage_source = 0.0f, power_source = 0.0f;
    float current_air = 0.0f, voltage_air = 0.0f, power_air = 0.0f;

    // Read data for each actuator
    ina260_read_current(INA260_LED_ADDRESS,    &current_led);
    ina260_read_voltage(INA260_LED_ADDRESS,    &voltage_led);
    ina260_read_power(INA260_LED_ADDRESS,      &power_led);

    ina260_read_current(INA260_DRAIN_ADDRESS,  &current_drain);
    ina260_read_voltage(INA260_DRAIN_ADDRESS,  &voltage_drain);
    ina260_read_power(INA260_DRAIN_ADDRESS,    &power_drain);

    ina260_read_current(INA260_SOURCE_ADDRESS, &current_source);
    ina260_read_voltage(INA260_SOURCE_ADDRESS, &voltage_source);
    ina260_read_power(INA260_SOURCE_ADDRESS,   &power_source);

    ina260_read_current(INA260_AIR_ADDRESS,    &current_air);
    ina260_read_voltage(INA260_AIR_ADDRESS,    &voltage_air);
    ina260_read_power(INA260_AIR_ADDRESS,      &power_air);

    // Flow rates
    float drain_flow    = get_drain_flow_rate();
    float source_flow   = get_source_flow_rate();
    float overflow_flow = get_overflow_flow_rate();

    // LED row
    cJSON *row_led = cJSON_CreateObject();
    cJSON_AddStringToObject(row_led, "actuator", "LED");
    cJSON_AddNumberToObject(row_led, "current_mA",   current_led);
    cJSON_AddNumberToObject(row_led, "voltage_mV",   voltage_led);
    cJSON_AddNumberToObject(row_led, "power_mW",     power_led);
    cJSON_AddItemToArray(table, row_led);

    // Drain row
    cJSON *row_drain = cJSON_CreateObject();
    cJSON_AddStringToObject(row_drain, "actuator", "Drain");
    cJSON_AddNumberToObject(row_drain, "current_mA",   current_drain);
    cJSON_AddNumberToObject(row_drain, "voltage_mV",   voltage_drain);
    cJSON_AddNumberToObject(row_drain, "power_mW",     power_drain);
    cJSON_AddNumberToObject(row_drain, "flow_rate_L_min", drain_flow);
    cJSON_AddItemToArray(table, row_drain);

    // Source row
    cJSON *row_source = cJSON_CreateObject();
    cJSON_AddStringToObject(row_source, "actuator", "Source");
    cJSON_AddNumberToObject(row_source, "current_mA",   current_source);
    cJSON_AddNumberToObject(row_source, "voltage_mV",   voltage_source);
    cJSON_AddNumberToObject(row_source, "power_mW",     power_source);
    cJSON_AddNumberToObject(row_source, "flow_rate_L_min", source_flow);
    cJSON_AddItemToArray(table, row_source);

    // Overflow row (example usage)
    cJSON *row_overflow = cJSON_CreateObject();
    cJSON_AddStringToObject(row_overflow, "actuator", "Overflow");
    cJSON_AddNumberToObject(row_overflow, "flow_rate_L_min", overflow_flow);
    cJSON_AddItemToArray(table, row_overflow);

    // Air pump row
    cJSON *row_air = cJSON_CreateObject();
    cJSON_AddStringToObject(row_air, "actuator", "Air Pump");
    cJSON_AddNumberToObject(row_air, "current_mA",   current_air);
    cJSON_AddNumberToObject(row_air, "voltage_mV",   voltage_air);
    cJSON_AddNumberToObject(row_air, "power_mW",     power_air);
    cJSON_AddItemToArray(table, row_air);

    // Add the table to the main JSON object
    cJSON_AddItemToObject(json, "sensors_data", table);

    // Get current system state
    system_state_t state = get_system_state();
    const char *system_status_str = get_system_status_string(state);
    cJSON_AddStringToObject(json, "system_status", system_status_str);

    // Convert JSON to string
    char *response = cJSON_PrintUnformatted(json);

    // Send response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);

    // Clean up
    cJSON_Delete(json);
    free(response);

    return ESP_OK;
}
