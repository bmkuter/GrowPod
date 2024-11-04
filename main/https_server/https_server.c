// https_server/https_server.c

#include "https_server.h"
#include "esp_https_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "actuator_control.h"
#include "ina260.h"
#include "flowmeter_control.h"

static const char *TAG = "HTTPS_SERVER";

// Forward declarations of request handlers
static esp_err_t airpump_post_handler(httpd_req_t *req);
static esp_err_t waterpump_post_handler(httpd_req_t *req);
static esp_err_t solenoid_post_handler(httpd_req_t *req);
static esp_err_t led_post_handler(httpd_req_t *req);
static esp_err_t sensors_get_handler(httpd_req_t *req);

// HTTPS server task
void start_https_server(void) {
    httpd_ssl_config_t config = HTTPD_SSL_CONFIG_DEFAULT();

    // Access embedded server certificate and key
    extern const unsigned char server_cert_pem_start[] asm("_binary_server_crt_start");
    extern const unsigned char server_cert_pem_end[] asm("_binary_server_crt_end");
    extern const unsigned char server_key_pem_start[] asm("_binary_server_key_start");
    extern const unsigned char server_key_pem_end[] asm("_binary_server_key_end");

    config.servercert = server_cert_pem_start;
    config.servercert_len = server_cert_pem_end - server_cert_pem_start;
    config.prvtkey_pem = server_key_pem_start;
    config.prvtkey_len = server_key_pem_end - server_key_pem_start;

    // (Optional) If using mTLS, access the Root CA certificate to verify client certs
    extern const unsigned char ca_cert_pem_start[] asm("_binary_root_ca_crt_start");
    extern const unsigned char ca_cert_pem_end[] asm("_binary_root_ca_crt_end");

    // Uncomment the following lines to enable mutual TLS (mTLS)
    config.cacert_pem = ca_cert_pem_start;
    config.cacert_len = ca_cert_pem_end - ca_cert_pem_start;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_ssl_start(&server, &config);
    if (ret == ESP_OK) {
        // Register URI handler for air pump
        httpd_uri_t airpump_post_uri = {
            .uri = "/api/actuators/airpump",
            .method = HTTP_POST,
            .handler = airpump_post_handler,  // This should be your handler function for air pump
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &airpump_post_uri);

        // Register URI handler for water pump
        httpd_uri_t waterpump_post_uri = {
            .uri = "/api/actuators/waterpump",
            .method = HTTP_POST,
            .handler = waterpump_post_handler,  // This should be your handler function for water pump
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &waterpump_post_uri);

        // Register URI handler for solenoid valve
        httpd_uri_t solenoid_post_uri = {
            .uri = "/api/actuators/solenoid",
            .method = HTTP_POST,
            .handler = solenoid_post_handler,  // This should be your handler function for solenoid valve
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &solenoid_post_uri);

        // Register URI handler for LED array
        httpd_uri_t led_post_uri = {
            .uri = "/api/actuators/led",
            .method = HTTP_POST,
            .handler = led_post_handler,  // This should be your handler function for LED array
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &led_post_uri);

        // Register URI handler for fetching sensor data
        httpd_uri_t sensors_get_uri = {
            .uri = "/api/sensors",
            .method = HTTP_GET,
            .handler = sensors_get_handler,  // This should be your handler function for sensors
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &sensors_get_uri);

        ESP_LOGI("HTTPS_SERVER", "HTTPS server started successfully");
    } else {
        ESP_LOGE("HTTPS_SERVER", "Failed to start HTTPS server");
    }
}

// Handler for airpump POST request
static esp_err_t airpump_post_handler(httpd_req_t *req) {
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

// Handler for water pump POST request
static esp_err_t waterpump_post_handler(httpd_req_t *req) {
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

// Handler for solenoid POST request
static esp_err_t solenoid_post_handler(httpd_req_t *req) {
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

    cJSON *state = cJSON_GetObjectItem(json, "state");
    if (!state || !cJSON_IsString(state)) {
        ESP_LOGE(TAG, "Invalid or missing 'state' in JSON");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    // Set solenoid state: "on" -> 1, "off" -> 0
    if (strcmp(state->valuestring, "on") == 0) {
        set_solenoid_valve(1);
        httpd_resp_sendstr(req, "Solenoid valve turned ON");
    } else if (strcmp(state->valuestring, "off") == 0) {
        set_solenoid_valve(0);
        httpd_resp_sendstr(req, "Solenoid valve turned OFF");
    } else {
        ESP_LOGE(TAG, "Invalid 'state' value, must be 'on' or 'off'");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    cJSON_Delete(json);
    return ESP_OK;
}

// Handler for LED POST request
static esp_err_t led_post_handler(httpd_req_t *req) {
    char content[100];
    int ret = httpd_req_recv(req, content, req->content_len);
    if (ret <= 0) {
        ESP_LOGE("HTTPS_SERVER", "Failed to receive request data");
        return ESP_FAIL;
    }
    content[ret] = '\0';  // Null-terminate the content

    // Log the received JSON content for debugging
    ESP_LOGI("HTTPS_SERVER", "Received content: %s", content);

    // Parse JSON
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        ESP_LOGE("HTTPS_SERVER", "Invalid JSON received");
        return ESP_FAIL;
    }

    cJSON *state = cJSON_GetObjectItem(json, "state");
    if (!state || !cJSON_IsString(state)) {
        ESP_LOGE("HTTPS_SERVER", "Invalid or missing 'state' in JSON");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    // Set LED array state: "on" -> 1, "off" -> 0
    if (strcmp(state->valuestring, "on") == 0) {
        set_led_array_binary(1);
        httpd_resp_sendstr(req, "LED array turned ON");
    } else if (strcmp(state->valuestring, "off") == 0) {
        set_led_array_binary(0);
        httpd_resp_sendstr(req, "LED array turned OFF");
    } else {
        ESP_LOGE("HTTPS_SERVER", "Invalid 'state' value, must be 'on' or 'off'");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    cJSON_Delete(json);
    return ESP_OK;
}

// Handler for sensors GET request
static esp_err_t sensors_get_handler(httpd_req_t *req) {
    // Create JSON response
    cJSON *json = cJSON_CreateObject();
    cJSON *table = cJSON_CreateArray();  // Create a JSON array for the table

    // Variables to hold sensor data
    float current_led, voltage_led, power_led;
    float current_drain, voltage_drain, power_drain;
    float current_source, voltage_source, power_source;
    float current_air, voltage_air, power_air;

    // Read data for each actuator
    ina260_read_current(INA260_LED_ADDRESS, &current_led);
    ina260_read_voltage(INA260_LED_ADDRESS, &voltage_led);
    ina260_read_power(INA260_LED_ADDRESS, &power_led);

    ina260_read_current(INA260_DRAIN_ADDRESS, &current_drain);
    ina260_read_voltage(INA260_DRAIN_ADDRESS, &voltage_drain);
    ina260_read_power(INA260_DRAIN_ADDRESS, &power_drain);

    ina260_read_current(INA260_SOURCE_ADDRESS, &current_source);
    ina260_read_voltage(INA260_SOURCE_ADDRESS, &voltage_source);
    ina260_read_power(INA260_SOURCE_ADDRESS, &power_source);

    ina260_read_current(INA260_AIR_ADDRESS, &current_air);
    ina260_read_voltage(INA260_AIR_ADDRESS, &voltage_air);
    ina260_read_power(INA260_AIR_ADDRESS, &power_air);

    float drain_flow = 42;//get_drain_flow_rate();
    float source_flow = 42;//get_source_flow_rate();

    // Add rows to the table for each actuator (current, voltage, and power)
    cJSON *row_led = cJSON_CreateObject();
    cJSON_AddStringToObject(row_led, "actuator", "LED");
    cJSON_AddNumberToObject(row_led, "current_mA", current_led);
    cJSON_AddNumberToObject(row_led, "voltage_mV", voltage_led);
    cJSON_AddNumberToObject(row_led, "power_mW", power_led);
    cJSON_AddItemToArray(table, row_led);

    cJSON *row_drain = cJSON_CreateObject();
    cJSON_AddStringToObject(row_drain, "actuator", "Drain");
    cJSON_AddNumberToObject(row_drain, "current_mA", current_drain);
    cJSON_AddNumberToObject(row_drain, "voltage_mV", voltage_drain);
    cJSON_AddNumberToObject(row_drain, "power_mW", power_drain);
    cJSON_AddNumberToObject(row_drain, "flow_rate_L_min", drain_flow);
    cJSON_AddItemToArray(table, row_drain);

    cJSON *row_source = cJSON_CreateObject();
    cJSON_AddStringToObject(row_source, "actuator", "Source");
    cJSON_AddNumberToObject(row_source, "current_mA", current_source);
    cJSON_AddNumberToObject(row_source, "voltage_mV", voltage_source);
    cJSON_AddNumberToObject(row_source, "power_mW", power_source);
    cJSON_AddNumberToObject(row_source, "flow_rate_L_min", source_flow);
    cJSON_AddItemToArray(table, row_source);

    cJSON *row_air = cJSON_CreateObject();
    cJSON_AddStringToObject(row_air, "actuator", "Air Pump");
    cJSON_AddNumberToObject(row_air, "current_mA", current_air);
    cJSON_AddNumberToObject(row_air, "voltage_mV", voltage_air);
    cJSON_AddNumberToObject(row_air, "power_mW", power_air);
    cJSON_AddItemToArray(table, row_air);

    // Add the table to the main JSON object
    cJSON_AddItemToObject(json, "sensors_data", table);

    // Convert JSON to string
    char *response = cJSON_Print(json);

    // Send response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);

    // Clean up
    cJSON_Delete(json);
    free(response);

    return ESP_OK;
}
