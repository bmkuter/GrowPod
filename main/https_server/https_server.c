// https_server/https_server.c

#include "https_server.h"
#include "esp_https_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <time.h>
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <esp_wifi.h>

#include "actuator_control.h"   // For set_air_pump_pwm, set_source_pump_pwm, set_drain_pump_pwm, set_planter_pump_pwm, set_led_array_binary (or set_led_array_pwm)
#include "ina260.h"             // For sensor reading
#include "flowmeter_control.h"  // For flow rate reading
#include "control_logic.h"      // For system_state_t, get_system_state, etc.
#include "distance_sensor.h"    // For distance_sensor_read_mm()

static const char *TAG = "HTTPS_SERVER";

/* ==========================================
 *         ROUTINE + SCHEDULE STRUCTS
 * ========================================== */

#define MAX_ROUTINE_RECORDS 10

typedef enum {
    ROUTINE_STATUS_PENDING = 0,
    ROUTINE_STATUS_RUNNING,
    ROUTINE_STATUS_COMPLETED,
    ROUTINE_STATUS_FAILED
} routine_status_t;

typedef struct {
    bool in_use;
    int  routine_id;
    char routine_name[32];
    routine_status_t status;

    // Water-level data: for empty/fill/calibrate
    int start_height_cm;
    int end_height_cm;
    int min_height_cm;
    int max_height_cm;

    // Performance metrics
    float time_elapsed_s;
    float power_used_mW_s;
} routine_record_t;

static routine_record_t s_routines[MAX_ROUTINE_RECORDS];
static int s_next_routine_id = 1;

// For schedule routines, define fixed IDs so they don’t accumulate:
#define LIGHT_SCHEDULE_ID     1001
#define PLANTER_SCHEDULE_ID   1002
#define AIR_SCHEDULE_ID       1003

typedef enum {
    SCHEDULE_TYPE_LIGHT = 0,
    SCHEDULE_TYPE_PLANTER,
    SCHEDULE_TYPE_AIR
} schedule_type_t;

typedef struct {
    schedule_type_t type;
    uint8_t schedule[24];  // 24 values (0..100)
} schedule_message_t;

/* Queue for schedule messages */
static QueueHandle_t schedule_msg_queue = NULL;

/* Current active schedules for the manager task */
static uint8_t current_light_schedule[24]   = {0};
static uint8_t current_planter_schedule[24] = {0};
static uint8_t current_air_schedule[24]     = {0};


/* ==========================================
 * Forward Declarations
 * ========================================== */
static esp_err_t routines_post_handler(httpd_req_t *req);
static esp_err_t routines_get_handler(httpd_req_t *req);
static esp_err_t saved_schedule_get_handler(httpd_req_t *req);

static int allocate_routine_slot(const char *routine_name);
static routine_record_t* find_routine_by_id(int rid);

static void routine_empty_pod_task(void *pvParam);
static void routine_fill_pod_task(void *pvParam);
static void routine_calibrate_pod_task(void *pvParam);
static void routine_store_schedule_task(void *pvParam);
static esp_err_t airpump_post_handler(httpd_req_t *req);
static esp_err_t sourcepump_post_handler(httpd_req_t *req);
static esp_err_t planterpump_post_handler(httpd_req_t *req);
static esp_err_t drainpump_post_handler(httpd_req_t *req);
static esp_err_t led_post_handler(httpd_req_t *req);
static esp_err_t unit_metrics_get_handler(httpd_req_t *req);
static esp_err_t hostname_suffix_post_handler(httpd_req_t *req);
static esp_err_t device_restart_post_handler(httpd_req_t *req);


/* ==========================================
 * SNTP-based function to get current hour
 * ========================================== */
int get_current_hour(void)
{
    time_t now;
    struct tm timeinfo;

    time(&now);
    if (now < 1000000000) { // time not yet synchronized
        return -1;
    }
    localtime_r(&now, &timeinfo);
    return timeinfo.tm_hour;
}

/* ==========================================
 * NVS Helpers (to persist schedules)
 * ========================================== */
static esp_err_t save_schedule_to_nvs(schedule_type_t type, const uint8_t schedule[24]) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("user_settings", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open error: %s", esp_err_to_name(err));
        return err;
    }
    const char* key = "";
    switch (type) {
       case SCHEDULE_TYPE_LIGHT:   key = "light_schedule";   break;
       case SCHEDULE_TYPE_PLANTER: key = "planter_schedule"; break;
       case SCHEDULE_TYPE_AIR:     key = "air_schedule";     break;
       default: nvs_close(my_handle); return ESP_ERR_INVALID_ARG;
    }
    err = nvs_set_blob(my_handle, key, schedule, 24 * sizeof(uint8_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS set blob error: %s", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }
    err = nvs_commit(my_handle);
    nvs_close(my_handle);
    return err;
}

static esp_err_t load_schedule_from_nvs(schedule_type_t type, uint8_t schedule[24]) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("user_settings", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open error (read): %s", esp_err_to_name(err));
        return err;
    }

    const char* key = "";
    switch (type) {
       case SCHEDULE_TYPE_LIGHT:   key = "light_schedule";   break;
       case SCHEDULE_TYPE_PLANTER: key = "planter_schedule"; break;
       case SCHEDULE_TYPE_AIR:     key = "air_schedule";     break;
       default:
         nvs_close(my_handle);
         return ESP_ERR_INVALID_ARG;
    }

    size_t required_size = 24 * sizeof(uint8_t);
    err = nvs_get_blob(my_handle, key, schedule, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // If the key doesn't exist, default to all zeros
        memset(schedule, 0, 24);
        err = ESP_OK; // Not an error, just means no existing schedule
    }

    nvs_close(my_handle);

    // Build a comma-separated string of the 24 elements
    char buf[128];
    snprintf(buf, sizeof(buf),
             "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
             schedule[0],  schedule[1],  schedule[2],  schedule[3],
             schedule[4],  schedule[5],  schedule[6],  schedule[7],
             schedule[8],  schedule[9],  schedule[10], schedule[11],
             schedule[12], schedule[13], schedule[14], schedule[15],
             schedule[16], schedule[17], schedule[18], schedule[19],
             schedule[20], schedule[21], schedule[22], schedule[23]);

    ESP_LOGI(TAG, "load_schedule_from_nvs('%s') => err=%s, schedule=[%s]",
             key, esp_err_to_name(err), buf);

    return err;
}


/* ==========================================
 * Schedule Manager Task
 * ==========================================
 * Waits for new schedule messages, updates
 * current schedules, saves to NVS, and
 * applies them if hour changes or schedule updated.
 */
static void schedule_manager_task(void *pvParam) {
    int last_hour = -1;
    schedule_message_t msg;
    while (1) {
        bool schedule_updated = false;

        // Wait up to 10s for a message
        if (xQueueReceive(schedule_msg_queue, &msg, pdMS_TO_TICKS(10000)) == pdTRUE) {
            switch (msg.type) {
                case SCHEDULE_TYPE_LIGHT:
                    memcpy(current_light_schedule, msg.schedule, 24);
                    ESP_LOGI(TAG, "Schedule Manager: Updated LIGHT schedule");
                    schedule_updated = true;
                    // save_schedule_to_nvs() is done by the routine task as well,
                    // but if you'd prefer, you could do it here.
                    break;
                case SCHEDULE_TYPE_PLANTER:
                    memcpy(current_planter_schedule, msg.schedule, 24);
                    ESP_LOGI(TAG, "Schedule Manager: Updated PLANTER schedule");
                    schedule_updated = true;
                    break;
                case SCHEDULE_TYPE_AIR:
                    memcpy(current_air_schedule, msg.schedule, 24);
                    ESP_LOGI(TAG, "Schedule Manager: Updated AIR schedule");
                    schedule_updated = true;
                    break;
                default:
                    ESP_LOGW(TAG, "Schedule Manager: Unknown schedule type");
                    break;
            }
        }

        // Check time
        int current_hour = get_current_hour();
        if (schedule_updated || (current_hour != -1 && current_hour != last_hour)) {
            last_hour = current_hour;
            uint8_t light_duty   = current_light_schedule[current_hour];
            uint8_t planter_duty = current_planter_schedule[current_hour];
            uint8_t air_duty     = current_air_schedule[current_hour];

            // Apply them
            set_led_array_pwm(light_duty);
            ESP_LOGI(TAG, "Schedule Manager: Set LED array to %d%% at hour %d", light_duty, current_hour);

            set_planter_pump_pwm(planter_duty);
            ESP_LOGI(TAG, "Schedule Manager: Set Planter Pump to %d%% at hour %d", planter_duty, current_hour);

            set_air_pump_pwm(air_duty);
            ESP_LOGI(TAG, "Schedule Manager: Set Air Pump to %d%% at hour %d", air_duty, current_hour);
        }
    }
}

void init_schedule_manager(void)
{
    schedule_msg_queue = xQueueCreate(10, sizeof(schedule_message_t));
    if (schedule_msg_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create schedule message queue");
        return;
    }

    // Optionally load stored schedules from NVS here for immediate usage
    esp_err_t err_light   = load_schedule_from_nvs(SCHEDULE_TYPE_LIGHT,   current_light_schedule);
    esp_err_t err_planter = load_schedule_from_nvs(SCHEDULE_TYPE_PLANTER, current_planter_schedule);
    esp_err_t err_air     = load_schedule_from_nvs(SCHEDULE_TYPE_AIR,     current_air_schedule);

    // Log the schedules we just loaded
    // Convert them to a string for logging
    char buf[128];
    
    // Light schedule
    snprintf(buf, sizeof(buf), 
             "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             current_light_schedule[0],  current_light_schedule[1],  current_light_schedule[2],  current_light_schedule[3],
             current_light_schedule[4],  current_light_schedule[5],  current_light_schedule[6],  current_light_schedule[7],
             current_light_schedule[8],  current_light_schedule[9],  current_light_schedule[10], current_light_schedule[11],
             current_light_schedule[12], current_light_schedule[13], current_light_schedule[14], current_light_schedule[15],
             current_light_schedule[16], current_light_schedule[17], current_light_schedule[18], current_light_schedule[19],
             current_light_schedule[20], current_light_schedule[21], current_light_schedule[22], current_light_schedule[23]);
    ESP_LOGI(TAG, "Light schedule (err=%s): [%s]", esp_err_to_name(err_light), buf);

    // Planter schedule
    snprintf(buf, sizeof(buf),
             "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             current_planter_schedule[0],  current_planter_schedule[1],  current_planter_schedule[2],  current_planter_schedule[3],
             current_planter_schedule[4],  current_planter_schedule[5],  current_planter_schedule[6],  current_planter_schedule[7],
             current_planter_schedule[8],  current_planter_schedule[9],  current_planter_schedule[10], current_planter_schedule[11],
             current_planter_schedule[12], current_planter_schedule[13], current_planter_schedule[14], current_planter_schedule[15],
             current_planter_schedule[16], current_planter_schedule[17], current_planter_schedule[18], current_planter_schedule[19],
             current_planter_schedule[20], current_planter_schedule[21], current_planter_schedule[22], current_planter_schedule[23]);
    ESP_LOGI(TAG, "Planter schedule (err=%s): [%s]", esp_err_to_name(err_planter), buf);

    // Air schedule
    snprintf(buf, sizeof(buf),
             "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             current_air_schedule[0],  current_air_schedule[1],  current_air_schedule[2],  current_air_schedule[3],
             current_air_schedule[4],  current_air_schedule[5],  current_air_schedule[6],  current_air_schedule[7],
             current_air_schedule[8],  current_air_schedule[9],  current_air_schedule[10], current_air_schedule[11],
             current_air_schedule[12], current_air_schedule[13], current_air_schedule[14], current_air_schedule[15],
             current_air_schedule[16], current_air_schedule[17], current_air_schedule[18], current_air_schedule[19],
             current_air_schedule[20], current_air_schedule[21], current_air_schedule[22], current_air_schedule[23]);
    ESP_LOGI(TAG, "Air schedule (err=%s): [%s]", esp_err_to_name(err_air), buf);

    // Finally, create the schedule manager task
    xTaskCreate(schedule_manager_task, "schedule_manager", 4096, NULL,
                configMAX_PRIORITIES - 2, NULL);
}


/* ==========================================
 * Start HTTPS server with wildcard matching
 * ==========================================
 */
void start_https_server(void) {
    httpd_ssl_config_t config = HTTPD_SSL_CONFIG_DEFAULT();
    config.httpd.uri_match_fn = httpd_uri_match_wildcard;  // function pointer
    config.httpd.max_uri_handlers = 15;

    extern const unsigned char server_cert_pem_start[] asm("_binary_server_crt_start");
    extern const unsigned char server_cert_pem_end[]   asm("_binary_server_crt_end");
    extern const unsigned char server_key_pem_start[]  asm("_binary_server_key_start");
    extern const unsigned char server_key_pem_end[]    asm("_binary_server_key_end");
    config.servercert     = server_cert_pem_start;
    config.servercert_len = server_cert_pem_end - server_cert_pem_start;
    config.prvtkey_pem    = server_key_pem_start;
    config.prvtkey_len    = server_key_pem_end - server_key_pem_start;

    extern const unsigned char ca_cert_pem_start[] asm("_binary_root_ca_crt_start");
    extern const unsigned char ca_cert_pem_end[]   asm("_binary_root_ca_crt_end");
    config.cacert_pem     = ca_cert_pem_start;
    config.cacert_len     = ca_cert_pem_end - ca_cert_pem_start;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_ssl_start(&server, &config);
    if (ret == ESP_OK) {
        // Register existing actuator and sensor endpoints
        httpd_uri_t airpump_post_uri = {
            .uri      = "/api/actuators/airpump",
            .method   = HTTP_POST,
            .handler  = airpump_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &airpump_post_uri);
        
        httpd_uri_t sourcepump_post_uri = {
            .uri      = "/api/actuators/sourcepump",
            .method   = HTTP_POST,
            .handler  = sourcepump_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &sourcepump_post_uri);
        
        httpd_uri_t planterpump_post_uri = {
            .uri      = "/api/actuators/planterpump",
            .method   = HTTP_POST,
            .handler  = planterpump_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &planterpump_post_uri);
        
        httpd_uri_t drainpump_post_uri = {
            .uri      = "/api/actuators/drainpump",
            .method   = HTTP_POST,
            .handler  = drainpump_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &drainpump_post_uri);
        
        httpd_uri_t led_post_uri = {
            .uri      = "/api/actuators/led",
            .method   = HTTP_POST,
            .handler  = led_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &led_post_uri);
        
        httpd_uri_t unit_metrics_uri = {
            .uri       = "/api/unit-metrics",
            .method    = HTTP_GET,
            .handler   = unit_metrics_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &unit_metrics_uri);

        // Example: GET /api/routines/status
        httpd_uri_t routines_status_uri = {
            .uri      = "/api/routines/status",
            .method   = HTTP_GET,
            .handler  = routines_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &routines_status_uri);

        // Example: GET /api/routines/saved?type=<light|planter|air>
        httpd_uri_t saved_schedule_uri = {
            .uri      = "/api/routines/saved",
            .method   = HTTP_GET,
            .handler  = saved_schedule_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &saved_schedule_uri);

        // Wildcard for POST /api/routines/<name>
        httpd_uri_t routines_uri = {
            .uri      = "/api/routines/*",
            .method   = HTTP_ANY,
            .handler  = routines_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &routines_uri);

        httpd_uri_t suffix_post_uri = {
            .uri = "/api/hostnameSuffix",
            .method = HTTP_POST,
            .handler = hostname_suffix_post_handler
        };
        httpd_register_uri_handler(server, &suffix_post_uri);

        httpd_uri_t restart_uri = {
            .uri      = "/api/restart",
            .method   = HTTP_POST,
            .handler  = device_restart_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &restart_uri);

        ESP_LOGI(TAG, "HTTPS server started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTPS server");
    }
}

/* ==========================================
 * REST Endpoints for routines
 * ==========================================
 */

/**
 * @brief Handler for POST /api/routines/<routine_name>.
 * 
 * For one-shot routines (empty_pod, fill_pod, calibrate_pod), we allocate a new record.
 * For schedule routines (light_schedule, planter_pod_schedule, air_pump_schedule),
 * we use a fixed ID to avoid multiple records for the same schedule.
 */
static esp_err_t routines_post_handler(httpd_req_t *req)
{
    const char *uri = req->uri;
    const char *routine_name = strrchr(uri, '/');
    if (!routine_name || strlen(routine_name) < 2) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No routine name");
        return ESP_FAIL;
    }
    routine_name++; // skip '/'

    char content[512];
    int ret = httpd_req_recv(req, content,
                    req->content_len < (int)sizeof(content) ? req->content_len : (int)sizeof(content)-1);
    if (ret > 0) {
        content[ret] = '\0';
    } else {
        content[0] = '\0';
    }

    int rid = -1;
    // One-shot routines
    if (strcasecmp(routine_name, "empty_pod") == 0) {
        int slot = allocate_routine_slot(routine_name);
        if (slot < 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No routine slots left");
            return ESP_FAIL;
        }
        rid = s_routines[slot].routine_id;
        s_routines[slot].status = ROUTINE_STATUS_PENDING;
        xTaskCreate(routine_empty_pod_task, "empty_pod", 4096, (void*)(intptr_t)rid,
                    configMAX_PRIORITIES - 1, NULL);
    }
    else if (strcasecmp(routine_name, "fill_pod") == 0) {
        int slot = allocate_routine_slot(routine_name);
        if (slot < 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No routine slots left");
            return ESP_FAIL;
        }
        rid = s_routines[slot].routine_id;
        s_routines[slot].status = ROUTINE_STATUS_PENDING;
        xTaskCreate(routine_fill_pod_task, "fill_pod", 4096, (void*)(intptr_t)rid,
                    configMAX_PRIORITIES - 1, NULL);
    }
    else if (strcasecmp(routine_name, "calibrate_pod") == 0) {
        int slot = allocate_routine_slot(routine_name);
        if (slot < 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No routine slots left");
            return ESP_FAIL;
        }
        rid = s_routines[slot].routine_id;
        s_routines[slot].status = ROUTINE_STATUS_PENDING;
        xTaskCreate(routine_calibrate_pod_task, "calibrate_pod", 4096, (void*)(intptr_t)rid,
                    configMAX_PRIORITIES - 1, NULL);
    }
    // Schedule routines
    else if (strcasecmp(routine_name, "light_schedule") == 0 ||
             strcasecmp(routine_name, "planter_pod_schedule") == 0 ||
             strcasecmp(routine_name, "air_pump_schedule") == 0)
    {
        if (strcasecmp(routine_name, "light_schedule") == 0) {
            rid = LIGHT_SCHEDULE_ID;
        } else if (strcasecmp(routine_name, "planter_pod_schedule") == 0) {
            rid = PLANTER_SCHEDULE_ID;
        } else { // "air_pump_schedule"
            rid = AIR_SCHEDULE_ID;
        }
        // Use or create a persistent record for this schedule
        routine_record_t *rec = find_routine_by_id(rid);
        if (!rec) {
            for (int i = 0; i < MAX_ROUTINE_RECORDS; i++) {
                if (!s_routines[i].in_use) {
                    s_routines[i].in_use = true;
                    s_routines[i].routine_id = rid;
                    s_routines[i].status = ROUTINE_STATUS_PENDING;
                    strncpy(s_routines[i].routine_name, routine_name, sizeof(s_routines[i].routine_name)-1);
                    s_routines[i].routine_name[sizeof(s_routines[i].routine_name)-1] = '\0';
                    rec = &s_routines[i];
                    break;
                }
            }
            if (!rec) {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No routine slot available");
                return ESP_FAIL;
            }
        } else {
            rec->status = ROUTINE_STATUS_PENDING;
        }
        // Parse the 24-value array
        uint8_t tmp_schedule[24] = {0};
        cJSON *root = cJSON_Parse(content);
        if (root) {
            cJSON *arr = cJSON_GetObjectItem(root, "schedule");
            if (cJSON_IsArray(arr) && cJSON_GetArraySize(arr) == 24) {
                for (int i = 0; i < 24; i++) {
                    cJSON *val = cJSON_GetArrayItem(arr, i);
                    int pwm = cJSON_IsNumber(val) ? val->valueint : 0;
                    if (pwm < 0) pwm = 0;
                    if (pwm > 100) pwm = 100;
                    tmp_schedule[i] = (uint8_t)pwm;
                }
            }
            cJSON_Delete(root);
        }
        typedef struct {
            int routine_id;
            char name[32];
            uint8_t sched[24];
        } sched_param_t;
        sched_param_t *p = malloc(sizeof(sched_param_t));
        p->routine_id = rid;
        strncpy(p->name, routine_name, sizeof(p->name)-1);
        p->name[sizeof(p->name)-1] = '\0';
        memcpy(p->sched, tmp_schedule, 24);

        xTaskCreate(routine_store_schedule_task, "sched_store", 4096,
                    (void*)p, configMAX_PRIORITIES - 1, NULL);
    }
    else {
        ESP_LOGW(TAG, "Unknown routine name: %s", routine_name);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Unknown routine");
        return ESP_FAIL;
    }

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddNumberToObject(resp, "routine_id", rid);
    cJSON_AddStringToObject(resp, "message", "Routine started");
    char *json_str = cJSON_PrintUnformatted(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_str);
    free(json_str);
    cJSON_Delete(resp);
    return ESP_OK;
}

/* ==========================================
 * GET /api/routines/status?id=<rid>
 * Returns JSON about routine's status
 * ==========================================
 */
static esp_err_t routines_get_handler(httpd_req_t *req)
{
    char param[64];
    if (httpd_req_get_url_query_str(req, param, sizeof(param)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(param, "id", val, sizeof(val)) == ESP_OK) {
            int rid = atoi(val);
            routine_record_t *rec = find_routine_by_id(rid);
            if (!rec) {
                httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Routine not found");
                return ESP_FAIL;
            }
            cJSON *root = cJSON_CreateObject();
            cJSON_AddNumberToObject(root, "routine_id", rid);
            cJSON_AddStringToObject(root, "routine_name", rec->routine_name);

            const char *status_str = "UNKNOWN";
            switch (rec->status) {
                case ROUTINE_STATUS_PENDING:   status_str = "PENDING";   break;
                case ROUTINE_STATUS_RUNNING:   status_str = "RUNNING";   break;
                case ROUTINE_STATUS_COMPLETED: status_str = "COMPLETED"; break;
                case ROUTINE_STATUS_FAILED:    status_str = "FAILED";    break;
            }
            cJSON_AddStringToObject(root, "status", status_str);

            cJSON_AddNumberToObject(root, "start_height_mm", rec->start_height_cm);
            cJSON_AddNumberToObject(root, "end_height_mm",   rec->end_height_cm);
            cJSON_AddNumberToObject(root, "min_height_mm",   rec->min_height_cm);
            cJSON_AddNumberToObject(root, "max_height_mm",   rec->max_height_cm);
            cJSON_AddNumberToObject(root, "time_elapsed_s",  rec->time_elapsed_s);
            cJSON_AddNumberToObject(root, "power_used_mW_s", rec->power_used_mW_s);

            char *json_str = cJSON_PrintUnformatted(root);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);
            free(json_str);
            cJSON_Delete(root);
            return ESP_OK;
        }
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid id");
    return ESP_FAIL;
}

/* ==========================================
 * GET /api/routines/saved?type=<light|planter|air>
 * Returns the schedule stored in NVS
 * ==========================================
 */
static esp_err_t saved_schedule_get_handler(httpd_req_t *req)
{
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char type_str[16];
        if (httpd_query_key_value(query, "type", type_str, sizeof(type_str)) == ESP_OK) {
            schedule_type_t type;
            if (strcasecmp(type_str, "light") == 0) {
                type = SCHEDULE_TYPE_LIGHT;
            } else if (strcasecmp(type_str, "planter") == 0) {
                type = SCHEDULE_TYPE_PLANTER;
            } else if (strcasecmp(type_str, "air") == 0) {
                type = SCHEDULE_TYPE_AIR;
            } else {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid schedule type");
                return ESP_FAIL;
            }

            uint8_t schedule[24];
            esp_err_t err = load_schedule_from_nvs(type, schedule);
            if (err != ESP_OK) {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to load schedule");
                return err;
            }

            // Build the JSON response
            cJSON *root = cJSON_CreateObject();
            cJSON *arr  = cJSON_CreateArray();

            // Manually add each schedule value to the array
            for (int i = 0; i < 24; i++) {
                cJSON_AddItemToArray(arr, cJSON_CreateNumber(schedule[i]));
            }

            cJSON_AddItemToObject(root, "schedule", arr);

            char *json_str = cJSON_PrintUnformatted(root);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);

            free(json_str);
            cJSON_Delete(root);
            return ESP_OK;
        }
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid type parameter");
    return ESP_FAIL;
}


/* ==========================================
 * Routine Tasks Implementation
 * ==========================================
 */
#define FILL_DRAIN_MAX_TIMER_SEC 60.0f * 4.0f

//------------------------------------------
// 1) empty_pod
//------------------------------------------
static void routine_empty_pod_task(void *pvParam)
{
    int rid = (int)(intptr_t) pvParam;
    routine_record_t *rec = find_routine_by_id(rid);
    if (!rec) {
        vTaskDelete(NULL);
        return;
    }
    rec->status = ROUTINE_STATUS_RUNNING;
    ESP_LOGI(TAG, "[empty_pod] start ID=%d", rid);

    int dist_mm = distance_sensor_read_mm();
    // If dist_mm < 0 => invalid; store -1 in that case
    int dist_cm = (dist_mm < 0) ? -1 : (dist_mm / 10);  // truncated integer cm
    rec->start_height_cm = dist_cm;

    set_drain_pump_pwm(70);

    float t0 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    float total_power = 0.0f;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));

        float cur_mA=0, volt_mV=0, pwr_mW=0;
        ina260_read_current(INA260_ADDRESS, &cur_mA);
        ina260_read_voltage(INA260_ADDRESS, &volt_mV);
        ina260_read_power(INA260_ADDRESS, &pwr_mW);
        total_power += (pwr_mW * 0.5f);

        dist_mm = distance_sensor_read_mm();
        dist_cm = (dist_mm < 0) ? -1 : (dist_mm / 10);

        ESP_LOGW(TAG, "DRAINING: Distance: %d cm", dist_cm);

        // Old threshold was dist_mm > 115 => break. 
        // Now dist_cm > 11 means the same as >110 mm (rounded down).
        if (dist_cm > 11) {
            break;
        }

        float now_s = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
        if ((now_s - t0) > FILL_DRAIN_MAX_TIMER_SEC) {
            break;
        }
    }

    set_drain_pump_pwm(0);

    float t1 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;

    dist_mm = distance_sensor_read_mm();
    dist_cm = (dist_mm < 0) ? -1 : (dist_mm / 10);
    rec->end_height_cm = dist_cm;
    rec->time_elapsed_s = t1 - t0;
    rec->power_used_mW_s = total_power;
    rec->status = ROUTINE_STATUS_COMPLETED;

    ESP_LOGI(TAG,
        "[empty_pod] done ID=%d: time=%.1f s, power=%.1f mW·s, start=%d cm, end=%d cm",
        rid, rec->time_elapsed_s, total_power,
        rec->start_height_cm, rec->end_height_cm);

    vTaskDelete(NULL);
}


//------------------------------------------
// 2) fill_pod
//------------------------------------------
static void routine_fill_pod_task(void *pvParam)
{
    int rid = (int)(intptr_t) pvParam;
    routine_record_t *rec = find_routine_by_id(rid);
    if (!rec) {
        vTaskDelete(NULL);
        return;
    }
    rec->status = ROUTINE_STATUS_RUNNING;
    ESP_LOGI(TAG, "[fill_pod] start ID=%d", rid);

    int dist_mm = distance_sensor_read_mm();
    int dist_cm = (dist_mm < 0) ? -1 : (dist_mm / 10);
    rec->start_height_cm = dist_cm;

    set_source_pump_pwm(70);

    float t0 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    float total_power = 0.0f;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));

        float cur_mA=0, volt_mV=0, pwr_mW=0;
        ina260_read_current(INA260_ADDRESS, &cur_mA);
        ina260_read_voltage(INA260_ADDRESS, &volt_mV);
        ina260_read_power(INA260_ADDRESS, &pwr_mW);
        total_power += (pwr_mW * 0.5f);

        dist_mm = distance_sensor_read_mm();
        dist_cm = (dist_mm < 0) ? -1 : (dist_mm / 10);

        ESP_LOGW(TAG, "FILLING: Distance: %d cm", dist_cm);

        // Original check was dist_mm > 0 && dist_mm < 50 => break
        // Now we do dist_cm > 0 && dist_cm < 5 => break
        // or if you want to exactly match "less than 50 mm", 
        // that is truncated to 4 cm for anything < 50 mm.
        if ((dist_cm > 0) && (dist_cm < 5)) {
            break;
        }

        float now_s = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
        if ((now_s - t0) > FILL_DRAIN_MAX_TIMER_SEC) {
            break;
        }
    }

    set_source_pump_pwm(0);

    float t1 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    dist_mm = distance_sensor_read_mm();
    dist_cm = (dist_mm < 0) ? -1 : (dist_mm / 10);
    rec->end_height_cm = dist_cm;
    rec->time_elapsed_s = t1 - t0;
    rec->power_used_mW_s = total_power;
    rec->status = ROUTINE_STATUS_COMPLETED;

    ESP_LOGI(TAG,
        "[fill_pod] done ID=%d: time=%.1f s, power=%.1f mW·s, start=%d cm, end=%d cm",
        rid, rec->time_elapsed_s, total_power,
        rec->start_height_cm, rec->end_height_cm);

    vTaskDelete(NULL);
}


//------------------------------------------
// 3) calibrate_pod
//------------------------------------------
static void routine_calibrate_pod_task(void *pvParam)
{
    int rid = (int)(intptr_t) pvParam;
    routine_record_t *rec = find_routine_by_id(rid);
    if (!rec) {
        vTaskDelete(NULL);
        return;
    }
    rec->status = ROUTINE_STATUS_RUNNING;
    ESP_LOGI(TAG, "[calibrate_pod] start ID=%d", rid);

    float t0 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    float total_power = 0.0f;

    // Initialize min/max to large / small integers
    rec->min_height_cm = 999999;
    rec->max_height_cm = 0;

    for (int i = 0; i < 20; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));

        float cur_mA=0, volt_mV=0, pwr_mW=0;
        ina260_read_current(INA260_ADDRESS, &cur_mA);
        ina260_read_voltage(INA260_ADDRESS, &volt_mV);
        ina260_read_power(INA260_ADDRESS, &pwr_mW);
        total_power += (pwr_mW * 0.5f);

        int dist_mm = distance_sensor_read_mm();
        if (dist_mm > 0) {
            // Truncated integer cm
            int dist_cm = dist_mm / 10;
            if (dist_cm < rec->min_height_cm) {
                rec->min_height_cm = dist_cm;
            }
            if (dist_cm > rec->max_height_cm) {
                rec->max_height_cm = dist_cm;
            }
        }
    }

    float t1 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    rec->time_elapsed_s = t1 - t0;
    rec->power_used_mW_s = total_power;
    rec->status = ROUTINE_STATUS_COMPLETED;

    ESP_LOGI(TAG,
        "[calibrate_pod] done ID=%d: time=%.1f s, power=%.1f mW·s, min=%d cm, max=%d cm",
        rid, rec->time_elapsed_s, total_power,
        rec->min_height_cm, rec->max_height_cm);

    vTaskDelete(NULL);
}


//------------------------------------------
// 4) schedule routine task (store schedule)
//------------------------------------------
static void routine_store_schedule_task(void *pvParam) {
    typedef struct {
        int routine_id;
        char name[32];
        uint8_t sched[24];
    } sched_param_t;

    sched_param_t *p = (sched_param_t*) pvParam;
    int rid = p->routine_id;

    routine_record_t *rec = find_routine_by_id(rid);
    if (!rec) {
        ESP_LOGE(TAG, "schedule task: invalid routine ID %d", rid);
        free(p);
        vTaskDelete(NULL);
        return;
    }
    rec->status = ROUTINE_STATUS_RUNNING;
    float t0 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;

    schedule_message_t sched_msg;
    memset(&sched_msg, 0, sizeof(sched_msg));
    if (strcasecmp(p->name, "light_schedule") == 0) {
        sched_msg.type = SCHEDULE_TYPE_LIGHT;
    } else if (strcasecmp(p->name, "planter_pod_schedule") == 0) {
        sched_msg.type = SCHEDULE_TYPE_PLANTER;
    } else if (strcasecmp(p->name, "air_pump_schedule") == 0) {
        sched_msg.type = SCHEDULE_TYPE_AIR;
    } else {
        ESP_LOGW(TAG, "schedule task: Unknown schedule name: %s", p->name);
        rec->status = ROUTINE_STATUS_FAILED;
        free(p);
        vTaskDelete(NULL);
        return;
    }
    memcpy(sched_msg.schedule, p->sched, 24);

    // Send to schedule manager
    if (xQueueSend(schedule_msg_queue, &sched_msg, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "schedule task: Failed to send schedule message");
        rec->status = ROUTINE_STATUS_FAILED;
    }
    // Also save to NVS (persist)
    esp_err_t err = ESP_OK;
    switch (sched_msg.type) {
        case SCHEDULE_TYPE_LIGHT:
            err = save_schedule_to_nvs(SCHEDULE_TYPE_LIGHT, sched_msg.schedule);
            break;
        case SCHEDULE_TYPE_PLANTER:
            err = save_schedule_to_nvs(SCHEDULE_TYPE_PLANTER, sched_msg.schedule);
            break;
        case SCHEDULE_TYPE_AIR:
            err = save_schedule_to_nvs(SCHEDULE_TYPE_AIR, sched_msg.schedule);
            break;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "schedule task: Failed to save schedule to NVS");
        rec->status = ROUTINE_STATUS_FAILED;
    }
    else
    {
        ESP_LOGW(TAG, "schedule task: Success to save schedule to NVS!");
    }
    free(p);
    float t1 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    rec->time_elapsed_s = t1 - t0;
    rec->power_used_mW_s = 0.0f;
    rec->status = ROUTINE_STATUS_COMPLETED;
    ESP_LOGI(TAG, "Schedule task completed for %s (ID=%d)", rec->routine_name, rid);
    vTaskDelete(NULL);
}

/* ==========================================
 * Helper Functions: allocate/find routine record
 * ==========================================
 */
static int allocate_routine_slot(const char *routine_name)
{
    for (int i = 0; i < MAX_ROUTINE_RECORDS; i++) {
        if (!s_routines[i].in_use) {
            s_routines[i].in_use = true;
            s_routines[i].routine_id = s_next_routine_id++;
            s_routines[i].status = ROUTINE_STATUS_PENDING;
            strncpy(s_routines[i].routine_name, routine_name, sizeof(s_routines[i].routine_name)-1);
            s_routines[i].routine_name[sizeof(s_routines[i].routine_name)-1] = '\0';
            s_routines[i].start_height_cm = -1;
            s_routines[i].end_height_cm   = -1;
            s_routines[i].min_height_cm   = 999999.0f;
            s_routines[i].max_height_cm   = 0.0f;
            s_routines[i].time_elapsed_s  = 0.0f;
            s_routines[i].power_used_mW_s = 0.0f;
            return i;
        }
    }
    return -1;
}

static routine_record_t* find_routine_by_id(int rid)
{
    for (int i = 0; i < MAX_ROUTINE_RECORDS; i++) {
        if (s_routines[i].in_use && s_routines[i].routine_id == rid) {
            return &s_routines[i];
        }
    }
    return NULL;
}


// Old functions:
/* ==========================================
 * Handler for actuator endpoints (airpump, sourcepump, planterpump, drainpump, led)
 * ========================================== */
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
// Handler for source pump POST request (JSON: {"value": <0..100>})
//-----------------------------------------------------------------------------------
static esp_err_t sourcepump_post_handler(httpd_req_t *req)
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
    set_source_pump_pwm(duty);

    cJSON_Delete(json);

    httpd_resp_sendstr(req, "Source pump updated");
    return ESP_OK;
}

//-----------------------------------------------------------------------------------
// Handler for planter pump POST request (JSON: {"value": <0..100>})
//-----------------------------------------------------------------------------------
static esp_err_t planterpump_post_handler(httpd_req_t *req)
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
    set_planter_pump_pwm(duty);

    cJSON_Delete(json);

    httpd_resp_sendstr(req, "Planter pump updated");
    return ESP_OK;
}

//-----------------------------------------------------------------------------------
// Handler for drain pump POST request (JSON: {"value": <0..100>})
// This replaces the old servo handler
//-----------------------------------------------------------------------------------
static esp_err_t drainpump_post_handler(httpd_req_t *req)
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
    set_drain_pump_pwm(duty);

    cJSON_Delete(json);

    httpd_resp_sendstr(req, "Drain pump updated");
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

    // Set LED array state: "on" -> true, "off" -> false
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

static esp_err_t unit_metrics_get_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();

    float cur_mA = 0.0f, volt_mV = 0.0f, pwr_mW = 0.0f;
    ina260_read_current(INA260_ADDRESS, &cur_mA);
    ina260_read_voltage(INA260_ADDRESS, &volt_mV);
    ina260_read_power(INA260_ADDRESS, &pwr_mW);

    int dist_mm = distance_sensor_read_mm();
    cJSON_AddNumberToObject(root, "current_mA", cur_mA);
    cJSON_AddNumberToObject(root, "voltage_mV", volt_mV);
    cJSON_AddNumberToObject(root, "power_consumption_mW", pwr_mW);
    cJSON_AddNumberToObject(root, "water_level_mm", (dist_mm >= 0) ? dist_mm : -1);

    // Retrieve MAC address
    uint8_t mac[6];
    if (esp_wifi_get_mac(ESP_IF_WIFI_STA, mac) == ESP_OK) {
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        cJSON_AddStringToObject(root, "mac_address", mac_str);
    }

    char *resp = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    free(resp);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t hostname_suffix_post_handler(httpd_req_t *req) {
    char body[128] = {0};
    int ret = httpd_req_recv(req, body, sizeof(body)-1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body");
        return ESP_FAIL;
    }
    cJSON *root = cJSON_Parse(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON");
        return ESP_FAIL;
    }
    cJSON *suffix_item = cJSON_GetObjectItem(root, "suffix");
    if (!suffix_item || !cJSON_IsString(suffix_item)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No suffix provided");
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    
    printf("Hostname suffix updated to: %s\n", suffix_item->valuestring);
    nvs_handle_t handle;
    if (nvs_open("user_settings", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_str(handle, "mdns_suffix", suffix_item->valuestring);
        nvs_commit(handle);
        nvs_close(handle);
    }
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Suffix updated");
    return ESP_OK;
}

static esp_err_t device_restart_post_handler(httpd_req_t *req) {
    esp_restart();
    return ESP_OK; 
}
