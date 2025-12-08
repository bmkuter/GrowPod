// https_server/https_server.c

#include "https_server.h"
#include "esp_https_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <time.h>
#include "esp_sntp.h"
#include <esp_wifi.h>
#include "../filesystem/config_manager.h"

#include "actuator_control.h"   // For set_air_pump_pwm, set_source_pump_pwm, set_drain_pump_pwm, set_planter_pump_pwm, set_led_array_binary (or set_led_array_pwm)
#include "power_monitor_HAL.h"  // For power monitoring abstraction
#include "control_logic.h"      // For system_state_t, get_system_state, etc.
#include "distance_sensor.h"    // For distance_sensor_read_mm()
#include "pod_state.h"  // Add pod_state

/* ==========================================
 *         WATER LEVEL CONSTANTS
 * ========================================== */

// Target thresholds for water levels (in mm)
#define TARGET_FILL_HEIGHT_MM     (TANK_EMPTY_HEIGHT_MM - TANK_HEADSPACE_MM)   // Pod considered filled when water level >= this value
#define READINGS_NEEDED           3     // Number of consistent readings to confirm state

static const char *TAG = "HTTPS_SERVER";

// Interactive calibration input (set via UART cmd_confirm_level)
static volatile int s_confirmed_level_mm = -1;

// Allow console or other modules to confirm measured water level during calibration
void confirm_level(int mm)
{
    s_confirmed_level_mm = mm;
}

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
    int start_height_mm;
    int end_height_mm;
    int min_height_mm;
    int max_height_mm;

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

// New structure for fill pod routine parameters
typedef struct {
    int routine_id;
    int target_mm;  // -1 means use default threshold
} fill_empty_param_t;

/* Queue for schedule messages */
static QueueHandle_t schedule_msg_queue = NULL;

/* Current active schedules for the manager task */
static uint8_t current_light_schedule[24]   = {0};
static uint8_t current_planter_schedule[24] = {0};
static uint8_t current_air_schedule[24]     = {0};

/* Mutexes for schedule access */
SemaphoreHandle_t schedule_led_mutex;
SemaphoreHandle_t schedule_planter_mutex;
SemaphoreHandle_t schedule_air_mutex;

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

extern void schedule_manager_task(void *pvParam);
extern void schedule_air_task(void *pvParam);
extern void schedule_led_task(void *pvParam); 
extern void schedule_planter_task(void *pvParam);

static esp_err_t schedules_print_get_handler(httpd_req_t *req);

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
static const char* schedule_type_to_string(schedule_type_t type) {
    switch (type) {
        case SCHEDULE_TYPE_LIGHT:   return "light";
        case SCHEDULE_TYPE_PLANTER: return "planter";
        case SCHEDULE_TYPE_AIR:     return "air";
        default:                    return NULL;
    }
}

static esp_err_t save_schedule_to_nvs(schedule_type_t type, const uint8_t schedule[24]) {
    const char* type_str = schedule_type_to_string(type);
    if (type_str == NULL) {
        ESP_LOGE(TAG, "Invalid schedule type: %d", type);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t err = config_save_schedule(type_str, schedule);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save schedule: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t load_schedule_from_nvs(schedule_type_t type, uint8_t schedule[24]) {
    const char* type_str = schedule_type_to_string(type);
    if (type_str == NULL) {
        ESP_LOGE(TAG, "Invalid schedule type: %d", type);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Loading schedule from JSON: %s", type_str);
    
    esp_err_t err = config_load_schedule(type_str, schedule);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load schedule (defaulting to zeros): %s", esp_err_to_name(err));
        // Default to all zeros if load fails
        memset(schedule, 0, 24);
        err = ESP_OK; // Not an error, just means no existing schedule
    }

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

    ESP_LOGI(TAG, "load_schedule_from_json => err=%s, schedule=[%s]",
             esp_err_to_name(err), buf);

    return err;
}

/* ==========================================
 * Schedule Manager Task
 * ==========================================
 * Waits for new schedule messages, updates
 * current schedules, saves to NVS, and
 * applies them if hour changes or schedule updated.
 */
void schedule_manager_task(void *pvParam) {
    schedule_message_t msg;
    while (1) {
        // Wait up to 10s for a schedule message and update appropriate global schedule
        if (xQueueReceive(schedule_msg_queue, &msg, pdMS_TO_TICKS(10000)) == pdTRUE) {
            ESP_LOGI(TAG, "Schedule Manager: Received schedule update for type %d", msg.type);
            switch (msg.type) {
                case SCHEDULE_TYPE_LIGHT:
                    xSemaphoreTake(schedule_led_mutex, portMAX_DELAY);
                    memcpy(current_light_schedule, msg.schedule, 24);
                    // Log new schedule
                    ESP_LOGI(TAG, "Schedule Manager: New LIGHT schedule: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                             current_light_schedule[0],  current_light_schedule[1],  current_light_schedule[2],  current_light_schedule[3],
                             current_light_schedule[4],  current_light_schedule[5],  current_light_schedule[6],  current_light_schedule[7],
                             current_light_schedule[8],  current_light_schedule[9],  current_light_schedule[10], current_light_schedule[11],
                             current_light_schedule[12], current_light_schedule[13], current_light_schedule[14], current_light_schedule[15],
                             current_light_schedule[16], current_light_schedule[17], current_light_schedule[18], current_light_schedule[19],
                             current_light_schedule[20], current_light_schedule[21], current_light_schedule[22], current_light_schedule[23]);
                    xSemaphoreGive(schedule_led_mutex);
                    ESP_LOGI(TAG, "Schedule Manager: Updated LIGHT schedule");
                    break;
                case SCHEDULE_TYPE_PLANTER:
                    xSemaphoreTake(schedule_planter_mutex, portMAX_DELAY);
                    memcpy(current_planter_schedule, msg.schedule, 24);
                    xSemaphoreGive(schedule_planter_mutex);
                    ESP_LOGI(TAG, "Schedule Manager: Updated PLANTER schedule");
                    break;
                case SCHEDULE_TYPE_AIR:
                    xSemaphoreTake(schedule_air_mutex, portMAX_DELAY);
                    memcpy(current_air_schedule, msg.schedule, 24);
                    xSemaphoreGive(schedule_air_mutex);
                    ESP_LOGI(TAG, "Schedule Manager: Updated AIR schedule");
                    break;
                default:
                    ESP_LOGW(TAG, "Schedule Manager: Unknown schedule type");
                    break;
            }
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
    
    // Create mutexes for each schedule array
    schedule_air_mutex = xSemaphoreCreateMutex();
    if (schedule_air_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create air schedule mutex");
        return;
    }
    schedule_planter_mutex = xSemaphoreCreateMutex();
    if (schedule_planter_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create planter schedule mutex");
        return;
    }
    schedule_led_mutex = xSemaphoreCreateMutex();
    if (schedule_led_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create LED schedule mutex");
        return;
    }

    // Load stored schedules from NVS:
    ESP_LOGI(TAG, "Loading schedules from NVS...");
    esp_err_t err_light   = load_schedule_from_nvs(SCHEDULE_TYPE_LIGHT,   current_light_schedule);
    esp_err_t err_planter = load_schedule_from_nvs(SCHEDULE_TYPE_PLANTER, current_planter_schedule);
    esp_err_t err_air     = load_schedule_from_nvs(SCHEDULE_TYPE_AIR,     current_air_schedule);

    // Log the schedules we just loaded
    // Convert them to a string for logging
    char buf[128];
    
    // Light schedule
    xSemaphoreTake(schedule_led_mutex, portMAX_DELAY);
    snprintf(buf, sizeof(buf), 
             "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             current_light_schedule[0],  current_light_schedule[1],  current_light_schedule[2],  current_light_schedule[3],
             current_light_schedule[4],  current_light_schedule[5],  current_light_schedule[6],  current_light_schedule[7],
             current_light_schedule[8],  current_light_schedule[9],  current_light_schedule[10], current_light_schedule[11],
             current_light_schedule[12], current_light_schedule[13], current_light_schedule[14], current_light_schedule[15],
             current_light_schedule[16], current_light_schedule[17], current_light_schedule[18], current_light_schedule[19],
             current_light_schedule[20], current_light_schedule[21], current_light_schedule[22], current_light_schedule[23]);
    xSemaphoreGive(schedule_led_mutex);
    ESP_LOGI(TAG, "Light schedule (err=%s): [%s]", esp_err_to_name(err_light), buf);

    // Planter schedule
    xSemaphoreTake(schedule_planter_mutex, portMAX_DELAY);
    snprintf(buf, sizeof(buf),
             "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             current_planter_schedule[0],  current_planter_schedule[1],  current_planter_schedule[2],  current_planter_schedule[3],
             current_planter_schedule[4],  current_planter_schedule[5],  current_planter_schedule[6],  current_planter_schedule[7],
             current_planter_schedule[8],  current_planter_schedule[9],  current_planter_schedule[10], current_planter_schedule[11],
             current_planter_schedule[12], current_planter_schedule[13], current_planter_schedule[14], current_planter_schedule[15],
             current_planter_schedule[16], current_planter_schedule[17], current_planter_schedule[18], current_planter_schedule[19],
             current_planter_schedule[20], current_planter_schedule[21], current_planter_schedule[22], current_planter_schedule[23]);
    xSemaphoreGive(schedule_planter_mutex);
    ESP_LOGI(TAG, "Planter schedule (err=%s): [%s]", esp_err_to_name(err_planter), buf);

    // Air schedule
    xSemaphoreTake(schedule_air_mutex, portMAX_DELAY);
    snprintf(buf, sizeof(buf),
             "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             current_air_schedule[0],  current_air_schedule[1],  current_air_schedule[2],  current_air_schedule[3],
             current_air_schedule[4],  current_air_schedule[5],  current_air_schedule[6],  current_air_schedule[7],
             current_air_schedule[8],  current_air_schedule[9],  current_air_schedule[10], current_air_schedule[11],
             current_air_schedule[12], current_air_schedule[13], current_air_schedule[14], current_air_schedule[15],
             current_air_schedule[16], current_air_schedule[17], current_air_schedule[18], current_air_schedule[19],
             current_air_schedule[20], current_air_schedule[21], current_air_schedule[22], current_air_schedule[23]);
    xSemaphoreGive(schedule_air_mutex);
    ESP_LOGI(TAG, "Air schedule (err=%s): [%s]", esp_err_to_name(err_air), buf);
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
    // Log the result
    ESP_LOGW(TAG, "Starting HTTPS server: %s", esp_err_to_name(ret));
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
       // GET /api/schedules -> print schedules
       httpd_uri_t schedules_uri = {
           .uri      = "/api/schedules",
           .method   = HTTP_GET,
           .handler  = schedules_print_get_handler,
           .user_ctx = NULL
       };
       httpd_register_uri_handler(server, &schedules_uri);
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
        // Parse optional target_mm from JSON (used as target percentage)
        int target_pct = -1;
        cJSON *root = cJSON_Parse(content);
        if (root) {
            cJSON *t = cJSON_GetObjectItem(root, "target_mm");
            if (cJSON_IsNumber(t)) {
                target_pct = t->valueint;
            }
            cJSON_Delete(root);
        }

        int slot = allocate_routine_slot(routine_name);
        if (slot < 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No routine slots left");
            return ESP_FAIL;
        }
        rid = s_routines[slot].routine_id;
        s_routines[slot].status = ROUTINE_STATUS_PENDING;
        
        // Pass target via parameter block
        fill_empty_param_t *p = malloc(sizeof(fill_empty_param_t));
        p->routine_id = rid;
        p->target_mm = target_pct;
        
        xTaskCreate(routine_empty_pod_task, "empty_pod", 4096,
                    (void*)p, configMAX_PRIORITIES - 1, NULL);
    }
    else if (strcasecmp(routine_name, "fill_pod") == 0) {
        // Parse optional target_mm from JSON
        int target_mm = -1;
        cJSON *root = cJSON_Parse(content);
        if (root) {
            cJSON *t = cJSON_GetObjectItem(root, "target_mm");
            if (cJSON_IsNumber(t)) {
                target_mm = t->valueint;
            }
            cJSON_Delete(root);
        }

        int slot = allocate_routine_slot(routine_name);
        if (slot < 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No routine slots left");
            return ESP_FAIL;
        }
        rid = s_routines[slot].routine_id;
        s_routines[slot].status = ROUTINE_STATUS_PENDING;
        // Pass target via parameter block
        fill_empty_param_t *p = malloc(sizeof(fill_empty_param_t));
        p->routine_id = rid;
        p->target_mm = target_mm;
        xTaskCreate(routine_fill_pod_task, "fill_pod", 4096,
                    (void*)p, configMAX_PRIORITIES - 1, NULL);
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

            cJSON_AddNumberToObject(root, "start_height_mm", rec->start_height_mm);
            cJSON_AddNumberToObject(root, "end_height_mm",   rec->end_height_mm);
            cJSON_AddNumberToObject(root, "min_height_mm",   rec->min_height_mm);
            cJSON_AddNumberToObject(root, "max_height_mm",   rec->max_height_mm);
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
#define FILL_DRAIN_MAX_TIMER_SEC 60.0f * 6.0f

//------------------------------------------
// 1) empty_pod
//------------------------------------------

// API for console to start the drain routine (target_pct < 0 uses default)
void start_empty_pod_routine(int target_pct)
{
    int slot = allocate_routine_slot("empty_pod");
    if (slot < 0) {
        ESP_LOGE(TAG, "start_empty_pod_routine: no slots");
        return;
    }
    int rid = s_routines[slot].routine_id;
    s_routines[slot].status = ROUTINE_STATUS_PENDING;
    
    // Create parameter block for empty task
    fill_empty_param_t *p = malloc(sizeof(fill_empty_param_t));
    p->routine_id = rid;
    p->target_mm = target_pct;  // used as target percentage
    
    xTaskCreate(routine_empty_pod_task, "empty_pod", 4096,
                (void*)p, configMAX_PRIORITIES - 1, NULL);
}

static void routine_empty_pod_task(void *pvParam)
{
    // Unpack parameters
    fill_empty_param_t *p = (fill_empty_param_t*) pvParam;
    int rid = p->routine_id;
    // Interpret argument as percentage to remain
    int target_pct = p->target_mm;
    free(p);
    
    // Compute sensor threshold distance (mm) from calibration
    int empty_raw = s_pod_state.raw_empty_mm;
    int full_raw = s_pod_state.raw_full_mm;
    int range = (empty_raw - full_raw);
    ESP_LOGI(TAG, "[empty_pod] Pod empty raw: %d mm, full raw: %d mm, range: %d mm",
             empty_raw, full_raw, range);

    // By default, drain completely (0%)
    if (target_pct < 0) target_pct = 0;
    else if (target_pct > 100) target_pct = 100;
    
    // Calculate target distance for sensor
    // Higher sensor reading = less fill
    int target_mm;
    
    if (range <= 0 || !s_pod_state.calibrated) {
        // Fallback defaults if not calibrated
        ESP_LOGW(TAG, "[empty_pod] Pod not calibrated, using default values");
        target_mm = TANK_EMPTY_HEIGHT_MM - TANK_DISTANCE_SENSOR_OFFSET_MM;
    } else {
        // Calculate target mm based on target percentage
        // When target_pct = 0, target_mm = empty_raw
        // When target_pct = 100, target_mm = headspace
        target_mm = empty_raw - (range * target_pct) / 100;
    }
    
    ESP_LOGI(TAG, "[empty_pod] Target remaining: %d%% -> %d mm sensor threshold", 
             target_pct, target_mm);
    
    routine_record_t *rec = find_routine_by_id(rid);
    if (!rec) { vTaskDelete(NULL); return; }
    rec->status = ROUTINE_STATUS_RUNNING;
    ESP_LOGI(TAG, "[empty_pod] start ID=%d", rid);

    int dist_mm = distance_sensor_read_mm();
    // Store raw mm values
    rec->start_height_mm = dist_mm;

    set_drain_pump_pwm(90);
    float t0 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    float last_log_s = t0;
    float total_power = 0.0f;
    
    // Counter for consecutive valid readings at target
    int target_readings = 0;
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));              // poll every 250 ms

        float cur_mA=0, volt_mV=0, pwr_mW=0;
        power_monitor_read_current(&cur_mA);
        power_monitor_read_voltage(&volt_mV);
        power_monitor_read_power(&pwr_mW);
        total_power += (pwr_mW * 0.25f);             // integrate 0.25 s slices

        dist_mm = distance_sensor_read_mm();
        if (dist_mm < 0) {
            // Invalid reading, skip and continue
            ESP_LOGW(TAG, "DRAINING: Invalid distance reading, skipping");
            continue;
        }

        float now_s = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
        // Use computed target or default empty threshold
        bool approaching_target = (dist_mm >= target_mm);

        if ((now_s - last_log_s >= 1.0f) || approaching_target) {
            ESP_LOGW(TAG, "DRAINING: Water level: %d mm (target: %d mm)", dist_mm, target_mm);
            last_log_s = now_s;
        }
        
        // Handle sensor noise when approaching target water level
        if (approaching_target) {
            target_readings++;
            ESP_LOGD(TAG, "DRAINING: Target readings count: %d/%d (dist = %d mm)", 
                     target_readings, READINGS_NEEDED, dist_mm);
            if (target_readings >= READINGS_NEEDED) {
                // Confirmed at target - multiple consistent readings
                ESP_LOGI(TAG, "Target level confirmed with %d consecutive readings >= %d mm",
                         READINGS_NEEDED, target_mm);
                break;
            }
        } else {
            // Reset counter if we get inconsistent readings
            target_readings = 0;
        }

        // Safety timeout
        if (now_s - t0 > FILL_DRAIN_MAX_TIMER_SEC) {
            ESP_LOGW(TAG, "Empty pod timeout reached");
            break;
        }
    }

    set_drain_pump_pwm(0);
    float t1 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    dist_mm = distance_sensor_read_mm();
    rec->end_height_mm = dist_mm;
    rec->time_elapsed_s  = t1 - t0;
    rec->power_used_mW_s = total_power;
    rec->status          = ROUTINE_STATUS_COMPLETED;

    ESP_LOGI(TAG,
        "[empty_pod] done ID=%d: time=%.1f s, power=%.1f mW·s, start=%d mm, end=%d mm",
        rid, (double)rec->time_elapsed_s, (double)total_power,
        rec->start_height_mm, rec->end_height_mm);

    vTaskDelete(NULL);
}


//------------------------------------------
// 2) fill_pod (tweaked polling & logging)
//------------------------------------------
void start_fill_pod_routine(int target_mm)
{
    int slot = allocate_routine_slot("fill_pod");
    if (slot < 0) {
        ESP_LOGE(TAG, "start_fill_pod_routine: no slots");
        return;
    }
    int rid = s_routines[slot].routine_id;
    s_routines[slot].status = ROUTINE_STATUS_PENDING;
    // Create parameter block for fill task
    fill_empty_param_t *p = malloc(sizeof(fill_empty_param_t));
    p->routine_id = rid;
    p->target_mm = target_mm;  // provided or -1 for default
    xTaskCreate(routine_fill_pod_task, "fill_pod", 4096,
                (void*)p, configMAX_PRIORITIES - 1, NULL);
}

static void routine_fill_pod_task(void *pvParam)
{
    // Unpack parameters
    fill_empty_param_t *p = (fill_empty_param_t*) pvParam;
    int rid = p->routine_id;
    // Interpret argument as percentage full
    int target_pct = p->target_mm;
    free(p);
    // Compute sensor threshold distance (mm) from calibration
    int empty_raw = s_pod_state.raw_empty_mm;
    int full_raw  = s_pod_state.raw_full_mm;
    int range     = (empty_raw - full_raw);
    // Print pod state
    ESP_LOGI(TAG, "[fill_pod] Pod state: empty=%d mm, full=%d mm, range=%d mm",
             empty_raw, full_raw, range);
    if (range <= 0) {
        // Fallback defaults
        empty_raw = TANK_EMPTY_HEIGHT_MM;
        full_raw  = TANK_HEADSPACE_MM;
        range     = empty_raw - full_raw;
    }
    // Clamp percentage
    if (target_pct < 0)        target_pct = 100;
    else if (target_pct > 100) target_pct = 100;
    // Lower sensor reading = more fill
    int target_mm = empty_raw - (range * target_pct) / 100;
    ESP_LOGI(TAG, "[fill_pod] Target fill: %d%% -> %d mm sensor threshold", target_pct, target_mm);

    routine_record_t *rec = find_routine_by_id(rid);
    if (!rec) { vTaskDelete(NULL); return; }
    rec->status = ROUTINE_STATUS_RUNNING;
    ESP_LOGI(TAG, "[fill_pod] start ID=%d", rid);

    int dist_mm = distance_sensor_read_mm();
    // Store raw mm values
    rec->start_height_mm = dist_mm;

    set_source_pump_pwm(70);

    float t0 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    float last_log_s = t0;
    float total_power = 0.0f;

    // Counter for consecutive valid readings at target
    int target_readings = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));

        float cur_mA=0, volt_mV=0, pwr_mW=0;
        power_monitor_read_current(&cur_mA);
        power_monitor_read_voltage(&volt_mV);
        power_monitor_read_power(&pwr_mW);
        total_power += (pwr_mW * 0.25f);

        dist_mm = distance_sensor_read_mm();
        if (dist_mm < 0) {
            // Invalid reading, skip and continue
            ESP_LOGW(TAG, "FILLING: Invalid distance reading, skipping");
            continue;
        }

        float now_s = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
        // Use computed sensor threshold
        int fill_threshold = target_mm;
        bool approaching_target = (dist_mm <= fill_threshold);

        if ((now_s - last_log_s >= 1.0f) || approaching_target) {
            ESP_LOGD(TAG, "FILLING: Water level: %d mm", dist_mm);
            last_log_s = now_s;
        }

        // Handle sensor noise when approaching target water level
        if (approaching_target) {
            target_readings++;
            ESP_LOGI(TAG, "FILLING: Target readings count: %d/%d (dist = %d mm)", target_readings, READINGS_NEEDED, dist_mm);
            if (target_readings >= READINGS_NEEDED) {
                ESP_LOGI(TAG, "Fill confirmed with %d consecutive readings <= %d mm", READINGS_NEEDED, fill_threshold);
                break;
            }
        } else {
            target_readings = 0;
        }

        // Safety timeout
        if ((now_s - t0) > FILL_DRAIN_MAX_TIMER_SEC) {
            ESP_LOGW(TAG, "Fill pod timeout reached");
            break;
        }
    }

    set_source_pump_pwm(0);

    float t1 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    dist_mm = distance_sensor_read_mm();
    rec->end_height_mm = dist_mm;
    rec->time_elapsed_s  = t1 - t0;
    rec->power_used_mW_s = total_power;
    rec->status          = ROUTINE_STATUS_COMPLETED;

    ESP_LOGI(TAG,
        "[fill_pod] done ID=%d: time=%.1f s, power=%.1f mW·s, start=%d mm, end=%d mm",
        rid, (double)rec->time_elapsed_s, (double)total_power,
        rec->start_height_mm, rec->end_height_mm);

    vTaskDelete(NULL);
}


//------------------------------------------
// 3) calibrate_pod
//------------------------------------------
void start_calibrate_pod_routine(void)
{
    int slot = allocate_routine_slot("calibrate_pod");
    if (slot < 0) {
        ESP_LOGE(TAG, "start_calibrate_pod_routine: no slots");
        return;
    }
    int rid = s_routines[slot].routine_id;
    s_routines[slot].status = ROUTINE_STATUS_PENDING;
    xTaskCreate(routine_calibrate_pod_task, "calibrate_pod", 4096, (void*)(intptr_t)rid,
                configMAX_PRIORITIES - 1, NULL);
}

static void routine_calibrate_pod_task(void *pvParam)
{
    int rid = (int)(intptr_t) pvParam;
    routine_record_t *rec = find_routine_by_id(rid);
    if (!rec) { vTaskDelete(NULL); return; }
    rec->status = ROUTINE_STATUS_RUNNING;
    ESP_LOGI(TAG, "[calibrate_pod] start ID=%d", rid);

    // Phase 1: Drain until user confirms empty
    ESP_LOGI(TAG, "[calibrate_pod] Phase 1: Draining pod; please confirm empty level via 'confirm_level <mm>'");
    set_drain_pump_pwm(90);
    while (s_confirmed_level_mm < 0) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    int raw_empty_mm = distance_sensor_read_mm();
    rec->min_height_mm = raw_empty_mm;
    ESP_LOGI(TAG, "[calibrate_pod] raw empty sensor distance: %d mm", raw_empty_mm);
    set_drain_pump_pwm(0);
    s_confirmed_level_mm = -1;

    // Phase 2: Fill until headspace threshold or timeout
    ESP_LOGI(TAG, "[calibrate_pod] Phase 2: Filling pod until headspace threshold (%d mm)", TANK_HEADSPACE_MM);
    set_source_pump_pwm(85);
    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS((FILL_DRAIN_MAX_TIMER_SEC * 1000)); // 6-minute timeout
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250));
        int dist_mm = distance_sensor_read_mm();
        if (dist_mm <= TANK_HEADSPACE_MM) {
            ESP_LOGI(TAG, "[calibrate_pod] reached headspace threshold: %d mm", dist_mm);
            break;
        }
        if ((xTaskGetTickCount() - start_tick) > timeout_ticks) {
            ESP_LOGW(TAG, "[calibrate_pod] fill timeout");
            break;
        }
    }
    set_source_pump_pwm(0);

    // Prompt user to confirm full sensor distance
    ESP_LOGI(TAG, "[calibrate_pod] Please confirm full sensor distance via 'confirm_level <mm>'");
    while (s_confirmed_level_mm < 0) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    int raw_full_mm = distance_sensor_read_mm();
    rec->max_height_mm = raw_full_mm;
    ESP_LOGI(TAG, "[calibrate_pod] raw full sensor distance: %d mm", raw_full_mm);
    s_confirmed_level_mm = -1;

    // Initialize pod state with new calibration
    pod_state_init(&s_pod_state, raw_empty_mm, raw_full_mm, raw_full_mm);
    ESP_LOGI(TAG, "[calibrate_pod] Pod state initialized: empty_raw=%d mm, full_raw=%d mm", raw_empty_mm, raw_full_mm);

    // Persist calibration to NVS
    pod_state_save_settings(&s_pod_state, raw_empty_mm, raw_full_mm);
    ESP_LOGI(TAG, "[calibrate_pod] Calibration saved to NVS");

    // Finalize calibration
    rec->status = ROUTINE_STATUS_COMPLETED;
    ESP_LOGI(TAG, "[calibrate_pod] done ID=%d: empty=%d mm, full=%d mm", rid, rec->min_height_mm, rec->max_height_mm);
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

    // Short delay before processing to allow system stabilization
    // This helps prevent crashes when schedules are sent in quick succession
    vTaskDelay(pdMS_TO_TICKS(100));

    routine_record_t *rec = find_routine_by_id(rid);
    if (!rec) {
        // allocate a persistent slot for this fixed-ID schedule
        for (int i = 0; i < MAX_ROUTINE_RECORDS; i++) {
            if (!s_routines[i].in_use) {
                s_routines[i].in_use = true;
                s_routines[i].routine_id = rid;
                s_routines[i].status = ROUTINE_STATUS_PENDING;
                strncpy(s_routines[i].routine_name, p->name, sizeof(s_routines[i].routine_name)-1);
                s_routines[i].routine_name[sizeof(s_routines[i].routine_name)-1] = '\0';
                rec = &s_routines[i];
                break;
            }
        }
        if (!rec) {
            ESP_LOGE(TAG, "schedule task: no slots for routine ID %d", rid);
            free(p);
            vTaskDelete(NULL);
            return;
        }
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

    // Send to schedule manager with longer timeout
    ESP_LOGI(TAG, "Sending schedule to manager for %s...", p->name);
    if (xQueueSend(schedule_msg_queue, &sched_msg, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGW(TAG, "schedule task: Failed to send schedule message - queue might be full");
        rec->status = ROUTINE_STATUS_FAILED;
        free(p);
        float t1 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
        rec->time_elapsed_s = t1 - t0;
        vTaskDelete(NULL);
        return;
    }
    else {
        ESP_LOGI(TAG, "schedule task: Successfully sent schedule message to queue");
    }
    
    // Small delay to allow schedule manager to process the queue message
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Also save to NVS (persist)
    esp_err_t err = ESP_OK;
    
    // Add a small delay before NVS operations to reduce contention
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "Saving schedule to NVS for %s...", p->name);
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
        ESP_LOGW(TAG, "schedule task: Failed to save schedule to NVS: %s", esp_err_to_name(err));
        rec->status = ROUTINE_STATUS_FAILED;
    }
    else {
        ESP_LOGI(TAG, "schedule task: Successfully saved schedule to NVS");
    }
    
    // Clean up and mark as completed
    free(p);
    float t1 = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
    rec->time_elapsed_s = t1 - t0;
    rec->power_used_mW_s = 0.0f;
    rec->status = ROUTINE_STATUS_COMPLETED;
    ESP_LOGI(TAG, "Schedule task completed for %s (ID=%d)", rec->routine_name, rid);

    // Close out task
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
            // Initialize water level fields
            s_routines[i].start_height_mm = -1;       // Invalid value indicates not yet measured
            s_routines[i].end_height_mm   = -1;       // Invalid value indicates not yet measured
            s_routines[i].min_height_mm   = 999999;   // Min = full height (small distance value)
            s_routines[i].max_height_mm   = 0;        // Max = empty height (large distance value)
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


// Print current stored schedules (LED, planter, air)
void print_schedules(void) {
    ESP_LOGI(TAG, "=== Current LED schedule ===");
    xSemaphoreTake(schedule_led_mutex, portMAX_DELAY);
    for (int h=0; h<24; ++h) {
        ESP_LOGI(TAG, "  Hour %2d: %3d%%", h, current_light_schedule[h]);
    }
    xSemaphoreGive(schedule_led_mutex);

    ESP_LOGI(TAG, "=== Current Planter schedule ===");
    xSemaphoreTake(schedule_planter_mutex, portMAX_DELAY);
    for (int h=0; h<24; ++h) {
        ESP_LOGI(TAG, "  Hour %2d: %3d%%", h, current_planter_schedule[h]);
    }
    xSemaphoreGive(schedule_planter_mutex);

    ESP_LOGI(TAG, "=== Current Air schedule ===");
    xSemaphoreTake(schedule_air_mutex, portMAX_DELAY);
    for (int h=0; h<24; ++h) {
        ESP_LOGI(TAG, "  Hour %2d: %3d%%", h, current_air_schedule[h]);
    }
    xSemaphoreGive(schedule_air_mutex);
}

static esp_err_t schedules_print_get_handler(httpd_req_t *req) {
    print_schedules();
    httpd_resp_sendstr(req, "Schedules printed");
    return ESP_OK;
}

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
        set_led_array_pwm(100);
        // set_led_array_binary(true);
        httpd_resp_sendstr(req, "LED array turned ON");
    } else if (strcmp(state->valuestring, "off") == 0) {
        set_led_array_pwm(0);
        // set_led_array_binary(false);
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
    power_monitor_read_current(&cur_mA);
    power_monitor_read_voltage(&volt_mV);
    power_monitor_read_power(&pwr_mW);

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
    
    // Save to JSON configuration
    esp_err_t err = config_save_mdns_suffix(suffix_item->valuestring);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save mDNS suffix: %s", esp_err_to_name(err));
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save suffix");
        return ESP_FAIL;
    }
    
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Suffix updated");
    return ESP_OK;
}

static esp_err_t device_restart_post_handler(httpd_req_t *req) {
    esp_restart();
    return ESP_OK; 
}

// wrappers for console‐driven schedule start
void start_light_schedule(uint8_t schedule[24])
{
    ESP_LOGI(TAG, "Starting light schedule with %d entries", 24);
    typedef struct { int routine_id; char name[32]; uint8_t sched[24]; } sched_param_t;
    sched_param_t *p = malloc(sizeof(*p));
    p->routine_id = LIGHT_SCHEDULE_ID;
    strcpy(p->name, "light_schedule");
    memcpy(p->sched, schedule, 24);
    xTaskCreate(routine_store_schedule_task, "sched_store", 4096,
                p, configMAX_PRIORITIES - 1, NULL);
}

void start_planter_schedule(uint8_t schedule[24])
{
    ESP_LOGI(TAG, "Starting planter schedule with %d entries", 24);
    typedef struct { int routine_id; char name[32]; uint8_t sched[24]; } sched_param_t;
    sched_param_t *p = malloc(sizeof(*p));
    p->routine_id = PLANTER_SCHEDULE_ID;
    strcpy(p->name, "planter_pod_schedule");
    memcpy(p->sched, schedule, 24);
    xTaskCreate(routine_store_schedule_task, "sched_store", 4096,
                p, configMAX_PRIORITIES - 1, NULL);
}

void start_air_schedule(uint8_t schedule[24])
{
    ESP_LOGI(TAG, "Starting air schedule with %d entries", 24);
    typedef struct { int routine_id; char name[32]; uint8_t sched[24]; } sched_param_t;
    sched_param_t *p = malloc(sizeof(*p));
    p->routine_id = AIR_SCHEDULE_ID;
    strcpy(p->name, "air_pump_schedule");
    memcpy(p->sched, schedule, 24);
    xTaskCreate(routine_store_schedule_task, "sched_store", 4096,
                p, configMAX_PRIORITIES - 1, NULL);
}

// New tasks to update actuators based on schedule with change detection

void schedule_air_task(void *pvParam) {
    static uint8_t last_air_duty = 0xFF; // initial invalid value
    QueueHandle_t actuator_queue = get_actuator_queue();
    while (1) {
        int hour = get_current_hour();
        if (hour >= 0 && hour < 24) {
            uint8_t duty;
            xSemaphoreTake(schedule_air_mutex, portMAX_DELAY);
            duty = current_air_schedule[hour];
            xSemaphoreGive(schedule_air_mutex);
            if (duty != last_air_duty) {
               actuator_command_t cmd = { .cmd_type = ACTUATOR_CMD_AIR_PUMP_PWM, .value = duty };
               xQueueSend(actuator_queue, &cmd, portMAX_DELAY);
                last_air_duty = duty;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void schedule_led_task(void *pvParam) {
    static uint8_t last_led_duty = 0xFF; // initial invalid value
    QueueHandle_t actuator_queue = get_actuator_queue();
    while (1) {
        int hour = get_current_hour();
        if (hour >= 0 && hour < 24) {
            uint8_t duty;
            xSemaphoreTake(schedule_led_mutex, portMAX_DELAY);
            duty = current_light_schedule[hour];
            xSemaphoreGive(schedule_led_mutex);
            if (duty != last_led_duty) {
               actuator_command_t cmd = { .cmd_type = ACTUATOR_CMD_LED_ARRAY_PWM, .value = duty };
               xQueueSend(actuator_queue, &cmd, portMAX_DELAY);
                last_led_duty = duty;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void schedule_planter_task(void *pvParam) {
    static uint8_t last_planter_duty = 0xFF; // initial invalid value
    QueueHandle_t actuator_queue = get_actuator_queue();
    while (1) {
        int hour = get_current_hour();
        if (hour >= 0 && hour < 24) {
            uint8_t duty;
            xSemaphoreTake(schedule_planter_mutex, portMAX_DELAY);
            duty = current_planter_schedule[hour];
            xSemaphoreGive(schedule_planter_mutex);
            if (last_planter_duty == 0xFF) {
                // First run, set initial duty
                last_planter_duty = duty;
            }
            // If Duty is 0 or 100, and the current duty is the same as last, skip sending command
            // This avoids unnecessary commands for 0% or 100% duty cycles
            if ((duty == last_planter_duty) && (duty == 0 || duty == 100)) {
                // No change, skip sending command
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            if (duty == 0) {
                { actuator_command_t cmd = { .cmd_type = ACTUATOR_CMD_PLANTER_PUMP_PWM, .value = 0 }; xQueueSend(actuator_queue, &cmd, portMAX_DELAY); }
            } else if (duty == 100) {
                { actuator_command_t cmd = { .cmd_type = ACTUATOR_CMD_PLANTER_PUMP_PWM, .value = 100 }; xQueueSend(actuator_queue, &cmd, portMAX_DELAY); }
            } else {
                time_t now;
                struct tm tm_now;
                time(&now);
                localtime_r(&now, &tm_now);
                int sec_into_hour = tm_now.tm_min * 60 + tm_now.tm_sec;
                int on_seconds = (duty * 3600) / 100;
                // If the current time is within the duty cycle, turn on the pump
                if (sec_into_hour < on_seconds) {
                    ESP_LOGI(TAG, "Planter duty: %d%%, sec_into_hour: %d, on_seconds: %d", duty, sec_into_hour, on_seconds);


                    { actuator_command_t cmd = { .cmd_type = ACTUATOR_CMD_PLANTER_PUMP_PWM, .value = 100 }; xQueueSend(actuator_queue, &cmd, portMAX_DELAY); }
                } else {
                    { actuator_command_t cmd = { .cmd_type = ACTUATOR_CMD_PLANTER_PUMP_PWM, .value = 0 }; xQueueSend(actuator_queue, &cmd, portMAX_DELAY); }
                }
            }
            last_planter_duty = duty;
         }
         vTaskDelay(pdMS_TO_TICKS(500));
     }
}
