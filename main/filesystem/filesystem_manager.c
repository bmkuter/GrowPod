#include "filesystem_manager.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include <sys/stat.h>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

static const char *TAG = "FILESYSTEM";
static bool s_fs_mounted = false;

esp_err_t filesystem_init(void)
{
    if (s_fs_mounted) {
        ESP_LOGW(TAG, "Filesystem already mounted");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing LittleFS filesystem...");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = LFS_MOUNT_POINT,
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    s_fs_mounted = true;

    // Get filesystem information
    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "LittleFS mounted successfully");
        ESP_LOGI(TAG, "  Partition size: total: %d bytes, used: %d bytes (%.1f%%)", 
                 total, used, 100.0 * used / total);
    }

    return ESP_OK;
}

esp_err_t filesystem_deinit(void)
{
    if (!s_fs_mounted) {
        ESP_LOGW(TAG, "Filesystem not mounted");
        return ESP_OK;
    }

    esp_err_t ret = esp_vfs_littlefs_unregister("littlefs");
    if (ret == ESP_OK) {
        s_fs_mounted = false;
        ESP_LOGI(TAG, "LittleFS unmounted successfully");
    } else {
        ESP_LOGE(TAG, "Failed to unmount LittleFS (%s)", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t filesystem_format(void)
{
    ESP_LOGW(TAG, "Formatting LittleFS filesystem...");
    
    if (s_fs_mounted) {
        esp_err_t ret = filesystem_deinit();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    esp_err_t ret = esp_littlefs_format("littlefs");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to format LittleFS (%s)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "LittleFS formatted successfully");
    
    // Remount after format
    return filesystem_init();
}

esp_err_t filesystem_get_usage(size_t *total_bytes, size_t *used_bytes)
{
    if (!s_fs_mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    return esp_littlefs_info("littlefs", total_bytes, used_bytes);
}

bool filesystem_is_mounted(void)
{
    return s_fs_mounted;
}

esp_err_t filesystem_test(void)
{
    if (!s_fs_mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting filesystem test...");

    const char *test_file = LFS_MOUNT_POINT "/test.txt";
    const char *test_data = "Hello from GrowPod LittleFS!\nThis is a test file.\nLine 3 here.";
    
    // Write test
    ESP_LOGI(TAG, "Writing test file: %s", test_file);
    FILE *f = fopen(test_file, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }

    size_t written = fwrite(test_data, 1, strlen(test_data), f);
    fclose(f);

    if (written != strlen(test_data)) {
        ESP_LOGE(TAG, "Failed to write complete data (wrote %d of %d bytes)", written, strlen(test_data));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "  Written %d bytes", written);

    // Read test
    ESP_LOGI(TAG, "Reading test file: %s", test_file);
    f = fopen(test_file, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }

    char buffer[256];
    size_t read_bytes = fread(buffer, 1, sizeof(buffer) - 1, f);
    fclose(f);
    buffer[read_bytes] = '\0';

    ESP_LOGI(TAG, "  Read %d bytes", read_bytes);
    ESP_LOGI(TAG, "  Content: %s", buffer);

    // Verify
    if (strcmp(buffer, test_data) == 0) {
        ESP_LOGI(TAG, "Filesystem test PASSED!");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Filesystem test FAILED - data mismatch");
        return ESP_FAIL;
    }
}

esp_err_t filesystem_list_dir(const char *path)
{
    if (!s_fs_mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    DIR *dir = opendir(path);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory: %s", path);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Listing directory: %s", path);
    
    struct dirent *entry;
    int count = 0;
    while ((entry = readdir(dir)) != NULL) {
        struct stat st;
        char full_path[LFS_MAX_PATH_LEN];
        int path_len = snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);
        
        if (path_len >= sizeof(full_path)) {
            ESP_LOGW(TAG, "Path too long, skipping: %s/%s", path, entry->d_name);
            continue;
        }
        
        if (stat(full_path, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                ESP_LOGI(TAG, "  [DIR]  %s", entry->d_name);
            } else {
                ESP_LOGI(TAG, "  [FILE] %s (%ld bytes)", entry->d_name, st.st_size);
            }
            count++;
        }
    }
    
    closedir(dir);
    ESP_LOGI(TAG, "Total entries: %d", count);
    
    return ESP_OK;
}

static void list_dir_recursive_helper(const char *path, int depth)
{
    DIR *dir = opendir(path);
    if (dir == NULL) {
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        struct stat st;
        char full_path[LFS_MAX_PATH_LEN];
        int path_len = snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);
        
        if (path_len >= sizeof(full_path)) {
            continue;
        }
        
        if (stat(full_path, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                ESP_LOGI(TAG, "  %s/", full_path);
                list_dir_recursive_helper(full_path, depth + 1);
            } else {
                ESP_LOGI(TAG, "  %s (%ld bytes)", full_path, st.st_size);
            }
        }
    }
    
    closedir(dir);
}

esp_err_t filesystem_list_dir_recursive(const char *path)
{
    if (!s_fs_mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Listing directory recursively: %s", path);
    list_dir_recursive_helper(path, 0);
    
    return ESP_OK;
}

esp_err_t filesystem_read_file(const char *path, char **buffer, size_t *size)
{
    if (!s_fs_mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", path);
        return ESP_FAIL;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (file_size < 0 || file_size > 64 * 1024) {  // Limit to 64KB for safety
        ESP_LOGE(TAG, "Invalid file size: %ld bytes", file_size);
        fclose(f);
        return ESP_FAIL;
    }

    // Allocate buffer (+1 for null terminator)
    *buffer = (char *)malloc(file_size + 1);
    if (*buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for file");
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    // Read file
    size_t bytes_read = fread(*buffer, 1, file_size, f);
    fclose(f);

    if (bytes_read != file_size) {
        ESP_LOGE(TAG, "Failed to read complete file");
        free(*buffer);
        *buffer = NULL;
        return ESP_FAIL;
    }

    // Null terminate
    (*buffer)[file_size] = '\0';
    *size = file_size;

    return ESP_OK;
}

esp_err_t filesystem_delete_file(const char *path)
{
    if (!s_fs_mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    if (unlink(path) == 0) {
        ESP_LOGI(TAG, "Deleted file: %s", path);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to delete file: %s", path);
        return ESP_FAIL;
    }
}

esp_err_t filesystem_create_dir(const char *path)
{
    if (!s_fs_mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    if (mkdir(path, 0775) == 0) {
        ESP_LOGI(TAG, "Created directory: %s", path);
        return ESP_OK;
    } else {
        // Check if directory already exists
        struct stat st;
        if (stat(path, &st) == 0 && S_ISDIR(st.st_mode)) {
            ESP_LOGW(TAG, "Directory already exists: %s", path);
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Failed to create directory: %s", path);
        return ESP_FAIL;
    }
}
