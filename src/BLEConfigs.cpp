/*******************************************************************************
 * @file BLEConfigs.cpp
 * @brief Implementation of common BLE functionality including initialization,
 * utilities, and shared resources.
 *
 * @version 0.0.1
 * @date 2025-06-15
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "BLEConfigs.hpp"
#include "esp_random.h" 
#include <stdarg.h>

/******************************************************************************/
/*                              Version Information                           */
/******************************************************************************/

// ✅ AGREGADO: Definiciones de versión faltantes
#define BLE_LIBRARY_VERSION_MAJOR  0
#define BLE_LIBRARY_VERSION_MINOR  0
#define BLE_LIBRARY_VERSION_PATCH  1

/******************************************************************************/
/*                              Static Variables                              */
/******************************************************************************/

static const char* BLE_TAG = "BLE_COMMON";
static ble_state_t ble_global_state = BLE_STATE_UNINITIALIZED;
static ble_event_callback_t global_event_callback = nullptr;
static ble_log_callback_t global_log_callback = nullptr;

/******************************************************************************/
/*                              Default UUIDs                                 */
/******************************************************************************/

/**
 * @brief Default custom service UUID (128-bit) for enhanced security
 */
const uint8_t BLE_DEFAULT_SERVICE_UUID_128[BLE_UUID_128_LEN] = {
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0
};

/**
 * @brief Default custom battery characteristic UUID (128-bit) 
 */
const uint8_t BLE_DEFAULT_BATTERY_CHAR_UUID_128[BLE_UUID_128_LEN] = {
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00
};

/**
 * @brief Default custom data characteristic UUID (128-bit)
 */
const uint8_t BLE_DEFAULT_CUSTOM_CHAR_UUID_128[BLE_UUID_128_LEN] = {
    0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
    0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99
};

/******************************************************************************/
/*                              Core Functions                                */
/******************************************************************************/

esp_err_t ble_common_init(void) {
    esp_err_t ret;

    ble_log(ESP_LOG_INFO, "Initializing BLE common subsystem");

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Release memory for Classic Bluetooth (we only use BLE)
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_WARN, "Failed to release Classic BT memory: %s", esp_err_to_name(ret));
    }

    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to initialize BT controller: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable BLE mode only
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to enable BT controller: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to initialize Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }

    ble_global_state = BLE_STATE_INITIALIZED;
    ble_log(ESP_LOG_INFO, "BLE common subsystem initialized successfully");

    return ESP_OK;
}

esp_err_t ble_common_deinit(void) {
    ble_log(ESP_LOG_INFO, "Deinitializing BLE common subsystem");

    esp_err_t ret = ESP_OK;

    // Disable and deinitialize Bluedroid
    if (esp_bluedroid_disable() != ESP_OK) {
        ble_log(ESP_LOG_WARN, "Failed to disable Bluedroid");
        ret = ESP_FAIL;
    }

    if (esp_bluedroid_deinit() != ESP_OK) {
        ble_log(ESP_LOG_WARN, "Failed to deinitialize Bluedroid");
        ret = ESP_FAIL;
    }

    // Disable and deinitialize Bluetooth controller
    if (esp_bt_controller_disable() != ESP_OK) {
        ble_log(ESP_LOG_WARN, "Failed to disable BT controller");
        ret = ESP_FAIL;
    }

    if (esp_bt_controller_deinit() != ESP_OK) {
        ble_log(ESP_LOG_WARN, "Failed to deinitialize BT controller");
        ret = ESP_FAIL;
    }

    ble_global_state = BLE_STATE_UNINITIALIZED;
    ble_log(ESP_LOG_INFO, "BLE common subsystem deinitialized");

    return ret;
}

void ble_set_event_callback(ble_event_callback_t callback) {
    global_event_callback = callback;
    ble_log(ESP_LOG_DEBUG, "Global event callback set");
}

void ble_set_log_callback(ble_log_callback_t callback) {
    global_log_callback = callback;
    ble_log(ESP_LOG_DEBUG, "Global log callback set");
}

ble_state_t ble_get_state(void) {
    return ble_global_state;
}

/******************************************************************************/
/*                              Utility Functions                             */
/******************************************************************************/

void ble_addr_to_string(esp_bd_addr_t bda, char *str) {
    if (str == nullptr) {
        return;
    }
    
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

bool ble_compare_uuid128(const uint8_t* uuid1, const uint8_t* uuid2) {
    if (uuid1 == nullptr || uuid2 == nullptr) {
        return false;
    }
    
    return memcmp(uuid1, uuid2, BLE_UUID_128_LEN) == 0;
}

esp_err_t ble_generate_auth_key(char* key_buffer, size_t key_length) {
    if (key_buffer == nullptr || key_length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Simple random key generation
    const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    
    for (size_t i = 0; i < key_length - 1; i++) {
        uint32_t random_val = esp_random();  // ✅ CORREGIDO: Ahora funciona con el include
        key_buffer[i] = charset[random_val % (sizeof(charset) - 1)];
    }
    key_buffer[key_length - 1] = '\0';

    ble_log(ESP_LOG_DEBUG, "Generated authentication key of length %d", key_length - 1);
    return ESP_OK;
}

bool ble_validate_device_name(const char* name) {
    if (name == nullptr) {
        return false;
    }

    size_t len = strlen(name);
    if (len == 0 || len >= BLE_MAX_DEVICE_NAME_LEN) {
        return false;
    }

    // Check for valid characters (printable ASCII)
    for (size_t i = 0; i < len; i++) {
        if (name[i] < 32 || name[i] > 126) {
            return false;
        }
    }

    return true;
}

esp_err_t ble_create_default_security_config(ble_security_config_t* config, 
                                            ble_security_level_t level) {
    if (config == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(config, 0, sizeof(ble_security_config_t));
    
    config->level = level;
    
    switch (level) {
        case BLE_SECURITY_NONE:
            config->use_custom_uuids = false;
            config->require_authentication = false;
            break;
            
        case BLE_SECURITY_BASIC:
            config->use_custom_uuids = true;
            config->require_authentication = false;
            memcpy(config->service_uuid, BLE_DEFAULT_SERVICE_UUID_128, BLE_UUID_128_LEN);
            memcpy(config->battery_char_uuid, BLE_DEFAULT_BATTERY_CHAR_UUID_128, BLE_UUID_128_LEN);
            memcpy(config->custom_char_uuid, BLE_DEFAULT_CUSTOM_CHAR_UUID_128, BLE_UUID_128_LEN);
            break;
            
        case BLE_SECURITY_AUTHENTICATED:
            config->use_custom_uuids = true;
            config->require_authentication = true;
            strncpy(config->auth_key, BLE_DEFAULT_AUTH_KEY, BLE_MAX_AUTH_KEY_LEN - 1);
            memcpy(config->service_uuid, BLE_DEFAULT_SERVICE_UUID_128, BLE_UUID_128_LEN);
            memcpy(config->battery_char_uuid, BLE_DEFAULT_BATTERY_CHAR_UUID_128, BLE_UUID_128_LEN);
            memcpy(config->custom_char_uuid, BLE_DEFAULT_CUSTOM_CHAR_UUID_128, BLE_UUID_128_LEN);
            break;
            
        case BLE_SECURITY_ENCRYPTED:
            // TODO: Implement BLE pairing/bonding configuration
            config->use_custom_uuids = true;
            config->require_authentication = true;
            strncpy(config->auth_key, BLE_DEFAULT_AUTH_KEY, BLE_MAX_AUTH_KEY_LEN - 1);
            memcpy(config->service_uuid, BLE_DEFAULT_SERVICE_UUID_128, BLE_UUID_128_LEN);
            memcpy(config->battery_char_uuid, BLE_DEFAULT_BATTERY_CHAR_UUID_128, BLE_UUID_128_LEN);
            memcpy(config->custom_char_uuid, BLE_DEFAULT_CUSTOM_CHAR_UUID_128, BLE_UUID_128_LEN);
            break;
            
        default:
            return ESP_ERR_INVALID_ARG;
    }

    ble_log(ESP_LOG_DEBUG, "Created default security config for level %d", level);
    return ESP_OK;
}

void ble_print_version_info(void) {
    ble_log(ESP_LOG_INFO, "=== BLE Library Information ===");
    ble_log(ESP_LOG_INFO, "Version: %d.%d.%d", 
            BLE_LIBRARY_VERSION_MAJOR, BLE_LIBRARY_VERSION_MINOR, BLE_LIBRARY_VERSION_PATCH);  // ✅ CORREGIDO
    ble_log(ESP_LOG_INFO, "ESP-IDF Version: %s", esp_get_idf_version());
    ble_log(ESP_LOG_INFO, "Compile Time: %s %s", __DATE__, __TIME__);
    ble_log(ESP_LOG_INFO, "===============================");
}

/******************************************************************************/
/*                              Logging Function                              */
/******************************************************************************/

void ble_log(esp_log_level_t level, const char* format, ...) {
    if (format == nullptr) {
        return;
    }

    char log_buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(log_buffer, sizeof(log_buffer), format, args);
    va_end(args);

    // Use custom log callback if available
    if (global_log_callback != nullptr) {
        global_log_callback(level, BLE_TAG, log_buffer);
    } else {
        // Use ESP-IDF logging
        esp_log_write(level, BLE_TAG, "%s", log_buffer);
    }
}

uint64_t ble_get_timestamp(void) {
    return esp_timer_get_time();
}

uint8_t ble_rssi_to_quality(int8_t rssi) {
    // Convert RSSI to quality percentage
    // RSSI range typically: -100 dBm (worst) to -30 dBm (best)
    if (rssi >= -30) {
        return 100;  // Excellent signal
    } else if (rssi >= -50) {
        return 80 + ((rssi + 50) * 20) / 20;  // Very good: 80-100%
    } else if (rssi >= -70) {
        return 50 + ((rssi + 70) * 30) / 20;  // Good: 50-80%
    } else if (rssi >= -90) {
        return 20 + ((rssi + 90) * 30) / 20;  // Fair: 20-50%
    } else if (rssi >= -100) {
        return ((rssi + 100) * 20) / 10;      // Poor: 0-20%
    } else {
        return 0;    // No signal
    }
}

bool ble_validate_uuid(const uint8_t* uuid, size_t length) {
    if (uuid == nullptr) {
        return false;
    }

    // Check valid UUID lengths
    if (length != 2 && length != 4 && length != 16) {
        return false;
    }

    // For now, just check that it's not all zeros
    bool all_zeros = true;
    for (size_t i = 0; i < length; i++) {
        if (uuid[i] != 0) {
            all_zeros = false;
            break;
        }
    }

    return !all_zeros;
}