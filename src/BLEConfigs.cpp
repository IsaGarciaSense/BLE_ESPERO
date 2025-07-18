/*******************************************************************************
 * @file BLEConfigs.cpp
 * @brief Implementation of common BLE functionality including initialization,
 * utilities, and shared resources.
 *
 * @version 0.0.6
 * @date 2025-06-26
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "BLEConfigs.hpp"

#include <stdarg.h>

#include "esp_random.h"

#define BLE_LIBRARY_VERSION_MAJOR  0
#define BLE_LIBRARY_VERSION_MINOR  0
#define BLE_LIBRARY_VERSION_PATCH  5

static const char* s_bleTag = "BLE_COMMON";
static ble_state_t s_bleGlobalState = BLE_STATE_UNINITIALIZED;
static ble_event_callback_t s_globalEventCallback = nullptr;
static ble_log_callback_t s_globalLogCallback = nullptr;
static bool s_bleCommonInitialized = false;

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

esp_err_t ble_common_init(void) {
    esp_err_t ret;

    if (s_bleCommonInitialized) {
        ble_log(ESP_LOG_INFO, "BLE common subsystem already initialized, skipping...");
        return ESP_OK;
    }

    ble_log(ESP_LOG_INFO, "Initializing BLE common subsystem");

    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_ERROR, "Failed to release Classic BT memory: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_DEBUG, "Classic BT memory already released");
    } else {
        ble_log(ESP_LOG_DEBUG, "Classic BT memory released successfully");
    }

    esp_bt_controller_config_t btConfig = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&btConfig);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_ERROR, "Failed to initialize BT controller: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_DEBUG, "BT controller already initialized");
    } else {
        ble_log(ESP_LOG_DEBUG, "BT controller initialized successfully");
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_ERROR, "Failed to enable BT controller: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_DEBUG, "BT controller already enabled");
    } else {
        ble_log(ESP_LOG_DEBUG, "BT controller enabled successfully");
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_ERROR, "Failed to initialize Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_DEBUG, "Bluedroid already initialized");
    } else {
        ble_log(ESP_LOG_DEBUG, "Bluedroid initialized successfully");
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_ERROR, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_DEBUG, "Bluedroid already enabled");
    } else {
        ble_log(ESP_LOG_DEBUG, "Bluedroid enabled successfully");
    }

    s_bleCommonInitialized = true;
    s_bleGlobalState = BLE_STATE_INITIALIZED;
    ble_log(ESP_LOG_INFO, "BLE common subsystem initialized successfully");

    return ESP_OK;
}

esp_err_t ble_common_deinit(void) {
    if (!s_bleCommonInitialized) {
        ble_log(ESP_LOG_DEBUG, "BLE common subsystem not initialized, skipping deinit");
        return ESP_OK;
    }

    ble_log(ESP_LOG_INFO, "Deinitializing BLE common subsystem");

    esp_err_t ret = ESP_OK;
    esp_err_t finalRet = ESP_OK;

    ret = esp_bluedroid_disable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_WARN, "Failed to disable Bluedroid: %s", esp_err_to_name(ret));
        finalRet = ESP_FAIL;
    }

    ret = esp_bluedroid_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_WARN, "Failed to deinitialize Bluedroid: %s", esp_err_to_name(ret));
        finalRet = ESP_FAIL;
    }

    ret = esp_bt_controller_disable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_WARN, "Failed to disable BT controller: %s", esp_err_to_name(ret));
        finalRet = ESP_FAIL;
    }

    ret = esp_bt_controller_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ble_log(ESP_LOG_WARN, "Failed to deinitialize BT controller: %s", esp_err_to_name(ret));
        finalRet = ESP_FAIL;
    }

    s_bleCommonInitialized = false;
    s_bleGlobalState = BLE_STATE_UNINITIALIZED;
    ble_log(ESP_LOG_INFO, "BLE common subsystem deinitialized");

    return finalRet;
}

bool ble_common_is_initialized(void) {
    return s_bleCommonInitialized;
}

void ble_set_event_callback(ble_event_callback_t callback) {
    s_globalEventCallback = callback;
    ble_log(ESP_LOG_DEBUG, "Global event callback set");
}

void ble_set_log_callback(ble_log_callback_t callback) {
    s_globalLogCallback = callback;
    ble_log(ESP_LOG_DEBUG, "Global log callback set");
}

ble_state_t ble_get_state(void) {
    return s_bleGlobalState;
}

void ble_addr_to_string(esp_bd_addr_t bda, char *_str) {
    if (_str == nullptr) {
        return;
    }
    
    sprintf(_str, "%02X:%02X:%02X:%02X:%02X:%02X",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

bool ble_compare_uuid128(const uint8_t* uuid1, const uint8_t* uuid2) {
    if (uuid1 == nullptr || uuid2 == nullptr) {
        return false;
    }
    
    return memcmp(uuid1, uuid2, BLE_UUID_128_LEN) == 0;
}

esp_err_t ble_generate_auth_key(char* _keyBuffer, size_t keyLength) {
    if (_keyBuffer == nullptr || keyLength == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Simple random key generation
    const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    
    for (size_t i = 0; i < keyLength - 1; i++) {
        uint32_t randomVal = esp_random();
        _keyBuffer[i] = charset[randomVal % (sizeof(charset) - 1)];
    }
    _keyBuffer[keyLength - 1] = '\0';

    ble_log(ESP_LOG_DEBUG, "Generated authentication key of length %d", keyLength - 1);
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

    for (size_t i = 0; i < len; i++) {
        if (name[i] < 32 || name[i] > 126) {
            return false;
        }
    }

    return true;
}

esp_err_t ble_create_default_security_config(ble_security_config_t* _config, 
                                              ble_security_level_t level) {
    if (_config == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(_config, 0, sizeof(ble_security_config_t));
    
    _config->level = level;
    
    switch (level) {
        case BLE_SECURITY_NONE:
            _config->useCustomUUIDS = false;
            _config->requireAuthentication = false;
            break;
            
        case BLE_SECURITY_BASIC:
            _config->useCustomUUIDS = true;
            _config->requireAuthentication = false;
            memcpy(_config->serviceUUID, BLE_DEFAULT_SERVICE_UUID_128, BLE_UUID_128_LEN);
            memcpy(_config->batteryCharUUID, BLE_DEFAULT_BATTERY_CHAR_UUID_128, BLE_UUID_128_LEN);
            memcpy(_config->customCharUUID, BLE_DEFAULT_CUSTOM_CHAR_UUID_128, BLE_UUID_128_LEN);
            break;
            
        case BLE_SECURITY_AUTHENTICATED:
            _config->useCustomUUIDS = true;
            _config->requireAuthentication = true;
            strncpy(_config->authKey, BLE_DEFAULT_AUTH_KEY, BLE_MAX_AUTH_KEY_LEN - 1);
            memcpy(_config->serviceUUID, BLE_DEFAULT_SERVICE_UUID_128, BLE_UUID_128_LEN);
            memcpy(_config->batteryCharUUID, BLE_DEFAULT_BATTERY_CHAR_UUID_128, BLE_UUID_128_LEN);
            memcpy(_config->customCharUUID, BLE_DEFAULT_CUSTOM_CHAR_UUID_128, BLE_UUID_128_LEN);
            break;
            
        case BLE_SECURITY_ENCRYPTED:
            _config->useCustomUUIDS = true;
            _config->requireAuthentication = true;
            strncpy(_config->authKey, BLE_DEFAULT_AUTH_KEY, BLE_MAX_AUTH_KEY_LEN - 1);
            memcpy(_config->serviceUUID, BLE_DEFAULT_SERVICE_UUID_128, BLE_UUID_128_LEN);
            memcpy(_config->batteryCharUUID, BLE_DEFAULT_BATTERY_CHAR_UUID_128, BLE_UUID_128_LEN);
            memcpy(_config->customCharUUID, BLE_DEFAULT_CUSTOM_CHAR_UUID_128, BLE_UUID_128_LEN);
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
            BLE_LIBRARY_VERSION_MAJOR, BLE_LIBRARY_VERSION_MINOR, BLE_LIBRARY_VERSION_PATCH);
    ble_log(ESP_LOG_INFO, "ESP-IDF Version: %s", esp_get_idf_version());
    ble_log(ESP_LOG_INFO, "Compile Time: %s %s", __DATE__, __TIME__);
    ble_log(ESP_LOG_INFO, "Initialization State: %s", s_bleCommonInitialized ? "INITIALIZED" : "NOT INITIALIZED");
    ble_log(ESP_LOG_INFO, "===============================");
}

void ble_log(esp_log_level_t level, const char* format, ...) {
    if (format == nullptr) {
        return;
    }

    char logBuffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(logBuffer, sizeof(logBuffer), format, args);
    va_end(args);

    if (s_globalLogCallback != nullptr) {
        s_globalLogCallback(level, s_bleTag, logBuffer);
    } else {
        esp_log_write(level, s_bleTag, "%s", logBuffer);
    }
}

uint64_t ble_get_timestamp(void) {
    return esp_timer_get_time();
}

uint8_t ble_rssi_to_quality(int8_t rssi) {
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

    if (length != 2 && length != 4 && length != 16) {
        return false;
    }

    bool allZeros = true;
    for (size_t i = 0; i < length; i++) {
        if (uuid[i] != 0) {
            allZeros = false;
            break;
        }
    }

    return !allZeros;
}