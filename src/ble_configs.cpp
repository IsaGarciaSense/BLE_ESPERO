/*******************************************************************************
 * @file ble_configs.cpp
 * @brief Implementation of common BLE functionality including initialization,
 * utilities, and shared resources.
 *
 * @version 0.0.5
 * @date 2025-06-24
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "ble_configs.hpp"

#include <stdarg.h>

#include "esp_random.h"

static const char* TAG = "BLE_COMMON";
static bleState_t ble_global_state = BLE_STATE_UNINITIALIZED;
static bleEventCB_t global_event_callback = nullptr;

static bool ble_common_initialized = false;

/**
 * @brief SenseAI App Service UUID (128-bit)
 * @note Main service identifier for SenseAI mobile app communication
 */
const uint8_t kBleDefaultServiceUUID128[BLE_UUID_128_LEN] = {
    0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
    0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f
};

/**
 * @brief SenseAI App RX Characteristic UUID (128-bit)
 * @note Characteristic where ESP32 SENDS data TO mobile app (app reads from here)
 * @note Corresponds to app's RX UUID: 6e400003-b5a3-f393-e0a9-e50e24dcca9e
 */
const uint8_t KBleDefaultBatteryCharUUID128[BLE_UUID_128_LEN] = {
    0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
    0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe
};

/**
 * @brief SenseAI App TX Characteristic UUID (128-bit) 
 * @note Characteristic where ESP32 RECEIVES data FROM mobile app (app writes to here)
 * @note Corresponds to app's TX UUID: beb5483e-36e1-4688-b7f5-ea07361b26a8
 */
const uint8_t KBleDefaultCustomCharUUID128[BLE_UUID_128_LEN] = {
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e
};

esp_err_t bleCommonInit(void) {
    esp_err_t ret;

    if (ble_common_initialized) {
        ESP_LOGI(TAG, "BLE common subsystem already initialized, skipping...");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing BLE common subsystem");

    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to release Classic BT memory: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "Classic BT memory already released");
    } else {
        ESP_LOGD(TAG, "Classic BT memory released successfully");
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize BT controller: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "BT controller already initialized");
    } else {
        ESP_LOGD(TAG, "BT controller initialized successfully");
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to enable BT controller: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "BT controller already enabled");
    } else {
        ESP_LOGD(TAG, "BT controller enabled successfully");
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "Bluedroid already initialized");
    } else {
        ESP_LOGD(TAG, "Bluedroid initialized successfully");
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "Bluedroid already enabled");
    } else {
        ESP_LOGD(TAG, "Bluedroid enabled successfully");
    }

    ble_common_initialized = true;
    ble_global_state = BLE_STATE_INITIALIZED;
    ESP_LOGI(TAG, "BLE common subsystem initialized successfully");

    return ESP_OK;
}

esp_err_t bleCommonDeinit(void) {
    if (!ble_common_initialized) {
        ESP_LOGD(TAG, "BLE common subsystem not initialized, skipping deinit");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing BLE common subsystem");

    esp_err_t ret = ESP_OK;
    esp_err_t final_ret = ESP_OK;

    ret = esp_bluedroid_disable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Failed to disable Bluedroid: %s", esp_err_to_name(ret));
        final_ret = ESP_FAIL;
    }

    ret = esp_bluedroid_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Failed to deinitialize Bluedroid: %s", esp_err_to_name(ret));
        final_ret = ESP_FAIL;
    }

    ret = esp_bt_controller_disable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Failed to disable BT controller: %s", esp_err_to_name(ret));
        final_ret = ESP_FAIL;
    }

    ret = esp_bt_controller_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Failed to deinitialize BT controller: %s", esp_err_to_name(ret));
        final_ret = ESP_FAIL;
    }

    ble_common_initialized = false;
    ble_global_state = BLE_STATE_UNINITIALIZED;
    ESP_LOGI(TAG, "BLE common subsystem deinitialized");

    return final_ret;
}

bool bleCommonIsInitialized(void) {
    return ble_common_initialized;
}

void bleSetEventCallback(bleEventCB_t callback) {
    global_event_callback = callback;
    ESP_LOGD(TAG, "Global event callback set");
}

bleState_t bleGetState(void) {
    return ble_global_state;
}

void bleMacaddToString(esp_bd_addr_t bda, char *str) {
    if (str == nullptr) {
        return;
    }
    
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

bool bleCompareUUID128(const uint8_t* uuid1, const uint8_t* uuid2) {
    if (uuid1 == nullptr || uuid2 == nullptr) {
        return false;
    }
    
    return memcmp(uuid1, uuid2, BLE_UUID_128_LEN) == 0;
}

esp_err_t bleGenerateAuthKey(char* key_buffer, size_t key_length) {
    if (key_buffer == nullptr || key_length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Simple random key generation
    const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    
    for (size_t i = 0; i < key_length - 1; i++) {
        uint32_t random_val = esp_random();
        key_buffer[i] = charset[random_val % (sizeof(charset) - 1)];
    }
    key_buffer[key_length - 1] = '\0';

    ESP_LOGD(TAG, "Generated authentication key of length %d", key_length - 1);
    return ESP_OK;
}

bool bleValidateDeviceName(const char* name) {
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

esp_err_t bleCreateDefaultSecurityConfig(bleSecurityConfig_t* config, 
                                            bleSecurityLevel_t level) {
    if (config == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(config, 0, sizeof(bleSecurityConfig_t));
    
    config->level = level;
    
    switch (level) {
        case BLE_SECURITY_NONE:
            config->useCustomUUIDS = false;
            config->requireAuthentication = false;
            break;
            
        case BLE_SECURITY_BASIC:
            config->useCustomUUIDS = true;
            config->requireAuthentication = false;
            memcpy(config->serviceUUID, kBleDefaultServiceUUID128, BLE_UUID_128_LEN);
            memcpy(config->batteryCharUUID, KBleDefaultBatteryCharUUID128, BLE_UUID_128_LEN);
            memcpy(config->customCharUUID, KBleDefaultCustomCharUUID128, BLE_UUID_128_LEN);
            break;
            
        case BLE_SECURITY_AUTHENTICATED:
            config->useCustomUUIDS = true;
            config->requireAuthentication = true;
            strncpy(config->authKey, BLE_DEFAULT_AUTH_KEY, BLE_MAX_AUTH_KEY_LEN - 1);
            memcpy(config->serviceUUID, kBleDefaultServiceUUID128, BLE_UUID_128_LEN);
            memcpy(config->batteryCharUUID, KBleDefaultBatteryCharUUID128, BLE_UUID_128_LEN);
            memcpy(config->customCharUUID, KBleDefaultCustomCharUUID128, BLE_UUID_128_LEN);
            break;
            
        case BLE_SECURITY_ENCRYPTED:
            // Highest security level with encryption
            config->useCustomUUIDS = true;
            config->requireAuthentication = true;
            strncpy(config->authKey, BLE_DEFAULT_AUTH_KEY, BLE_MAX_AUTH_KEY_LEN - 1);
            memcpy(config->serviceUUID, kBleDefaultServiceUUID128, BLE_UUID_128_LEN);
            memcpy(config->batteryCharUUID, KBleDefaultBatteryCharUUID128, BLE_UUID_128_LEN);
            memcpy(config->customCharUUID, KBleDefaultCustomCharUUID128, BLE_UUID_128_LEN);
            break;
            
        default:
            return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Created default security config for level %d", level);
    return ESP_OK;
}

uint64_t bleGetTimestamp(void) {
    return esp_timer_get_time();
}

// uint8_t bleRSSIToQuality(int8_t rssi) {
//     // RSSI range typically: -100 dBm (worst) to -30 dBm (best)
//     if (rssi >= -30) {
//         return 100;  // Excellent signal
//     } else if (rssi >= -50) {
//         return 80 + ((rssi + 50) * 20) / 20;  // Very good: 80-100%
//     } else if (rssi >= -70) {
//         return 50 + ((rssi + 70) * 30) / 20;  // Good: 50-80%
//     } else if (rssi >= -90) {
//         return 20 + ((rssi + 90) * 30) / 20;  // Fair: 20-50%
//     } else if (rssi >= -100) {
//         return ((rssi + 100) * 20) / 10;      // Poor: 0-20%
//     } else {
//         return 0;    // No signal
//     }
// }

bool bleValidateUUID(const uint8_t* uuid, size_t length) {
    if (uuid == nullptr) {
        return false;
    }

    if (length != 2 && length != 4 && length != 16) {
        return false;
    }

    bool all_zeros = true;
    for (size_t i = 0; i < length; i++) {
        if (uuid[i] != 0) {
            all_zeros = false;
            break;
        }
    }

    return !all_zeros;
}
