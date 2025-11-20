/*******************************************************************************
 * @file ble_configs.hpp
 * @brief Contains common declarations for BLE library including security 
 * configuration, states, and shared functionality between client and server.
 *
 * @version 0.0.5
 * @date 2025-06-24
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/******************************************************************************/
/*              Build Flags Validation (from platformio.ini)                */
/******************************************************************************/

// Validate critical BLE build flags from platformio.ini
#ifndef CONFIG_BT_BLE_ENABLED
    #error "CONFIG_BT_BLE_ENABLED must be defined as 1 in build_flags. Add -DCONFIG_BT_BLE_ENABLED=1 to platformio.ini"
#elif CONFIG_BT_BLE_ENABLED != 1
    #error "CONFIG_BT_BLE_ENABLED must be set to 1. Current value is incorrect. Use -DCONFIG_BT_BLE_ENABLED=1 in platformio.ini"
#endif

#ifndef CONFIG_BT_BLUEDROID_ENABLED
    #error "CONFIG_BT_BLUEDROID_ENABLED must be defined as 1 in build_flags. Add -DCONFIG_BT_BLUEDROID_ENABLED=1 to platformio.ini"
#elif CONFIG_BT_BLUEDROID_ENABLED != 1
    #error "CONFIG_BT_BLUEDROID_ENABLED must be set to 1. Current value is incorrect. Use -DCONFIG_BT_BLUEDROID_ENABLED=1 in platformio.ini"
#endif

#ifndef CONFIG_BT_BLE_42_FEATURES_SUPPORTED
    #error "CONFIG_BT_BLE_42_FEATURES_SUPPORTED must be defined as 1 in build_flags. Add -DCONFIG_BT_BLE_42_FEATURES_SUPPORTED=1 to platformio.ini"
#elif CONFIG_BT_BLE_42_FEATURES_SUPPORTED != 1
    #error "CONFIG_BT_BLE_42_FEATURES_SUPPORTED must be set to 1. Current value is incorrect. Use -DCONFIG_BT_BLE_42_FEATURES_SUPPORTED=1 in platformio.ini"
#endif

#ifndef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    #error "CONFIG_BT_BLE_50_FEATURES_SUPPORTED must be defined as 0 in build_flags. Add -DCONFIG_BT_BLE_50_FEATURES_SUPPORTED=0 to platformio.ini"
#elif CONFIG_BT_BLE_50_FEATURES_SUPPORTED != 0
    #error "CONFIG_BT_BLE_50_FEATURES_SUPPORTED must be set to 0 for compatibility. Current value is incorrect. Use -DCONFIG_BT_BLE_50_FEATURES_SUPPORTED=0 in platformio.ini"
#endif

// Validate that IDF_MAINTAINER is set (helps with debugging)
#ifndef IDF_MAINTAINER
    #warning "IDF_MAINTAINER is not defined. Consider adding -DIDF_MAINTAINER=1 to build_flags in platformio.ini for better debugging support"
#endif

/******************************************************************************/
/*                    Additional Build Flags Warnings                        */
/******************************************************************************/

// Warning for missing optional but recommended flags
#ifndef CONFIG_BT_CONTROLLER_ENABLED
    #warning "CONFIG_BT_CONTROLLER_ENABLED is not defined. Consider adding -DCONFIG_BT_CONTROLLER_ENABLED=1 to build_flags"
#endif

#ifndef CONFIG_BT_GATTS_ENABLE  
    #warning "CONFIG_BT_GATTS_ENABLE is not defined. BLE Server functionality may not work. Consider adding -DCONFIG_BT_GATTS_ENABLE=1 to build_flags"
#endif

#ifndef CONFIG_BT_GATTC_ENABLE
    #warning "CONFIG_BT_GATTC_ENABLE is not defined. BLE Client functionality may not work. Consider adding -DCONFIG_BT_GATTC_ENABLE=1 to build_flags"  
#endif

#ifndef CONFIG_BT_BLE_SMP_ENABLE
    #warning "CONFIG_BT_BLE_SMP_ENABLE is not defined. BLE security features may not work. Consider adding -DCONFIG_BT_BLE_SMP_ENABLE=1 to build_flags"
#endif


#define BLE_MAX_DEVICE_NAME_LEN     32
#define BLE_MAX_AUTH_KEY_LEN        32
#define BLE_MAX_CUSTOM_DATA_LEN     100
#define BLE_UUID_128_LEN            16

#define BLE_MAX_MTU_SIZE            512
#define BLE_DEFAULT_MTU_SIZE        23

#define BLE_INVALID_HANDLE          0
#define BLE_DEFAULT_SCAN_TIMEOUT    10000   // 10 seconds
#define BLE_DEFAULT_RECONNECT_TIME  5000    // 5 seconds


/******************************************************************************/
/*                      Compile-time Configuration Validation                */
/******************************************************************************/

// Essential BLE configuration validation - These will cause compilation errors if missing
#ifndef CONFIG_BT_BLE_ENABLED
    static_assert(false, "CONFIG_BT_BLE_ENABLED must be defined as 1 in build_flags. Add -DCONFIG_BT_BLE_ENABLED=1 to platformio.ini");
#elif CONFIG_BT_BLE_ENABLED != 1
    static_assert(false, "CONFIG_BT_BLE_ENABLED must be set to 1. Current value is incorrect. Use -DCONFIG_BT_BLE_ENABLED=1 in platformio.ini");
#endif

#ifndef CONFIG_BT_BLUEDROID_ENABLED
    static_assert(false, "CONFIG_BT_BLUEDROID_ENABLED must be defined as 1 in build_flags. Add -DCONFIG_BT_BLUEDROID_ENABLED=1 to platformio.ini");
#elif CONFIG_BT_BLUEDROID_ENABLED != 1
    static_assert(false, "CONFIG_BT_BLUEDROID_ENABLED must be set to 1. Current value is incorrect. Use -DCONFIG_BT_BLUEDROID_ENABLED=1 in platformio.ini");
#endif

#ifndef CONFIG_BT_BLE_42_FEATURES_SUPPORTED
    static_assert(false, "CONFIG_BT_BLE_42_FEATURES_SUPPORTED must be defined as 1 in build_flags. Add -DCONFIG_BT_BLE_42_FEATURES_SUPPORTED=1 to platformio.ini");
#elif CONFIG_BT_BLE_42_FEATURES_SUPPORTED != 1
    static_assert(false, "CONFIG_BT_BLE_42_FEATURES_SUPPORTED must be set to 1. Current value is incorrect. Use -DCONFIG_BT_BLE_42_FEATURES_SUPPORTED=1 in platformio.ini");
#endif

#ifndef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    static_assert(false, "CONFIG_BT_BLE_50_FEATURES_SUPPORTED must be defined as 0 in build_flags. Add -DCONFIG_BT_BLE_50_FEATURES_SUPPORTED=0 to platformio.ini");
#elif CONFIG_BT_BLE_50_FEATURES_SUPPORTED != 0
    static_assert(false, "CONFIG_BT_BLE_50_FEATURES_SUPPORTED must be set to 0 for compatibility. Current value is incorrect. Use -DCONFIG_BT_BLE_50_FEATURES_SUPPORTED=0 in platformio.ini");
#endif

/**
 * @enum bleMode_t
 * @brief Operating modes for the BLE library
 */
typedef enum {
    BLE_MODE_CLIENT_ONLY,                           ///< Device only as client
    BLE_MODE_SERVER_ONLY,                           ///< Device only as server
    BLE_MODE_DUAL                                   ///< Device as both client 
                                                    ///< and server simultaneously
} bleMode_t;

/**
 * @enum bleState_t
 * @brief General states for the BLE library
 */
typedef enum {
    BLE_STATE_UNINITIALIZED,                        ///< Library not initialized
    BLE_STATE_INITIALIZED,                          ///< Library initialized but not started
    BLE_STATE_STARTING,                             ///< Library starting up
    BLE_STATE_READY,                                ///< Library ready for operations
    BLE_STATE_ERROR                                 ///< Library in error state
} bleState_t;

/**
 * @enum bleSecurityLevel_t
 * @brief Security levels for BLE connections
 */
typedef enum {
    BLE_SECURITY_NONE,                              ///< No security (name matching only)
    BLE_SECURITY_BASIC,                             ///< UUID verification + name matching
    BLE_SECURITY_AUTHENTICATED,                     ///< Basic + authentication key
    BLE_SECURITY_ENCRYPTED                          ///< Authenticated + BLE pairing/bonding
} bleSecurityLevel_t;

/**
 * @struct bleSecurityConfig_t
 * @brief Configuration structure for BLE security settings
 */
typedef struct {
    bleSecurityLevel_t level;                       ///< Security level to use
    bool useCustomUUIDS;                            ///< Use custom 128-bit UUIDs
    bool requireAuthentication;                     ///< Require authentication key
    char authKey[BLE_MAX_AUTH_KEY_LEN];             ///< Authentication key
    uint8_t serviceUUID[BLE_UUID_128_LEN];          ///< SenseAI service UUID (128-bit)
    uint8_t batteryCharUUID[BLE_UUID_128_LEN];      ///< ESP32 TX characteristic (app RX)
    uint8_t customCharUUID[BLE_UUID_128_LEN];       ///< ESP32 RX characteristic (app TX)
} bleSecurityConfig_t;

/**
 * @struct bleDeviceInfo_t
 * @brief Information about a discovered BLE device
 */
typedef struct {
    esp_bd_addr_t address;                          ///< Device MAC address
    char name[BLE_MAX_DEVICE_NAME_LEN];             ///< Device name
    int8_t rssi;                                    ///< Signal strength (dBm)
    bool authenticated;                             ///< Authentication status
    uint64_t lastSeen;                              ///< Last seen timestamp
} bleDeviceInfo_t;

/**
 * @struct bleDataPacket_t
 * @brief Data packet structure for BLE communication
 */
typedef struct {
    uint8_t batteryLevel;                           ///< Battery level (0-100%)
    char customData[BLE_MAX_CUSTOM_DATA_LEN];       ///< Custom data string
    uint64_t timeStamp;                             ///< Data timestamp
    bool valid;                                     ///< Data validity flag
} bleDataPacket_t;

/**
 * @brief General event callback function type
 * @param eventType Type of event that occurred
 * @param eventData Pointer to event-specific data
 */
typedef void (*bleEventCB_t)(int eventType, void *eventData);

/**
 * @brief Default UUIDs for basic security (16-bit fallback)
 * @note These are legacy UUIDs used only when 128-bit custom UUIDs are disabled
 * @note For SenseAI app integration, use the 128-bit UUIDs below
 */
#define BLE_DEFAULT_SERVICE_UUID_16     0x180F      ///< Battery Service (legacy)
#define BLE_DEFAULT_BATTERY_CHAR_UUID   0x2A19      ///< Battery Level (legacy) 
#define BLE_DEFAULT_CUSTOM_CHAR_UUID    0xFF01      ///< Custom Data (legacy)

/**
 * @brief SenseAI App BLE UUIDs (128-bit) for enhanced security and app integration
 * @note These UUIDs must match exactly with the mobile app configuration
 */
extern const uint8_t kBleDefaultServiceUUID128[BLE_UUID_128_LEN];        ///< Main service UUID
extern const uint8_t KBleDefaultBatteryCharUUID128[BLE_UUID_128_LEN];    ///< ESP32 TX (app RX) characteristic 
extern const uint8_t KBleDefaultCustomCharUUID128[BLE_UUID_128_LEN];     ///< ESP32 RX (app TX) characteristic

/**
 * @brief Default authentication key
 */
#define BLE_DEFAULT_AUTH_KEY "15453n53"

/**
 * @brief Initializes the common BLE subsystem
 * 
 * Initializes NVS, Bluetooth controller, and Bluedroid stack.
 * Must be called before any other BLE operations.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t bleCommonInit(void);

/**
 * @brief Deinitializes the common BLE subsystem
 * 
 * Cleans up all BLE resources and deinitializes the Bluetooth stack.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t bleCommonDeinit(void);

/**
 * @brief Checks if BLE common subsystem is initialized
 * 
 * @return bool True if initialized, false otherwise
 */
bool bleCommonIsInitialized(void);

/**
 * @brief Sets the general event callback function
 * 
 * @param callback Function to call for general BLE events
 */
void bleSetEventCallback(bleEventCB_t callback);

/**
 * @brief Gets the current BLE library state
 * 
 * @return bleState_t Current state
 */
bleState_t bleGetState(void);

/**
 * @brief Converts a MAC address to string format
 * 
 * @param MACadd MAC address bytes
 * @param str Output string buffer (minimum 18 characters)
 */
void bleMacaddToString(esp_bd_addr_t MACadd, char *str);

/**
 * @brief Compares two 128-bit UUIDs
 * 
 * @param uuid1 First UUID
 * @param uuid2 Second UUID
 * @return bool True if UUIDs match, false otherwise
 */
bool bleCompareUUID128(const uint8_t* uuid1, const uint8_t* uuid2);

/**
 * @brief Generates a random authentication key
 * 
 * @param keyBuffer Buffer to store the generated key
 * @param keyLength Length of the key to generate
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t bleGenerateAuthKey(char* keyBuffer, size_t keyLength);

/**
 * @brief Validates BLE device name
 * 
 * @param name Device name to validate
 * @return bool True if name is valid, false otherwise
 */
bool bleValidateDeviceName(const char* name);

/**
 * @brief Creates a default security configuration
 * 
 * @param config Pointer to security configuration structure to fill
 * @param level Security level to configure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t bleCreateDefaultSecurityConfig(bleSecurityConfig_t* config, 
                                            bleSecurityLevel_t level);

/**
 * @brief Gets current timestamp in microseconds
 * 
 * @return uint64_t Current timestamp
 */
uint64_t bleGetTimestamp(void);

// /**
//  * @brief Calculates RSSI quality percentage
//  * 
//  * @param rssi RSSI value in dBm
//  * @return uint8_t Quality percentage (0-100%)
//  */
// uint8_t bleRSSIToQuality(int8_t rssi);

/**
 * @brief Validates UUID format
 * 
 * @param uuid UUID bytes to validate
 * @param length UUID length (2, 4, or 16 bytes)
 * @return bool True if UUID is valid, false otherwise
 */
bool bleValidateUUID(const uint8_t* uuid, size_t length);