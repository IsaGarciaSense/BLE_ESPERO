/*******************************************************************************
 * @file BLEConfigs.hpp
 * @brief Contains common declarations for BLE library including security 
 * configuration, states, and shared functionality between client and server.
 *
 * @version 0.0.6
 * @date 2025-06-26
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

#define BLE_MAX_DEVICE_NAME_LEN     32
#define BLE_MAX_AUTH_KEY_LEN        32
#define BLE_MAX_CUSTOM_DATA_LEN     64
#define BLE_UUID_128_LEN            16

#define BLE_INVALID_HANDLE          0
#define BLE_DEFAULT_SCAN_TIMEOUT    10000   // 10 seconds
#define BLE_DEFAULT_RECONNECT_TIME  5000    // 5 seconds

/**
 * @enum ble_mode_t
 * @brief Operating modes for the BLE library
 */
typedef enum {
    BLE_MODE_CLIENT_ONLY,       ///< Device acts only as BLE client
    BLE_MODE_SERVER_ONLY,       ///< Device acts only as BLE server
    BLE_MODE_DUAL               ///< Device acts as both client and server simultaneously
} ble_mode_t;

/**
 * @enum ble_state_t
 * @brief General states for the BLE library
 */
typedef enum {
    BLE_STATE_UNINITIALIZED,    ///< Library not initialized
    BLE_STATE_INITIALIZED,      ///< Library initialized but not started
    BLE_STATE_STARTING,         ///< Library starting up
    BLE_STATE_READY,            ///< Library ready for operations
    BLE_STATE_ERROR             ///< Library in error state
} ble_state_t;

/**
 * @enum ble_security_level_t
 * @brief Security levels for BLE connections
 */
typedef enum {
    BLE_SECURITY_NONE,           ///< No security (name matching only)
    BLE_SECURITY_BASIC,          ///< UUID verification + name matching
    BLE_SECURITY_AUTHENTICATED,  ///< Basic + authentication key
    BLE_SECURITY_ENCRYPTED       ///< Authenticated + BLE pairing/bonding
} ble_security_level_t;

/**
 * @struct ble_security_config_t
 * @brief Configuration structure for BLE security settings
 */
typedef struct {
    ble_security_level_t level;                        ///< Security level to use
    bool useCustomUUIDS;                               ///< Use custom 128-bit UUIDs
    bool requireAuthentication;                        ///< Require authentication key
    char authKey[BLE_MAX_AUTH_KEY_LEN];                ///< Authentication key
    uint8_t serviceUUID[BLE_UUID_128_LEN];             ///< Custom service UUID (128-bit)
    uint8_t batteryCharUUID[BLE_UUID_128_LEN];         ///< Custom battery characteristic UUID
    uint8_t customCharUUID[BLE_UUID_128_LEN];          ///< Custom data characteristic UUID
} ble_security_config_t;

/**
 * @struct ble_device_info_t
 * @brief Information about a discovered BLE device
 */
typedef struct {
    esp_bd_addr_t address;                             ///< Device MAC address
    char name[BLE_MAX_DEVICE_NAME_LEN];                ///< Device name
    int8_t rssi;                                       ///< Signal strength (dBm)
    bool authenticated;                                ///< Authentication status
    uint64_t lastSeen;                                 ///< Last seen timestamp
} ble_device_info_t;

/**
 * @struct ble_data_packet_t
 * @brief Data packet structure for BLE communication
 */
typedef struct {
    uint8_t batteryLevel;                              ///< Battery level (0-100%)
    char customData[BLE_MAX_CUSTOM_DATA_LEN];          ///< Custom data string
    uint64_t timeStamp;                                ///< Data timestamp
    bool valid;                                        ///< Data validity flag
} ble_data_packet_t;

/**
 * @brief General event callback function type
 * @param eventType Type of event that occurred
 * @param eventData Pointer to event-specific data
 */
typedef void (*ble_event_callback_t)(int eventType, void *eventData);

/**
 * @brief Log callback function type for custom logging
 * @param level Log level (ESP_LOG_ERROR, ESP_LOG_WARN, etc.)
 * @param tag Log tag
 * @param message Log message
 */
typedef void (*ble_log_callback_t)(esp_log_level_t level, const char* tag, const char* message);

/**
 * @brief Default UUIDs for basic security (16-bit)
 */
#define BLE_DEFAULT_SERVICE_UUID_16     0x180F  ///< Battery Service
#define BLE_DEFAULT_BATTERY_CHAR_UUID   0x2A19  ///< Battery Level
#define BLE_DEFAULT_CUSTOM_CHAR_UUID    0xFF01  ///< Custom Data

/**
 * @brief Default custom UUIDs for enhanced security (128-bit)
 */
extern const uint8_t BLE_DEFAULT_SERVICE_UUID_128[BLE_UUID_128_LEN];
extern const uint8_t BLE_DEFAULT_BATTERY_CHAR_UUID_128[BLE_UUID_128_LEN];
extern const uint8_t BLE_DEFAULT_CUSTOM_CHAR_UUID_128[BLE_UUID_128_LEN];

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
esp_err_t ble_common_init(void);

/**
 * @brief Deinitializes the common BLE subsystem
 * 
 * Cleans up all BLE resources and deinitializes the Bluetooth stack.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ble_common_deinit(void);

/**
 * @brief Checks if BLE common subsystem is initialized
 * 
 * @return bool True if initialized, false otherwise
 */
bool ble_common_is_initialized(void);

/**
 * @brief Sets the general event callback function
 * 
 * @param callback Function to call for general BLE events
 */
void ble_set_event_callback(ble_event_callback_t callback);

/**
 * @brief Sets the log callback function for custom logging
 * 
 * @param callback Function to call for log messages
 */
void ble_set_log_callback(ble_log_callback_t callback);

/**
 * @brief Gets the current BLE library state
 * 
 * @return ble_state_t Current state
 */
ble_state_t ble_get_state(void);

/**
 * @brief Converts a MAC address to string format
 * 
 * @param macAddress MAC address bytes
 * @param _str Output string buffer (minimum 18 characters)
 */
void ble_addr_to_string(esp_bd_addr_t macAddress, char *_str);

/**
 * @brief Compares two 128-bit UUIDs
 * 
 * @param uuid1 First UUID
 * @param uuid2 Second UUID
 * @return bool True if UUIDs match, false otherwise
 */
bool ble_compare_uuid128(const uint8_t* uuid1, const uint8_t* uuid2);

/**
 * @brief Generates a random authentication key
 * 
 * @param _keyBuffer Buffer to store the generated key
 * @param keyLength Length of the key to generate
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ble_generate_auth_key(char* _keyBuffer, size_t keyLength);

/**
 * @brief Validates BLE device name
 * 
 * @param name Device name to validate
 * @return bool True if name is valid, false otherwise
 */
bool ble_validate_device_name(const char* name);

/**
 * @brief Creates a default security configuration
 * 
 * @param _config Pointer to security configuration structure to fill
 * @param level Security level to configure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ble_create_default_security_config(ble_security_config_t* _config, 
                                              ble_security_level_t level);

/**
 * @brief Prints BLE library version and build information
 */
void ble_print_version_info(void);

/**
 * @brief Custom logging function with BLE tag
 * 
 * @param level Log level
 * @param format Printf-style format string
 * @param ... Format arguments
 */
void ble_log(esp_log_level_t level, const char* format, ...);

/**
 * @brief Gets current timestamp in microseconds
 * 
 * @return uint64_t Current timestamp
 */
uint64_t ble_get_timestamp(void);

/**
 * @brief Calculates RSSI quality percentage
 * 
 * @param rssi RSSI value in dBm
 * @return uint8_t Quality percentage (0-100%)
 */
uint8_t ble_rssi_to_quality(int8_t rssi);

/**
 * @brief Validates UUID format
 * 
 * @param uuid UUID bytes to validate
 * @param length UUID length (2, 4, or 16 bytes)
 * @return bool True if UUID is valid, false otherwise
 */
bool ble_validate_uuid(const uint8_t* uuid, size_t length);