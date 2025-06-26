/*******************************************************************************
 * @file BLEServer.cpp
 * @brief Implementation of BLE server functionality including advertising,
 * client management, service provisioning, and JSON command processing.
 *
 * @version 0.0.5
 * @date 2025-06-24
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "BLEServer.hpp"

#include <cJSON.h>

/******************************************************************************/
/*                               Static Variables                             */
/******************************************************************************/

static const char* s_bleServerTag = "BLE_SERVER";
BLEServer* BLEServer::s_instance = nullptr;

// Advertising data structure
static esp_ble_adv_data_t s_advData = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = nullptr,
    .service_data_len = 0,
    .p_service_data = nullptr,
    .service_uuid_len = 16,
    .p_service_uuid = nullptr,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
};

// Advertising parameters
static esp_ble_adv_params_t s_advParams = {
    .adv_int_min = 0x60, // 60 ms
    .adv_int_max = 0x80, // 80 ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = {0},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/******************************************************************************/
/*                              Default Configuration                         */
/******************************************************************************/

static const ble_server_config_t s_defaultServerConfig = {
    .deviceName = "154-BLE-Server",
    .advertisingInterval = 1000,
    .autoStartAdvertising = true,
    .dataUpdateInterval = 5000,
    .maxClients = 4,
    .clientTimeout = 30000,
    .requireAuthentication = false,
    .enableNotifications = true,
    .enableJsonCommands = true,
    .maxJsonSize = 512
};

/******************************************************************************/
/*                                Constructor/Destructor                      */
/******************************************************************************/

BLEServer::BLEServer() : BLEServer(s_defaultServerConfig) {
}

BLEServer::BLEServer(const ble_server_config_t& config) :
    config_(config),
    state_(BLE_SERVER_IDLE),
    gattsIf_(ESP_GATT_IF_NONE),
    serviceHandle_(BLE_INVALID_HANDLE),
    batteryCharHandle_(BLE_INVALID_HANDLE),
    customCharHandle_(BLE_INVALID_HANDLE),
    batteryDescrHandle_(BLE_INVALID_HANDLE),
    customDescrHandle_(BLE_INVALID_HANDLE),
    maxClients_(config.maxClients),
    clientConnectedCB_(nullptr),
    clientDisconnectedCB_(nullptr),
    dataWrittenCB_(nullptr),
    dataReadCB_(nullptr),
    clientAuthCB_(nullptr),
    advertisingCB_(nullptr),
    jsonCommandCB_(nullptr),
    dataUpdateTaskHandle_(nullptr),
    clientTimeoutTaskHandle_(nullptr),
    clientsMutex_(nullptr),
    dataMutex_(nullptr),
    batteryLevel_(100)
{
    // Initialize static instance
    s_instance = this;
    
    // Initialize client sessions
    memset(clientSessions_, 0, sizeof(clientSessions_));
    
    // Initialize status
    memset(&status_, 0, sizeof(status_));
    status_.state = state_;
    status_.batteryLevel = batteryLevel_;
    status_.jsonProcessingEnabled = config_.enableJsonCommands;
    strcpy(status_.customData, "Hello from ESP32!");
    
    // Initialize statistics
    memset(&stats_, 0, sizeof(stats_));
    stats_.startTime = ble_get_timestamp();
    
    // Initialize custom data
    strcpy(customData_, status_.customData);
    
    // Create default security configuration
    ble_create_default_security_config(&securityConfig_, BLE_SECURITY_BASIC);

    ble_log(ESP_LOG_INFO, "BLEServer created with device name: %s", 
            config_.deviceName);
    ble_log(ESP_LOG_INFO, "JSON commands: %s", 
            config_.enableJsonCommands ? "ENABLED" : "DISABLED");
}

BLEServer::~BLEServer() {
    // Stop all tasks
    if (dataUpdateTaskHandle_) {
        vTaskDelete(dataUpdateTaskHandle_);
    }
    if (clientTimeoutTaskHandle_) {
        vTaskDelete(clientTimeoutTaskHandle_);
    }
    
    // Delete mutexes
    if (clientsMutex_) {
        vSemaphoreDelete(clientsMutex_);
    }
    if (dataMutex_) {
        vSemaphoreDelete(dataMutex_);
    }
    
    // Disconnect all clients
    disconnectAllClients();
    
    // Stop advertising
    stopAdvertising();
    
    ble_log(ESP_LOG_INFO, "BLEServer destroyed");
    s_instance = nullptr;
}

/******************************************************************************/
/*                              Initialization                                */
/******************************************************************************/

esp_err_t BLEServer::init() {
    ble_log(ESP_LOG_INFO, "Initializing BLE server...");
    
    // Create mutexes
    clientsMutex_ = xSemaphoreCreateMutex();
    dataMutex_ = xSemaphoreCreateMutex();
    
    if (!clientsMutex_ || !dataMutex_) {
        ble_log(ESP_LOG_ERROR, "Failed to create mutexes");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize common BLE
    esp_err_t ret = ble_common_init();
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to initialize BLE common: %s", 
                esp_err_to_name(ret));
        return ret;
    }
    
    // Register GAP callback
    ret = esp_ble_gap_register_callback(gapCallback);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to register GAP callback: %s", 
                esp_err_to_name(ret));
        return ret;
    }
    
    // Register GATT server callback
    ret = esp_ble_gatts_register_callback(gattsCallback);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to register GATT server callback: %s", 
                esp_err_to_name(ret));
        return ret;
    }
    
    // Register application
    ret = esp_ble_gatts_app_register(0);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to register GATT server app: %s", 
                esp_err_to_name(ret));
        return ret;
    }
    
    // Set device name
    ret = esp_ble_gap_set_device_name(config_.deviceName);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to set device name: %s", 
                esp_err_to_name(ret));
        return ret;
    }
    
    // Configure advertising data
    if (securityConfig_.useCustomUUIDS) {
        s_advData.p_service_uuid = (uint8_t*)securityConfig_.serviceUUID;
    } else {
        // Use 16-bit UUID for basic security
        static uint8_t serviceUuid16[2] = {
            (uint8_t)(BLE_DEFAULT_SERVICE_UUID_16 & 0xFF),
            (uint8_t)(BLE_DEFAULT_SERVICE_UUID_16 >> 8)
        };
        s_advData.p_service_uuid = serviceUuid16;
        s_advData.service_uuid_len = 2;
    }
    
    ret = esp_ble_gap_config_adv_data(&s_advData);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to configure advertising data: %s", 
                esp_err_to_name(ret));
        return ret;
    }
    
    // Update advertising interval
    setAdvertisingInterval(config_.advertisingInterval);

    // Start tasks only if intervals are set
    if (config_.dataUpdateInterval > 0) {
        xTaskCreate(dataUpdateTask, "ble_server_data", 4096, this, 5, 
                    &dataUpdateTaskHandle_);
    }
    
    if (config_.clientTimeout > 0) {
        xTaskCreate(clientTimeoutTask, "ble_server_timeout", 4096, this, 3, 
                    &clientTimeoutTaskHandle_);
    }
    
    state_ = BLE_SERVER_READY;
    status_.state = state_;
    
    ble_log(ESP_LOG_INFO, "BLE server initialized successfully");
    
    // Auto-start advertising if configured
    if (config_.autoStartAdvertising) {
        startAdvertising();
    }
    
    return ESP_OK;
}

/******************************************************************************/
/*                              Advertising Control                           */
/******************************************************************************/

esp_err_t BLEServer::startAdvertising() {
    if (state_ != BLE_SERVER_READY && state_ != BLE_SERVER_CONNECTED) {
        ble_log(ESP_LOG_WARN, "Cannot start advertising - server not ready");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = esp_ble_gap_start_advertising(&s_advParams);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to start advertising: %s", 
                esp_err_to_name(ret));
        return ret;
    }
    
    state_ = BLE_SERVER_ADVERTISING;
    status_.state = state_;
    status_.advertisingActive = true;
    stats_.advertisingCycles++;
    
    ble_log(ESP_LOG_INFO, "Started advertising");
    
    if (advertisingCB_) {
        advertisingCB_(true, ESP_OK);
    }
    
    return ESP_OK;
}

esp_err_t BLEServer::stopAdvertising() {
    if (!status_.advertisingActive) {
        return ESP_OK;  // Already stopped
    }
    
    esp_err_t ret = esp_ble_gap_stop_advertising();
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to stop advertising: %s", 
                esp_err_to_name(ret));
        return ret;
    }
    
    status_.advertisingActive = false;
    
    // Update state based on connections
    if (hasConnectedClients()) {
        state_ = BLE_SERVER_CONNECTED;
    } else {
        state_ = BLE_SERVER_READY;
    }
    status_.state = state_;
    
    ble_log(ESP_LOG_INFO, "Stopped advertising");
    
    if (advertisingCB_) {
        advertisingCB_(false, ESP_OK);
    }
    
    return ESP_OK;
}

/******************************************************************************/
/*                              Data Management                               */
/******************************************************************************/

esp_err_t BLEServer::setBatteryLevel(uint8_t level) {
    if (level > 100) {
        ble_log(ESP_LOG_WARN, "Battery level clamped to 100%%");
        level = 100;
    }
    
    if (xSemaphoreTake(dataMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        batteryLevel_ = level;
        status_.batteryLevel = level;
        status_.lastUpdateTime = ble_get_timestamp();
        xSemaphoreGive(dataMutex_);
        
        ble_log(ESP_LOG_DEBUG, "Battery level updated to %d%%", level);
        
        // Notify connected clients if notifications are enabled
        if (config_.enableNotifications) {
            notifyAllClients();
        }
        
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

/******************************************************************************/
/*                              JSON Response Methods                         */
/******************************************************************************/

esp_err_t BLEServer::sendJsonResponse(uint16_t connId, const char* _jsonResponse) {
    if (!_jsonResponse) {
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t jsonLen = strlen(_jsonResponse);
    const size_t kChunkSize = 200; // Conservative size for BLE
    
    ble_log(ESP_LOG_INFO, "📤 Sending JSON response to client %d (%zu bytes)", 
            connId, jsonLen);
    ble_log(ESP_LOG_DEBUG, "JSON: %s", _jsonResponse);
    
    if (jsonLen <= kChunkSize) {
        // Send in one piece if small enough
        esp_err_t ret = esp_ble_gatts_send_indicate(gattsIf_, connId, 
                                                   customCharHandle_,
                                                   jsonLen, 
                                                   (uint8_t*)_jsonResponse, 
                                                   false);
        if (ret == ESP_OK) {
            stats_.dataPacketsSent++;
        }
        return ret;
    } else {
        // Send in chunks
        esp_err_t result = ESP_OK;
        for (size_t i = 0; i < jsonLen; i += kChunkSize) {
            size_t copyLen = (jsonLen - i < kChunkSize) ? jsonLen - i : kChunkSize;
            
            esp_err_t ret = esp_ble_gatts_send_indicate(gattsIf_, connId, 
                                                       customCharHandle_,
                                                       copyLen, 
                                                       (uint8_t*)(_jsonResponse + i), 
                                                       false);
            if (ret == ESP_OK) {
                stats_.dataPacketsSent++;
                vTaskDelay(pdMS_TO_TICKS(50)); // Small delay between chunks
            } else {
                result = ret;
                break;
            }
        }
        return result;
    }
}

esp_err_t BLEServer::sendJsonResponseToAll(const char* _jsonResponse) {
    if (!_jsonResponse) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t result = ESP_OK;
    
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID != 0) {
                esp_err_t ret = sendJsonResponse(clientSessions_[i].connID, 
                                               _jsonResponse);
                if (ret != ESP_OK) {
                    result = ret;
                }
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    return result;
}

esp_err_t BLEServer::notifyAllClients() {
    esp_err_t result = ESP_OK;
    
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID != 0 && 
                clientSessions_[i].notificationsEnabled) {
                esp_err_t ret = notifyClient(clientSessions_[i].connID, "both");
                if (ret != ESP_OK) {
                    result = ret;
                }
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    return result;
}

esp_err_t BLEServer::notifyClient(uint16_t connId, 
                                  const char* characteristicType) {
    if (!characteristicType) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t result = ESP_OK;
    
    // Notify battery characteristic
    if (strcmp(characteristicType, "battery") == 0 || 
        strcmp(characteristicType, "both") == 0) {
        if (batteryCharHandle_ != BLE_INVALID_HANDLE) {
            esp_err_t ret = esp_ble_gatts_send_indicate(gattsIf_, connId, 
                                                       batteryCharHandle_,
                                                       sizeof(batteryLevel_), 
                                                       &batteryLevel_, false);
            if (ret == ESP_OK) {
                stats_.dataPacketsSent++;
                
                // Update client session stats
                if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
                    for (int i = 0; i < maxClients_; i++) {
                        if (clientSessions_[i].connID == connId) {
                            clientSessions_[i].dataPacketsSent++;
                            clientSessions_[i].lastActivity = ble_get_timestamp();
                            break;
                        }
                    }
                    xSemaphoreGive(clientsMutex_);
                }
            } else {
                result = ret;
            }
        }
    }
    
    // Notify custom characteristic
    if (strcmp(characteristicType, "custom") == 0 || 
        strcmp(characteristicType, "both") == 0) {
        if (customCharHandle_ != BLE_INVALID_HANDLE) {
            esp_err_t ret = esp_ble_gatts_send_indicate(gattsIf_, connId, 
                                                       customCharHandle_,
                                                       strlen(customData_), 
                                                       (uint8_t*)customData_, 
                                                       false);
            if (ret == ESP_OK) {
                stats_.dataPacketsSent++;
            } else {
                result = ret;
            }
        }
    }
    
    return result;
}

/******************************************************************************/
/*                              Client Management                             */
/******************************************************************************/

esp_err_t BLEServer::disconnectClient(uint16_t connId) {
    esp_err_t ret = esp_ble_gatts_close(gattsIf_, connId);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to disconnect client %d: %s", 
                connId, esp_err_to_name(ret));
        return ret;
    }
    
    ble_log(ESP_LOG_INFO, "Disconnecting client %d", connId);
    return ESP_OK;
}

esp_err_t BLEServer::disconnectAllClients() {
    esp_err_t result = ESP_OK;
    
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID != 0) {
                esp_err_t ret = disconnectClient(clientSessions_[i].connID);
                if (ret != ESP_OK) {
                    result = ret;
                }
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    return result;
}

esp_err_t BLEServer::addClientSession(uint16_t connId, 
                                     const esp_bd_addr_t _clientAddr) {
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Find empty slot
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == 0) {
                clientSessions_[i].connID = connId;
                memcpy(clientSessions_[i].address, _clientAddr, 
                       sizeof(esp_bd_addr_t));
                clientSessions_[i].authenticated = !config_.requireAuthentication;
                clientSessions_[i].connectTime = ble_get_timestamp();
                clientSessions_[i].lastActivity = clientSessions_[i].connectTime;
                clientSessions_[i].dataPacketsSent = 0;
                clientSessions_[i].dataPacketsReceived = 0;
                clientSessions_[i].notificationsEnabled = config_.enableNotifications;
                clientSessions_[i].jsonBufferPos = 0;
                memset(clientSessions_[i].jsonBuffer, 0, 
                       sizeof(clientSessions_[i].jsonBuffer));
                
                status_.connectedClients++;
                stats_.totalConnections++;
                stats_.currentClients++;
                
                xSemaphoreGive(clientsMutex_);
                
                ble_log(ESP_LOG_INFO, "Added client session %d (total clients: %d)", 
                        connId, status_.connectedClients);
                return ESP_OK;
            }
        }
        xSemaphoreGive(clientsMutex_);
        
        ble_log(ESP_LOG_WARN, "No available slots for new client %d", connId);
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t BLEServer::removeClientSession(uint16_t connId) {
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connId) {
                memset(&clientSessions_[i], 0, sizeof(ble_client_session_t));
                status_.connectedClients--;
                stats_.currentClients--;
                
                xSemaphoreGive(clientsMutex_);

                ble_log(ESP_LOG_INFO, 
                        "Removed client session %d (remaining clients: %d)", 
                        connId, status_.connectedClients);

                // Update server state
                if (status_.connectedClients == 0) {
                    if (status_.advertisingActive) {
                        state_ = BLE_SERVER_ADVERTISING;
                    } else {
                        state_ = BLE_SERVER_READY;
                    }
                    status_.state = state_;
                }
                
                return ESP_OK;
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    ble_log(ESP_LOG_WARN, "Client session %d not found", connId);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t BLEServer::setCustomData(const char* data) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(data) >= BLE_MAX_CUSTOM_DATA_LEN) {
        ble_log(ESP_LOG_WARN, "Custom data truncated to fit buffer");
    }
    
    if (xSemaphoreTake(dataMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        strncpy(customData_, data, BLE_MAX_CUSTOM_DATA_LEN - 1);
        customData_[BLE_MAX_CUSTOM_DATA_LEN - 1] = '\0';
        
        strncpy(status_.customData, customData_, BLE_MAX_CUSTOM_DATA_LEN - 1);
        status_.customData[BLE_MAX_CUSTOM_DATA_LEN - 1] = '\0';
        status_.lastUpdateTime = ble_get_timestamp();
        
        xSemaphoreGive(dataMutex_);
        
        ble_log(ESP_LOG_DEBUG, "Custom data updated: %s", customData_);
        
        // Notify connected clients if notifications are enabled
        if (config_.enableNotifications) {
            notifyAllClients();
        }
        
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}