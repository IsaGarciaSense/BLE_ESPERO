/*******************************************************************************
 * @file BLEServer.cpp
 * @brief Implementation of BLE server functionality including advertising,
 * client management, service provisioning, and JSON command processing.
 *
 * @version 0.0.3
 * @date 2025-09-24
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "ble_server.hpp"

#include <cJSON.h>

/******************************************************************************/
/*                               Static Variables                             */
/******************************************************************************/

static const char* TAG = "BLEServer";
BLEServer* BLEServer::instance_ = nullptr;

// Advertising data structure
static esp_ble_adv_data_t adv_data_ = {
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
static esp_ble_adv_params_t advParams_ = {
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

static const bleServerConfig_t default_server_config = {
    .deviceName = "SenseAI_BLE",
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

BLEServer::BLEServer() : BLEServer(default_server_config) {
}

BLEServer::BLEServer(const bleServerConfig_t& config) :
    config_(config),
    state_(BLE_SERVER_IDLE),
    gattsInter_(ESP_GATT_IF_NONE),
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
    instance_ = this;
    
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
    stats_.startTime = bleGetTimestamp();
    
    // Initialize custom data
    strcpy(customData_, status_.customData);
    
    // Create default security configuration
    bleCreateDefaultSecurityConfig(&securityConfig_, BLE_SECURITY_BASIC);

    ESP_LOGI(TAG, "BLEServer created with device name: %s", config_.deviceName);
    ESP_LOGI(TAG, "JSON commands: %s", config_.enableJsonCommands ? "ENABLED" : "DISABLED");
}

BLEServer::~BLEServer() {
    ESP_LOGW(TAG, "BLEServer destructor called - performing enhanced cleanup");
    
    // CRITICAL: Stop all operations before cleanup to avoid spinlock
    if (state_ != BLE_SERVER_IDLE) {
        ESP_LOGW(TAG, "Server not properly stopped before destruction");
        
        // Emergency stop advertising
        if (isAdvertising()) {
            ESP_LOGI(TAG, "Emergency stop advertising");
            stopAdvertising();
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        
        // Emergency disconnect all clients
        if (hasConnectedClients()) {
            ESP_LOGI(TAG, "Emergency disconnect all clients");
            disconnectAllClients();
            vTaskDelay(pdMS_TO_TICKS(800)); // Longer wait for client cleanup
        }
    }
    
    // Stop all tasks first to avoid accessing deleted resources
    if (dataUpdateTaskHandle_ != nullptr) {
        ESP_LOGI(TAG, "Stopping data update task");
        vTaskDelete(dataUpdateTaskHandle_);
        dataUpdateTaskHandle_ = nullptr;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (clientTimeoutTaskHandle_ != nullptr) {
        ESP_LOGI(TAG, "Stopping client timeout task");
        vTaskDelete(clientTimeoutTaskHandle_);
        clientTimeoutTaskHandle_ = nullptr;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // CRITICAL: Clean mutexes last and safely to avoid spinlock
    if (clientsMutex_ != nullptr) {
        ESP_LOGI(TAG, "Cleaning clients mutex");
        // Try to take mutex with timeout before deleting
        if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(200)) == pdTRUE) {
            xSemaphoreGive(clientsMutex_);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Critical delay
        vSemaphoreDelete(clientsMutex_);
        clientsMutex_ = nullptr;
    }
    
    if (dataMutex_ != nullptr) {
        ESP_LOGI(TAG, "Cleaning data mutex");
        // Try to take mutex with timeout before deleting
        if (xSemaphoreTake(dataMutex_, pdMS_TO_TICKS(200)) == pdTRUE) {
            xSemaphoreGive(dataMutex_);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Critical delay
        vSemaphoreDelete(dataMutex_);
        dataMutex_ = nullptr;
    }
    
    // Final cleanup delay to ensure all BLE operations complete
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "BLEServer destructor completed safely");
    instance_ = nullptr;
}

/******************************************************************************/
/*                              Initialization                                */
/******************************************************************************/

esp_err_t BLEServer::init() {
    ESP_LOGI(TAG, "Initializing BLE server...");
    
    // Create mutexes
    clientsMutex_ = xSemaphoreCreateMutex();
    dataMutex_ = xSemaphoreCreateMutex();
    
    if (!clientsMutex_ || !dataMutex_) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize common BLE
    esp_err_t ret = bleCommonInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE common: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_ble_gatt_set_local_mtu(BLE_MAX_MTU_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set local MTU: %s", esp_err_to_name(ret));
        // return ret;
    }else {
        ESP_LOGI(TAG, "Local MTU set to %d", BLE_MAX_MTU_SIZE);
    }
    
    // Register GAP callback
    ret = esp_ble_gap_register_callback(gapCallback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register GATT server callback
    ret = esp_ble_gatts_register_callback(gattsCallback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GATT server callback: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register application
    ret = esp_ble_gatts_app_register(0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GATT server app: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set device name
    ret = esp_ble_gap_set_device_name(config_.deviceName);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set device name: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure advertising data
    if (securityConfig_.useCustomUUIDS) {
        adv_data_.p_service_uuid = (uint8_t*)securityConfig_.serviceUUID;
    } else {
        // Use 16-bit UUID for basic security
        static uint8_t service_uuid_16[2] = {
            (uint8_t)(BLE_DEFAULT_SERVICE_UUID_16 & 0xFF),
            (uint8_t)(BLE_DEFAULT_SERVICE_UUID_16 >> 8)
        };
        adv_data_.p_service_uuid = service_uuid_16;
        adv_data_.service_uuid_len = 2;
    }
    
    ret = esp_ble_gap_config_adv_data(&adv_data_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure advertising data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Update advertising interval
    setAdvertisingInterval(config_.advertisingInterval);

    // Start tasks only if intervals are set
    if (config_.dataUpdateInterval > 0) {
        xTaskCreate(dataUpdateTask, "ble_server_data", 4096, this, 5, &dataUpdateTaskHandle_);
    }
    
    if (config_.clientTimeout > 0) {
        xTaskCreate(clientTimeoutTask, "ble_server_timeout", 4096, this, 3, &clientTimeoutTaskHandle_);
    }
    
    state_ = BLE_SERVER_READY;
    status_.state = state_;
    
    ESP_LOGI(TAG, "BLE server initialized successfully");
    
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
        ESP_LOGW(TAG, "Cannot start advertising - server not ready");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = esp_ble_gap_start_advertising(&advParams_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
        return ret;
    }
    
    state_ = BLE_SERVER_ADVERTISING;
    status_.state = state_;
    status_.advertisingActive = true;
    stats_.advertisingCycles++;
    
    ESP_LOGI(TAG, "Started advertising");
    
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
        ESP_LOGE(TAG, "Failed to stop advertising: %s", esp_err_to_name(ret));
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
    
    ESP_LOGI(TAG, "Stopped advertising");
    
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
        ESP_LOGW(TAG, "Battery level clamped to 100%%");
        level = 100;
    }
    
    if (xSemaphoreTake(dataMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        batteryLevel_ = level;
        status_.batteryLevel = level;
        status_.lastUpdateTime = bleGetTimestamp();
        xSemaphoreGive(dataMutex_);
        
        ESP_LOGD(TAG, "Battery level updated to %d%%", level);
        
        // Notify connected clients if notifications are enabled
        if (config_.enableNotifications) {
            notifyAllClients();
        }
        
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t BLEServer::setCustomData(const char* data) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(data) >= BLE_MAX_CUSTOM_DATA_LEN) {
        ESP_LOGW(TAG, "Custom data truncated to fit buffer");
    }
    
    if (xSemaphoreTake(dataMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        strncpy(customData_, data, BLE_MAX_CUSTOM_DATA_LEN - 1);
        customData_[BLE_MAX_CUSTOM_DATA_LEN - 1] = '\0';
        
        strncpy(status_.customData, customData_, BLE_MAX_CUSTOM_DATA_LEN - 1);
        status_.customData[BLE_MAX_CUSTOM_DATA_LEN - 1] = '\0';
        status_.lastUpdateTime = bleGetTimestamp();
        
        xSemaphoreGive(dataMutex_);
        
        ESP_LOGD(TAG, "Custom data updated: %s", customData_);
        
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

esp_err_t BLEServer::sendDataSizeNotification(uint16_t connID, size_t dataSize, uint16_t chunkSize) {
    cJSON *notification = cJSON_CreateObject();
    
    cJSON_AddStringToObject(notification, "type", "large_data_incoming");
    cJSON_AddNumberToObject(notification, "total_size", dataSize);
    cJSON_AddNumberToObject(notification, "chunk_size", chunkSize);
    cJSON_AddNumberToObject(notification, "estimated_chunks", (dataSize + chunkSize - 1) / chunkSize);
    cJSON_AddStringToObject(notification, "status", "prepare_to_receive");
    
    char *json_string = cJSON_Print(notification);
    
    // Enviar como una notificación pequeña
    esp_err_t ret = esp_ble_gatts_send_indicate(gattsInter_, connID, customCharHandle_,
                                               strlen(json_string), (uint8_t*)json_string, false);
    
    free(json_string);
    cJSON_Delete(notification);
    
    return ret;
}

esp_err_t BLEServer::sendTransferCompleteNotification(uint16_t connID, size_t totalSize, size_t chunksUsed) {
    cJSON *notification = cJSON_CreateObject();
    
    cJSON_AddStringToObject(notification, "type", "transfer_complete");
    cJSON_AddNumberToObject(notification, "total_size", totalSize);
    cJSON_AddNumberToObject(notification, "chunks_sent", chunksUsed);
    cJSON_AddStringToObject(notification, "status", "success");
    
    char *json_string = cJSON_Print(notification);
    
    esp_err_t ret = esp_ble_gatts_send_indicate(gattsInter_, connID, customCharHandle_,
                                               strlen(json_string), (uint8_t*)json_string, false);
    
    free(json_string);
    cJSON_Delete(notification);
    
    return ret;
}


esp_err_t BLEServer::sendJsonResponse(uint16_t connID, const char* jsonResponse) {
    if (!jsonResponse) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Obtener MTU negociado específico del cliente
    uint16_t clientMTU = getClientMTU(connID);
    uint16_t chunk_size = clientMTU - 3;  // Restar header ATT
    size_t json_len = strlen(jsonResponse);
    
    ESP_LOGI(TAG, "Sending JSON response to client %d (%zu bytes) using MTU %d (chunk size: %d)", 
             connID, json_len, clientMTU, chunk_size);
    
    // Si es muy grande, notificar al cliente primero
    if (json_len > chunk_size) {
        esp_err_t prep_ret = sendDataSizeNotification(connID, json_len, chunk_size);
        if (prep_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send size notification: %s", esp_err_to_name(prep_ret));
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Dar tiempo al cliente para prepararse
    }
    
    if (json_len <= chunk_size) {
        // Enviar en una sola pieza
        esp_err_t ret = esp_ble_gatts_send_indicate(gattsInter_, connID, customCharHandle_,
                                                   json_len, (uint8_t*)jsonResponse, false);
        if (ret == ESP_OK) {
            stats_.dataSent++;
        }
        return ret;
    } else {
        // Enviar en chunks con información de progreso
        esp_err_t result = ESP_OK;
        size_t total_chunks = (json_len + chunk_size - 1) / chunk_size;
        
        for (size_t i = 0; i < json_len; i += chunk_size) {
            size_t copy_len = (json_len - i < chunk_size) ? json_len - i : chunk_size;
            size_t current_chunk = (i / chunk_size) + 1;
            
            ESP_LOGD(TAG, "Sending chunk %zu/%zu (%zu bytes) to client %d", 
                     current_chunk, total_chunks, copy_len, connID);
            
            esp_err_t ret = esp_ble_gatts_send_indicate(gattsInter_, connID, customCharHandle_,
                                                       copy_len, (uint8_t*)(jsonResponse + i), false);
            if (ret == ESP_OK) {
                stats_.dataSent++;
                
                // Delay adaptivo basado en MTU
                uint32_t delay = (clientMTU > 100) ? 20 : 50;
                vTaskDelay(pdMS_TO_TICKS(delay));
            } else {
                ESP_LOGE(TAG, "Failed to send chunk %zu/%zu: %s", current_chunk, total_chunks, esp_err_to_name(ret));
                result = ret;
                break;
            }
        }
        
        // Enviar notificación de completado
        if (result == ESP_OK) {
            sendTransferCompleteNotification(connID, json_len, total_chunks);
        }
        
        return result;
    }
}

esp_err_t BLEServer::sendJsonResponseToAll(const char* jsonResponse) {
    if (!jsonResponse) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t result = ESP_OK;
    
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID != 0) {
                esp_err_t ret = sendJsonResponse(clientSessions_[i].connID, jsonResponse);
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
            if (clientSessions_[i].connID != 0 && clientSessions_[i].notificationsEnabled) {
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

esp_err_t BLEServer::notifyClient(uint16_t connID, const char* characteristicType) {
    if (!characteristicType) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t result = ESP_OK;
    
    // Notify battery characteristic
    if (strcmp(characteristicType, "battery") == 0 || strcmp(characteristicType, "both") == 0) {
        if (batteryCharHandle_ != BLE_INVALID_HANDLE) {
            esp_err_t ret = esp_ble_gatts_send_indicate(gattsInter_, connID, batteryCharHandle_,
                                                       sizeof(batteryLevel_), &batteryLevel_, false);
            if (ret == ESP_OK) {
                stats_.dataSent++;
                
                // Update client session stats
                if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
                    for (int i = 0; i < maxClients_; i++) {
                        if (clientSessions_[i].connID == connID) {
                            clientSessions_[i].dataSent++;
                            clientSessions_[i].lastActivity = bleGetTimestamp();
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
    if (strcmp(characteristicType, "custom") == 0 || strcmp(characteristicType, "both") == 0) {
        if (customCharHandle_ != BLE_INVALID_HANDLE) {
            esp_err_t ret = esp_ble_gatts_send_indicate(gattsInter_, connID, customCharHandle_,
                                                       strlen(customData_), (uint8_t*)customData_, false);
            if (ret == ESP_OK) {
                stats_.dataSent++;
            } else {
                result = ret;
            }
        }
    }
    
    return result;
}

/******************************************************************************/
/*                              JSON Processing Methods                       */
/******************************************************************************/

void BLEServer::processJsonData(uint16_t connID, const uint8_t* data, uint16_t length) {
    if (!config_.enableJsonCommands || !data || length == 0) {
        return;
    }
    
    // Find client session
    bleClientSession_t* session = nullptr;
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connID) {
                session = &clientSessions_[i];
                break;
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    if (!session) {
        ESP_LOGW(TAG, "Session not found for client %d", connID);
        return;
    }
    
    // Accumulate data in client's JSON buffer
    for (uint16_t i = 0; i < length; i++) {
        if (session->jsonBufferPos < sizeof(session->jsonBuffer) - 1) {
            session->jsonBuffer[session->jsonBufferPos++] = data[i];
            
            // Check for complete JSON (look for closing brace)
            if (data[i] == '}') {
                session->jsonBuffer[session->jsonBufferPos] = '\0';
                
                // Process the complete JSON command
                processJsonCommand(connID, session->jsonBuffer);
                
                // Reset buffer
                session->jsonBufferPos = 0;
                memset(session->jsonBuffer, 0, sizeof(session->jsonBuffer));
            }
        } else {
            // Buffer overflow, reset
            ESP_LOGW(TAG, "JSON buffer overflow for client %d", connID);
            session->jsonBufferPos = 0;
            memset(session->jsonBuffer, 0, sizeof(session->jsonBuffer));
        }
    }
}

void BLEServer::processJsonCommand(uint16_t connID, const char* json_string) {
    if (!json_string) {
        return;
    }
    
    ESP_LOGI(TAG, "Processing JSON command from client %d: %s", connID, json_string);
    
    // Parse JSON to check if it's an acknowledgment
    cJSON *json = cJSON_Parse(json_string);
    if (json != nullptr) {
        cJSON *type = cJSON_GetObjectItem(json, "type");
        if (type != nullptr && cJSON_IsString(type)) {
            // Check if this is an acknowledgment
            if (strcmp(type->valuestring, "ack") == 0 || strcmp(type->valuestring, "acknowledgment") == 0) {
                cJSON *dataId = cJSON_GetObjectItem(json, "data_id");
                uint32_t ackDataId = 0;
                
                if (dataId != nullptr && cJSON_IsNumber(dataId)) {
                    ackDataId = (uint32_t)dataId->valueint;
                }
                
                ESP_LOGI(TAG, "Received acknowledgment from client %d for data ID %lu", connID, ackDataId);
                
                // Handle the acknowledgment
                esp_err_t ret = handleClientAck(connID, ackDataId);
                if (ret == ESP_OK) {
                    // Send acknowledgment response
                    cJSON *response = cJSON_CreateObject();
                    cJSON_AddStringToObject(response, "type", "ack_received");
                    cJSON_AddNumberToObject(response, "data_id", ackDataId);
                    cJSON_AddStringToObject(response, "status", "success");
                    
                    char *response_string = cJSON_Print(response);
                    sendJsonResponse(connID, response_string);
                    
                    free(response_string);
                    cJSON_Delete(response);
                }
                
                cJSON_Delete(json);
                return;
            }
        }
        cJSON_Delete(json);
    }
    
    stats_.jsonCommandsProcessed++;
    
    // Call user callback if registered
    if (jsonCommandCB_) {
        bleJsonCommand_t command;
        command.connID = connID;
        strncpy(command.commandJson, json_string, sizeof(command.commandJson) - 1);
        command.commandJson[sizeof(command.commandJson) - 1] = '\0';
        command.timestamp = bleGetTimestamp();
        
        bool handled = jsonCommandCB_(&command);
        if (!handled) {
            ESP_LOGW(TAG, "JSON command not handled by user callback");
            stats_.jsonCommandsFailed++;
        }
    } else {
        ESP_LOGW(TAG, "No JSON command callback registered");
        stats_.jsonCommandsFailed++;
    }
}

/******************************************************************************/
/*                              Client Management                             */
/******************************************************************************/

esp_err_t BLEServer::disconnectClient(uint16_t connID) {
    esp_err_t ret = esp_ble_gatts_close(gattsInter_, connID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect client %d: %s", connID, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Disconnecting client %d", connID);
    return ESP_OK;
}

esp_err_t BLEServer::disconnectAllClients() {
    ESP_LOGI(TAG, "Disconnecting all clients with enhanced safety");
    
    if (state_ != BLE_SERVER_CONNECTED && state_ != BLE_SERVER_READY) {
        ESP_LOGW(TAG, "No clients to disconnect");
        return ESP_OK;
    }
    
    esp_err_t result = ESP_OK;
    
    // CRITICAL FIX: Take mutex with longer timeout and handle failure gracefully
    if (clientsMutex_ != nullptr && xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(2000)) == pdTRUE) {
        ESP_LOGI(TAG, "Mutex acquired for client disconnection");
        
        // Disconnect all active sessions
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID != 0) {
                ESP_LOGI(TAG, "Disconnecting client %d (connID: %d)", i, clientSessions_[i].connID);
                
                // Force disconnect - don't wait for callback
                esp_err_t ret = esp_ble_gatts_close(gattsInter_, clientSessions_[i].connID);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to close connection %d: %s", 
                             clientSessions_[i].connID, esp_err_to_name(ret));
                    result = ret;
                }
                
                // Clear session immediately - don't wait for event
                memset(&clientSessions_[i], 0, sizeof(bleClientSession_t));
                clientSessions_[i].connID = 0; // Reset to invalid
            }
        }
        
        // Reset client count immediately
        status_.connectedClients = 0;
        stats_.currentClients = 0;
        
        xSemaphoreGive(clientsMutex_);
        ESP_LOGI(TAG, "Mutex released after client disconnection");
        
    } else {
        ESP_LOGE(TAG, "Failed to take clients mutex for disconnect - performing emergency disconnect");
        
        // Emergency disconnect without mutex - risky but better than spinlock
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID != 0) {
                ESP_LOGW(TAG, "Emergency disconnect client %d", clientSessions_[i].connID);
                esp_ble_gatts_close(gattsInter_, clientSessions_[i].connID);
                clientSessions_[i].connID = 0; // Reset to invalid
            }
        }
        
        // Reset counts
        status_.connectedClients = 0;
        stats_.currentClients = 0;
        result = ESP_ERR_TIMEOUT;
    }
    
    // Give extended time for disconnections to process
    vTaskDelay(pdMS_TO_TICKS(800));
    
    // Update state
    if (status_.connectedClients == 0) {
        state_ = BLE_SERVER_IDLE;
    }
    
    ESP_LOGI(TAG, "All clients disconnected (result: %s)", esp_err_to_name(result));
    return result;
}

esp_err_t BLEServer::addClientSession(uint16_t connID, const esp_bd_addr_t client_addr) {
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Find empty slot
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == 0) {
                clientSessions_[i].connID = connID;
                memcpy(clientSessions_[i].address, client_addr, sizeof(esp_bd_addr_t));
                clientSessions_[i].authenticated = !config_.requireAuthentication;
                clientSessions_[i].connectTime = bleGetTimestamp();
                clientSessions_[i].lastActivity = clientSessions_[i].connectTime;
                clientSessions_[i].dataSent = 0;
                clientSessions_[i].dataReceived = 0;
                clientSessions_[i].notificationsEnabled = config_.enableNotifications;
                clientSessions_[i].jsonBufferPos = 0;
                memset(clientSessions_[i].jsonBuffer, 0, sizeof(clientSessions_[i].jsonBuffer));
                
                clientSessions_[i].negotiatedMTU = BLE_DEFAULT_MTU_SIZE; // Default MTU
                clientSessions_[i].effectiveDataSize = BLE_DEFAULT_MTU_SIZE - 3; // Default effective data size
                clientSessions_[i].maxJsonSize = 80; // 4 chunks of 20 bytes
                
                // Initialize confirmation status
                clientSessions_[i].confirmationStatus.dataAcknowledged = true;
                clientSessions_[i].confirmationStatus.lastDataSentTime = 0;
                clientSessions_[i].confirmationStatus.pendingDataId = 0;
                clientSessions_[i].confirmationStatus.status = BLE_CONFIRM_NOT_REQUIRED;
                clientSessions_[i].confirmationStatus.autoDisconnectAfterAck = false;
                clientSessions_[i].confirmationStatus.ackTimeoutMs = 5000; // Default 5 second timeout

                status_.connectedClients++;
                stats_.totalConnections++;
                stats_.currentClients++;
                
                xSemaphoreGive(clientsMutex_);
                
                ESP_LOGI(TAG, "Added client session %d (total clients: %d)", connID, status_.connectedClients);
                return ESP_OK;
            }
        }
        xSemaphoreGive(clientsMutex_);
        
        ESP_LOGW(TAG, "No available slots for new client %d", connID);
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t BLEServer::removeClientSession(uint16_t connID) {
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connID) {
                memset(&clientSessions_[i], 0, sizeof(bleClientSession_t));
                status_.connectedClients--;
                stats_.currentClients--;
                
                xSemaphoreGive(clientsMutex_);

                ESP_LOGI(TAG, "Removed client session %d (remaining clients: %d)", connID, status_.connectedClients);

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
    
    ESP_LOGW(TAG, "Client session %d not found", connID);
    return ESP_ERR_NOT_FOUND;
}

/******************************************************************************/
/*                              Configuration                                 */
/******************************************************************************/

esp_err_t BLEServer::setConfig(const bleServerConfig_t& config) {
    config_ = config;
    maxClients_ = config.maxClients;
    status_.jsonProcessingEnabled = config.enableJsonCommands;

    ESP_LOGI(TAG, "Configuration updated");
    ESP_LOGI(TAG, "JSON commands: %s", config_.enableJsonCommands ? "ENABLED" : "DISABLED");
    return ESP_OK;
}

bleServerConfig_t BLEServer::getConfig() const {
    return config_;
}

esp_err_t BLEServer::setDeviceName(const char* deviceName) {
    if (!deviceName || strlen(deviceName) >= BLE_MAX_DEVICE_NAME_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    strcpy(config_.deviceName, deviceName);

    esp_err_t ret = esp_ble_gap_set_device_name(deviceName);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set device name: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Device name updated to: %s", deviceName);
    return ESP_OK;
}

esp_err_t BLEServer::setAdvertisingInterval(uint32_t interval) {
    config_.advertisingInterval = interval;
    
    // Convert to BLE units (0.625ms units)
    uint16_t intervalBle = (uint16_t)(interval * 1000 / 625);
    
    advParams_.adv_int_min = intervalBle;
    advParams_.adv_int_max = intervalBle + 10;  // Small range for consistent timing
    
    ESP_LOGI(TAG, "Advertising interval updated to %ld ms", interval);
    return ESP_OK;
}

/******************************************************************************/
/*                              Callback Registration                         */
/******************************************************************************/

void BLEServer::setClientConnectedCallback(bleServerClientConnectedCB_t callback) {
    clientConnectedCB_ = callback;
}

void BLEServer::setClientDisconnectedCallback(bleServerClientDisconnectedCB_t callback) {
    clientDisconnectedCB_ = callback;
}

void BLEServer::setDataWrittenCallback(bleServerDataWrittenCB_t callback) {
    dataWrittenCB_ = callback;
}

void BLEServer::setDataReadCallback(bleServerDataReadCB_t callback) {
    dataReadCB_ = callback;
}

void BLEServer::setClientAuthCallback(bleServerClientAuthCB_t callback) {
    clientAuthCB_ = callback;
}

void BLEServer::setAdvertisingCallback(bleServerAdvertisingCB_t callback) {
    advertisingCB_ = callback;
}

void BLEServer::setJsonCommandCallback(bleServerJsonCommandCB_t callback) {
    jsonCommandCB_ = callback;
    ESP_LOGI(TAG, "JSON command callback %s", callback ? "registered" : "unregistered");
}

/******************************************************************************/
/*                              Status Methods                                */
/******************************************************************************/

bleServerState_t BLEServer::getState() const {
    return state_;
}

bool BLEServer::isAdvertising() const {
    return status_.advertisingActive;
}

bool BLEServer::hasConnectedClients() const {
    return status_.connectedClients > 0;
}

uint8_t BLEServer::getConnectedClientCount() const {
    return status_.connectedClients;
}

bleServerStatus_t BLEServer::getStatus() const {
    return status_;
}

const bleClientSession_t* BLEServer::getClientSession(uint16_t connID) const {
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connID) {
                xSemaphoreGive(clientsMutex_);
                return &clientSessions_[i];
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    return nullptr;
}

uint8_t BLEServer::getAllClientSessions(bleClientSession_t* sessions, uint8_t max_sessions) const {
    if (!sessions || max_sessions == 0) {
        return 0;
    }
    
    uint8_t count = 0;
    
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < maxClients_ && count < max_sessions; i++) {
            if (clientSessions_[i].connID != 0) {
                sessions[count] = clientSessions_[i];
                count++;
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    return count;
}

bleServerStats_t BLEServer::getStats() const {
    bleServerStats_t stats = stats_;
    stats.totalUpTime = (bleGetTimestamp() - stats_.startTime) / 1000;  // Convert to ms
    return stats;
}

void BLEServer::resetStats() {
    memset(&stats_, 0, sizeof(stats_));
    stats_.startTime = bleGetTimestamp();
    ESP_LOGI(TAG, "Statistics reset");
}

/******************************************************************************/
/*                              Utility Methods                               */
/******************************************************************************/

const char* BLEServer::kgetStateString() const {
    switch (state_) {
        case BLE_SERVER_IDLE: return "IDLE";
        case BLE_SERVER_ADVERTISING: return "ADVERTISING";
        case BLE_SERVER_CONNECTED: return "CONNECTED";
        case BLE_SERVER_READY: return "READY";
        case BLE_SERVER_STOPPING: return "STOPPING";
        default: return "UNKNOWN";
    }
}

uint64_t BLEServer::getUptime() const {
    return (bleGetTimestamp() - stats_.startTime) / 1000;  // Convert to ms
}

bool BLEServer::performHealthCheck() {
    bool healthy = true;
    
    // Check server state
    if (state_ == BLE_SERVER_IDLE) {
        ESP_LOGW(TAG, "Health check: Server is idle");
        healthy = false;
    }
    
    // Check memory
    uint32_t free_heap = esp_get_free_heap_size();
    if (free_heap < 10000) {  // Less than 10KB free
        ESP_LOGW(TAG, "Health check: Low memory (%ld bytes)", free_heap);
        healthy = false;
    }
    
    // Check mutexes
    if (!clientsMutex_ || !dataMutex_) {
        ESP_LOGE(TAG, "Health check: Missing mutexes");
        healthy = false;
    }
    
    ESP_LOGI(TAG, "Health check: %s (JSON commands: %s)", 
             healthy ? "PASS" : "FAIL",
             config_.enableJsonCommands ? "ON" : "OFF");
    return healthy;
}

void BLEServer::getMemoryInfo(uint32_t* free_heap, uint32_t* min_free_heap) const {
    if (free_heap) {
        *free_heap = esp_get_free_heap_size();
    }
    if (min_free_heap) {
        *min_free_heap = esp_get_minimum_free_heap_size();
    }
}

esp_err_t BLEServer::generateStatusReport(char* buffer, size_t buffer_size) const {
    if (!buffer || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    bleServerStats_t stats = getStats();
    
    int written = snprintf(buffer, buffer_size,
        "=== BLE Server Status Report ===\n"
        "State: %s\n"
        "Device Name: %s\n"
        "Advertising: %s\n"
        "Connected Clients: %d/%d\n"
        "Battery Level: %d%%\n"
        "Custom Data: %s\n"
        "Uptime: %lld ms\n"
        "Total Connections: %ld\n"
        "Data Packets Sent: %ld\n"
        "Data Packets Received: %ld\n"
        "JSON Commands Processed: %ld\n"
        "JSON Commands Failed: %ld\n"
        "JSON Processing: %s\n"
        "Free Heap: %ld bytes\n",
        kgetStateString(),
        config_.deviceName,
        status_.advertisingActive ? "Active" : "Inactive",
        status_.connectedClients, maxClients_,
        status_.batteryLevel,
        status_.customData,
        stats.totalUpTime,
        stats.totalConnections,
        stats.dataSent,
        stats.dataReceived,
        stats.jsonCommandsProcessed,
        stats.jsonCommandsFailed,
        config_.enableJsonCommands ? "ENABLED" : "DISABLED",
        esp_get_free_heap_size()
    );
    
    if (written >= (int)buffer_size) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    return ESP_OK;
}

esp_err_t BLEServer::setClientMTU(uint16_t connID, uint16_t mtu) {
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connID) {
                clientSessions_[i].negotiatedMTU = mtu;
                clientSessions_[i].effectiveDataSize = mtu - 3;  // Restar header ATT
                clientSessions_[i].maxJsonSize = (mtu - 3) * 4;  // Permitir hasta 4 chunks
                
                ESP_LOGI(TAG, "Client %d MTU set to %d bytes (effective data: %d bytes, max JSON: %d bytes)", 
                         connID, mtu, clientSessions_[i].effectiveDataSize, clientSessions_[i].maxJsonSize);
                
                xSemaphoreGive(clientsMutex_);
                return ESP_OK;
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    ESP_LOGW(TAG, "Client %d not found for MTU update", connID);
    return ESP_ERR_NOT_FOUND;
}

uint16_t BLEServer::getClientMTU(uint16_t connID) const {
    uint16_t mtu = BLE_DEFAULT_MTU_SIZE;  // MTU por defecto
    
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connID) {
                mtu = clientSessions_[i].negotiatedMTU;
                break;
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    return mtu;
}

esp_err_t BLEServer::sendMTUCapabilitiesInfo(uint16_t connID, uint16_t negotiatedMTU) {
    // Crear JSON con información de las capacidades negociadas
    cJSON *capabilities = cJSON_CreateObject();
    cJSON *mtu_info = cJSON_CreateObject();
    
    cJSON_AddNumberToObject(mtu_info, "negotiated_mtu", negotiatedMTU);
    cJSON_AddNumberToObject(mtu_info, "max_data_size", negotiatedMTU - 3);
    cJSON_AddNumberToObject(mtu_info, "recommended_chunk_size", (negotiatedMTU - 3));
    cJSON_AddStringToObject(mtu_info, "chunking_support", "automatic");
    
    cJSON_AddItemToObject(capabilities, "mtu_capabilities", mtu_info);
    cJSON_AddStringToObject(capabilities, "type", "mtu_negotiation_complete");
    cJSON_AddNumberToObject(capabilities, "timestamp", bleGetTimestamp() / 1000);
    
    char *json_string = cJSON_Print(capabilities);
    
    esp_err_t ret = sendJsonResponse(connID, json_string);
    
    free(json_string);
    cJSON_Delete(capabilities);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MTU capabilities sent to client %d", connID);
    } else {
        ESP_LOGW(TAG, "Failed to send MTU capabilities to client %d: %s", connID, esp_err_to_name(ret));
    }
    
    return ret;
}


/******************************************************************************/
/*                         Confirmation Tracking Methods                     */
/******************************************************************************/

esp_err_t BLEServer::waitForClientAck(uint16_t connID, uint32_t timeoutMs) {
    const TickType_t checkInterval = pdMS_TO_TICKS(50); // Check every 50ms
    TickType_t startTime = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeoutMs);
    
    ESP_LOGI(TAG, "Waiting for acknowledgment from client %d (timeout: %lu ms)", connID, timeoutMs);
    
    while ((xTaskGetTickCount() - startTime) < timeout) {
        if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (int i = 0; i < maxClients_; i++) {
                if (clientSessions_[i].connID == connID) {
                    if (clientSessions_[i].confirmationStatus.status == BLE_CONFIRM_ACKNOWLEDGED) {
                        xSemaphoreGive(clientsMutex_);
                        ESP_LOGI(TAG, "Acknowledgment received from client %d", connID);
                        return ESP_OK;
                    }
                    break;
                }
            }
            xSemaphoreGive(clientsMutex_);
        }
        vTaskDelay(checkInterval);
    }
    
    ESP_LOGW(TAG, "Acknowledgment timeout for client %d", connID);
    
    // Mark as timeout
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connID) {
                clientSessions_[i].confirmationStatus.status = BLE_CONFIRM_TIMEOUT;
                break;
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t BLEServer::handleClientAck(uint16_t connID, uint32_t dataId) {
    ESP_LOGI(TAG, "Processing acknowledgment from client %d for data ID %lu", connID, dataId);
    
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connID) {
                if (clientSessions_[i].confirmationStatus.pendingDataId == dataId || dataId == 0) {
                    clientSessions_[i].confirmationStatus.dataAcknowledged = true;
                    clientSessions_[i].confirmationStatus.status = BLE_CONFIRM_ACKNOWLEDGED;
                    
                    ESP_LOGI(TAG, "Acknowledgment confirmed for client %d", connID);
                    
                    // Check if auto-disconnect is enabled
                    if (clientSessions_[i].confirmationStatus.autoDisconnectAfterAck) {
                        xSemaphoreGive(clientsMutex_);
                        ESP_LOGI(TAG, "Auto-disconnecting client %d after acknowledgment", connID);
                        
                        // Delay to ensure ack response is sent before disconnecting
                        vTaskDelay(pdMS_TO_TICKS(100));
                        return disconnectClient(connID);
                    }
                    
                    xSemaphoreGive(clientsMutex_);
                    return ESP_OK;
                }
                break;
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    ESP_LOGW(TAG, "No matching pending data found for ack from client %d", connID);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t BLEServer::setClientConfirmationMode(uint16_t connID, bool autoDisconnect, uint32_t timeoutMs) {
    ESP_LOGI(TAG, "Setting confirmation mode for client %d: auto-disconnect=%s, timeout=%lu ms", 
             connID, autoDisconnect ? "true" : "false", timeoutMs);
    
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connID) {
                clientSessions_[i].confirmationStatus.autoDisconnectAfterAck = autoDisconnect;
                clientSessions_[i].confirmationStatus.ackTimeoutMs = timeoutMs;
                clientSessions_[i].confirmationStatus.status = BLE_CONFIRM_NOT_REQUIRED;
                
                xSemaphoreGive(clientsMutex_);
                return ESP_OK;
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t BLEServer::sendDataWithConfirmation(uint16_t connID, const char* data, bool requireAck, bool autoDisconnect) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    static uint32_t dataIdCounter = 1;
    uint32_t currentDataId = dataIdCounter++;
    
    ESP_LOGI(TAG, "Sending data with confirmation tracking to client %d (ID: %lu, require_ack: %s)", 
             connID, currentDataId, requireAck ? "true" : "false");
    
    // Set up confirmation tracking
    if (requireAck && xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connID) {
                clientSessions_[i].confirmationStatus.dataAcknowledged = false;
                clientSessions_[i].confirmationStatus.lastDataSentTime = bleGetTimestamp();
                clientSessions_[i].confirmationStatus.pendingDataId = currentDataId;
                clientSessions_[i].confirmationStatus.status = BLE_CONFIRM_PENDING;
                clientSessions_[i].confirmationStatus.autoDisconnectAfterAck = autoDisconnect;
                break;
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    // Create JSON message with confirmation info
    cJSON *message = cJSON_CreateObject();
    cJSON_AddStringToObject(message, "data", data);
    cJSON_AddNumberToObject(message, "data_id", currentDataId);
    cJSON_AddBoolToObject(message, "requires_ack", requireAck);
    cJSON_AddNumberToObject(message, "timestamp", bleGetTimestamp() / 1000);
    
    if (requireAck) {
        cJSON_AddStringToObject(message, "type", "data_with_ack");
    } else {
        cJSON_AddStringToObject(message, "type", "data");
    }
    
    char *json_string = cJSON_Print(message);
    esp_err_t ret = sendJsonResponse(connID, json_string);
    
    free(json_string);
    cJSON_Delete(message);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send data with confirmation: %s", esp_err_to_name(ret));
        
        // Clear pending confirmation on send failure
        if (requireAck && xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (int i = 0; i < maxClients_; i++) {
                if (clientSessions_[i].connID == connID) {
                    clientSessions_[i].confirmationStatus.status = BLE_CONFIRM_NOT_REQUIRED;
                    break;
                }
            }
            xSemaphoreGive(clientsMutex_);
        }
    }
    
    return ret;
}

bleConfirmationStatus_t BLEServer::getClientConfirmationStatus(uint16_t connID) const {
    if (xSemaphoreTake(clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < maxClients_; i++) {
            if (clientSessions_[i].connID == connID) {
                bleConfirmationStatus_t status = clientSessions_[i].confirmationStatus.status;
                xSemaphoreGive(clientsMutex_);
                return status;
            }
        }
        xSemaphoreGive(clientsMutex_);
    }
    
    return BLE_CONFIRM_NOT_REQUIRED;
}


/******************************************************************************/
/*                              Internal Tasks                                */
/******************************************************************************/

void BLEServer::dataUpdateTask(void *pvParameters) {
    BLEServer* server = static_cast<BLEServer*>(pvParameters);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(server->config_.dataUpdateInterval);
    
    ESP_LOGI(TAG, "Data update task started (interval: %ld ms)", server->config_.dataUpdateInterval);
    
    while (true) {
        vTaskDelayUntil(&last_wake_time, task_period);
        // Update custom data with timestamp
        char timestamp_data[BLE_MAX_CUSTOM_DATA_LEN];
        snprintf(timestamp_data, sizeof(timestamp_data), "Data-%lld", bleGetTimestamp() / 1000000);
        server->setCustomData(timestamp_data);
    }
}

void BLEServer::clientTimeoutTask(void *pvParameters) {
    BLEServer* server = static_cast<BLEServer*>(pvParameters);
    
    const TickType_t check_interval = pdMS_TO_TICKS(5000);  // Check every 5 seconds
    
    ESP_LOGI(TAG, "Client timeout task started (timeout: %ld ms)", server->config_.clientTimeout);
    
    while (true) {
        vTaskDelay(check_interval);
        
        uint64_t current_time = bleGetTimestamp();
        uint64_t timeout_threshold = server->config_.clientTimeout * 1000;  // Convert to microseconds
        
        if (xSemaphoreTake(server->clientsMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (int i = 0; i < server->maxClients_; i++) {
                if (server->clientSessions_[i].connID != 0) {
                    uint64_t inactive_time = current_time - server->clientSessions_[i].lastActivity;
                    
                    if (inactive_time > timeout_threshold) {
                        ESP_LOGW(TAG, "Client %d timed out after %lld ms", 
                                server->clientSessions_[i].connID, inactive_time / 1000);
                        
                        // Disconnect timed-out client
                        uint16_t connID = server->clientSessions_[i].connID;
                        xSemaphoreGive(server->clientsMutex_);
                        
                        server->disconnectClient(connID);
                        
                        // Re-acquire mutex for next iteration
                        if (xSemaphoreTake(server->clientsMutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
                            break;
                        }
                    }
                }
            }
            xSemaphoreGive(server->clientsMutex_);
        }
    }
}

/******************************************************************************/
/*                              Internal Methods                              */
/******************************************************************************/

bool BLEServer::validateClientAuthentication(uint16_t connID, const char* auth_data) {
    if (!config_.requireAuthentication) {
        return true;  // Authentication not required
    }
    
    if (!auth_data) {
        ESP_LOGW(TAG, "Client %d: No authentication data provided", connID);
        return false;
    }
    
    // Check against configured authentication key
    if (strcmp(auth_data, securityConfig_.authKey) == 0) {
        ESP_LOGI(TAG, "Client %d: Authentication successful", connID);
        return true;
    }
    
    ESP_LOGW(TAG, "Client %d: Authentication failed", connID);
    return false;
}

/******************************************************************************/
/*                              Static Callbacks                              */
/******************************************************************************/

void BLEServer::gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (!instance_) return;
    
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set complete");
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
                instance_->status_.advertisingActive = false;
                if (instance_->advertisingCB_) {
                    instance_->advertisingCB_(false, ESP_FAIL);
                }
            } else {
                ESP_LOGI(TAG, "Advertising started successfully");
                instance_->status_.advertisingActive = true;
                if (instance_->advertisingCB_) {
                    instance_->advertisingCB_(true, ESP_OK);
                }
            }
            break;
            
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising stop failed: %d", param->adv_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising stopped successfully");
                instance_->status_.advertisingActive = false;
                
                // Si auto-advertising está habilitado y no hay clientes máximos conectados, reiniciar advertising
                if (instance_->config_.autoStartAdvertising && 
                    instance_->status_.connectedClients < instance_->maxClients_) {
                    ESP_LOGI(TAG, "Auto-restarting advertising (auto-advertising enabled)");
                    vTaskDelay(pdMS_TO_TICKS(100)); // Pequeño delay antes de reiniciar
                    instance_->startAdvertising();
                }
            }
            break;
            
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Connection parameters updated - interval: %d, latency: %d", 
                    param->update_conn_params.conn_int, param->update_conn_params.latency);
            break;
            
        default:
            ESP_LOGD(TAG, "GAP event: %d", event);
            break;
    }
}

void BLEServer::gattsCallback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, 
                             esp_ble_gatts_cb_param_t *param) {
    if (!instance_) return;
    
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server registered: app_id=%d, status=%d", 
                    param->reg.app_id, param->reg.status);
            
            if (param->reg.status == ESP_GATT_OK) {
                instance_->gattsInter_ = gatts_if;
                
                // Create service
                esp_gatt_srvc_id_t service_id;
                service_id.is_primary = true;
                service_id.id.inst_id = 0x00;
                
                if (instance_->securityConfig_.useCustomUUIDS) {
                    service_id.id.uuid.len = ESP_UUID_LEN_128;
                    memcpy(service_id.id.uuid.uuid.uuid128, 
                           instance_->securityConfig_.serviceUUID, 16);
                } else {
                    service_id.id.uuid.len = ESP_UUID_LEN_16;
                    service_id.id.uuid.uuid.uuid16 = BLE_DEFAULT_SERVICE_UUID_16;
                }
                
                esp_ble_gatts_create_service(gatts_if, &service_id, 10);
            }
            break;
         
        case ESP_GATTS_MTU_EVT:
        {
            ESP_LOGI(TAG, " MTU exchange completed: conn_id=%d, MTU=%d", 
                    param->mtu.conn_id, param->mtu.mtu);
            
            // Actualizar MTU en la sesión del cliente si es necesario
            instance_->setClientMTU(param->mtu.conn_id, param->mtu.mtu);

            esp_err_t cap_ret = instance_->sendMTUCapabilitiesInfo(param->mtu.conn_id, param->mtu.mtu);
            if (cap_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send MTU capabilities: %s", esp_err_to_name(cap_ret));
            }
            break;
        }
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Service created: service_handle=%d", param->create.service_handle);
            instance_->serviceHandle_ = param->create.service_handle;
            
            // Start the service
            esp_ble_gatts_start_service(param->create.service_handle);
            
            // Add battery characteristic
            esp_bt_uuid_t battery_char_uuid;
            if (instance_->securityConfig_.useCustomUUIDS) {
                battery_char_uuid.len = ESP_UUID_LEN_128;
                memcpy(battery_char_uuid.uuid.uuid128, 
                       instance_->securityConfig_.batteryCharUUID, 16);
            } else {
                battery_char_uuid.len = ESP_UUID_LEN_16;
                battery_char_uuid.uuid.uuid16 = BLE_DEFAULT_BATTERY_CHAR_UUID;
            }
            
            esp_ble_gatts_add_char(instance_->serviceHandle_, &battery_char_uuid,
                                  ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                  ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                  NULL, NULL);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            if (param->add_char.status == ESP_GATT_OK) {
                if (instance_->batteryCharHandle_ == BLE_INVALID_HANDLE) {
                    instance_->batteryCharHandle_ = param->add_char.attr_handle;
                    ESP_LOGI(TAG, "Battery characteristic added: handle=%d", 
                            instance_->batteryCharHandle_);
                    
                    // Add CCCD descriptor for battery characteristic (needed for notifications)
                    esp_bt_uuid_t descr_uuid;
                    descr_uuid.len = ESP_UUID_LEN_16;
                    descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
                    
                    esp_ble_gatts_add_char_descr(instance_->serviceHandle_, &descr_uuid,
                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                NULL, NULL);
                                                
                } else if (instance_->customCharHandle_ == BLE_INVALID_HANDLE) {
                    instance_->customCharHandle_ = param->add_char.attr_handle;
                    ESP_LOGI(TAG, "Custom characteristic added: handle=%d", 
                            instance_->customCharHandle_);
                    
                    // Add CCCD descriptor for custom characteristic (needed for notifications)
                    esp_bt_uuid_t descr_uuid;
                    descr_uuid.len = ESP_UUID_LEN_16;
                    descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
                    
                    esp_ble_gatts_add_char_descr(instance_->serviceHandle_, &descr_uuid,
                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                NULL, NULL);
                }
            }
            break;
            
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "Service started: service_handle=%d", param->start.service_handle);
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            {
                ESP_LOGI(TAG, "Client connected: conn_id=%d", param->connect.conn_id);
                
                // Add client session
                instance_->addClientSession(param->connect.conn_id, param->connect.remote_bda);
                
                ESP_LOGI(TAG, "Initiating MTU negotiation for client %d", param->connect.conn_id);
                esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gatts_if, param->connect.conn_id);
                if (mtu_ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to send MTU request to client %d: %s", 
                            param->connect.conn_id, esp_err_to_name(mtu_ret));
                    instance_->setClientMTU(param->connect.conn_id, BLE_DEFAULT_MTU_SIZE);
                }else{
                    ESP_LOGI(TAG, "MTU negotiation request sent succesfuly");
                }


                // Update server state
                instance_->state_ = BLE_SERVER_CONNECTED;
                instance_->status_.state = instance_->state_;
                
                // El ESP32 detiene automáticamente el advertising al conectarse un cliente
                // Actualizamos nuestro estado interno para reflejar esto
                instance_->status_.advertisingActive = false;
                
                // Call user callback
                if (instance_->clientConnectedCB_) {
                    bleDeviceInfo_t client_info;
                    memcpy(client_info.address, param->connect.remote_bda, sizeof(esp_bd_addr_t));
                    client_info.rssi = 0;  // RSSI not available in connect event
                    client_info.authenticated = !instance_->config_.requireAuthentication;
                    client_info.lastSeen = bleGetTimestamp();
                    strcpy(client_info.name, "Unknown");
                    
                    instance_->clientConnectedCB_(param->connect.conn_id, &client_info);
                }

                // Reiniciar advertising inmediatamente si no hemos alcanzado el máximo de clientes
                if (instance_->status_.connectedClients < instance_->maxClients_) {
                    if (instance_->config_.autoStartAdvertising) {
                        ESP_LOGI(TAG, "Restarting advertising to allow more connections (%d/%d clients)", 
                                instance_->status_.connectedClients, instance_->maxClients_);
                        vTaskDelay(pdMS_TO_TICKS(50)); // Pequeño delay para estabilidad
                        instance_->startAdvertising();
                    }
                } else {
                    ESP_LOGI(TAG, "Maximum clients connected (%d/%d), keeping advertising stopped", 
                            instance_->status_.connectedClients, instance_->maxClients_);
                }
                break;
            }
            
        case ESP_GATTS_DISCONNECT_EVT:
            {
                ESP_LOGI(TAG, "Client disconnected: conn_id=%d, reason=%d", 
                    param->disconnect.conn_id, param->disconnect.reason);
            
                // Remove client session
                instance_->removeClientSession(param->disconnect.conn_id);
                
                // Update server state if no more clients
                if (instance_->status_.connectedClients == 0) {
                    instance_->state_ = BLE_SERVER_READY;
                    instance_->status_.state = instance_->state_;
                }
                
                // Call user callback
                if (instance_->clientDisconnectedCB_) {
                    instance_->clientDisconnectedCB_(param->disconnect.conn_id, param->disconnect.reason);
                }
                
                // SIEMPRE reiniciar advertising después de desconexión (si auto-advertising está habilitado)
                if (instance_->config_.autoStartAdvertising) {
                    ESP_LOGI(TAG, "Client disconnected - restarting advertising (%d/%d clients remaining)", 
                            instance_->status_.connectedClients, instance_->maxClients_);
                    
                    // Pequeño delay para asegurar que la desconexión se complete
                    vTaskDelay(pdMS_TO_TICKS(100));
                    
                    esp_err_t ret = instance_->startAdvertising();
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to restart advertising after disconnection: %s", esp_err_to_name(ret));
                    } else {
                        ESP_LOGI(TAG, "Advertising restarted successfully after client disconnection");
                    }
                } else {
                    ESP_LOGW(TAG, "Auto-advertising disabled - advertising not restarted");
                }
                break;
            }
            
        case ESP_GATTS_READ_EVT:
            {
                ESP_LOGD(TAG, "Read request: conn_id=%d, handle=%d", 
                        param->read.conn_id, param->read.handle);
                
                esp_gatt_rsp_t response;
                memset(&response, 0, sizeof(response));
                response.attr_value.handle = param->read.handle;
                
                if (param->read.handle == instance_->batteryCharHandle_) {
                    response.attr_value.len = 1;
                    response.attr_value.value[0] = instance_->batteryLevel_;
                    
                    if (instance_->dataReadCB_) {
                        instance_->dataReadCB_(param->read.conn_id, "battery");
                    }
                } else if (param->read.handle == instance_->customCharHandle_) {
                    response.attr_value.len = strlen(instance_->customData_);
                    memcpy(response.attr_value.value, instance_->customData_, response.attr_value.len);
                    
                    if (instance_->dataReadCB_) {
                        instance_->dataReadCB_(param->read.conn_id, "custom");
                    }
                }
                
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &response);
                
                // Update client activity
                if (xSemaphoreTake(instance_->clientsMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
                    for (int i = 0; i < instance_->maxClients_; i++) {
                        if (instance_->clientSessions_[i].connID == param->read.conn_id) {
                            instance_->clientSessions_[i].lastActivity = bleGetTimestamp();
                            break;
                        }
                    }
                    xSemaphoreGive(instance_->clientsMutex_);
                }
                break;
            }
        case ESP_GATTS_WRITE_EVT:
            {
                ESP_LOGD(TAG, "Write request: conn_id=%d, handle=%d, len=%d", 
                        param->write.conn_id, param->write.handle, param->write.len);
                
                if (param->write.handle == instance_->customCharHandle_) {
                    // Process JSON data if enabled
                    if (instance_->config_.enableJsonCommands) {
                        instance_->processJsonData(param->write.conn_id, param->write.value, param->write.len);
                    } else {
                        // Update custom data from client write (legacy mode)
                        char new_data[BLE_MAX_CUSTOM_DATA_LEN];
                        size_t copy_len = (param->write.len < BLE_MAX_CUSTOM_DATA_LEN - 1) ? 
                                        param->write.len : BLE_MAX_CUSTOM_DATA_LEN - 1;
                        
                        memcpy(new_data, param->write.value, copy_len);
                        new_data[copy_len] = '\0';
                        
                        instance_->setCustomData(new_data);
                    }
                    
                    instance_->stats_.dataReceived++;
                    
                    if (instance_->dataWrittenCB_) {
                        instance_->dataWrittenCB_(param->write.conn_id, param->write.value, param->write.len);
                    }
                    
                    // Update client activity and stats
                    if (xSemaphoreTake(instance_->clientsMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
                        for (int i = 0; i < instance_->maxClients_; i++) {
                            if (instance_->clientSessions_[i].connID == param->write.conn_id) {
                                instance_->clientSessions_[i].lastActivity = bleGetTimestamp();
                                instance_->clientSessions_[i].dataReceived++;
                                break;
                            }
                        }
                        xSemaphoreGive(instance_->clientsMutex_);
                    }
                }
                
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                            ESP_GATT_OK, NULL);
                }
                break;
            }    
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            {
                if (param->add_char_descr.status == ESP_GATT_OK) {
                    ESP_LOGI(TAG, "CCCD descriptor added: handle=%d, service_handle=%d", 
                            param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
                            
                    // Check if this is the first descriptor (battery CCCD)
                    if (instance_->batteryDescrHandle_ == BLE_INVALID_HANDLE) {
                        instance_->batteryDescrHandle_ = param->add_char_descr.attr_handle;
                        ESP_LOGI(TAG, "Battery CCCD descriptor handle stored: %d", instance_->batteryDescrHandle_);
                        
                        // Now add the custom characteristic
                        esp_bt_uuid_t custom_char_uuid;
                        if (instance_->securityConfig_.useCustomUUIDS) {
                            custom_char_uuid.len = ESP_UUID_LEN_128;
                            memcpy(custom_char_uuid.uuid.uuid128, 
                                instance_->securityConfig_.customCharUUID, 16);
                        } else {
                            custom_char_uuid.len = ESP_UUID_LEN_16;
                            custom_char_uuid.uuid.uuid16 = BLE_DEFAULT_CUSTOM_CHAR_UUID;
                        }
                        
                        esp_ble_gatts_add_char(instance_->serviceHandle_, &custom_char_uuid,
                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | 
                                            ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                            NULL, NULL);
                    } else if (instance_->customDescrHandle_ == BLE_INVALID_HANDLE) {
                        instance_->customDescrHandle_ = param->add_char_descr.attr_handle;
                        ESP_LOGI(TAG, "Custom CCCD descriptor handle stored: %d", instance_->customDescrHandle_);
                        ESP_LOGI(TAG, "All characteristics and descriptors added successfully!");
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to add CCCD descriptor: status=%d", param->add_char_descr.status);
                }
                break;
            }    
        case ESP_GATTS_CONF_EVT:
            ESP_LOGD(TAG, "Notification confirmed: conn_id=%d, status=%d", 
                    param->conf.conn_id, param->conf.status);
            break;
        
        case ESP_GATTS_EXEC_WRITE_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_ADD_INCL_SRVC_EVT:
        case ESP_GATTS_DELETE_EVT:
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_RESPONSE_EVT:
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        case ESP_GATTS_SET_ATTR_VAL_EVT:
        case ESP_GATTS_SEND_SERVICE_CHANGE_EVT:
            // Casos no manejados actualmente
            ESP_LOGD(TAG, "Unhandled GATTS event: %d", event);
            break;

        default:
            ESP_LOGD(TAG, "GATTS event: %d", event);
            break;
    }
}
