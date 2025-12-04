/*******************************************************************************
 * @file ble_client.cpp
 * @brief Implementation of BLE client functionality including scanning,
 * connecting, and data communication with BLE servers.
 *
 * @version 0.0.5
 * @date 2025-06-24
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/
#include "ble_client.hpp"

#include "esp_random.h"
#include <cJSON.h>

/******************************************************************************/
/*                              Static Variables                              */
/******************************************************************************/

static const char* TAG = "BLE_CLIENT";
BLEClient* BLEClient::instance_ = nullptr;

// Default scan parameters
static esp_ble_scan_params_t defaultScanParams = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,    // 50ms
    .scan_window            = 0x30,    // 30ms
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

/******************************************************************************/
/*                              Constructor/Destructor                        */
/******************************************************************************/

BLEClient::BLEClient() 
    : state_(BLE_CLIENT_IDLE)
    , gattCInter_(0)
    , connID_(0)
    , serviceStartHandle_(0)
    , serviceEndHandle_(0)
    , batteryCharHandle_(BLE_INVALID_HANDLE)
    , customCharHandle_(BLE_INVALID_HANDLE)
    , negotiatedMTU_(23)                     // MTU por defecto
    , localMTU_(512)                         // MTU local máximo
    , mtuExchangeCompleted_(false)           // MTU exchange not completed
    , rxBufferPos_(0)                        // RX buffer position
    , expectingLargeData_(false)             // Not expecting large data
    , connectedCB_(nullptr)
    , disconnectedCB_(nullptr)
    , dataReceivedCB_(nullptr)
    , dataReceivedWithAckCB_(nullptr)
    , deviceFoundCB_(nullptr)
    , authCB_(nullptr)
    , autoSendAcknowledgments_(false)
    , disconnectAfterAck_(false)
    , lastReceivedDataId_(0)
    , dataReadTaskHandle_(nullptr)
    , reconnectTaskHandle_(nullptr)
    , stateMutex_(nullptr)
    , connectionStartTime_(0)
    , lastDataTime_(0)
    , anyDeviceFoundCB_(nullptr)
    , scanStartedCB_(nullptr)
    , scanCompletedCB_(nullptr)
    , discoveryMode_(false)
    , currentScanDuration_(0)
    , scanStartTime_(0)
    , foundDevicesCount_(0)
    , targetDeviceFoundInScan_(false) {

    // Initialize default configuration
    memset(&config_, 0, sizeof(config_));
    strncpy(config_.targetDeviceName, "SenseAI_BLE", BLE_MAX_DEVICE_NAME_LEN - 1);
    config_.scanTimeout = BLE_DEFAULT_SCAN_TIMEOUT;
    config_.autoReconnect = true;
    config_.reconnectInterval = BLE_DEFAULT_RECONNECT_TIME;
    config_.connectionTimeout = 10000;
    config_.enableNotifications = true;
    config_.readInterval = 5000;
    config_.autoSendAcknowledgments = false;
    config_.disconnectAfterAck = false;

    // Initialize security configuration
    bleCreateDefaultSecurityConfig(&securityConfig_, BLE_SECURITY_BASIC);

    // Initialize device and data structures
    memset(&connectedDevice_, 0, sizeof(connectedDevice_));
    memset(&lastData_, 0, sizeof(lastData_));
    memset(&stats_, 0, sizeof(stats_));

    // List of found devices
    memset(foundDevices_, 0, sizeof(foundDevices_));
    
    // Initialize RX buffer
    memset(rxBuffer_, 0, sizeof(rxBuffer_));
    
    // Initialize acknowledgment settings
    autoSendAcknowledgments_ = false;
    disconnectAfterAck_ = false;
    lastReceivedDataId_ = 0;

    // Create mutex
    stateMutex_ = xSemaphoreCreateMutex();
    
    // Set static instance for callbacks
    instance_ = this;

    ESP_LOGI(TAG, "BLE Client created");
}

BLEClient::BLEClient(const bleClientConfig_t& config) : BLEClient() {
    config_ = config;
    ESP_LOGI(TAG, "BLE Client created with custom configuration");
}

BLEClient::~BLEClient() {
    ESP_LOGI(TAG, "Destroying BLE Client");

    // Stop tasks
    if (dataReadTaskHandle_ != nullptr) {
        vTaskDelete(dataReadTaskHandle_);
    }
    if (reconnectTaskHandle_ != nullptr) {
        vTaskDelete(reconnectTaskHandle_);
    }

    // Disconnect if connected
    if (isConnected()) {
        disconnect(true);
    }

    // Clean up mutex
    if (stateMutex_ != nullptr) {
        vSemaphoreDelete(stateMutex_);
    }

    // Clear static instance
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

/******************************************************************************/
/*                              Core Methods                                  */
/******************************************************************************/

esp_err_t BLEClient::init() {
    ESP_LOGI(TAG, "Initializing BLE Client");

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    // Configure local MTU BEFORE registering callbacks
    ret = esp_ble_gatt_set_local_mtu(localMTU_);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set local MTU: %s", esp_err_to_name(ret));
        // Continue with default MTU
        localMTU_ = 23;
    } else {
        ESP_LOGI(TAG, "Local MTU set to %d bytes", localMTU_);
    }

    // Register GAP callback
    ret = esp_ble_gap_register_callback(gapCallback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        xSemaphoreGive(stateMutex_);
        return ret;
    }

    // Register GATTC callback
    ret = esp_ble_gattc_register_callback(gattcCallback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GATTC callback: %s", esp_err_to_name(ret));
        xSemaphoreGive(stateMutex_);
        return ret;
    }

    // Register GATTC application
    ret = esp_ble_gattc_app_register(0x56);  // App ID
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GATTC app: %s", esp_err_to_name(ret));
        xSemaphoreGive(stateMutex_);
        return ret;
    }

    state_ = BLE_CLIENT_IDLE;
    xSemaphoreGive(stateMutex_);

    ESP_LOGI(TAG, "BLE Client initialized successfully");
    return ESP_OK;
}

esp_err_t BLEClient::setMacTarget(const esp_bd_addr_t serverMACadd) {
    if (serverMACadd == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(config_.targetServerMACadd, serverMACadd, ESP_BD_ADDR_LEN);
    xSemaphoreGive(stateMutex_);

    char macStr[18];
    bleMacaddToString((uint8_t*)serverMACadd, macStr);
    ESP_LOGI(TAG, "Target MAC address set to: %s", macStr);
    
    return ESP_OK;
}

esp_err_t BLEClient::startScan() {
    char macStr[18];
    bleMacaddToString((uint8_t*)config_.targetServerMACadd, macStr);
    ESP_LOGI(TAG, "Starting BLE scan for target MAC: %s", macStr);

    if (state_ == BLE_CLIENT_SCANNING) {
        ESP_LOGW(TAG, "Already scanning");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    state_ = BLE_CLIENT_SCANNING;
    stats_.scanCount++;

    xSemaphoreGive(stateMutex_);

    // Set scan parameters
    esp_err_t ret = esp_ble_gap_set_scan_params(&defaultScanParams);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set scan params: %s", esp_err_to_name(ret));
        state_ = BLE_CLIENT_IDLE;
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEClient::stopScan() {
    ESP_LOGI(TAG, "Stopping BLE scan");
    
    return esp_ble_gap_stop_scanning();
}

esp_err_t BLEClient::connect(const esp_bd_addr_t serverMACadd) {
    if (state_ == BLE_CLIENT_CONNECTED || state_ == BLE_CLIENT_CONNECTING) {
        ESP_LOGW(TAG, "Already connected or connecting");
        return ESP_ERR_INVALID_STATE;
    }

    if (state_== BLE_CLIENT_SCANNING) {
        ESP_LOGW(TAG, "Stopping scanning before connection");
        esp_err_t stopResult = esp_ble_gap_stop_scanning();
        if (stopResult != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop scanning: %s", esp_err_to_name(stopResult));
            return stopResult;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    char MACaddStr[18];
    bleMacaddToString((uint8_t*)serverMACadd, MACaddStr);
    ESP_LOGI(TAG, "Connecting to server: %s", MACaddStr);

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    state_ = BLE_CLIENT_CONNECTING;
    stats_.connectionAttempts++;
    connectionStartTime_ = bleGetTimestamp();
    
    // Copy server address
    memcpy(connectedDevice_.address, serverMACadd, ESP_BD_ADDR_LEN);

    xSemaphoreGive(stateMutex_);

    // Attempt connection
    esp_err_t ret = esp_ble_gattc_open(gattCInter_, (uint8_t*)serverMACadd, BLE_ADDR_TYPE_PUBLIC, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate connection: %s", esp_err_to_name(ret));
        state_ = BLE_CLIENT_IDLE;
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEClient::disconnect(bool planned) {
    if (!isConnected()) {
        ESP_LOGW(TAG, "Not connected");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disconnecting from server (planned: %s)", planned ? "yes" : "no");

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    state_ = BLE_CLIENT_DISCONNECTING;
    xSemaphoreGive(stateMutex_);

    esp_err_t ret = esp_ble_gattc_close(gattCInter_, connID_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEClient::readBatteryLevel() {
    if (!isConnected() || batteryCharHandle_ == BLE_INVALID_HANDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Reading battery level");

    esp_err_t ret = esp_ble_gattc_read_char(gattCInter_, connID_, batteryCharHandle_, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read battery characteristic: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t BLEClient::readCustomData() {
    if (!isConnected() || customCharHandle_ == BLE_INVALID_HANDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Reading custom data");

    esp_err_t ret = esp_ble_gattc_read_char(gattCInter_, connID_, customCharHandle_, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read custom characteristic: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t BLEClient::writeCustomData(const char* data) {
    if (!isConnected() || customCharHandle_ == BLE_INVALID_HANDLE || data == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t dataLen = strlen(data);
    
    // Esperar a que se complete el MTU exchange
    uint32_t timeout = 0;
    while (!mtuExchangeCompleted_ && timeout < 50) {  // 5 segundos máximo
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout++;
    }
    
    if (!mtuExchangeCompleted_) {
        ESP_LOGW(TAG, "MTU exchange not completed, using default MTU");
        negotiatedMTU_ = 23;
        mtuExchangeCompleted_ = true;
    }

    // Calcular tamaño de chunk efectivo
    uint16_t effectiveChunkSize = negotiatedMTU_ - 3;  // Restar 3 bytes para header ATT
    
    ESP_LOGI(TAG, "Writing data (%zu bytes) using MTU %d (chunk size: %d)", 
             dataLen, negotiatedMTU_, effectiveChunkSize);

    if (dataLen <= effectiveChunkSize) {
        // Enviar todo en una sola escritura
        ESP_LOGI(TAG, "Sending data in single write (%zu bytes)", dataLen);
        
        esp_err_t ret = esp_ble_gattc_write_char(gattCInter_, connID_, customCharHandle_,
                                                dataLen, (uint8_t*)data,
                                                ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (ret == ESP_OK) {
            stats_.dataSent++;
        } else {
            ESP_LOGE(TAG, "Single write failed: %s", esp_err_to_name(ret));
        }
        return ret;
    } else {
        // Usar prepared write para datos largos
        ESP_LOGI(TAG, "Using prepared write for large data (%zu bytes, %zu chunks)", 
                 dataLen, (dataLen + effectiveChunkSize - 1) / effectiveChunkSize);
        
        esp_err_t ret = ESP_OK;
        size_t chunksSent = 0;
        
        for (size_t offset = 0; offset < dataLen; offset += effectiveChunkSize) {
            size_t writeLen = (dataLen - offset < effectiveChunkSize) ? 
                             dataLen - offset : effectiveChunkSize;
            
            ESP_LOGD(TAG, "Preparing chunk %zu: offset=%zu, length=%zu", 
                     chunksSent + 1, offset, writeLen);
            
            ret = esp_ble_gattc_prepare_write(gattCInter_, connID_, customCharHandle_,
                                            offset, writeLen, (uint8_t*)(data + offset),
                                            ESP_GATT_AUTH_REQ_NONE);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Prepare write chunk %zu failed: %s", 
                         chunksSent + 1, esp_err_to_name(ret));
                return ret;
            }
            
            chunksSent++;
            vTaskDelay(pdMS_TO_TICKS(20));  // Delay entre chunks
        }
        
        // Ejecutar todas las escrituras preparadas
        ESP_LOGI(TAG, "Executing prepared write (%zu chunks)", chunksSent);
        ret = esp_ble_gattc_execute_write(gattCInter_, connID_, true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Execute write failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, " Large data written successfully (%zu bytes in %zu chunks)", 
                     dataLen, chunksSent);
            stats_.dataSent++;
        }
        
        return ret;
    }
}

esp_err_t BLEClient::setNotifications(bool enable) {
    if (!isConnected()) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "%s notifications", enable ? "Enabling" : "Disabling");
    
    
    return ESP_OK;
}

/******************************************************************************/
/*                           Configuration Methods                            */
/******************************************************************************/

esp_err_t BLEClient::setConfig(const bleClientConfig_t& config) {
    if (isConnected()) {
        ESP_LOGW(TAG, "Cannot change config while connected");
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;
    ESP_LOGI(TAG, "Client configuration updated");
    return ESP_OK;
}

esp_err_t BLEClient::setSecurityConfig(const bleSecurityConfig_t& securityConfig) {
    if (isConnected()) {
        ESP_LOGW(TAG, "Cannot change security config while connected");
        return ESP_ERR_INVALID_STATE;
    }

    securityConfig_ = securityConfig;
    ESP_LOGI(TAG, "Security configuration updated:");
    ESP_LOGI(TAG, "  Level: %d", securityConfig_.level);
    ESP_LOGI(TAG, "  Use Custom UUIDs: %s", securityConfig_.useCustomUUIDS ? "YES" : "NO");
    ESP_LOGI(TAG, "  Require Authentication: %s", securityConfig_.requireAuthentication ? "YES" : "NO");
    return ESP_OK;
}

bleClientConfig_t BLEClient::getConfig() const {
    return config_;
}

esp_err_t BLEClient::setTargetDevice(const char* deviceName) {
    if (deviceName == nullptr || !bleValidateDeviceName(deviceName)) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(config_.targetDeviceName, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    config_.targetDeviceName[BLE_MAX_DEVICE_NAME_LEN - 1] = '\0';

    ESP_LOGI(TAG, "Target device set to: %s", config_.targetDeviceName);
    return ESP_OK;
}


/******************************************************************************/
/*                            Callback Registration                           */
/******************************************************************************/

void BLEClient::setConnectedCallback(bleClientConnectedCB_t callback) {
    connectedCB_ = callback;
    ESP_LOGD(TAG, "Connected callback registered");
}

void BLEClient::setDisconnectedCallback(bleClientDisconnectedCB_t callback) {
    disconnectedCB_ = callback;
    ESP_LOGD(TAG, "Disconnected callback registered");
}

void BLEClient::setDataReceivedCallback(bleClientDataReceivedCB_t callback) {
    dataReceivedCB_ = callback;
    ESP_LOGD(TAG, "Data received callback registered");
}

void BLEClient::setDeviceFoundCallback(bleClientDeviceFoundCB_t callback) {
    deviceFoundCB_ = callback;
    ESP_LOGD(TAG, "Device found callback registered");
}

void BLEClient::setAuthCallback(bleClientAuthCB_t callback) {
    authCB_ = callback;
    ESP_LOGD(TAG, "Authentication callback registered");
}

void BLEClient::setAnyDeviceFoundCallback(bleClientAnyDeviceFoundCB_t callback) {
    anyDeviceFoundCB_ = callback;
    ESP_LOGD(TAG, "Any device found callback registered");
}

void BLEClient::setScanStartedCallback(bleClientScanStartedCB_t callback) {
    scanStartedCB_ = callback;
    ESP_LOGD(TAG, "Scan started callback registered");
}

void BLEClient::setScanCompletedCallback(bleClientScanCompletedCB_t callback) {
    scanCompletedCB_ = callback;
    ESP_LOGD(TAG, "Scan completed callback registered");
}

void BLEClient::setDataReceivedWithAckCallback(bleClientDataReceivedWithAckCB_t callback) {
    dataReceivedWithAckCB_ = callback;
    ESP_LOGD(TAG, "Data received with acknowledgment callback registered");
}

/******************************************************************************/
/*                         Acknowledgment Methods                            */
/******************************************************************************/

esp_err_t BLEClient::sendDataAcknowledgment(uint32_t dataId) {
    if (!isConnected() || customCharHandle_ == BLE_INVALID_HANDLE) {
        ESP_LOGW(TAG, "Cannot send acknowledgment - not connected or no custom characteristic");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Sending acknowledgment for data ID %lu", dataId);

    // Create acknowledgment JSON
    cJSON *ack = cJSON_CreateObject();
    cJSON_AddStringToObject(ack, "type", "ack");
    cJSON_AddNumberToObject(ack, "data_id", dataId);
    cJSON_AddNumberToObject(ack, "timestamp", bleGetTimestamp() / 1000);
    cJSON_AddStringToObject(ack, "status", "received");

    char *json_string = cJSON_Print(ack);
    
    esp_err_t ret = writeCustomData(json_string);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Acknowledgment sent successfully for data ID %lu", dataId);
        
        // Check if we should disconnect after acknowledgment
        if (disconnectAfterAck_ || config_.disconnectAfterAck) {
            ESP_LOGI(TAG, "Disconnecting after acknowledgment as configured");
            vTaskDelay(pdMS_TO_TICKS(100)); // Allow ack to be sent before disconnecting
            ret = disconnect(true);
        }
    } else {
        ESP_LOGE(TAG, "Failed to send acknowledgment: %s", esp_err_to_name(ret));
    }
    
    free(json_string);
    cJSON_Delete(ack);
    
    return ret;
}

esp_err_t BLEClient::setAutoAcknowledgment(bool autoAck, bool disconnectAfterAck) {
    autoSendAcknowledgments_ = autoAck;
    disconnectAfterAck_ = disconnectAfterAck;
    
    // Also update config for consistency
    config_.autoSendAcknowledgments = autoAck;
    config_.disconnectAfterAck = disconnectAfterAck;
    
    ESP_LOGI(TAG, "Auto-acknowledgment configured: autoAck=%s, disconnectAfterAck=%s", 
             autoAck ? "true" : "false", disconnectAfterAck ? "true" : "false");
    
    return ESP_OK;
}


bleClientState_t BLEClient::getState() const {
    return state_;
}

bool BLEClient::isConnected() const {
    return state_ == BLE_CLIENT_CONNECTED || state_ == BLE_CLIENT_READY || 
           state_ == BLE_CLIENT_AUTHENTICATING || state_ == BLE_CLIENT_DISCOVERING;
}

bool BLEClient::isScanning() const {
    return state_ == BLE_CLIENT_SCANNING;
}

const bleDeviceInfo_t* BLEClient::kgetConnectedDevice() const {
    return isConnected() ? &connectedDevice_ : nullptr;
}

const bleDataPacket_t* BLEClient::kgetLastData() const {
    return lastData_.valid ? &lastData_ : nullptr;
}

bleClientStats_t BLEClient::getStats() const {
    bleClientStats_t stats = stats_;
    
    // Update uptime if connected
    if (isConnected() && connectionStartTime_ > 0) {
        stats.totalUptime = (bleGetTimestamp() - connectionStartTime_) / 1000;
    }
    
    return stats;
}

void BLEClient::resetStats() {
    memset(&stats_, 0, sizeof(stats_));
    ESP_LOGI(TAG, "Client statistics reset");
}

const char* BLEClient::kgetStateString() const {
    switch (state_) {
        case BLE_CLIENT_IDLE: return "IDLE";
        case BLE_CLIENT_SCANNING: return "SCANNING";
        case BLE_CLIENT_CONNECTING: return "CONNECTING";
        case BLE_CLIENT_CONNECTED: return "CONNECTED";
        case BLE_CLIENT_DISCOVERING: return "DISCOVERING";
        case BLE_CLIENT_AUTHENTICATING: return "AUTHENTICATING";
        case BLE_CLIENT_READY: return "READY";
        case BLE_CLIENT_DISCONNECTING: return "DISCONNECTING";
        default: return "UNKNOWN";
    }
}

// uint8_t BLEClient::getConnectionQuality() const {
//     if (!isConnected()) {
//         return 0;
//     }

//     return bleRSSIToQuality(connectedDevice_.rssi);
// }

uint64_t BLEClient::getConnectionUptime() const {
    if (!isConnected() || connectionStartTime_ == 0) {
        return 0;
    }

    return (bleGetTimestamp() - connectionStartTime_) / 1000;
}

bool BLEClient::performHealthCheck() {
    if (state_ == BLE_CLIENT_IDLE) {
        return true;
    }
    
    if (isConnected() && lastDataTime_ > 0) {
        uint64_t timeSinceData = (bleGetTimestamp() - lastDataTime_) / 1000;
        if (timeSinceData > 30000) {  // 30 seconds
            ESP_LOGW(TAG, "No data received for %llu ms", timeSinceData);
            return false;
        }
    }
    
    return true;
}

/******************************************************************************/
/*                          NUEVOS MÉTODOS EXTENDIDOS                        */
/******************************************************************************/

esp_err_t BLEClient::startDiscoveryScan(uint32_t duration) {
    ESP_LOGI(TAG, "Starting discovery scan for %lu ms (all devices)", duration);

    if (state_ == BLE_CLIENT_SCANNING) {
        ESP_LOGW(TAG, "Already scanning");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    discoveryMode_ = true;
    currentScanDuration_ = duration;
    scanStartTime_ = bleGetTimestamp();
    targetDeviceFoundInScan_ = false;

    state_ = BLE_CLIENT_SCANNING;
    stats_.scanCount++;

    xSemaphoreGive(stateMutex_);

    if (scanStartedCB_ != nullptr) {
        scanStartedCB_(duration);
    }

    esp_err_t ret = esp_ble_gap_set_scan_params(&defaultScanParams);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set scan params: %s", esp_err_to_name(ret));
        state_ = BLE_CLIENT_IDLE;
        discoveryMode_ = false;
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEClient::startTargetedScan() {
    char macStr[18];
    bleMacaddToString((uint8_t*)config_.targetServerMACadd, macStr);
    ESP_LOGI(TAG, "Starting targeted scan for MAC: %s", macStr);

    if (state_ == BLE_CLIENT_SCANNING) {
        ESP_LOGW(TAG, "Already scanning");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Configurar modo targeted (normal)
    discoveryMode_ = false;
    currentScanDuration_ = config_.scanTimeout;
    scanStartTime_ = bleGetTimestamp();
    targetDeviceFoundInScan_ = false;

    state_ = BLE_CLIENT_SCANNING;
    stats_.scanCount++;

    xSemaphoreGive(stateMutex_);

    ESP_LOGI(TAG, "Scan configured:");
    ESP_LOGI(TAG, "  Target MAC: %s", macStr);
    ESP_LOGI(TAG, "  Duration: %lu ms", config_.scanTimeout);

    if (scanStartedCB_ != nullptr) {
        scanStartedCB_(config_.scanTimeout);
    }

    esp_err_t ret = esp_ble_gap_set_scan_params(&defaultScanParams);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set scan params: %s", esp_err_to_name(ret));
        state_ = BLE_CLIENT_IDLE;
        discoveryMode_ = false;
        return ret;
    }

    ESP_LOGI(TAG, "Targeted MAC scan started successfully");
    return ESP_OK;
}

void BLEClient::setScanMode(bool discoveryMode) {
    discoveryMode_ = discoveryMode;
    ESP_LOGI(TAG, "Scan mode set to: %s", discoveryMode ? "DISCOVERY" : "TARGETED");
}

uint32_t BLEClient::getUniqueDevicesFound() const {
    return foundDevicesCount_;
}

void BLEClient::clearDeviceList() {
    foundDevicesCount_ = 0;
    memset(foundDevices_, 0, sizeof(foundDevices_));
    ESP_LOGI(TAG, "Device list cleared");
}

const bleDeviceInfo_t* BLEClient::kgetFoundDevice(uint32_t index) const {
    if (index >= foundDevicesCount_) {
        return nullptr;
    }
    return &foundDevices_[index];
}

bool BLEClient::addToFoundDevices(const bleDeviceInfo_t* deviceInfo) {
    if (deviceInfo == nullptr || foundDevicesCount_ >= kMaxFoundDevices) {
        return false;
    }

    if (isDeviceAlreadyFound(deviceInfo->address)) {
        return false; // Ya existe
    }

    // Agregar nuevo dispositivo
    memcpy(&foundDevices_[foundDevicesCount_], deviceInfo, sizeof(bleDeviceInfo_t));
    foundDevicesCount_++;

    ESP_LOGD(TAG, "Added device to list: %s (total: %lu)", 
            deviceInfo->name, foundDevicesCount_);
    return true;
}

bool BLEClient::isDeviceAlreadyFound(const esp_bd_addr_t address) const {
    for (uint32_t i = 0; i < foundDevicesCount_; i++) {
        if (memcmp(foundDevices_[i].address, address, ESP_BD_ADDR_LEN) == 0) {
            return true;
        }
    }
    return false;
}

void BLEClient::gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (instance_ == nullptr) {
        return;
    }
    
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGD(TAG, "Scan parameters set");
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                esp_ble_gap_start_scanning(instance_->currentScanDuration_ > 0 ? 
                                          instance_->currentScanDuration_ / 1000 : 
                                          instance_->config_.scanTimeout / 1000);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Scan start failed: %d", param->scan_start_cmpl.status);
                instance_->state_ = BLE_CLIENT_IDLE;
            } else {
                ESP_LOGI(TAG, "Scan started successfully");
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            instance_->handleScanResult(param);
            break;
            
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGD(TAG, "Scan stopped");
            if (instance_->state_ == BLE_CLIENT_SCANNING) {
                instance_->state_ = BLE_CLIENT_IDLE;
            }
            break;
            
        default:
            break;
    }
}

void BLEClient::gattcCallback(esp_gattc_cb_event_t event, esp_gatt_if_t gattCInter, 
                             esp_ble_gattc_cb_param_t *param) {
    if (instance_ == nullptr) {
        return;
    }
    
    switch (event) {
        case ESP_GATTC_REG_EVT:
            ESP_LOGD(TAG, "GATTC registered, app_id: %04x", param->reg.app_id);
            instance_->gattCInter_ = gattCInter;
            break;
            
        case ESP_GATTC_CONNECT_EVT:
            instance_->handleConnect(param);
            break;
            
        case ESP_GATTC_CFG_MTU_EVT:
            ESP_LOGI(TAG, "MTU exchange result: conn_id=%d, status=%d, MTU=%d", 
                    param->cfg_mtu.conn_id, param->cfg_mtu.status, param->cfg_mtu.mtu);
            
            if (param->cfg_mtu.status == ESP_GATT_OK) {
                instance_->negotiatedMTU_ = param->cfg_mtu.mtu;
                instance_->mtuExchangeCompleted_ = true;
                
                ESP_LOGI(TAG, "✅ MTU negotiated successfully: %d bytes (effective data: %d bytes)", 
                         instance_->negotiatedMTU_, instance_->negotiatedMTU_ - 3);
            } else {
                ESP_LOGW(TAG, "MTU exchange failed, using default MTU: 23 bytes");
                instance_->negotiatedMTU_ = 23;
                instance_->mtuExchangeCompleted_ = true;
            }
            
            // Continuar con service discovery después del MTU exchange
            if (instance_->state_ == BLE_CLIENT_CONNECTED) {
                ESP_LOGI(TAG, "Starting service discovery after MTU exchange");
                instance_->state_ = BLE_CLIENT_DISCOVERING;
                esp_ble_gattc_search_service(instance_->gattCInter_, instance_->connID_, nullptr);
            }
            break;
            
        case ESP_GATTC_DISCONNECT_EVT:
            instance_->handleDisconnect(param);
            break;
            
        case ESP_GATTC_SEARCH_RES_EVT:
            instance_->handleServiceFound(param);
            break;
            
        case ESP_GATTC_SEARCH_CMPL_EVT:
            instance_->handleServiceDiscoveryComplete(param);
            break;
            
        case ESP_GATTC_READ_CHAR_EVT:
            instance_->handleCharacteristicRead(param);
            break;
            
        case ESP_GATTC_WRITE_CHAR_EVT:
            instance_->handleCharacteristicWrite(param);
            break;
            
        case ESP_GATTC_NOTIFY_EVT:
            instance_->handleNotification(param);
            break;
            
        default:
            break;
    }
}

bool BLEClient::verifyTargetDevice(esp_ble_gap_cb_param_t *scanResult) {
    bool hasTargetService = false;

    bool macMatch = false;

    uint8_t zeroMac[ESP_BD_ADDR_LEN] = {0};
    
    if(memcmp(config_.targetServerMACadd, zeroMac, ESP_BD_ADDR_LEN) != 0) {
        macMatch = (memcmp(scanResult->scan_rst.bda, config_.targetServerMACadd, ESP_BD_ADDR_LEN) == 0);
    } 
    
    bool nameMatch =false;

    if(strlen(config_.targetDeviceName) > 0) {
        char deviceName[BLE_MAX_DEVICE_NAME_LEN] = {0};
        if (scanResult->scan_rst.adv_data_len > 0) {
            uint8_t *advData = scanResult->scan_rst.ble_adv;
            uint8_t advDataLen = scanResult->scan_rst.adv_data_len;

            for (int i = 0; i < advDataLen; ) {
                uint8_t length = advData[i];
                if (length == 0) break;

                uint8_t type = advData[i + 1];

                // Buscar nombre del dispositivo
                if (type == 0x09 || type == 0x08) {  // Complete o shortened local name
                    uint8_t nameLen = length - 1;
                    if (nameLen > 0 && nameLen < BLE_MAX_DEVICE_NAME_LEN) {
                        memcpy(deviceName, &advData[i + 2], nameLen);
                        deviceName[nameLen] = '\0';
                    }
                    break;
                }
                
                i += length + 1;
            }
        }

        nameMatch = (strcmp(deviceName, config_.targetDeviceName) == 0);
    } 


    if (securityConfig_.useCustomUUIDS && scanResult->scan_rst.adv_data_len > 0) {
        uint8_t *advData = scanResult->scan_rst.ble_adv;
        uint8_t advDataLen = scanResult->scan_rst.adv_data_len;

        for (int i = 0; i < advDataLen; ) {
            uint8_t length = advData[i];
            if (length == 0) break;

            uint8_t type = advData[i + 1];

            if (type == 0x07 && length == 17) {  // Complete list of 128-bit service UUIDs
                if (bleCompareUUID128(&advData[i + 2], securityConfig_.serviceUUID)) {
                    hasTargetService = true;
                    break;
                }
            }
            
            i += length + 1;
        }
    }

    bool deviceMatch = macMatch || nameMatch;
        
    bool securityOk = false;
    switch (securityConfig_.level) {
        case BLE_SECURITY_NONE:
            securityOk = true;
            break;
            
        case BLE_SECURITY_BASIC:
        case BLE_SECURITY_AUTHENTICATED:
        case BLE_SECURITY_ENCRYPTED:
            securityOk = (hasTargetService || !securityConfig_.useCustomUUIDS);
            break;
            
        default:
            securityOk = false;
            break;
    }

    bool finalResult = deviceMatch && securityOk;

    // Solo mostrar logs detallados cuando encontramos el target
    if (finalResult) {
        char foundMac[18];
        bleMacaddToString(scanResult->scan_rst.bda, foundMac);
        ESP_LOGI(TAG, "*** TARGET DEVICE VERIFIED ***");
        ESP_LOGI(TAG, "  MAC: %s, RSSI: %d dBm", foundMac, scanResult->scan_rst.rssi);
    }

    return finalResult;
}

esp_err_t BLEClient::authenticateWithServer() {
    if (customCharHandle_ == BLE_INVALID_HANDLE || !securityConfig_.requireAuthentication) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Authenticating with server");
    
    state_ = BLE_CLIENT_AUTHENTICATING;
    
    // Send authentication key
    esp_err_t ret = writeCustomData(securityConfig_.authKey);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send authentication: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

/******************************************************************************/
/*                       MÉTODO handleScanResult MODIFICADO                  */
/******************************************************************************/

void BLEClient::handleScanResult(esp_ble_gap_cb_param_t *param) {
    esp_ble_gap_cb_param_t *scanResult = param;

    switch (scanResult->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            // Ceder tiempo al sistema para evitar watchdog
            taskYIELD();
            
            bleDeviceInfo_t currentDevice;
            memset(&currentDevice, 0, sizeof(currentDevice));
            
            memcpy(currentDevice.address, scanResult->scan_rst.bda, ESP_BD_ADDR_LEN);
            currentDevice.rssi = scanResult->scan_rst.rssi;
            currentDevice.lastSeen = bleGetTimestamp();
            
            char deviceName[BLE_MAX_DEVICE_NAME_LEN] = {0};
            if (scanResult->scan_rst.adv_data_len > 0) {
                uint8_t *advData = scanResult->scan_rst.ble_adv;
                uint8_t advDataLen = scanResult->scan_rst.adv_data_len;

                for (int i = 0; i < advDataLen; ) {
                    uint8_t length = advData[i];
                    if (length == 0) break;

                    uint8_t type = advData[i + 1];

                    // Buscar nombre del dispositivo
                    if (type == 0x09 || type == 0x08) {  // Complete o shortened local name
                        uint8_t nameLen = length - 1;
                        if (nameLen > BLE_MAX_DEVICE_NAME_LEN - 1) {
                            nameLen = BLE_MAX_DEVICE_NAME_LEN - 1;
                        }
                        memcpy(deviceName, &advData[i + 2], nameLen);
                        deviceName[nameLen] = '\0';
                        break;
                    }
                    
                    i += length + 1;
                }
            }
            
            // No asignar "Unknown Device" - dejar vacío para filtrar
            strncpy(currentDevice.name, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);

            // Optimización: Solo verificar target si:
            // 1. El dispositivo tiene un nombre Y coincide con el target, O
            // 2. Tenemos una MAC específica configurada (no zeros)
            bool shouldVerify = false;
            uint8_t zeroMac[ESP_BD_ADDR_LEN] = {0};
            bool hasMacTarget = (memcmp(config_.targetServerMACadd, zeroMac, ESP_BD_ADDR_LEN) != 0);
            bool hasNameTarget = (strlen(config_.targetDeviceName) > 0);
            
            if (hasMacTarget) {
                // Verificar si MAC coincide antes de hacer verificación completa
                shouldVerify = (memcmp(scanResult->scan_rst.bda, config_.targetServerMACadd, ESP_BD_ADDR_LEN) == 0);
            } else if (hasNameTarget && strlen(deviceName) > 0) {
                // Solo verificar si el nombre coincide
                shouldVerify = (strcmp(deviceName, config_.targetDeviceName) == 0);
            }
            
            bool isTarget = false;
            if (shouldVerify) {
                isTarget = verifyTargetDevice(scanResult);
                if (isTarget) {
                    targetDeviceFoundInScan_ = true;
                }
            }

            // Solo llamar callback si el dispositivo tiene nombre (filtrar "Unknown")
            if (anyDeviceFoundCB_ != nullptr && strlen(deviceName) > 0) {
                anyDeviceFoundCB_(&currentDevice, isTarget);
            }
            
            // Manejar dispositivo objetivo - conectar si es target (en cualquier modo)
            if (isTarget) {
                char macStr[18];
                bleMacaddToString(scanResult->scan_rst.bda, macStr);

                ESP_LOGI(TAG, "Target device found: %s, RSSI: %d dBm",
                        macStr, scanResult->scan_rst.rssi);

                esp_err_t stopResult = esp_ble_gap_stop_scanning();
                if (stopResult != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to stop scanning: %s", esp_err_to_name(stopResult));
                }

                vTaskDelay(pdMS_TO_TICKS(100));

                // Actualizar información del dispositivo conectado
                memcpy(&connectedDevice_, &currentDevice, sizeof(bleDeviceInfo_t));

                // Llamar callback original de dispositivo encontrado
                bool shouldConnect = true;
                if (deviceFoundCB_ != nullptr) {
                    deviceFoundCB_(&connectedDevice_, &shouldConnect);
                }
                
                // Conectar al dispositivo target
                if (shouldConnect) {
                    ESP_LOGI(TAG, "Connecting to target device: %s", macStr);
                    connect(scanResult->scan_rst.bda);
                }

                return;
            }
            break;
        }
        
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(TAG, "Scan completed - Found %lu unique devices (%s target)", 
                    foundDevicesCount_, targetDeviceFoundInScan_ ? "WITH" : "WITHOUT");
            
            if (state_ == BLE_CLIENT_SCANNING) {
                state_ = BLE_CLIENT_IDLE;
                
                if (scanCompletedCB_ != nullptr) {
                    scanCompletedCB_(foundDevicesCount_, targetDeviceFoundInScan_);
                }
                
                // Auto-retry solo en modo targeted y si no se encontró el target
                if (!discoveryMode_ && config_.autoReconnect && !targetDeviceFoundInScan_) {
                    vTaskDelay(pdMS_TO_TICKS(config_.reconnectInterval));
                    startTargetedScan();
                }
            }
            break;
            
        default:
            break;
    }
}

void BLEClient::handleConnect(esp_ble_gattc_cb_param_t *param) {
    ESP_LOGI(TAG, "Connected to server, conn_id: %d", param->connect.conn_id);

    connID_ = param->connect.conn_id;
    state_ = BLE_CLIENT_CONNECTED;
    stats_.successfulConnections++;
    
    // Reset MTU state for new connection
    mtuExchangeCompleted_ = false;
    negotiatedMTU_ = 23;  // Reset to default
    
    // Iniciar negociación MTU inmediatamente
    ESP_LOGI(TAG, "Requesting MTU exchange");
    esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattCInter_, connID_);
    if (mtu_ret != ESP_OK) {
        ESP_LOGW(TAG, "MTU request failed: %s - proceeding with default", esp_err_to_name(mtu_ret));
        negotiatedMTU_ = 23;
        mtuExchangeCompleted_ = true;
        
        // Proceder con service discovery
        ESP_LOGI(TAG, "Starting service discovery");
        state_ = BLE_CLIENT_DISCOVERING;
        esp_ble_gattc_search_service(gattCInter_, connID_, nullptr);
    }
    // Si MTU request es exitoso, el service discovery se iniciará en ESP_GATTC_CFG_MTU_EVT
}

void BLEClient::handleDisconnect(esp_ble_gattc_cb_param_t *param) {
    ESP_LOGI(TAG, "Disconnected from server, reason: %d", param->disconnect.reason);
    
    // Update statistics
    stats_.disconnections++;
    if (connectionStartTime_ > 0) {
        stats_.totalUptime += (bleGetTimestamp() - connectionStartTime_) / 1000;
    }
    
    // Reset connection state
    state_ = BLE_CLIENT_IDLE;
    connID_ = 0;
    batteryCharHandle_ = BLE_INVALID_HANDLE;
    customCharHandle_ = BLE_INVALID_HANDLE;
    connectionStartTime_ = 0;
    
    // Reset MTU state
    negotiatedMTU_ = 23;
    mtuExchangeCompleted_ = false;
    
    // Reset chunking state
    rxBufferPos_ = 0;
    expectingLargeData_ = false;
    memset(rxBuffer_, 0, sizeof(rxBuffer_));

    // Call disconnection callback
    if (disconnectedCB_ != nullptr) {
        disconnectedCB_(param->disconnect.reason, false);
    }
    
    // Auto-reconnect if configured
    if (config_.autoReconnect) {
        ESP_LOGI(TAG, "Scheduling reconnection in %lu ms", config_.reconnectInterval);
        vTaskDelay(pdMS_TO_TICKS(config_.reconnectInterval));
        startTargetedScan();
    }
}

void BLEClient::handleServiceFound(esp_ble_gattc_cb_param_t *param) {
    ESP_LOGD(TAG, "Service found: UUID len %d", param->search_res.srvc_id.uuid.len);
    
    // Check if this is our target service
    bool serviceMatch = false;

    if (securityConfig_.level == BLE_SECURITY_NONE && !securityConfig_.useCustomUUIDS) {
        // En modo SECURITY_NONE sin UUIDs custom, aceptar el primer servicio que no sea genérico
        // Evitar servicios genéricos de BLE (Generic Access 0x1800, Generic Attribute 0x1801)
        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
            uint16_t uuid16 = param->search_res.srvc_id.uuid.uuid.uuid16;
            if (uuid16 != 0x1800 && uuid16 != 0x1801) {
                serviceMatch = true;
                ESP_LOGI(TAG, "Accepting service UUID16: 0x%04X (SECURITY_NONE mode)", uuid16);
            }
        } else if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
            // Aceptar servicios 128-bit en modo SECURITY_NONE
            serviceMatch = true;
            ESP_LOGI(TAG, "Accepting 128-bit service (SECURITY_NONE mode)");
        }
    } else if (securityConfig_.useCustomUUIDS && param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
        serviceMatch = bleCompareUUID128(param->search_res.srvc_id.uuid.uuid.uuid128, 
                                          securityConfig_.serviceUUID);
    } else if (!securityConfig_.useCustomUUIDS && param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
        serviceMatch = (param->search_res.srvc_id.uuid.uuid.uuid16 == BLE_DEFAULT_SERVICE_UUID_16);
    }
    
    if (serviceMatch) {
        ESP_LOGI(TAG, "Target service found: handles %d-%d", 
                param->search_res.start_handle, param->search_res.end_handle);
        serviceStartHandle_ = param->search_res.start_handle;
        serviceEndHandle_ = param->search_res.end_handle;
    }
}

void BLEClient::handleServiceDiscoveryComplete(esp_ble_gattc_cb_param_t *param) {
    ESP_LOGI(TAG, "Service discovery completed");
    
    if (serviceStartHandle_ != 0) {
        ESP_LOGI(TAG, "Discovering characteristics");
        
        uint16_t count = 10;
        esp_gattc_char_elem_t charElemResult[10];
        uint16_t charCount = count;

        esp_err_t ret = esp_ble_gattc_get_all_char(gattCInter_, connID_, 
                                                  serviceStartHandle_, serviceEndHandle_,
                                                  charElemResult, &charCount, 0);
        
        if (ret == ESP_GATT_OK) {
            ESP_LOGI(TAG, "Found %d characteristics", charCount);
            
            for (int i = 0; i < charCount; i++) {
                esp_gattc_char_elem_t* char_elem = &charElemResult[i];
                
                bool isBattery = false;
                bool isCustom = false;
                
                // En modo SECURITY_NONE, asignar la primera característica con propiedades de lectura/escritura
                if (securityConfig_.level == BLE_SECURITY_NONE && !securityConfig_.useCustomUUIDS) {
                    // Usar la primera característica legible como battery
                    if (batteryCharHandle_ == BLE_INVALID_HANDLE && 
                        (char_elem->properties & ESP_GATT_CHAR_PROP_BIT_READ)) {
                        isBattery = true;
                    }
                    // Usar la primera característica escribible como custom
                    else if (customCharHandle_ == BLE_INVALID_HANDLE && 
                             (char_elem->properties & (ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR))) {
                        isCustom = true;
                    }
                } else if (securityConfig_.useCustomUUIDS && char_elem->uuid.len == ESP_UUID_LEN_128) {
                    isBattery = bleCompareUUID128(char_elem->uuid.uuid.uuid128, 
                                                   securityConfig_.batteryCharUUID);
                    isCustom = bleCompareUUID128(char_elem->uuid.uuid.uuid128, 
                                                  securityConfig_.customCharUUID);
                } else if (!securityConfig_.useCustomUUIDS && char_elem->uuid.len == ESP_UUID_LEN_16) {
                    isBattery = (char_elem->uuid.uuid.uuid16 == BLE_DEFAULT_BATTERY_CHAR_UUID);
                    isCustom = (char_elem->uuid.uuid.uuid16 == BLE_DEFAULT_CUSTOM_CHAR_UUID);
                }
                
                if (isBattery) {
                    batteryCharHandle_ = char_elem->char_handle;
                    ESP_LOGI(TAG, "Battery/Read characteristic found: handle=%d", batteryCharHandle_);
                } else if (isCustom) {
                    customCharHandle_ = char_elem->char_handle;
                    ESP_LOGI(TAG, "Custom/Write characteristic found: handle=%d", customCharHandle_);
                }
            }
            
            if (securityConfig_.requireAuthentication) {
                authenticateWithServer();
            } else {
                state_ = BLE_CLIENT_READY;
                
                if (connectedCB_ != nullptr) {
                    connectedCB_(&connectedDevice_);
                }
                
                if (config_.readInterval > 0) {
                    xTaskCreate(dataReadTask, "ble_client_read", 4096, this, 5, &dataReadTaskHandle_);
                }
            }
        } else {
            ESP_LOGE(TAG, "Failed to get characteristics: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "Target service not found");
        disconnect(false);
    }
}

void BLEClient::handleCharacteristicFound(esp_ble_gattc_cb_param_t *param) {
    ESP_LOGD(TAG, "Characteristic event received");
}

void BLEClient::handleAllCharacteristicsFound(esp_ble_gattc_cb_param_t *param) {
    ESP_LOGD(TAG, "All characteristics event received");
}

void BLEClient::handleCharacteristicRead(esp_ble_gattc_cb_param_t *param) {
    if (param->read.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "Read failed: %d", param->read.status);
        return;
    }
    
    stats_.dataReceived++;
    lastDataTime_ = bleGetTimestamp();

    if (param->read.handle == batteryCharHandle_) {
        lastData_.batteryLevel = param->read.value[0];
        ESP_LOGI(TAG, "Battery level: %d%%", lastData_.batteryLevel);
        
        lastData_.timeStamp = bleGetTimestamp();
        lastData_.valid = true;
        
        if (dataReceivedCB_ != nullptr) {
            dataReceivedCB_(&lastData_);
        }
    } else if (param->read.handle == customCharHandle_) {
        // Use the new method for handling chunked data
        handleIncomingData(param->read.value, param->read.value_len);
    }
}

void BLEClient::handleCharacteristicWrite(esp_ble_gattc_cb_param_t *param) {
    if (param->write.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "Write failed: %d", param->write.status);
        return;
    }
    
    ESP_LOGD(TAG, "Write successful");
    
    if (state_ == BLE_CLIENT_AUTHENTICATING) {
        ESP_LOGI(TAG, "Authentication sent, waiting for response");
        readCustomData();
    }
}

void BLEClient::handleNotification(esp_ble_gattc_cb_param_t *param) {
    ESP_LOGD(TAG, "Notification received from handle: %d", param->notify.handle);
    
    stats_.dataReceived++;
    lastDataTime_ = bleGetTimestamp();

    if (param->notify.handle == batteryCharHandle_) {
        lastData_.batteryLevel = param->notify.value[0];
        ESP_LOGI(TAG, "Battery notification: %d%%", lastData_.batteryLevel);
        
        lastData_.timeStamp = bleGetTimestamp();
        lastData_.valid = true;
        
        if (dataReceivedCB_ != nullptr) {
            dataReceivedCB_(&lastData_);
        }
    } else if (param->notify.handle == customCharHandle_) {
        // Use the new method for handling chunked data
        handleIncomingData(param->notify.value, param->notify.value_len);
        
        // Handle authentication (existing code)
        if (state_ == BLE_CLIENT_AUTHENTICATING) {
            char tempData[BLE_MAX_CUSTOM_DATA_LEN];
            memcpy(tempData, param->notify.value, param->notify.value_len);
            tempData[param->notify.value_len] = '\0';
            
            if (strcmp(tempData, "AUTH_OK") == 0) {
                ESP_LOGI(TAG, "Authentication successful");
                state_ = BLE_CLIENT_READY;

                if (authCB_ != nullptr) {
                    authCB_(true, ESP_OK);
                }
                
                if (connectedCB_ != nullptr) {
                    connectedCB_(&connectedDevice_);
                }
                
                if (config_.readInterval > 0) {
                    xTaskCreate(dataReadTask, "ble_client_read", 4096, this, 5, &dataReadTaskHandle_);
                }
            }
        }
    }
}

void BLEClient::dataReadTask(void *pvParameters) {
    BLEClient* client = static_cast<BLEClient*>(pvParameters);
    
    ESP_LOGI(TAG, "Data read task started");
    
    while (client->isConnected()) {
        // Read battery level
        if (client->batteryCharHandle_ != BLE_INVALID_HANDLE) {
            client->readBatteryLevel();
            vTaskDelay(pdMS_TO_TICKS(1000));  // Wait between reads
        }
        
        // Read custom data
        if (client->customCharHandle_ != BLE_INVALID_HANDLE) {
            client->readCustomData();
        }

        vTaskDelay(pdMS_TO_TICKS(client->config_.readInterval));
    }
    
    ESP_LOGI(TAG, "Data read task ended");
    client->dataReadTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}

void BLEClient::reconnectTask(void *pvParameters) {
    BLEClient* client = static_cast<BLEClient*>(pvParameters);
    
    ESP_LOGI(TAG, "Reconnect task started");
    
    while (client->config_.autoReconnect && !client->isConnected()) {
        vTaskDelay(pdMS_TO_TICKS(client->config_.reconnectInterval));

        if (!client->isConnected() && client->state_ == BLE_CLIENT_IDLE) {
            ESP_LOGI(TAG, "Attempting reconnection");
            client->startTargetedScan();
        }
    }
    
    ESP_LOGI(TAG, "Reconnect task ended");
    client->reconnectTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}

/******************************************************************************/
/*                         MTU and Chunking Support                          */
/******************************************************************************/

void BLEClient::handleIncomingData(const uint8_t* data, uint16_t length) {
    // Verificar si es una notificación de tamaño de datos grandes
    if (length < 100) {  // Mensajes de control son pequeños
        char tempBuffer[256];
        memcpy(tempBuffer, data, length);
        tempBuffer[length] = '\0';
        
        // Parse JSON to check for acknowledgment requirements
        cJSON *json = cJSON_Parse(tempBuffer);
        bool requiresAck = false;
        bool shouldDisconnect = false;
        uint32_t dataId = 0;
        
        if (json != nullptr) {
            cJSON *type = cJSON_GetObjectItem(json, "type");
            cJSON *requiresAckJson = cJSON_GetObjectItem(json, "requires_ack");
            cJSON *dataIdJson = cJSON_GetObjectItem(json, "data_id");
            
            if (requiresAckJson != nullptr && cJSON_IsBool(requiresAckJson)) {
                requiresAck = cJSON_IsTrue(requiresAckJson);
            }
            
            if (dataIdJson != nullptr && cJSON_IsNumber(dataIdJson)) {
                dataId = (uint32_t)dataIdJson->valueint;
                lastReceivedDataId_ = dataId;
            }
            
            // Check for specific control messages
            if (type != nullptr && cJSON_IsString(type)) {
                if (strcmp(type->valuestring, "data_with_ack") == 0) {
                    requiresAck = true;
                }
                
                // Handle ack_received response
                if (strcmp(type->valuestring, "ack_received") == 0) {
                    ESP_LOGI(TAG, "Server confirmed receipt of our acknowledgment");
                    cJSON_Delete(json);
                    return;
                }
            }
            
            cJSON_Delete(json);
        }
        
        // Verificar si es JSON de control
        if (strstr(tempBuffer, "large_data_incoming") != nullptr) {
            ESP_LOGI(TAG, " Large data incoming notification received");
            expectingLargeData_ = true;
            rxBufferPos_ = 0;
            memset(rxBuffer_, 0, sizeof(rxBuffer_));
            return;
        } else if (strstr(tempBuffer, "transfer_complete") != nullptr) {
            ESP_LOGI(TAG, " Transfer complete notification received");
            
            if (rxBufferPos_ > 0) {
                rxBuffer_[rxBufferPos_] = '\0';
                
                // Procesar datos completos
                memcpy(lastData_.customData, rxBuffer_, 
                       rxBufferPos_ < BLE_MAX_CUSTOM_DATA_LEN ? rxBufferPos_ : BLE_MAX_CUSTOM_DATA_LEN - 1);
                lastData_.customData[rxBufferPos_] = '\0';
                lastData_.timeStamp = bleGetTimestamp();
                lastData_.valid = true;
                
                ESP_LOGI(TAG, " Large data assembled (%d bytes): %.*s", 
                         rxBufferPos_, (rxBufferPos_ > 50 ? 50 : rxBufferPos_), rxBuffer_);
                
                // Handle acknowledgment for large data
                if (requiresAck && (autoSendAcknowledgments_ || config_.autoSendAcknowledgments)) {
                    ESP_LOGI(TAG, "Auto-sending acknowledgment for large data (ID: %lu)", dataId);
                    sendDataAcknowledgment(dataId);
                }
                
                // Call enhanced callback if available
                if (dataReceivedWithAckCB_ != nullptr) {
                    dataReceivedWithAckCB_(&lastData_, requiresAck, disconnectAfterAck_);
                } else if (dataReceivedCB_ != nullptr) {
                    dataReceivedCB_(&lastData_);
                }
            }
            
            expectingLargeData_ = false;
            rxBufferPos_ = 0;
            return;
        } else if (strstr(tempBuffer, "mtu_capabilities") != nullptr) {
            ESP_LOGI(TAG, " MTU capabilities received");
            // Procesar información de capacidades MTU del servidor
            return;
        }
        
        // Normal data with possible ack requirement
        memcpy(lastData_.customData, data, 
               length < BLE_MAX_CUSTOM_DATA_LEN ? length : BLE_MAX_CUSTOM_DATA_LEN - 1);
        lastData_.customData[length] = '\0';
        lastData_.timeStamp = bleGetTimestamp();
        lastData_.valid = true;
        
        ESP_LOGI(TAG, " Data received (%d bytes)%s: %s", length, 
                 requiresAck ? " [ACK REQUIRED]" : "", lastData_.customData);
        
        // Handle automatic acknowledgment
        if (requiresAck && (autoSendAcknowledgments_ || config_.autoSendAcknowledgments)) {
            ESP_LOGI(TAG, "Auto-sending acknowledgment for data (ID: %lu)", dataId);
            sendDataAcknowledgment(dataId);
        }
        
        // Call enhanced callback if available
        if (dataReceivedWithAckCB_ != nullptr) {
            dataReceivedWithAckCB_(&lastData_, requiresAck, disconnectAfterAck_);
        } else if (dataReceivedCB_ != nullptr) {
            dataReceivedCB_(&lastData_);
        }
        
        return;
    }
    
    // Si esperamos datos grandes, acumular en buffer
    if (expectingLargeData_) {
        if (rxBufferPos_ + length < sizeof(rxBuffer_)) {
            memcpy(rxBuffer_ + rxBufferPos_, data, length);
            rxBufferPos_ += length;
            ESP_LOGD(TAG, " Accumulated chunk (%d bytes, total: %d)", length, rxBufferPos_);
        } else {
            ESP_LOGW(TAG, "RX buffer overflow, discarding data");
        }
        return;
    }
    
    // Datos normales (no fragmentados)
    memcpy(lastData_.customData, data, 
           length < BLE_MAX_CUSTOM_DATA_LEN ? length : BLE_MAX_CUSTOM_DATA_LEN - 1);
    lastData_.customData[length] = '\0';
    lastData_.timeStamp = bleGetTimestamp();
    lastData_.valid = true;
    
    ESP_LOGI(TAG, " Data received (%d bytes): %s", length, lastData_.customData);
    
    if (dataReceivedCB_ != nullptr) {
        dataReceivedCB_(&lastData_);
    }
}

uint16_t BLEClient::getNegotiatedMTU() const {
    return negotiatedMTU_;
}

bool BLEClient::isMTUExchangeCompleted() const {
    return mtuExchangeCompleted_;
}
