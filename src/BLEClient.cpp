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
#include "BLEClient.hpp"

#include "esp_random.h"

/******************************************************************************/
/*                              Static Variables                              */
/******************************************************************************/

// static const char* BLE_CLIENT_TAG = "BLE_CLIENT";
BLEClient* BLEClient::instance_ = nullptr;

// Default scan parameters
static esp_ble_scan_params_t default_scan_params = {
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
    , connectedCB_(nullptr)
    , disconnectedCB_(nullptr)
    , dataReceivedCB_(nullptr)
    , deviceFoundCB_(nullptr)
    , authCB_(nullptr)
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
    strncpy(config_.targetDeviceName, "154_BLE_Server", BLE_MAX_DEVICE_NAME_LEN - 1);
    config_.scanTimeout = BLE_DEFAULT_SCAN_TIMEOUT;
    config_.autoReconnect = true;
    config_.reconnectInterval = BLE_DEFAULT_RECONNECT_TIME;
    config_.connectionTimeout = 10000;
    config_.enableNotifications = true;
    config_.readInterval = 5000;

    // Initialize security configuration
    ble_create_default_security_config(&securityConfig_, BLE_SECURITY_NONE);

    // Initialize device and data structures
    memset(&connectedDevice_, 0, sizeof(connectedDevice_));
    memset(&lastData_, 0, sizeof(lastData_));
    memset(&stats_, 0, sizeof(stats_));

    // List of found devices
    memset(foundDevices_, 0, sizeof(foundDevices_));

    // Create mutex
    stateMutex_ = xSemaphoreCreateMutex();
    
    // Set static instance for callbacks
    instance_ = this;

    ble_log(ESP_LOG_INFO, "BLE Client created");
}

BLEClient::BLEClient(const ble_client_config_t& config) : BLEClient() {
    config_ = config;
    ble_log(ESP_LOG_INFO, "BLE Client created with custom configuration");
}

BLEClient::~BLEClient() {
    ble_log(ESP_LOG_INFO, "Destroying BLE Client");

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
    ble_log(ESP_LOG_INFO, "Initializing BLE Client");

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    // Register GAP callback
    ret = esp_ble_gap_register_callback(gapCallback);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        xSemaphoreGive(stateMutex_);
        return ret;
    }

    // Register GATTC callback
    ret = esp_ble_gattc_register_callback(gattcCallback);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to register GATTC callback: %s", esp_err_to_name(ret));
        xSemaphoreGive(stateMutex_);
        return ret;
    }

    // Register GATTC application
    ret = esp_ble_gattc_app_register(0x56);  // App ID
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to register GATTC app: %s", esp_err_to_name(ret));
        xSemaphoreGive(stateMutex_);
        return ret;
    }

    state_ = BLE_CLIENT_IDLE;
    xSemaphoreGive(stateMutex_);

    ble_log(ESP_LOG_INFO, "BLE Client initialized successfully");
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

    char addr_str[18];
    ble_addr_to_string((uint8_t*)serverMACadd, addr_str);
    ble_log(ESP_LOG_INFO, "Target MAC address set to: %s", addr_str);
    
    return ESP_OK;
}

esp_err_t BLEClient::startScan() {
    char macStr[18];
    ble_addr_to_string((uint8_t*)config_.targetServerMACadd, macStr);
    ble_log(ESP_LOG_INFO, "Starting BLE scan for target MAC: %s", macStr);

    if (state_ == BLE_CLIENT_SCANNING) {
        ble_log(ESP_LOG_WARN, "Already scanning");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    state_ = BLE_CLIENT_SCANNING;
    stats_.scanCount++;

    xSemaphoreGive(stateMutex_);

    // Set scan parameters
    esp_err_t ret = esp_ble_gap_set_scan_params(&default_scan_params);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to set scan params: %s", esp_err_to_name(ret));
        state_ = BLE_CLIENT_IDLE;
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEClient::stopScan() {
    ble_log(ESP_LOG_INFO, "Stopping BLE scan");
    
    return esp_ble_gap_stop_scanning();
}

esp_err_t BLEClient::connect(const esp_bd_addr_t serverMACadd) {
    if (state_ == BLE_CLIENT_CONNECTED || state_ == BLE_CLIENT_CONNECTING) {
        ble_log(ESP_LOG_WARN, "Already connected or connecting");
        return ESP_ERR_INVALID_STATE;
    }

    char MACaddStr[18];
    ble_addr_to_string((uint8_t*)serverMACadd, MACaddStr);
    ble_log(ESP_LOG_INFO, "Connecting to server: %s", MACaddStr);

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    state_ = BLE_CLIENT_CONNECTING;
    stats_.connectionAttempts++;
    connectionStartTime_ = ble_get_timestamp();
    
    // Copy server address
    memcpy(connectedDevice_.address, serverMACadd, ESP_BD_ADDR_LEN);

    xSemaphoreGive(stateMutex_);

    // Attempt connection
    esp_err_t ret = esp_ble_gattc_open(gattCInter_, (uint8_t*)serverMACadd, BLE_ADDR_TYPE_PUBLIC, true);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to initiate connection: %s", esp_err_to_name(ret));
        state_ = BLE_CLIENT_IDLE;
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEClient::disconnect(bool planned) {
    if (!isConnected()) {
        ble_log(ESP_LOG_WARN, "Not connected");
        return ESP_ERR_INVALID_STATE;
    }

    ble_log(ESP_LOG_INFO, "Disconnecting from server (planned: %s)", planned ? "yes" : "no");

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    state_ = BLE_CLIENT_DISCONNECTING;
    xSemaphoreGive(stateMutex_);

    esp_err_t ret = esp_ble_gattc_close(gattCInter_, connID_);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to disconnect: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEClient::readBatteryLevel() {
    if (!isConnected() || batteryCharHandle_ == BLE_INVALID_HANDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_log(ESP_LOG_DEBUG, "Reading battery level");

    esp_err_t ret = esp_ble_gattc_read_char(gattCInter_, connID_, batteryCharHandle_, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to read battery characteristic: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t BLEClient::readCustomData() {
    if (!isConnected() || customCharHandle_ == BLE_INVALID_HANDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_log(ESP_LOG_DEBUG, "Reading custom data");

    esp_err_t ret = esp_ble_gattc_read_char(gattCInter_, connID_, customCharHandle_, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to read custom characteristic: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t BLEClient::writeCustomData(const char* data) {
    if (!isConnected() || customCharHandle_ == BLE_INVALID_HANDLE || data == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t dataLen = strlen(data);
    if (dataLen > BLE_MAX_CUSTOM_DATA_LEN - 1) {
        ble_log(ESP_LOG_WARN, "Data too long, truncating");
        dataLen = BLE_MAX_CUSTOM_DATA_LEN - 1;
    }

    ble_log(ESP_LOG_DEBUG, "Writing custom data: %.*s", (int)dataLen, data);

    esp_err_t ret = esp_ble_gattc_write_char(gattCInter_, connID_, customCharHandle_,
                                           dataLen, (uint8_t*)data,
                                           ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to write custom characteristic: %s", esp_err_to_name(ret));
    } else {
        stats_.dataSent++;
    }
    
    return ret;
}

esp_err_t BLEClient::setNotifications(bool enable) {
    if (!isConnected()) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_log(ESP_LOG_INFO, "%s notifications", enable ? "Enabling" : "Disabling");
    
    
    return ESP_OK;
}

/******************************************************************************/
/*                           Configuration Methods                            */
/******************************************************************************/

esp_err_t BLEClient::setConfig(const ble_client_config_t& config) {
    if (isConnected()) {
        ble_log(ESP_LOG_WARN, "Cannot change config while connected");
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;
    ble_log(ESP_LOG_INFO, "Client configuration updated");
    return ESP_OK;
}

ble_client_config_t BLEClient::getConfig() const {
    return config_;
}

esp_err_t BLEClient::setTargetDevice(const char* deviceName) {
    if (deviceName == nullptr || !ble_validate_device_name(deviceName)) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(config_.targetDeviceName, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    config_.targetDeviceName[BLE_MAX_DEVICE_NAME_LEN - 1] = '\0';

    ble_log(ESP_LOG_INFO, "Target device set to: %s", config_.targetDeviceName);
    return ESP_OK;
}


/******************************************************************************/
/*                            Callback Registration                           */
/******************************************************************************/

void BLEClient::setConnectedCallback(ble_client_connected_cb_t callback) {
    connectedCB_ = callback;
    ble_log(ESP_LOG_DEBUG, "Connected callback registered");
}

void BLEClient::setDisconnectedCallback(ble_client_disconnected_cb_t callback) {
    disconnectedCB_ = callback;
    ble_log(ESP_LOG_DEBUG, "Disconnected callback registered");
}

void BLEClient::setDataReceivedCallback(ble_client_data_received_cb_t callback) {
    dataReceivedCB_ = callback;
    ble_log(ESP_LOG_DEBUG, "Data received callback registered");
}

void BLEClient::setDeviceFoundCallback(ble_client_device_found_cb_t callback) {
    deviceFoundCB_ = callback;
    ble_log(ESP_LOG_DEBUG, "Device found callback registered");
}

void BLEClient::setAuthCallback(ble_client_auth_cb_t callback) {
    authCB_ = callback;
    ble_log(ESP_LOG_DEBUG, "Authentication callback registered");
}

void BLEClient::setAnyDeviceFoundCallback(ble_client_any_device_found_cb_t callback) {
    anyDeviceFoundCB_ = callback;
    ble_log(ESP_LOG_DEBUG, "Any device found callback registered");
}

void BLEClient::setScanStartedCallback(ble_client_scan_started_cb_t callback) {
    scanStartedCB_ = callback;
    ble_log(ESP_LOG_DEBUG, "Scan started callback registered");
}

void BLEClient::setScanCompletedCallback(ble_client_scan_completed_cb_t callback) {
    scanCompletedCB_ = callback;
    ble_log(ESP_LOG_DEBUG, "Scan completed callback registered");
}


ble_client_state_t BLEClient::getState() const {
    return state_;
}

bool BLEClient::isConnected() const {
    return state_ == BLE_CLIENT_CONNECTED || state_ == BLE_CLIENT_READY || 
           state_ == BLE_CLIENT_AUTHENTICATING || state_ == BLE_CLIENT_DISCOVERING;
}

bool BLEClient::isScanning() const {
    return state_ == BLE_CLIENT_SCANNING;
}

const ble_device_info_t* BLEClient::getConnectedDevice() const {
    return isConnected() ? &connectedDevice_ : nullptr;
}

const ble_data_packet_t* BLEClient::getLastData() const {
    return lastData_.valid ? &lastData_ : nullptr;
}

ble_client_stats_t BLEClient::getStats() const {
    ble_client_stats_t stats = stats_;
    
    // Update uptime if connected
    if (isConnected() && connectionStartTime_ > 0) {
        stats.totalUptime = (ble_get_timestamp() - connectionStartTime_) / 1000;
    }
    
    return stats;
}

void BLEClient::resetStats() {
    memset(&stats_, 0, sizeof(stats_));
    ble_log(ESP_LOG_INFO, "Client statistics reset");
}

const char* BLEClient::getStateString() const {
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

uint8_t BLEClient::getConnectionQuality() const {
    if (!isConnected()) {
        return 0;
    }

    return ble_rssi_to_quality(connectedDevice_.rssi);
}

uint64_t BLEClient::getConnectionUptime() const {
    if (!isConnected() || connectionStartTime_ == 0) {
        return 0;
    }

    return (ble_get_timestamp() - connectionStartTime_) / 1000;
}

bool BLEClient::performHealthCheck() {
    if (state_ == BLE_CLIENT_IDLE) {
        return true;
    }
    
    if (isConnected() && lastDataTime_ > 0) {
        uint64_t timeSinceData = (ble_get_timestamp() - lastDataTime_) / 1000;
        if (timeSinceData > 30000) {  // 30 seconds
            ble_log(ESP_LOG_WARN, "No data received for %llu ms", timeSinceData);
            return false;
        }
    }
    
    return true;
}

/******************************************************************************/
/*                          NUEVOS MÉTODOS EXTENDIDOS                        */
/******************************************************************************/

esp_err_t BLEClient::startDiscoveryScan(uint32_t duration) {
    ble_log(ESP_LOG_INFO, "Starting discovery scan for %lu ms (all devices)", duration);

    if (state_ == BLE_CLIENT_SCANNING) {
        ble_log(ESP_LOG_WARN, "Already scanning");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    discoveryMode_ = true;
    currentScanDuration_ = duration;
    scanStartTime_ = ble_get_timestamp();
    targetDeviceFoundInScan_ = false;

    state_ = BLE_CLIENT_SCANNING;
    stats_.scanCount++;

    xSemaphoreGive(stateMutex_);

    if (scanStartedCB_ != nullptr) {
        scanStartedCB_(duration);
    }

    esp_err_t ret = esp_ble_gap_set_scan_params(&default_scan_params);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to set scan params: %s", esp_err_to_name(ret));
        state_ = BLE_CLIENT_IDLE;
        discoveryMode_ = false;
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEClient::startTargetedScan() {
    char macStr[18];
    ble_addr_to_string((uint8_t*)config_.targetServerMACadd, macStr);
    ble_log(ESP_LOG_INFO, "Starting targeted scan for MAC: %s", macStr);

    if (state_ == BLE_CLIENT_SCANNING) {
        ble_log(ESP_LOG_WARN, "Already scanning");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Configurar modo targeted (normal)
    discoveryMode_ = false;
    currentScanDuration_ = config_.scanTimeout;
    scanStartTime_ = ble_get_timestamp();
    targetDeviceFoundInScan_ = false;

    state_ = BLE_CLIENT_SCANNING;
    stats_.scanCount++;

    xSemaphoreGive(stateMutex_);

    ble_log(ESP_LOG_INFO, "Scan configured:");
    ble_log(ESP_LOG_INFO, "  Target MAC: %s", macStr);
    ble_log(ESP_LOG_INFO, "  Duration: %lu ms", config_.scanTimeout);

    if (scanStartedCB_ != nullptr) {
        scanStartedCB_(config_.scanTimeout);
    }

    esp_err_t ret = esp_ble_gap_set_scan_params(&default_scan_params);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to set scan params: %s", esp_err_to_name(ret));
        state_ = BLE_CLIENT_IDLE;
        discoveryMode_ = false;
        return ret;
    }

    ble_log(ESP_LOG_INFO, "Targeted MAC scan started successfully");
    return ESP_OK;
}

void BLEClient::setScanMode(bool discoveryMode) {
    discoveryMode_ = discoveryMode;
    ble_log(ESP_LOG_INFO, "Scan mode set to: %s", discoveryMode ? "DISCOVERY" : "TARGETED");
}

uint32_t BLEClient::getUniqueDevicesFound() const {
    return foundDevicesCount_;
}

void BLEClient::clearDeviceList() {
    foundDevicesCount_ = 0;
    memset(foundDevices_, 0, sizeof(foundDevices_));
    ble_log(ESP_LOG_INFO, "Device list cleared");
}

const ble_device_info_t* BLEClient::getFoundDevice(uint32_t index) const {
    if (index >= foundDevicesCount_) {
        return nullptr;
    }
    return &foundDevices_[index];
}

bool BLEClient::addToFoundDevices(const ble_device_info_t* deviceInfo) {
    if (deviceInfo == nullptr || foundDevicesCount_ >= MAX_FOUND_DEVICES) {
        return false;
    }

    if (isDeviceAlreadyFound(deviceInfo->address)) {
        return false; // Ya existe
    }

    // Agregar nuevo dispositivo
    memcpy(&foundDevices_[foundDevicesCount_], deviceInfo, sizeof(ble_device_info_t));
    foundDevicesCount_++;

    ble_log(ESP_LOG_DEBUG, "Added device to list: %s (total: %lu)", 
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
            ble_log(ESP_LOG_DEBUG, "Scan parameters set");
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                esp_ble_gap_start_scanning(instance_->currentScanDuration_ > 0 ? 
                                          instance_->currentScanDuration_ / 1000 : 
                                          instance_->config_.scanTimeout / 1000);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ble_log(ESP_LOG_ERROR, "Scan start failed: %d", param->scan_start_cmpl.status);
                instance_->state_ = BLE_CLIENT_IDLE;
            } else {
                ble_log(ESP_LOG_INFO, "Scan started successfully");
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            instance_->handleScanResult(param);
            break;
            
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ble_log(ESP_LOG_DEBUG, "Scan stopped");
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
            ble_log(ESP_LOG_DEBUG, "GATTC registered, app_id: %04x", param->reg.app_id);
            instance_->gattCInter_ = gattCInter;
            break;
            
        case ESP_GATTC_CONNECT_EVT:
            instance_->handleConnect(param);
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

    bool macMatch = (memcmp(scanResult->scan_rst.bda, config_.targetServerMACadd, ESP_BD_ADDR_LEN) == 0);
    
    if (securityConfig_.useCustomUUIDS && scanResult->scan_rst.adv_data_len > 0) {
        uint8_t *advData = scanResult->scan_rst.ble_adv;
        uint8_t advDataLen = scanResult->scan_rst.adv_data_len;

        for (int i = 0; i < advDataLen; ) {
            uint8_t length = advData[i];
            if (length == 0) break;

            uint8_t type = advData[i + 1];

            if (type == 0x07 && length == 17) {  // Complete list of 128-bit service UUIDs
                if (ble_compare_uuid128(&advData[i + 2], securityConfig_.serviceUUID)) {
                    hasTargetService = true;
                    break;
                }
            }
            
            i += length + 1;
        }
    }
    
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

    bool finalResult = macMatch && securityOk;

    // Debug logging simplificado - ONLY MAC
    char foundMac[18], targetMac[18];
    ble_addr_to_string(scanResult->scan_rst.bda, foundMac);
    ble_addr_to_string((uint8_t*)config_.targetServerMACadd, targetMac);

    ble_log(ESP_LOG_INFO, "=== MAC-Based Device Verification ===");
    ble_log(ESP_LOG_INFO, "Found MAC: %s", foundMac);
    ble_log(ESP_LOG_INFO, "Target MAC: %s", targetMac);
    ble_log(ESP_LOG_INFO, "RSSI: %d dBm", scanResult->scan_rst.rssi);
    ble_log(ESP_LOG_INFO, "Verification Results:");
    ble_log(ESP_LOG_INFO, "  MAC match: %s", macMatch ? "YES" : "NO");
    ble_log(ESP_LOG_INFO, "  Security OK: %s", securityOk ? "YES" : "NO");
    ble_log(ESP_LOG_INFO, "  FINAL RESULT: %s", finalResult ? "TARGET VERIFIED" : "NOT TARGET");
    ble_log(ESP_LOG_INFO, "====================================");

    return finalResult;
}

esp_err_t BLEClient::authenticateWithServer() {
    if (customCharHandle_ == BLE_INVALID_HANDLE || !securityConfig_.requireAuthentication) {
        return ESP_OK;
    }
    
    ble_log(ESP_LOG_INFO, "Authenticating with server");
    
    state_ = BLE_CLIENT_AUTHENTICATING;
    
    // Send authentication key
    esp_err_t ret = writeCustomData(securityConfig_.authKey);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to send authentication: %s", esp_err_to_name(ret));
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
            ble_device_info_t currentDevice;
            memset(&currentDevice, 0, sizeof(currentDevice));
            
            memcpy(currentDevice.address, scanResult->scan_rst.bda, ESP_BD_ADDR_LEN);
            currentDevice.rssi = scanResult->scan_rst.rssi;
            currentDevice.lastSeen = ble_get_timestamp();
            
            char deviceName[BLE_MAX_DEVICE_NAME_LEN] = {0};
            if (scanResult->scan_rst.adv_data_len > 0) {
                uint8_t *advData = scanResult->scan_rst.ble_adv;
                uint8_t advDataLen = scanResult->scan_rst.adv_data_len;

                for (int i = 0; i < advDataLen; ) {
                    uint8_t length = advData[i];
                    if (length == 0) break;

                    uint8_t type = advData[i + 1];

                    // Buscar nombre del dispositivo
                    if (type == 0x09 || type == 0x08) {  // Complete o shortened local name                     REVISAR!!!
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
            
            if (strlen(deviceName) == 0) {
                snprintf(deviceName, sizeof(deviceName), "Unknown Device");
            }

            strncpy(currentDevice.name, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);

            bool isNewDevice = addToFoundDevices(&currentDevice);

            // Verificar si es el dispositivo objetivo
            bool isTarget = verifyTargetDevice(scanResult);
            if (isTarget) {
                targetDeviceFoundInScan_ = true;
            }

            if (anyDeviceFoundCB_ != nullptr) {
                anyDeviceFoundCB_(&currentDevice, isTarget);
            }
            
            // Funcionalidad original: manejar dispositivo objetivo
            if (isTarget) {
                char MACaddStr[18];
                ble_addr_to_string(scanResult->scan_rst.bda, MACaddStr);

                ble_log(ESP_LOG_INFO, "Target device found: %s, RSSI: %d dBm",
                        MACaddStr, scanResult->scan_rst.rssi);

                // Actualizar información del dispositivo conectado
                memcpy(&connectedDevice_, &currentDevice, sizeof(ble_device_info_t));

                // Llamar callback original de dispositivo encontrado
                bool shouldConnect = true;
                if (deviceFoundCB_ != nullptr) {
                    deviceFoundCB_(&connectedDevice_, &shouldConnect);
                }
                
                // Solo conectar si no estamos en modo discovery y se debe conectar
                if (!discoveryMode_ && shouldConnect) {
                    esp_ble_gap_stop_scanning();
                    connect(scanResult->scan_rst.bda);
                }
            }
            break;
        }
        
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ble_log(ESP_LOG_INFO, "Scan completed - Found %lu unique devices (%s target)", 
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
    ble_log(ESP_LOG_INFO, "Connected to server, conn_id: %d", param->connect.conn_id);

    connID_ = param->connect.conn_id;
    state_ = BLE_CLIENT_CONNECTED;
    stats_.successfulConnections++;
    
    // Start service discovery
    ble_log(ESP_LOG_INFO, "Starting service discovery");
    state_ = BLE_CLIENT_DISCOVERING;
    esp_ble_gattc_search_service(gattCInter_, connID_, nullptr);
}

void BLEClient::handleDisconnect(esp_ble_gattc_cb_param_t *param) {
    ble_log(ESP_LOG_INFO, "Disconnected from server, reason: %d", param->disconnect.reason);
    
    // Update statistics
    stats_.disconnections++;
    if (connectionStartTime_ > 0) {
        stats_.totalUptime += (ble_get_timestamp() - connectionStartTime_) / 1000;
    }
    
    // Reset connection state
    state_ = BLE_CLIENT_IDLE;
    connID_ = 0;
    batteryCharHandle_ = BLE_INVALID_HANDLE;
    customCharHandle_ = BLE_INVALID_HANDLE;
    connectionStartTime_ = 0;

    // Call disconnection callback
    if (disconnectedCB_ != nullptr) {
        disconnectedCB_(param->disconnect.reason, false);
    }
    
    // Auto-reconnect if configured
    if (config_.autoReconnect) {
        ble_log(ESP_LOG_INFO, "Scheduling reconnection in %lu ms", config_.reconnectInterval);
        vTaskDelay(pdMS_TO_TICKS(config_.reconnectInterval));
        startTargetedScan();
    }
}

void BLEClient::handleServiceFound(esp_ble_gattc_cb_param_t *param) {
    ble_log(ESP_LOG_DEBUG, "Service found: UUID len %d", param->search_res.srvc_id.uuid.len);
    
    // Check if this is our target service
    bool serviceMatch = false;

    if (securityConfig_.useCustomUUIDS && param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
        serviceMatch = ble_compare_uuid128(param->search_res.srvc_id.uuid.uuid.uuid128, 
                                          securityConfig_.serviceUUID);
    } else if (!securityConfig_.useCustomUUIDS && param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
        serviceMatch = (param->search_res.srvc_id.uuid.uuid.uuid16 == BLE_DEFAULT_SERVICE_UUID_16);
    }
    
    if (serviceMatch) {
        ble_log(ESP_LOG_INFO, "Target service found");
        serviceStartHandle_ = param->search_res.start_handle;
        serviceEndHandle_ = param->search_res.end_handle;
    }
}

void BLEClient::handleServiceDiscoveryComplete(esp_ble_gattc_cb_param_t *param) {
    ble_log(ESP_LOG_INFO, "Service discovery completed");
    
    if (serviceStartHandle_ != 0) {
        ble_log(ESP_LOG_INFO, "Discovering characteristics");
        
        uint16_t count = 10;
        esp_gattc_char_elem_t charElemResult[10];
        uint16_t charCount = count;

        esp_err_t ret = esp_ble_gattc_get_all_char(gattCInter_, connID_, 
                                                  serviceStartHandle_, serviceEndHandle_,
                                                  charElemResult, &charCount, 0);
        
        if (ret == ESP_GATT_OK) {
            ble_log(ESP_LOG_INFO, "Found %d characteristics", charCount);
            
            for (int i = 0; i < charCount; i++) {
                esp_gattc_char_elem_t* char_elem = &charElemResult[i];
                
                bool isBattery = false;
                bool isCustom = false;
                if (securityConfig_.useCustomUUIDS && char_elem->uuid.len == ESP_UUID_LEN_128) {
                    isBattery = ble_compare_uuid128(char_elem->uuid.uuid.uuid128, 
                                                   securityConfig_.batteryCharUUID);
                    isCustom = ble_compare_uuid128(char_elem->uuid.uuid.uuid128, 
                                                  securityConfig_.customCharUUID);
                } else if (!securityConfig_.useCustomUUIDS && char_elem->uuid.len == ESP_UUID_LEN_16) {
                    isBattery = (char_elem->uuid.uuid.uuid16 == BLE_DEFAULT_BATTERY_CHAR_UUID);
                    isCustom = (char_elem->uuid.uuid.uuid16 == BLE_DEFAULT_CUSTOM_CHAR_UUID);
                }
                
                if (isBattery) {
                    batteryCharHandle_ = char_elem->char_handle;
                    ble_log(ESP_LOG_INFO, "Battery characteristic found: handle=%d", batteryCharHandle_);
                } else if (isCustom) {
                    customCharHandle_ = char_elem->char_handle;
                    ble_log(ESP_LOG_INFO, "Custom characteristic found: handle=%d", customCharHandle_);
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
            ble_log(ESP_LOG_ERROR, "Failed to get characteristics: %s", esp_err_to_name(ret));
        }
    } else {
        ble_log(ESP_LOG_ERROR, "Target service not found");
        disconnect(false);
    }
}

void BLEClient::handleCharacteristicFound(esp_ble_gattc_cb_param_t *param) {
    ble_log(ESP_LOG_DEBUG, "Characteristic event received");
}

void BLEClient::handleAllCharacteristicsFound(esp_ble_gattc_cb_param_t *param) {
    ble_log(ESP_LOG_DEBUG, "All characteristics event received");
}

void BLEClient::handleCharacteristicRead(esp_ble_gattc_cb_param_t *param) {
    if (param->read.status != ESP_GATT_OK) {
        ble_log(ESP_LOG_ERROR, "Read failed: %d", param->read.status);
        return;
    }
    
    stats_.dataReceived++;
    lastDataTime_ = ble_get_timestamp();

    if (param->read.handle == batteryCharHandle_) {
        lastData_.batteryLevel = param->read.value[0];
        ble_log(ESP_LOG_INFO, "Battery level: %d%%", lastData_.batteryLevel);
    } else if (param->read.handle == customCharHandle_) {
        memcpy(lastData_.customData, param->read.value, 
               param->read.value_len < BLE_MAX_CUSTOM_DATA_LEN ? param->read.value_len : BLE_MAX_CUSTOM_DATA_LEN - 1);
        lastData_.customData[param->read.value_len] = '\0';
        ble_log(ESP_LOG_INFO, "Custom data: %s", lastData_.customData);
    }

    lastData_.timeStamp = ble_get_timestamp();
    lastData_.valid = true;

    if (dataReceivedCB_ != nullptr) {
        dataReceivedCB_(&lastData_);
    }
}

void BLEClient::handleCharacteristicWrite(esp_ble_gattc_cb_param_t *param) {
    if (param->write.status != ESP_GATT_OK) {
        ble_log(ESP_LOG_ERROR, "Write failed: %d", param->write.status);
        return;
    }
    
    ble_log(ESP_LOG_DEBUG, "Write successful");
    
    if (state_ == BLE_CLIENT_AUTHENTICATING) {
        ble_log(ESP_LOG_INFO, "Authentication sent, waiting for response");
        readCustomData();
    }
}

void BLEClient::handleNotification(esp_ble_gattc_cb_param_t *param) {
    ble_log(ESP_LOG_DEBUG, "Notification received from handle: %d", param->notify.handle);
    
    stats_.dataReceived++;
    lastDataTime_ = ble_get_timestamp();

    if (param->notify.handle == batteryCharHandle_) {
        lastData_.batteryLevel = param->notify.value[0];
        ble_log(ESP_LOG_INFO, "Battery notification: %d%%", lastData_.batteryLevel);
    } else if (param->notify.handle == customCharHandle_) {
        memcpy(lastData_.customData, param->notify.value,
               param->notify.value_len < BLE_MAX_CUSTOM_DATA_LEN ? param->notify.value_len : BLE_MAX_CUSTOM_DATA_LEN - 1);
        lastData_.customData[param->notify.value_len] = '\0';
        ble_log(ESP_LOG_INFO, "Custom notification: %s", lastData_.customData);

        if (state_ == BLE_CLIENT_AUTHENTICATING && strcmp(lastData_.customData, "AUTH_OK") == 0) {
            ble_log(ESP_LOG_INFO, "Authentication successful");
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

    lastData_.timeStamp = ble_get_timestamp();
    lastData_.valid = true;

    if (dataReceivedCB_ != nullptr) {
        dataReceivedCB_(&lastData_);
    }
}

void BLEClient::dataReadTask(void *pvParameters) {
    BLEClient* client = static_cast<BLEClient*>(pvParameters);
    
    ble_log(ESP_LOG_INFO, "Data read task started");
    
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
    
    ble_log(ESP_LOG_INFO, "Data read task ended");
    client->dataReadTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}

void BLEClient::reconnectTask(void *pvParameters) {
    BLEClient* client = static_cast<BLEClient*>(pvParameters);
    
    ble_log(ESP_LOG_INFO, "Reconnect task started");
    
    while (client->config_.autoReconnect && !client->isConnected()) {
        vTaskDelay(pdMS_TO_TICKS(client->config_.reconnectInterval));

        if (!client->isConnected() && client->state_ == BLE_CLIENT_IDLE) {
            ble_log(ESP_LOG_INFO, "Attempting reconnection");
            client->startTargetedScan();
        }
    }
    
    ble_log(ESP_LOG_INFO, "Reconnect task ended");
    client->reconnectTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}