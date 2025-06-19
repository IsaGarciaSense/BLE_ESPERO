/*******************************************************************************
 * @file ble_client.cpp
 * @brief Implementation of BLE client functionality including scanning,
 * connecting, and data communication with BLE servers.
 *
 * @version 0.0.1
 * @date 2025-06-15
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/
#include "esp_random.h"

#include "BLEClient.hpp"

/******************************************************************************/
/*                              Static Variables                              */
/******************************************************************************/

static const char* BLE_CLIENT_TAG = "BLE_CLIENT";
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
    , gattc_if_(0)
    , conn_id_(0)
    , service_start_handle_(0)
    , service_end_handle_(0)
    , battery_char_handle_(BLE_INVALID_HANDLE)
    , custom_char_handle_(BLE_INVALID_HANDLE)
    , connected_cb_(nullptr)
    , disconnected_cb_(nullptr)
    , data_received_cb_(nullptr)
    , device_found_cb_(nullptr)
    , auth_cb_(nullptr)
    , data_read_task_handle_(nullptr)
    , reconnect_task_handle_(nullptr)
    , state_mutex_(nullptr)
    , connection_start_time_(0)
    , last_data_time_(0)
    // ✅ NUEVAS INICIALIZACIONES
    , any_device_found_cb_(nullptr)
    , scan_started_cb_(nullptr)
    , scan_completed_cb_(nullptr)
    , discovery_mode_(false)
    , current_scan_duration_(0)
    , scan_start_time_(0)
    , found_devices_count_(0)
    , target_device_found_in_scan_(false) {
    
    // Initialize default configuration
    memset(&config_, 0, sizeof(config_));
    strncpy(config_.target_device_name, "154_BLE_Server", BLE_MAX_DEVICE_NAME_LEN - 1);
    config_.scan_timeout_ms = BLE_DEFAULT_SCAN_TIMEOUT;
    config_.min_rssi = -90;
    config_.auto_reconnect = true;
    config_.reconnect_interval_ms = BLE_DEFAULT_RECONNECT_TIME;
    config_.connection_timeout_ms = 10000;
    config_.enable_notifications = true;
    config_.read_interval_ms = 5000;

    // Initialize security configuration
    ble_create_default_security_config(&security_config_, BLE_SECURITY_NONE);

    // Initialize device and data structures
    memset(&connected_device_, 0, sizeof(connected_device_));
    memset(&last_data_, 0, sizeof(last_data_));
    memset(&stats_, 0, sizeof(stats_));
    
    // ✅ NUEVA INICIALIZACIÓN: Lista de dispositivos encontrados
    memset(found_devices_, 0, sizeof(found_devices_));

    // Create mutex
    state_mutex_ = xSemaphoreCreateMutex();
    
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
    if (data_read_task_handle_ != nullptr) {
        vTaskDelete(data_read_task_handle_);
    }
    if (reconnect_task_handle_ != nullptr) {
        vTaskDelete(reconnect_task_handle_);
    }

    // Disconnect if connected
    if (isConnected()) {
        disconnect(true);
    }

    // Clean up mutex
    if (state_mutex_ != nullptr) {
        vSemaphoreDelete(state_mutex_);
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

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    // Register GAP callback
    ret = esp_ble_gap_register_callback(gapCallback);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        xSemaphoreGive(state_mutex_);
        return ret;
    }

    // Register GATTC callback
    ret = esp_ble_gattc_register_callback(gattcCallback);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to register GATTC callback: %s", esp_err_to_name(ret));
        xSemaphoreGive(state_mutex_);
        return ret;
    }

    // Register GATTC application
    ret = esp_ble_gattc_app_register(0x56);  // App ID
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to register GATTC app: %s", esp_err_to_name(ret));
        xSemaphoreGive(state_mutex_);
        return ret;
    }

    state_ = BLE_CLIENT_IDLE;
    xSemaphoreGive(state_mutex_);

    ble_log(ESP_LOG_INFO, "BLE Client initialized successfully");
    return ESP_OK;
}

esp_err_t BLEClient::startScan() {
    ble_log(ESP_LOG_INFO, "Starting BLE scan for target: %s", config_.target_device_name);

    if (state_ == BLE_CLIENT_SCANNING) {
        ble_log(ESP_LOG_WARN, "Already scanning");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    state_ = BLE_CLIENT_SCANNING;
    stats_.scan_count++;
    
    xSemaphoreGive(state_mutex_);

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

esp_err_t BLEClient::connect(const esp_bd_addr_t server_address) {
    if (state_ == BLE_CLIENT_CONNECTED || state_ == BLE_CLIENT_CONNECTING) {
        ble_log(ESP_LOG_WARN, "Already connected or connecting");
        return ESP_ERR_INVALID_STATE;
    }

    char addr_str[18];
    ble_addr_to_string((uint8_t*)server_address, addr_str);
    ble_log(ESP_LOG_INFO, "Connecting to server: %s", addr_str);

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    state_ = BLE_CLIENT_CONNECTING;
    stats_.connection_attempts++;
    connection_start_time_ = ble_get_timestamp();
    
    // Copy server address
    memcpy(connected_device_.address, server_address, ESP_BD_ADDR_LEN);
    
    xSemaphoreGive(state_mutex_);

    // Attempt connection
    esp_err_t ret = esp_ble_gattc_open(gattc_if_, (uint8_t*)server_address, BLE_ADDR_TYPE_PUBLIC, true);
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

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    state_ = BLE_CLIENT_DISCONNECTING;
    xSemaphoreGive(state_mutex_);

    esp_err_t ret = esp_ble_gattc_close(gattc_if_, conn_id_);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to disconnect: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEClient::readBatteryLevel() {
    if (!isConnected() || battery_char_handle_ == BLE_INVALID_HANDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_log(ESP_LOG_DEBUG, "Reading battery level");
    
    esp_err_t ret = esp_ble_gattc_read_char(gattc_if_, conn_id_, battery_char_handle_, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to read battery characteristic: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t BLEClient::readCustomData() {
    if (!isConnected() || custom_char_handle_ == BLE_INVALID_HANDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_log(ESP_LOG_DEBUG, "Reading custom data");
    
    esp_err_t ret = esp_ble_gattc_read_char(gattc_if_, conn_id_, custom_char_handle_, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to read custom characteristic: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t BLEClient::writeCustomData(const char* data) {
    if (!isConnected() || custom_char_handle_ == BLE_INVALID_HANDLE || data == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t data_len = strlen(data);
    if (data_len > BLE_MAX_CUSTOM_DATA_LEN - 1) {
        ble_log(ESP_LOG_WARN, "Data too long, truncating");
        data_len = BLE_MAX_CUSTOM_DATA_LEN - 1;
    }

    ble_log(ESP_LOG_DEBUG, "Writing custom data: %.*s", (int)data_len, data);
    
    esp_err_t ret = esp_ble_gattc_write_char(gattc_if_, conn_id_, custom_char_handle_, 
                                           data_len, (uint8_t*)data, 
                                           ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to write custom characteristic: %s", esp_err_to_name(ret));
    } else {
        stats_.data_packets_sent++;
    }
    
    return ret;
}

esp_err_t BLEClient::setNotifications(bool enable) {
    if (!isConnected()) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_log(ESP_LOG_INFO, "%s notifications", enable ? "Enabling" : "Disabling");
    
    // TODO: Implement notification enable/disable
    // This requires finding and writing to the CCCD (Client Characteristic Configuration Descriptor)
    
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

esp_err_t BLEClient::setTargetDevice(const char* device_name) {
    if (device_name == nullptr || !ble_validate_device_name(device_name)) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(config_.target_device_name, device_name, BLE_MAX_DEVICE_NAME_LEN - 1);
    config_.target_device_name[BLE_MAX_DEVICE_NAME_LEN - 1] = '\0';
    
    ble_log(ESP_LOG_INFO, "Target device set to: %s", config_.target_device_name);
    return ESP_OK;
}

esp_err_t BLEClient::setMinRssi(int8_t min_rssi) {
    if (min_rssi > -10 || min_rssi < -120) {
        return ESP_ERR_INVALID_ARG;
    }

    config_.min_rssi = min_rssi;
    ble_log(ESP_LOG_INFO, "Minimum RSSI set to: %d dBm", min_rssi);
    return ESP_OK;
}

/******************************************************************************/
/*                            Callback Registration                           */
/******************************************************************************/

void BLEClient::setConnectedCallback(ble_client_connected_cb_t callback) {
    connected_cb_ = callback;
    ble_log(ESP_LOG_DEBUG, "Connected callback registered");
}

void BLEClient::setDisconnectedCallback(ble_client_disconnected_cb_t callback) {
    disconnected_cb_ = callback;
    ble_log(ESP_LOG_DEBUG, "Disconnected callback registered");
}

void BLEClient::setDataReceivedCallback(ble_client_data_received_cb_t callback) {
    data_received_cb_ = callback;
    ble_log(ESP_LOG_DEBUG, "Data received callback registered");
}

void BLEClient::setDeviceFoundCallback(ble_client_device_found_cb_t callback) {
    device_found_cb_ = callback;
    ble_log(ESP_LOG_DEBUG, "Device found callback registered");
}

void BLEClient::setAuthCallback(ble_client_auth_cb_t callback) {
    auth_cb_ = callback;
    ble_log(ESP_LOG_DEBUG, "Authentication callback registered");
}

/******************************************************************************/
/*                          NUEVOS CALLBACKS EXTENDIDOS                      */
/******************************************************************************/

void BLEClient::setAnyDeviceFoundCallback(ble_client_any_device_found_cb_t callback) {
    any_device_found_cb_ = callback;
    ble_log(ESP_LOG_DEBUG, "Any device found callback registered");
}

void BLEClient::setScanStartedCallback(ble_client_scan_started_cb_t callback) {
    scan_started_cb_ = callback;
    ble_log(ESP_LOG_DEBUG, "Scan started callback registered");
}

void BLEClient::setScanCompletedCallback(ble_client_scan_completed_cb_t callback) {
    scan_completed_cb_ = callback;
    ble_log(ESP_LOG_DEBUG, "Scan completed callback registered");
}

/******************************************************************************/
/*                              Status Methods                                */
/******************************************************************************/

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
    return isConnected() ? &connected_device_ : nullptr;
}

const ble_data_packet_t* BLEClient::getLastData() const {
    return last_data_.is_valid ? &last_data_ : nullptr;
}

ble_client_stats_t BLEClient::getStats() const {
    ble_client_stats_t stats = stats_;
    
    // Update uptime if connected
    if (isConnected() && connection_start_time_ > 0) {
        stats.total_uptime_ms = (ble_get_timestamp() - connection_start_time_) / 1000;
    }
    
    return stats;
}

void BLEClient::resetStats() {
    memset(&stats_, 0, sizeof(stats_));
    ble_log(ESP_LOG_INFO, "Client statistics reset");
}

/******************************************************************************/
/*                              Utility Methods                               */
/******************************************************************************/

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
    
    return ble_rssi_to_quality(connected_device_.rssi);
}

uint64_t BLEClient::getConnectionUptime() const {
    if (!isConnected() || connection_start_time_ == 0) {
        return 0;
    }
    
    return (ble_get_timestamp() - connection_start_time_) / 1000;
}

bool BLEClient::performHealthCheck() {
    // Check basic state
    if (state_ == BLE_CLIENT_IDLE) {
        return true;
    }
    
    // Check if connected but haven't received data in a while
    if (isConnected() && last_data_time_ > 0) {
        uint64_t time_since_data = (ble_get_timestamp() - last_data_time_) / 1000;
        if (time_since_data > 30000) {  // 30 seconds
            ble_log(ESP_LOG_WARN, "No data received for %llu ms", time_since_data);
            return false;
        }
    }
    
    return true;
}

/******************************************************************************/
/*                          NUEVOS MÉTODOS EXTENDIDOS                        */
/******************************************************************************/

esp_err_t BLEClient::startDiscoveryScan(uint32_t duration_ms) {
    ble_log(ESP_LOG_INFO, "Starting discovery scan for %lu ms (all devices)", duration_ms);

    if (state_ == BLE_CLIENT_SCANNING) {
        ble_log(ESP_LOG_WARN, "Already scanning");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Configurar modo discovery
    discovery_mode_ = true;
    current_scan_duration_ = duration_ms;
    scan_start_time_ = ble_get_timestamp();
    target_device_found_in_scan_ = false;
    
    state_ = BLE_CLIENT_SCANNING;
    stats_.scan_count++;
    
    xSemaphoreGive(state_mutex_);

    // Llamar callback de inicio
    if (scan_started_cb_ != nullptr) {
        scan_started_cb_(duration_ms);
    }

    // Set scan parameters
    esp_err_t ret = esp_ble_gap_set_scan_params(&default_scan_params);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to set scan params: %s", esp_err_to_name(ret));
        state_ = BLE_CLIENT_IDLE;
        discovery_mode_ = false;
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEClient::startTargetedScan() {
    ble_log(ESP_LOG_INFO, "Starting targeted scan for: %s", config_.target_device_name);

    if (state_ == BLE_CLIENT_SCANNING) {
        ble_log(ESP_LOG_WARN, "Already scanning");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Configurar modo targeted (normal)
    discovery_mode_ = false;
    current_scan_duration_ = config_.scan_timeout_ms;
    scan_start_time_ = ble_get_timestamp();
    target_device_found_in_scan_ = false;
    
    state_ = BLE_CLIENT_SCANNING;
    stats_.scan_count++;
    
    xSemaphoreGive(state_mutex_);

    // Llamar callback de inicio
    if (scan_started_cb_ != nullptr) {
        scan_started_cb_(config_.scan_timeout_ms);
    }

    // Set scan parameters
    esp_err_t ret = esp_ble_gap_set_scan_params(&default_scan_params);
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to set scan params: %s", esp_err_to_name(ret));
        state_ = BLE_CLIENT_IDLE;
        discovery_mode_ = false;
        return ret;
    }

    return ESP_OK;
}

void BLEClient::setScanMode(bool discovery_mode) {
    discovery_mode_ = discovery_mode;
    ble_log(ESP_LOG_INFO, "Scan mode set to: %s", discovery_mode ? "DISCOVERY" : "TARGETED");
}

uint32_t BLEClient::getUniqueDevicesFound() const {
    return found_devices_count_;
}

void BLEClient::clearDeviceList() {
    found_devices_count_ = 0;
    memset(found_devices_, 0, sizeof(found_devices_));
    ble_log(ESP_LOG_INFO, "Device list cleared");
}

const ble_device_info_t* BLEClient::getFoundDevice(uint32_t index) const {
    if (index >= found_devices_count_) {
        return nullptr;
    }
    return &found_devices_[index];
}

bool BLEClient::addToFoundDevices(const ble_device_info_t* device_info) {
    if (device_info == nullptr || found_devices_count_ >= MAX_FOUND_DEVICES) {
        return false;
    }

    // Verificar si ya existe
    if (isDeviceAlreadyFound(device_info->address)) {
        return false; // Ya existe
    }

    // Agregar nuevo dispositivo
    memcpy(&found_devices_[found_devices_count_], device_info, sizeof(ble_device_info_t));
    found_devices_count_++;
    
    ble_log(ESP_LOG_DEBUG, "Added device to list: %s (total: %lu)", 
            device_info->name, found_devices_count_);
    return true;
}

bool BLEClient::isDeviceAlreadyFound(const esp_bd_addr_t address) const {
    for (uint32_t i = 0; i < found_devices_count_; i++) {
        if (memcmp(found_devices_[i].address, address, ESP_BD_ADDR_LEN) == 0) {
            return true;
        }
    }
    return false;
}

/******************************************************************************/
/*                              Internal Methods                              */
/******************************************************************************/

void BLEClient::gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (instance_ == nullptr) {
        return;
    }
    
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ble_log(ESP_LOG_DEBUG, "Scan parameters set");
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                esp_ble_gap_start_scanning(instance_->current_scan_duration_ > 0 ? 
                                          instance_->current_scan_duration_ / 1000 : 
                                          instance_->config_.scan_timeout_ms / 1000);
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

void BLEClient::gattcCallback(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, 
                             esp_ble_gattc_cb_param_t *param) {
    if (instance_ == nullptr) {
        return;
    }
    
    switch (event) {
        case ESP_GATTC_REG_EVT:
            ble_log(ESP_LOG_DEBUG, "GATTC registered, app_id: %04x", param->reg.app_id);
            instance_->gattc_if_ = gattc_if;
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

// Reemplaza la función verifyTargetDevice completa con esta versión corregida:

bool BLEClient::verifyTargetDevice(esp_ble_gap_cb_param_t *scan_result) {
    char device_name[BLE_MAX_DEVICE_NAME_LEN] = {0};
    bool has_target_service = false;
    bool name_match = false;
    
    // ✅ PRIMERO: Extraer nombre del dispositivo de los datos de advertising
    if (scan_result->scan_rst.adv_data_len > 0) {
        uint8_t *adv_data = scan_result->scan_rst.ble_adv;
        uint8_t adv_data_len = scan_result->scan_rst.adv_data_len;
        
        for (int i = 0; i < adv_data_len; ) {
            uint8_t length = adv_data[i];
            if (length == 0) break;
            
            uint8_t type = adv_data[i + 1];
            
            // Check device name FIRST
            if (type == 0x09 || type == 0x08) {  // Complete or shortened local name
                uint8_t name_len = length - 1;
                if (name_len > BLE_MAX_DEVICE_NAME_LEN - 1) {
                    name_len = BLE_MAX_DEVICE_NAME_LEN - 1;
                }
                memcpy(device_name, &adv_data[i + 2], name_len);
                device_name[name_len] = '\0';
                
                ble_log(ESP_LOG_DEBUG, "Extracted device name: '%s' (len=%d)", device_name, name_len);
            }
            // Check service UUIDs
            else if (security_config_.use_custom_uuids) {
                if (type == 0x07 && length == 17) {  // Complete list of 128-bit service UUIDs
                    if (ble_compare_uuid128(&adv_data[i + 2], security_config_.service_uuid)) {
                        has_target_service = true;
                        ble_log(ESP_LOG_DEBUG, "Found target service UUID");
                    }
                }
            }
            
            i += length + 1;
        }
    }
    
    // SEGUNDO: Debug logging para verificar la extracción
    ble_log(ESP_LOG_INFO, "=== Device Verification Debug ===");
    ble_log(ESP_LOG_INFO, "Extracted name: '%s'", device_name);
    ble_log(ESP_LOG_INFO, "Target name: '%s'", config_.target_device_name);
    ble_log(ESP_LOG_INFO, "Name length: %d", strlen(device_name));
    ble_log(ESP_LOG_INFO, "Target length: %d", strlen(config_.target_device_name));
    ble_log(ESP_LOG_INFO, "RSSI: %d dBm (min: %d)", scan_result->scan_rst.rssi, config_.min_rssi);
    ble_log(ESP_LOG_INFO, "Security level: %d", security_config_.level);
    ble_log(ESP_LOG_INFO, "Use custom UUIDs: %s", security_config_.use_custom_uuids ? "YES" : "NO");
    
    // TERCERO: Verificar name match
    if (strlen(device_name) > 0) {
        name_match = (strcmp(device_name, config_.target_device_name) == 0);
        ble_log(ESP_LOG_INFO, "String comparison result: %s", name_match ? "MATCH" : "NO MATCH");
    } else {
        ble_log(ESP_LOG_WARN, "Device name is empty!");
    }

    // Cuarto: Apply security level verification
    bool security_ok = false;
    switch (security_config_.level) {
        case BLE_SECURITY_NONE:
            security_ok = true;
            ble_log(ESP_LOG_DEBUG, "Security: NONE - always OK");
            break;
            
        case BLE_SECURITY_BASIC:
        case BLE_SECURITY_AUTHENTICATED:
        case BLE_SECURITY_ENCRYPTED:
            security_ok = (has_target_service || !security_config_.use_custom_uuids);
            ble_log(ESP_LOG_DEBUG, "Security: %s - %s", 
                    security_config_.use_custom_uuids ? "Custom UUIDs" : "Standard UUIDs",
                    security_ok ? "OK" : "FAIL");
            break;
            
        default:
            security_ok = false;
            ble_log(ESP_LOG_ERROR, "Unknown security level: %d", security_config_.level);
            break;
    }
    
    // ✅ QUINTO: Final result
    bool final_result = name_match && security_ok;
    
    ble_log(ESP_LOG_INFO, "Verification Summary:");
    ble_log(ESP_LOG_INFO, "  Name match: %s", name_match ? "✓" : "✗");
    ble_log(ESP_LOG_INFO, "  Security OK: %s", security_ok ? "✓" : "✗");
    ble_log(ESP_LOG_INFO, "  FINAL RESULT: %s", final_result ? "TARGET VERIFIED ✓" : "NOT TARGET ✗");
    ble_log(ESP_LOG_INFO, "==============================");
    
    return final_result;
}

esp_err_t BLEClient::authenticateWithServer() {
    if (custom_char_handle_ == BLE_INVALID_HANDLE || !security_config_.require_authentication) {
        return ESP_OK;
    }
    
    ble_log(ESP_LOG_INFO, "Authenticating with server");
    
    state_ = BLE_CLIENT_AUTHENTICATING;
    
    // Send authentication key
    esp_err_t ret = writeCustomData(security_config_.auth_key);
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
    esp_ble_gap_cb_param_t *scan_result = param;
    
    switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            // ✅ NUEVA FUNCIONALIDAD: Procesar TODOS los dispositivos encontrados
            
            // Extraer información del dispositivo
            ble_device_info_t current_device;
            memset(&current_device, 0, sizeof(current_device));
            
            // Copiar dirección MAC
            memcpy(current_device.address, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
            current_device.rssi = scan_result->scan_rst.rssi;
            current_device.last_seen = ble_get_timestamp();
            
            // Extraer nombre del dispositivo de los datos de advertising
            char device_name[BLE_MAX_DEVICE_NAME_LEN] = {0};
            if (scan_result->scan_rst.adv_data_len > 0) {
                uint8_t *adv_data = scan_result->scan_rst.ble_adv;
                uint8_t adv_data_len = scan_result->scan_rst.adv_data_len;
                
                for (int i = 0; i < adv_data_len; ) {
                    uint8_t length = adv_data[i];
                    if (length == 0) break;
                    
                    uint8_t type = adv_data[i + 1];
                    
                    // Buscar nombre del dispositivo
                    if (type == 0x09 || type == 0x08) {  // Complete o shortened local name
                        uint8_t name_len = length - 1;
                        if (name_len > BLE_MAX_DEVICE_NAME_LEN - 1) {
                            name_len = BLE_MAX_DEVICE_NAME_LEN - 1;
                        }
                        memcpy(device_name, &adv_data[i + 2], name_len);
                        device_name[name_len] = '\0';
                        break;
                    }
                    
                    i += length + 1;
                }
            }
            
            // Si no se encontró nombre, usar "Unknown Device"
            if (strlen(device_name) == 0) {
                snprintf(device_name, sizeof(device_name), "Unknown Device");
            }
            
            strncpy(current_device.name, device_name, BLE_MAX_DEVICE_NAME_LEN - 1);
            
            // ✅ NUEVA FUNCIONALIDAD: Agregar a lista de dispositivos encontrados
            bool is_new_device = addToFoundDevices(&current_device);
            
            // Verificar si es el dispositivo objetivo
            bool is_target = verifyTargetDevice(scan_result);
            if (is_target) {
                target_device_found_in_scan_ = true;
            }
            
            // ✅ NUEVA FUNCIONALIDAD: Llamar callback para CUALQUIER dispositivo
            if (any_device_found_cb_ != nullptr) {
                any_device_found_cb_(&current_device, is_target);
            }
            
            // Funcionalidad original: manejar dispositivo objetivo
            if (is_target) {
                char addr_str[18];
                ble_addr_to_string(scan_result->scan_rst.bda, addr_str);
                
                ble_log(ESP_LOG_INFO, "Target device found: %s, RSSI: %d dBm", 
                        addr_str, scan_result->scan_rst.rssi);
                
                // Actualizar información del dispositivo conectado
                memcpy(&connected_device_, &current_device, sizeof(ble_device_info_t));
                
                // Llamar callback original de dispositivo encontrado
                bool should_connect = true;
                if (device_found_cb_ != nullptr) {
                    device_found_cb_(&connected_device_, &should_connect);
                }
                
                // Solo conectar si no estamos en modo discovery y se debe conectar
                if (!discovery_mode_ && should_connect) {
                    esp_ble_gap_stop_scanning();
                    connect(scan_result->scan_rst.bda);
                }
            }
            break;
        }
        
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ble_log(ESP_LOG_INFO, "Scan completed - Found %lu unique devices (%s target)", 
                    found_devices_count_, target_device_found_in_scan_ ? "WITH" : "WITHOUT");
            
            if (state_ == BLE_CLIENT_SCANNING) {
                state_ = BLE_CLIENT_IDLE;
                
                // ✅ NUEVA FUNCIONALIDAD: Llamar callback de escaneo completado
                if (scan_completed_cb_ != nullptr) {
                    scan_completed_cb_(found_devices_count_, target_device_found_in_scan_);
                }
                
                // Auto-retry solo en modo targeted y si no se encontró el target
                if (!discovery_mode_ && config_.auto_reconnect && !target_device_found_in_scan_) {
                    vTaskDelay(pdMS_TO_TICKS(config_.reconnect_interval_ms));
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
    
    conn_id_ = param->connect.conn_id;
    state_ = BLE_CLIENT_CONNECTED;
    stats_.successful_connections++;
    
    // Start service discovery
    ble_log(ESP_LOG_INFO, "Starting service discovery");
    state_ = BLE_CLIENT_DISCOVERING;
    esp_ble_gattc_search_service(gattc_if_, conn_id_, nullptr);
}

void BLEClient::handleDisconnect(esp_ble_gattc_cb_param_t *param) {
    ble_log(ESP_LOG_INFO, "Disconnected from server, reason: %d", param->disconnect.reason);
    
    // Update statistics
    stats_.disconnections++;
    if (connection_start_time_ > 0) {
        stats_.total_uptime_ms += (ble_get_timestamp() - connection_start_time_) / 1000;
    }
    
    // Reset connection state
    state_ = BLE_CLIENT_IDLE;
    conn_id_ = 0;
    battery_char_handle_ = BLE_INVALID_HANDLE;
    custom_char_handle_ = BLE_INVALID_HANDLE;
    connection_start_time_ = 0;
    
    // Call disconnection callback
    if (disconnected_cb_ != nullptr) {
        disconnected_cb_(param->disconnect.reason, false);
    }
    
    // Auto-reconnect if configured
    if (config_.auto_reconnect) {
        ble_log(ESP_LOG_INFO, "Scheduling reconnection in %lu ms", config_.reconnect_interval_ms);
        vTaskDelay(pdMS_TO_TICKS(config_.reconnect_interval_ms));
        startTargetedScan();
    }
}

void BLEClient::handleServiceFound(esp_ble_gattc_cb_param_t *param) {
    ble_log(ESP_LOG_DEBUG, "Service found: UUID len %d", param->search_res.srvc_id.uuid.len);
    
    // Check if this is our target service
    bool service_match = false;
    
    if (security_config_.use_custom_uuids && param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
        service_match = ble_compare_uuid128(param->search_res.srvc_id.uuid.uuid.uuid128, 
                                          security_config_.service_uuid);
    } else if (!security_config_.use_custom_uuids && param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
        service_match = (param->search_res.srvc_id.uuid.uuid.uuid16 == BLE_DEFAULT_SERVICE_UUID_16);
    }
    
    if (service_match) {
        ble_log(ESP_LOG_INFO, "Target service found");
        service_start_handle_ = param->search_res.start_handle;
        service_end_handle_ = param->search_res.end_handle;
    }
}

void BLEClient::handleServiceDiscoveryComplete(esp_ble_gattc_cb_param_t *param) {
    ble_log(ESP_LOG_INFO, "Service discovery completed");
    
    if (service_start_handle_ != 0) {
        ble_log(ESP_LOG_INFO, "Discovering characteristics");
        
        // Usar la función correcta con todos los parámetros
        uint16_t count = 10;  // Máximo número de características a obtener
        esp_gattc_char_elem_t char_elem_result[10];
        uint16_t char_count = count;
        
        esp_err_t ret = esp_ble_gattc_get_all_char(gattc_if_, conn_id_, 
                                                  service_start_handle_, service_end_handle_,
                                                  char_elem_result, &char_count, 0);
        
        if (ret == ESP_GATT_OK) {
            ble_log(ESP_LOG_INFO, "Found %d characteristics", char_count);
            
            // Procesar las características encontradas
            for (int i = 0; i < char_count; i++) {
                esp_gattc_char_elem_t* char_elem = &char_elem_result[i];
                
                // Verificar si es característica de batería
                bool is_battery = false;
                bool is_custom = false;
                
                if (security_config_.use_custom_uuids && char_elem->uuid.len == ESP_UUID_LEN_128) {
                    is_battery = ble_compare_uuid128(char_elem->uuid.uuid.uuid128, 
                                                   security_config_.battery_char_uuid);
                    is_custom = ble_compare_uuid128(char_elem->uuid.uuid.uuid128, 
                                                  security_config_.custom_char_uuid);
                } else if (!security_config_.use_custom_uuids && char_elem->uuid.len == ESP_UUID_LEN_16) {
                    is_battery = (char_elem->uuid.uuid.uuid16 == BLE_DEFAULT_BATTERY_CHAR_UUID);
                    is_custom = (char_elem->uuid.uuid.uuid16 == BLE_DEFAULT_CUSTOM_CHAR_UUID);
                }
                
                if (is_battery) {
                    battery_char_handle_ = char_elem->char_handle;
                    ble_log(ESP_LOG_INFO, "Battery characteristic found: handle=%d", battery_char_handle_);
                } else if (is_custom) {
                    custom_char_handle_ = char_elem->char_handle;
                    ble_log(ESP_LOG_INFO, "Custom characteristic found: handle=%d", custom_char_handle_);
                }
            }
            
            // Continuar con autenticación o marcar como listo
            if (security_config_.require_authentication) {
                authenticateWithServer();
            } else {
                state_ = BLE_CLIENT_READY;
                
                if (connected_cb_ != nullptr) {
                    connected_cb_(&connected_device_);
                }
                
                // Iniciar tarea de lectura de datos
                if (config_.read_interval_ms > 0) {
                    xTaskCreate(dataReadTask, "ble_client_read", 4096, this, 5, &data_read_task_handle_);
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
    
    stats_.data_packets_received++;
    last_data_time_ = ble_get_timestamp();
    
    // Update data based on characteristic handle
    if (param->read.handle == battery_char_handle_) {
        last_data_.battery_level = param->read.value[0];
        ble_log(ESP_LOG_INFO, "Battery level: %d%%", last_data_.battery_level);
    } else if (param->read.handle == custom_char_handle_) {
        memcpy(last_data_.custom_data, param->read.value, 
               param->read.value_len < BLE_MAX_CUSTOM_DATA_LEN ? param->read.value_len : BLE_MAX_CUSTOM_DATA_LEN - 1);
        last_data_.custom_data[param->read.value_len] = '\0';
        ble_log(ESP_LOG_INFO, "Custom data: %s", last_data_.custom_data);
    }
    
    last_data_.timestamp = ble_get_timestamp();
    last_data_.is_valid = true;
    
    // Call data received callback
    if (data_received_cb_ != nullptr) {
        data_received_cb_(&last_data_);
    }
}

void BLEClient::handleCharacteristicWrite(esp_ble_gattc_cb_param_t *param) {
    if (param->write.status != ESP_GATT_OK) {
        ble_log(ESP_LOG_ERROR, "Write failed: %d", param->write.status);
        return;
    }
    
    ble_log(ESP_LOG_DEBUG, "Write successful");
    
    // Check if this was authentication
    if (state_ == BLE_CLIENT_AUTHENTICATING) {
        ble_log(ESP_LOG_INFO, "Authentication sent, waiting for response");
        // We should read the response or wait for notification
        readCustomData();
    }
}

void BLEClient::handleNotification(esp_ble_gattc_cb_param_t *param) {
    ble_log(ESP_LOG_DEBUG, "Notification received from handle: %d", param->notify.handle);
    
    stats_.data_packets_received++;
    last_data_time_ = ble_get_timestamp();
    
    // Process notification data similar to read
    if (param->notify.handle == battery_char_handle_) {
        last_data_.battery_level = param->notify.value[0];
        ble_log(ESP_LOG_INFO, "Battery notification: %d%%", last_data_.battery_level);
    } else if (param->notify.handle == custom_char_handle_) {
        memcpy(last_data_.custom_data, param->notify.value, 
               param->notify.value_len < BLE_MAX_CUSTOM_DATA_LEN ? param->notify.value_len : BLE_MAX_CUSTOM_DATA_LEN - 1);
        last_data_.custom_data[param->notify.value_len] = '\0';
        ble_log(ESP_LOG_INFO, "Custom notification: %s", last_data_.custom_data);
        
        // Check for authentication response
        if (state_ == BLE_CLIENT_AUTHENTICATING && strcmp(last_data_.custom_data, "AUTH_OK") == 0) {
            ble_log(ESP_LOG_INFO, "Authentication successful");
            state_ = BLE_CLIENT_READY;
            
            if (auth_cb_ != nullptr) {
                auth_cb_(true, ESP_OK);
            }
            
            if (connected_cb_ != nullptr) {
                connected_cb_(&connected_device_);
            }
            
            // Start data reading task
            if (config_.read_interval_ms > 0) {
                xTaskCreate(dataReadTask, "ble_client_read", 4096, this, 5, &data_read_task_handle_);
            }
        }
    }
    
    last_data_.timestamp = ble_get_timestamp();
    last_data_.is_valid = true;
    
    if (data_received_cb_ != nullptr) {
        data_received_cb_(&last_data_);
    }
}

/******************************************************************************/
/*                              Task Functions                                */
/******************************************************************************/

void BLEClient::dataReadTask(void *pvParameters) {
    BLEClient* client = static_cast<BLEClient*>(pvParameters);
    
    ble_log(ESP_LOG_INFO, "Data read task started");
    
    while (client->isConnected()) {
        // Read battery level
        if (client->battery_char_handle_ != BLE_INVALID_HANDLE) {
            client->readBatteryLevel();
            vTaskDelay(pdMS_TO_TICKS(1000));  // Wait between reads
        }
        
        // Read custom data
        if (client->custom_char_handle_ != BLE_INVALID_HANDLE) {
            client->readCustomData();
        }
        
        vTaskDelay(pdMS_TO_TICKS(client->config_.read_interval_ms));
    }
    
    ble_log(ESP_LOG_INFO, "Data read task ended");
    client->data_read_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}

void BLEClient::reconnectTask(void *pvParameters) {
    BLEClient* client = static_cast<BLEClient*>(pvParameters);
    
    ble_log(ESP_LOG_INFO, "Reconnect task started");
    
    while (client->config_.auto_reconnect && !client->isConnected()) {
        vTaskDelay(pdMS_TO_TICKS(client->config_.reconnect_interval_ms));
        
        if (!client->isConnected() && client->state_ == BLE_CLIENT_IDLE) {
            ble_log(ESP_LOG_INFO, "Attempting reconnection");
            client->startTargetedScan();
        }
    }
    
    ble_log(ESP_LOG_INFO, "Reconnect task ended");
    client->reconnect_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}