/*******************************************************************************
 * @file BLEServer.cpp
 * @brief Implementation of BLE server functionality including advertising,
 * client management, service provisioning, and JSON command processing.
 *
 * @version 0.0.2
 * @date 2025-06-19
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "BLEServer.hpp"
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
static esp_ble_adv_params_t adv_params_ = {
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

static const ble_server_config_t default_server_config = {
    .device_name = "ESP32-BLE-Server",
    .advertising_interval_ms = 1000,
    .auto_start_advertising = true,
    .data_update_interval_ms = 5000,
    .simulate_battery_drain = true,
    .max_clients = 4,
    .client_timeout_ms = 30000,
    .require_authentication = false,
    .enable_notifications = true,
    .enable_json_commands = true,
    .max_json_size = 512
};

/******************************************************************************/
/*                                Constructor/Destructor                      */
/******************************************************************************/

BLEServer::BLEServer() : BLEServer(default_server_config) {
}

BLEServer::BLEServer(const ble_server_config_t& config) :
    config_(config),
    state_(BLE_SERVER_IDLE),
    gatts_if_(ESP_GATT_IF_NONE),
    service_handle_(BLE_INVALID_HANDLE),
    battery_char_handle_(BLE_INVALID_HANDLE),
    custom_char_handle_(BLE_INVALID_HANDLE),
    battery_descr_handle_(BLE_INVALID_HANDLE),
    custom_descr_handle_(BLE_INVALID_HANDLE),
    max_clients_(config.max_clients),
    client_connected_cb_(nullptr),
    client_disconnected_cb_(nullptr),
    data_written_cb_(nullptr),
    data_read_cb_(nullptr),
    client_auth_cb_(nullptr),
    advertising_cb_(nullptr),
    json_command_cb_(nullptr),
    data_update_task_handle_(nullptr),
    client_timeout_task_handle_(nullptr),
    clients_mutex_(nullptr),
    data_mutex_(nullptr),
    battery_level_(100)
{
    // Initialize static instance
    instance_ = this;
    
    // Initialize client sessions
    memset(client_sessions_, 0, sizeof(client_sessions_));
    
    // Initialize status
    memset(&status_, 0, sizeof(status_));
    status_.state = state_;
    status_.battery_level = battery_level_;
    status_.json_processing_enabled = config_.enable_json_commands;
    strcpy(status_.custom_data, "Hello from ESP32!");
    
    // Initialize statistics
    memset(&stats_, 0, sizeof(stats_));
    stats_.start_time = ble_get_timestamp();
    
    // Initialize custom data
    strcpy(custom_data_, status_.custom_data);
    
    // Create default security configuration
    ble_create_default_security_config(&security_config_, BLE_SECURITY_BASIC);
    
    ESP_LOGI(TAG, "BLEServer created with device name: %s", config_.device_name);
    ESP_LOGI(TAG, "JSON commands: %s", config_.enable_json_commands ? "ENABLED" : "DISABLED");
}

BLEServer::~BLEServer() {
    // Stop all tasks
    if (data_update_task_handle_) {
        vTaskDelete(data_update_task_handle_);
    }
    if (client_timeout_task_handle_) {
        vTaskDelete(client_timeout_task_handle_);
    }
    
    // Delete mutexes
    if (clients_mutex_) {
        vSemaphoreDelete(clients_mutex_);
    }
    if (data_mutex_) {
        vSemaphoreDelete(data_mutex_);
    }
    
    // Disconnect all clients
    disconnectAllClients();
    
    // Stop advertising
    stopAdvertising();
    
    ESP_LOGI(TAG, "BLEServer destroyed");
    instance_ = nullptr;
}

/******************************************************************************/
/*                              Initialization                                */
/******************************************************************************/

esp_err_t BLEServer::init() {
    ESP_LOGI(TAG, "Initializing BLE server...");
    
    // Create mutexes
    clients_mutex_ = xSemaphoreCreateMutex();
    data_mutex_ = xSemaphoreCreateMutex();
    
    if (!clients_mutex_ || !data_mutex_) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize common BLE
    esp_err_t ret = ble_common_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE common: %s", esp_err_to_name(ret));
        return ret;
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
    ret = esp_ble_gap_set_device_name(config_.device_name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set device name: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure advertising data
    if (security_config_.use_custom_uuids) {
        adv_data_.p_service_uuid = (uint8_t*)security_config_.service_uuid;
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
    setAdvertisingInterval(config_.advertising_interval_ms);
    
    // Start tasks only if intervals are set
    if (config_.data_update_interval_ms > 0) {
        xTaskCreate(dataUpdateTask, "ble_server_data", 4096, this, 5, &data_update_task_handle_);
    }
    
    if (config_.client_timeout_ms > 0) {
        xTaskCreate(clientTimeoutTask, "ble_server_timeout", 4096, this, 3, &client_timeout_task_handle_);
    }
    
    state_ = BLE_SERVER_READY;
    status_.state = state_;
    
    ESP_LOGI(TAG, "BLE server initialized successfully");
    
    // Auto-start advertising if configured
    if (config_.auto_start_advertising) {
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
    
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
        return ret;
    }
    
    state_ = BLE_SERVER_ADVERTISING;
    status_.state = state_;
    status_.advertising_active = true;
    stats_.advertising_cycles++;
    
    ESP_LOGI(TAG, "Started advertising");
    
    if (advertising_cb_) {
        advertising_cb_(true, ESP_OK);
    }
    
    return ESP_OK;
}

esp_err_t BLEServer::stopAdvertising() {
    if (!status_.advertising_active) {
        return ESP_OK;  // Already stopped
    }
    
    esp_err_t ret = esp_ble_gap_stop_advertising();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop advertising: %s", esp_err_to_name(ret));
        return ret;
    }
    
    status_.advertising_active = false;
    
    // Update state based on connections
    if (hasConnectedClients()) {
        state_ = BLE_SERVER_CONNECTED;
    } else {
        state_ = BLE_SERVER_READY;
    }
    status_.state = state_;
    
    ESP_LOGI(TAG, "Stopped advertising");
    
    if (advertising_cb_) {
        advertising_cb_(false, ESP_OK);
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
    
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        battery_level_ = level;
        status_.battery_level = level;
        status_.last_update_time = ble_get_timestamp();
        xSemaphoreGive(data_mutex_);
        
        ESP_LOGD(TAG, "Battery level updated to %d%%", level);
        
        // Notify connected clients if notifications are enabled
        if (config_.enable_notifications) {
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
    
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        strncpy(custom_data_, data, BLE_MAX_CUSTOM_DATA_LEN - 1);
        custom_data_[BLE_MAX_CUSTOM_DATA_LEN - 1] = '\0';
        
        strncpy(status_.custom_data, custom_data_, BLE_MAX_CUSTOM_DATA_LEN - 1);
        status_.custom_data[BLE_MAX_CUSTOM_DATA_LEN - 1] = '\0';
        status_.last_update_time = ble_get_timestamp();
        
        xSemaphoreGive(data_mutex_);
        
        ESP_LOGD(TAG, "Custom data updated: %s", custom_data_);
        
        // Notify connected clients if notifications are enabled
        if (config_.enable_notifications) {
            notifyAllClients();
        }
        
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

/******************************************************************************/
/*                              JSON Response Methods                         */
/******************************************************************************/

esp_err_t BLEServer::sendJsonResponse(uint16_t conn_id, const char* json_response) {
    if (!json_response) {
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t json_len = strlen(json_response);
    const size_t chunk_size = 200; // Conservative size for BLE
    
    ESP_LOGI(TAG, "📤 Sending JSON response to client %d (%zu bytes)", conn_id, json_len);
    ESP_LOGD(TAG, "JSON: %s", json_response);
    
    if (json_len <= chunk_size) {
        // Send in one piece if small enough
        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_, conn_id, custom_char_handle_,
                                                   json_len, (uint8_t*)json_response, false);
        if (ret == ESP_OK) {
            stats_.data_packets_sent++;
        }
        return ret;
    } else {
        // Send in chunks
        esp_err_t result = ESP_OK;
        for (size_t i = 0; i < json_len; i += chunk_size) {
            size_t copy_len = (json_len - i < chunk_size) ? json_len - i : chunk_size;
            
            esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_, conn_id, custom_char_handle_,
                                                       copy_len, (uint8_t*)(json_response + i), false);
            if (ret == ESP_OK) {
                stats_.data_packets_sent++;
                vTaskDelay(pdMS_TO_TICKS(50)); // Small delay between chunks
            } else {
                result = ret;
                break;
            }
        }
        return result;
    }
}

esp_err_t BLEServer::sendJsonResponseToAll(const char* json_response) {
    if (!json_response) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t result = ESP_OK;
    
    if (xSemaphoreTake(clients_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < max_clients_; i++) {
            if (client_sessions_[i].conn_id != 0) {
                esp_err_t ret = sendJsonResponse(client_sessions_[i].conn_id, json_response);
                if (ret != ESP_OK) {
                    result = ret;
                }
            }
        }
        xSemaphoreGive(clients_mutex_);
    }
    
    return result;
}

esp_err_t BLEServer::notifyAllClients() {
    esp_err_t result = ESP_OK;
    
    if (xSemaphoreTake(clients_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < max_clients_; i++) {
            if (client_sessions_[i].conn_id != 0 && client_sessions_[i].notifications_enabled) {
                esp_err_t ret = notifyClient(client_sessions_[i].conn_id, "both");
                if (ret != ESP_OK) {
                    result = ret;
                }
            }
        }
        xSemaphoreGive(clients_mutex_);
    }
    
    return result;
}

esp_err_t BLEServer::notifyClient(uint16_t conn_id, const char* characteristic_type) {
    if (!characteristic_type) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t result = ESP_OK;
    
    // Notify battery characteristic
    if (strcmp(characteristic_type, "battery") == 0 || strcmp(characteristic_type, "both") == 0) {
        if (battery_char_handle_ != BLE_INVALID_HANDLE) {
            esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_, conn_id, battery_char_handle_,
                                                       sizeof(battery_level_), &battery_level_, false);
            if (ret == ESP_OK) {
                stats_.data_packets_sent++;
                
                // Update client session stats
                if (xSemaphoreTake(clients_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
                    for (int i = 0; i < max_clients_; i++) {
                        if (client_sessions_[i].conn_id == conn_id) {
                            client_sessions_[i].data_packets_sent++;
                            client_sessions_[i].last_activity = ble_get_timestamp();
                            break;
                        }
                    }
                    xSemaphoreGive(clients_mutex_);
                }
            } else {
                result = ret;
            }
        }
    }
    
    // Notify custom characteristic
    if (strcmp(characteristic_type, "custom") == 0 || strcmp(characteristic_type, "both") == 0) {
        if (custom_char_handle_ != BLE_INVALID_HANDLE) {
            esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_, conn_id, custom_char_handle_,
                                                       strlen(custom_data_), (uint8_t*)custom_data_, false);
            if (ret == ESP_OK) {
                stats_.data_packets_sent++;
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

void BLEServer::processJsonData(uint16_t conn_id, const uint8_t* data, uint16_t length) {
    if (!config_.enable_json_commands || !data || length == 0) {
        return;
    }
    
    // Find client session
    ble_client_session_t* session = nullptr;
    if (xSemaphoreTake(clients_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < max_clients_; i++) {
            if (client_sessions_[i].conn_id == conn_id) {
                session = &client_sessions_[i];
                break;
            }
        }
        xSemaphoreGive(clients_mutex_);
    }
    
    if (!session) {
        ESP_LOGW(TAG, "Session not found for client %d", conn_id);
        return;
    }
    
    // Accumulate data in client's JSON buffer
    for (uint16_t i = 0; i < length; i++) {
        if (session->json_buffer_pos < sizeof(session->json_buffer) - 1) {
            session->json_buffer[session->json_buffer_pos++] = data[i];
            
            // Check for complete JSON (look for closing brace)
            if (data[i] == '}') {
                session->json_buffer[session->json_buffer_pos] = '\0';
                
                // Process the complete JSON command
                processJsonCommand(conn_id, session->json_buffer);
                
                // Reset buffer
                session->json_buffer_pos = 0;
                memset(session->json_buffer, 0, sizeof(session->json_buffer));
            }
        } else {
            // Buffer overflow, reset
            ESP_LOGW(TAG, "JSON buffer overflow for client %d", conn_id);
            session->json_buffer_pos = 0;
            memset(session->json_buffer, 0, sizeof(session->json_buffer));
        }
    }
}

void BLEServer::processJsonCommand(uint16_t conn_id, const char* json_string) {
    if (!json_string) {
        return;
    }
    
    ESP_LOGI(TAG, "📥 Processing JSON command from client %d: %s", conn_id, json_string);
    
    stats_.json_commands_processed++;
    
    // Call user callback if registered
    if (json_command_cb_) {
        ble_json_command_t command;
        command.conn_id = conn_id;
        strncpy(command.command_json, json_string, sizeof(command.command_json) - 1);
        command.command_json[sizeof(command.command_json) - 1] = '\0';
        command.timestamp = ble_get_timestamp();
        
        bool handled = json_command_cb_(&command);
        if (!handled) {
            ESP_LOGW(TAG, "JSON command not handled by user callback");
            stats_.json_commands_failed++;
        }
    } else {
        ESP_LOGW(TAG, "No JSON command callback registered");
        stats_.json_commands_failed++;
    }
}

/******************************************************************************/
/*                              Client Management                             */
/******************************************************************************/

esp_err_t BLEServer::disconnectClient(uint16_t conn_id) {
    esp_err_t ret = esp_ble_gatts_close(gatts_if_, conn_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect client %d: %s", conn_id, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Disconnecting client %d", conn_id);
    return ESP_OK;
}

esp_err_t BLEServer::disconnectAllClients() {
    esp_err_t result = ESP_OK;
    
    if (xSemaphoreTake(clients_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < max_clients_; i++) {
            if (client_sessions_[i].conn_id != 0) {
                esp_err_t ret = disconnectClient(client_sessions_[i].conn_id);
                if (ret != ESP_OK) {
                    result = ret;
                }
            }
        }
        xSemaphoreGive(clients_mutex_);
    }
    
    return result;
}

esp_err_t BLEServer::addClientSession(uint16_t conn_id, const esp_bd_addr_t client_addr) {
    if (xSemaphoreTake(clients_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Find empty slot
        for (int i = 0; i < max_clients_; i++) {
            if (client_sessions_[i].conn_id == 0) {
                client_sessions_[i].conn_id = conn_id;
                memcpy(client_sessions_[i].address, client_addr, sizeof(esp_bd_addr_t));
                client_sessions_[i].authenticated = !config_.require_authentication;
                client_sessions_[i].connect_time = ble_get_timestamp();
                client_sessions_[i].last_activity = client_sessions_[i].connect_time;
                client_sessions_[i].data_packets_sent = 0;
                client_sessions_[i].data_packets_received = 0;
                client_sessions_[i].notifications_enabled = config_.enable_notifications;
                client_sessions_[i].json_buffer_pos = 0;
                memset(client_sessions_[i].json_buffer, 0, sizeof(client_sessions_[i].json_buffer));
                
                status_.connected_clients++;
                stats_.total_connections++;
                stats_.current_clients++;
                
                xSemaphoreGive(clients_mutex_);
                
                ESP_LOGI(TAG, "Added client session %d (total clients: %d)", conn_id, status_.connected_clients);
                return ESP_OK;
            }
        }
        xSemaphoreGive(clients_mutex_);
        
        ESP_LOGW(TAG, "No available slots for new client %d", conn_id);
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t BLEServer::removeClientSession(uint16_t conn_id) {
    if (xSemaphoreTake(clients_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < max_clients_; i++) {
            if (client_sessions_[i].conn_id == conn_id) {
                memset(&client_sessions_[i], 0, sizeof(ble_client_session_t));
                status_.connected_clients--;
                stats_.current_clients--;
                
                xSemaphoreGive(clients_mutex_);
                
                ESP_LOGI(TAG, "Removed client session %d (remaining clients: %d)", conn_id, status_.connected_clients);
                
                // Update server state
                if (status_.connected_clients == 0) {
                    if (status_.advertising_active) {
                        state_ = BLE_SERVER_ADVERTISING;
                    } else {
                        state_ = BLE_SERVER_READY;
                    }
                    status_.state = state_;
                }
                
                return ESP_OK;
            }
        }
        xSemaphoreGive(clients_mutex_);
    }
    
    ESP_LOGW(TAG, "Client session %d not found", conn_id);
    return ESP_ERR_NOT_FOUND;
}

/******************************************************************************/
/*                              Configuration                                 */
/******************************************************************************/

esp_err_t BLEServer::setConfig(const ble_server_config_t& config) {
    config_ = config;
    max_clients_ = config.max_clients;
    status_.json_processing_enabled = config.enable_json_commands;
    
    ESP_LOGI(TAG, "Configuration updated");
    ESP_LOGI(TAG, "JSON commands: %s", config_.enable_json_commands ? "ENABLED" : "DISABLED");
    return ESP_OK;
}

ble_server_config_t BLEServer::getConfig() const {
    return config_;
}

esp_err_t BLEServer::setDeviceName(const char* device_name) {
    if (!device_name || strlen(device_name) >= BLE_MAX_DEVICE_NAME_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    
    strcpy(config_.device_name, device_name);
    
    esp_err_t ret = esp_ble_gap_set_device_name(device_name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set device name: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Device name updated to: %s", device_name);
    return ESP_OK;
}

esp_err_t BLEServer::setAdvertisingInterval(uint32_t interval_ms) {
    config_.advertising_interval_ms = interval_ms;
    
    // Convert to BLE units (0.625ms units)
    uint16_t interval = (uint16_t)(interval_ms * 1000 / 625);
    
    adv_params_.adv_int_min = interval;
    adv_params_.adv_int_max = interval + 10;  // Small range for consistent timing
    
    ESP_LOGI(TAG, "Advertising interval updated to %ld ms", interval_ms);
    return ESP_OK;
}

/******************************************************************************/
/*                              Callback Registration                         */
/******************************************************************************/

void BLEServer::setClientConnectedCallback(ble_server_client_connected_cb_t callback) {
    client_connected_cb_ = callback;
}

void BLEServer::setClientDisconnectedCallback(ble_server_client_disconnected_cb_t callback) {
    client_disconnected_cb_ = callback;
}

void BLEServer::setDataWrittenCallback(ble_server_data_written_cb_t callback) {
    data_written_cb_ = callback;
}

void BLEServer::setDataReadCallback(ble_server_data_read_cb_t callback) {
    data_read_cb_ = callback;
}

void BLEServer::setClientAuthCallback(ble_server_client_auth_cb_t callback) {
    client_auth_cb_ = callback;
}

void BLEServer::setAdvertisingCallback(ble_server_advertising_cb_t callback) {
    advertising_cb_ = callback;
}

void BLEServer::setJsonCommandCallback(ble_server_json_command_cb_t callback) {
    json_command_cb_ = callback;
    ESP_LOGI(TAG, "JSON command callback %s", callback ? "registered" : "unregistered");
}

/******************************************************************************/
/*                              Status Methods                                */
/******************************************************************************/

ble_server_state_t BLEServer::getState() const {
    return state_;
}

bool BLEServer::isAdvertising() const {
    return status_.advertising_active;
}

bool BLEServer::hasConnectedClients() const {
    return status_.connected_clients > 0;
}

uint8_t BLEServer::getConnectedClientCount() const {
    return status_.connected_clients;
}

ble_server_status_t BLEServer::getStatus() const {
    return status_;
}

const ble_client_session_t* BLEServer::getClientSession(uint16_t conn_id) const {
    if (xSemaphoreTake(clients_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < max_clients_; i++) {
            if (client_sessions_[i].conn_id == conn_id) {
                xSemaphoreGive(clients_mutex_);
                return &client_sessions_[i];
            }
        }
        xSemaphoreGive(clients_mutex_);
    }
    
    return nullptr;
}

uint8_t BLEServer::getAllClientSessions(ble_client_session_t* sessions, uint8_t max_sessions) const {
    if (!sessions || max_sessions == 0) {
        return 0;
    }
    
    uint8_t count = 0;
    
    if (xSemaphoreTake(clients_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < max_clients_ && count < max_sessions; i++) {
            if (client_sessions_[i].conn_id != 0) {
                sessions[count] = client_sessions_[i];
                count++;
            }
        }
        xSemaphoreGive(clients_mutex_);
    }
    
    return count;
}

ble_server_stats_t BLEServer::getStats() const {
    ble_server_stats_t stats = stats_;
    stats.total_uptime_ms = (ble_get_timestamp() - stats_.start_time) / 1000;  // Convert to ms
    return stats;
}

void BLEServer::resetStats() {
    memset(&stats_, 0, sizeof(stats_));
    stats_.start_time = ble_get_timestamp();
    ESP_LOGI(TAG, "Statistics reset");
}

/******************************************************************************/
/*                              Utility Methods                               */
/******************************************************************************/

const char* BLEServer::getStateString() const {
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
    return (ble_get_timestamp() - stats_.start_time) / 1000;  // Convert to ms
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
    if (!clients_mutex_ || !data_mutex_) {
        ESP_LOGE(TAG, "Health check: Missing mutexes");
        healthy = false;
    }
    
    ESP_LOGI(TAG, "Health check: %s (JSON commands: %s)", 
             healthy ? "PASS" : "FAIL",
             config_.enable_json_commands ? "ON" : "OFF");
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
    
    ble_server_stats_t stats = getStats();
    
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
        getStateString(),
        config_.device_name,
        status_.advertising_active ? "Active" : "Inactive",
        status_.connected_clients, max_clients_,
        status_.battery_level,
        status_.custom_data,
        stats.total_uptime_ms,
        stats.total_connections,
        stats.data_packets_sent,
        stats.data_packets_received,
        stats.json_commands_processed,
        stats.json_commands_failed,
        config_.enable_json_commands ? "ENABLED" : "DISABLED",
        esp_get_free_heap_size()
    );
    
    if (written >= (int)buffer_size) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    return ESP_OK;
}

/******************************************************************************/
/*                              Internal Tasks                                */
/******************************************************************************/

void BLEServer::dataUpdateTask(void *pvParameters) {
    BLEServer* server = static_cast<BLEServer*>(pvParameters);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(server->config_.data_update_interval_ms);
    
    ESP_LOGI(TAG, "Data update task started (interval: %ld ms)", server->config_.data_update_interval_ms);
    
    while (true) {
        vTaskDelayUntil(&last_wake_time, task_period);
        
        // Simulate battery drain if enabled
        if (server->config_.simulate_battery_drain) {
            uint8_t current_battery = server->battery_level_;
            if (current_battery > 0) {
                // Simulate 1% drain every update cycle
                server->setBatteryLevel(current_battery - 1);
            } else {
                // Reset to 100% when battery is empty
                server->setBatteryLevel(100);
            }
        }
        
        // Update custom data with timestamp
        char timestamp_data[BLE_MAX_CUSTOM_DATA_LEN];
        snprintf(timestamp_data, sizeof(timestamp_data), "Data-%lld", ble_get_timestamp() / 1000000);
        server->setCustomData(timestamp_data);
    }
}

void BLEServer::clientTimeoutTask(void *pvParameters) {
    BLEServer* server = static_cast<BLEServer*>(pvParameters);
    
    const TickType_t check_interval = pdMS_TO_TICKS(5000);  // Check every 5 seconds
    
    ESP_LOGI(TAG, "Client timeout task started (timeout: %ld ms)", server->config_.client_timeout_ms);
    
    while (true) {
        vTaskDelay(check_interval);
        
        uint64_t current_time = ble_get_timestamp();
        uint64_t timeout_threshold = server->config_.client_timeout_ms * 1000;  // Convert to microseconds
        
        if (xSemaphoreTake(server->clients_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (int i = 0; i < server->max_clients_; i++) {
                if (server->client_sessions_[i].conn_id != 0) {
                    uint64_t inactive_time = current_time - server->client_sessions_[i].last_activity;
                    
                    if (inactive_time > timeout_threshold) {
                        ESP_LOGW(TAG, "Client %d timed out after %lld ms", 
                                server->client_sessions_[i].conn_id, inactive_time / 1000);
                        
                        // Disconnect timed-out client
                        uint16_t conn_id = server->client_sessions_[i].conn_id;
                        xSemaphoreGive(server->clients_mutex_);
                        
                        server->disconnectClient(conn_id);
                        
                        // Re-acquire mutex for next iteration
                        if (xSemaphoreTake(server->clients_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
                            break;
                        }
                    }
                }
            }
            xSemaphoreGive(server->clients_mutex_);
        }
    }
}

/******************************************************************************/
/*                              Internal Methods                              */
/******************************************************************************/

bool BLEServer::validateClientAuthentication(uint16_t conn_id, const char* auth_data) {
    if (!config_.require_authentication) {
        return true;  // Authentication not required
    }
    
    if (!auth_data) {
        ESP_LOGW(TAG, "Client %d: No authentication data provided", conn_id);
        return false;
    }
    
    // Check against configured authentication key
    if (strcmp(auth_data, security_config_.auth_key) == 0) {
        ESP_LOGI(TAG, "Client %d: Authentication successful", conn_id);
        return true;
    }
    
    ESP_LOGW(TAG, "Client %d: Authentication failed", conn_id);
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
                if (instance_->advertising_cb_) {
                    instance_->advertising_cb_(false, ESP_FAIL);
                }
            } else {
                ESP_LOGI(TAG, "Advertising started successfully");
            }
            break;
            
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising stop failed: %d", param->adv_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising stopped successfully");
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
                instance_->gatts_if_ = gatts_if;
                
                // Create service
                esp_gatt_srvc_id_t service_id;
                service_id.is_primary = true;
                service_id.id.inst_id = 0x00;
                
                if (instance_->security_config_.use_custom_uuids) {
                    service_id.id.uuid.len = ESP_UUID_LEN_128;
                    memcpy(service_id.id.uuid.uuid.uuid128, 
                           instance_->security_config_.service_uuid, 16);
                } else {
                    service_id.id.uuid.len = ESP_UUID_LEN_16;
                    service_id.id.uuid.uuid.uuid16 = BLE_DEFAULT_SERVICE_UUID_16;
                }
                
                esp_ble_gatts_create_service(gatts_if, &service_id, 10);
            }
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Service created: service_handle=%d", param->create.service_handle);
            instance_->service_handle_ = param->create.service_handle;
            
            // Start the service
            esp_ble_gatts_start_service(param->create.service_handle);
            
            // Add battery characteristic
            esp_bt_uuid_t battery_char_uuid;
            if (instance_->security_config_.use_custom_uuids) {
                battery_char_uuid.len = ESP_UUID_LEN_128;
                memcpy(battery_char_uuid.uuid.uuid128, 
                       instance_->security_config_.battery_char_uuid, 16);
            } else {
                battery_char_uuid.len = ESP_UUID_LEN_16;
                battery_char_uuid.uuid.uuid16 = BLE_DEFAULT_BATTERY_CHAR_UUID;
            }
            
            esp_ble_gatts_add_char(instance_->service_handle_, &battery_char_uuid,
                                  ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                  ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                  NULL, NULL);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            if (param->add_char.status == ESP_GATT_OK) {
                if (instance_->battery_char_handle_ == BLE_INVALID_HANDLE) {
                    instance_->battery_char_handle_ = param->add_char.attr_handle;
                    ESP_LOGI(TAG, "Battery characteristic added: handle=%d", 
                            instance_->battery_char_handle_);
                    
                    // Add custom characteristic
                    esp_bt_uuid_t custom_char_uuid;
                    if (instance_->security_config_.use_custom_uuids) {
                        custom_char_uuid.len = ESP_UUID_LEN_128;
                        memcpy(custom_char_uuid.uuid.uuid128, 
                               instance_->security_config_.custom_char_uuid, 16);
                    } else {
                        custom_char_uuid.len = ESP_UUID_LEN_16;
                        custom_char_uuid.uuid.uuid16 = BLE_DEFAULT_CUSTOM_CHAR_UUID;
                    }
                    
                    esp_ble_gatts_add_char(instance_->service_handle_, &custom_char_uuid,
                                          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                          ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | 
                                          ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                          NULL, NULL);
                } else if (instance_->custom_char_handle_ == BLE_INVALID_HANDLE) {
                    instance_->custom_char_handle_ = param->add_char.attr_handle;
                    ESP_LOGI(TAG, "Custom characteristic added: handle=%d", 
                            instance_->custom_char_handle_);
                }
            }
            break;
            
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "Service started: service_handle=%d", param->start.service_handle);
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected: conn_id=%d", param->connect.conn_id);
            
            // Add client session
            instance_->addClientSession(param->connect.conn_id, param->connect.remote_bda);
            
            // Update server state
            instance_->state_ = BLE_SERVER_CONNECTED;
            instance_->status_.state = instance_->state_;
            
            // Stop advertising if we reached max clients
            // if (instance_->status_.connected_clients >= instance_->max_clients_) {
            //     instance_->stopAdvertising();
            // }

            // Mantener advertising activo para permitir más conexiones
            if (!instance_->isAdvertising()) {
                ESP_LOGI(TAG, "Reactivating advertising to allow more connections");
                instance_->startAdvertising();
            }
            
            // Call user callback
            if (instance_->client_connected_cb_) {
                ble_device_info_t client_info;
                memcpy(client_info.address, param->connect.remote_bda, sizeof(esp_bd_addr_t));
                client_info.rssi = 0;  // RSSI not available in connect event
                client_info.authenticated = !instance_->config_.require_authentication;
                client_info.last_seen = ble_get_timestamp();
                strcpy(client_info.name, "Unknown");
                
                instance_->client_connected_cb_(param->connect.conn_id, &client_info);
            }
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected: conn_id=%d, reason=%d", 
                    param->disconnect.conn_id, param->disconnect.reason);
            
            // Remove client session
            instance_->removeClientSession(param->disconnect.conn_id);
            
            // Call user callback
            if (instance_->client_disconnected_cb_) {
                instance_->client_disconnected_cb_(param->disconnect.conn_id, param->disconnect.reason);
            }
            
            // Siempre asegurar que advertising está activo después de desconexión
            if (!instance_->isAdvertising()) {
                ESP_LOGI(TAG, "Restarting advertising after client disconnection");
                instance_->startAdvertising();
            }

            // Restart advertising si auto-advertising está habilitado (sin importar clientes)
            if (instance_->config_.auto_start_advertising) {
                if (!instance_->isAdvertising()) {
                    instance_->startAdvertising();
                }
            }
            break;
            
        case ESP_GATTS_READ_EVT:
            ESP_LOGD(TAG, "Read request: conn_id=%d, handle=%d", 
                    param->read.conn_id, param->read.handle);
            
            esp_gatt_rsp_t response;
            memset(&response, 0, sizeof(response));
            response.attr_value.handle = param->read.handle;
            
            if (param->read.handle == instance_->battery_char_handle_) {
                response.attr_value.len = 1;
                response.attr_value.value[0] = instance_->battery_level_;
                
                if (instance_->data_read_cb_) {
                    instance_->data_read_cb_(param->read.conn_id, "battery");
                }
            } else if (param->read.handle == instance_->custom_char_handle_) {
                response.attr_value.len = strlen(instance_->custom_data_);
                memcpy(response.attr_value.value, instance_->custom_data_, response.attr_value.len);
                
                if (instance_->data_read_cb_) {
                    instance_->data_read_cb_(param->read.conn_id, "custom");
                }
            }
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                       ESP_GATT_OK, &response);
            
            // Update client activity
            if (xSemaphoreTake(instance_->clients_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
                for (int i = 0; i < instance_->max_clients_; i++) {
                    if (instance_->client_sessions_[i].conn_id == param->read.conn_id) {
                        instance_->client_sessions_[i].last_activity = ble_get_timestamp();
                        break;
                    }
                }
                xSemaphoreGive(instance_->clients_mutex_);
            }
            break;
            
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGD(TAG, "Write request: conn_id=%d, handle=%d, len=%d", 
                    param->write.conn_id, param->write.handle, param->write.len);
            
            if (param->write.handle == instance_->custom_char_handle_) {
                // Process JSON data if enabled
                if (instance_->config_.enable_json_commands) {
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
                
                instance_->stats_.data_packets_received++;
                
                if (instance_->data_written_cb_) {
                    instance_->data_written_cb_(param->write.conn_id, param->write.value, param->write.len);
                }
                
                // Update client activity and stats
                if (xSemaphoreTake(instance_->clients_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
                    for (int i = 0; i < instance_->max_clients_; i++) {
                        if (instance_->client_sessions_[i].conn_id == param->write.conn_id) {
                            instance_->client_sessions_[i].last_activity = ble_get_timestamp();
                            instance_->client_sessions_[i].data_packets_received++;
                            break;
                        }
                    }
                    xSemaphoreGive(instance_->clients_mutex_);
                }
            }
            
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                           ESP_GATT_OK, NULL);
            }
            break;
            
        case ESP_GATTS_CONF_EVT:
            ESP_LOGD(TAG, "Notification confirmed: conn_id=%d, status=%d", 
                    param->conf.conn_id, param->conf.status);
            break;
            
        default:
            ESP_LOGD(TAG, "GATTS event: %d", event);
            break;
    }
}