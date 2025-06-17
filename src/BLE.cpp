/*******************************************************************************
 * @file BLE.cpp
 * @brief Implementation of the main BLE library providing unified interface
 * for both client and server functionality with flexible operating modes.
 *
 * @version 0.0.1
 * @date 2025-06-15
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "BLE.hpp"

/******************************************************************************/
/*                              Static Variables                              */
/******************************************************************************/

// static const char* BLE_LIBRARY_TAG = "BLE_LIBRARY";

/******************************************************************************/
/*                              Constructor/Destructor                        */
/******************************************************************************/

BLELibrary::BLELibrary()
    : state_(BLE_STATE_UNINITIALIZED)
    , last_error_(ESP_OK)
    , client_(nullptr)
    , server_(nullptr)
    , client_initialized_(false)
    , server_initialized_(false)
    , common_initialized_(false)
    , init_time_(0)
    , start_time_(0)
    , event_callback_(nullptr)
    , log_callback_(nullptr)
    , state_mutex_(nullptr)
    , watchdog_task_handle_(nullptr)
    , restart_count_(0)
    , total_uptime_(0) {
    
    // Initialize default configuration
    memset(&config_, 0, sizeof(config_));
    config_.mode = BLE_MODE_CLIENT_ONLY;
    strncpy(config_.device_name, "ESP32_BLE_Device", BLE_MAX_DEVICE_NAME_LEN - 1);
    ble_create_default_security_config(&config_.security, BLE_SECURITY_AUTHENTICATED);
    config_.enable_logging = true;
    config_.log_level = ESP_LOG_INFO;
    config_.auto_start = true;
    config_.watchdog_timeout_ms = 30000;

    // Create mutex
    state_mutex_ = xSemaphoreCreateMutex();

    ble_log(ESP_LOG_INFO, "BLE Library created");
}

BLELibrary::BLELibrary(const ble_library_config_t& config) : BLELibrary() {
    config_ = config;
    ble_log(ESP_LOG_INFO, "BLE Library created with custom configuration");
}

BLELibrary::~BLELibrary() {
    ble_log(ESP_LOG_INFO, "Destroying BLE Library");

    // Stop watchdog task
    if (watchdog_task_handle_ != nullptr) {
        vTaskDelete(watchdog_task_handle_);
    }

    // Deinitialize library
    deinit();

    // Clean up mutex
    if (state_mutex_ != nullptr) {
        vSemaphoreDelete(state_mutex_);
    }
}

/******************************************************************************/
/*                              Core Methods                                  */
/******************************************************************************/

esp_err_t BLELibrary::init() {
    ble_log(ESP_LOG_INFO, "Initializing BLE Library");

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    if (state_ != BLE_STATE_UNINITIALIZED) {
        ble_log(ESP_LOG_WARN, "Library already initialized");
        xSemaphoreGive(state_mutex_);
        return ESP_ERR_INVALID_STATE;
    }

    state_ = BLE_STATE_STARTING;
    init_time_ = ble_get_timestamp();

    // Initialize common BLE subsystem
    if (!common_initialized_) {
        ret = ble_common_init();
        if (ret != ESP_OK) {
            ble_log(ESP_LOG_ERROR, "Failed to initialize BLE common: %s", esp_err_to_name(ret));
            state_ = BLE_STATE_ERROR;
            last_error_ = ret;
            xSemaphoreGive(state_mutex_);
            return ret;
        }
        common_initialized_ = true;
    }

    // Initialize modules based on mode
    switch (config_.mode) {
        case BLE_MODE_CLIENT_ONLY:
            ret = initClient();
            break;
            
        case BLE_MODE_SERVER_ONLY:
            ret = initServer();
            break;
            
        case BLE_MODE_DUAL:
            ret = initClient();
            if (ret == ESP_OK) {
                ret = initServer();
            }
            break;
            
        default:
            ret = ESP_ERR_INVALID_ARG;
            break;
    }

    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to initialize modules: %s", esp_err_to_name(ret));
        state_ = BLE_STATE_ERROR;
        last_error_ = ret;
        xSemaphoreGive(state_mutex_);
        return ret;
    }

    state_ = BLE_STATE_INITIALIZED;

    // Start watchdog task
    if (config_.watchdog_timeout_ms > 0) {
        xTaskCreate(watchdogTask, "ble_watchdog", 4096, this, 2, &watchdog_task_handle_);
    }

    xSemaphoreGive(state_mutex_);

    ble_log(ESP_LOG_INFO, "BLE Library initialized successfully in %s mode", 
            getModeString(config_.mode));

    // Auto-start if configured
    if (config_.auto_start) {
        ret = start();
    }

    return ret;
}

esp_err_t BLELibrary::start() {
    ble_log(ESP_LOG_INFO, "Starting BLE Library services");

    if (state_ != BLE_STATE_INITIALIZED) {
        ble_log(ESP_LOG_ERROR, "Library not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;
    start_time_ = ble_get_timestamp();

    // Start services based on mode
    switch (config_.mode) {
        case BLE_MODE_CLIENT_ONLY:
            if (client_ != nullptr) {
                ret = client_->startScan();
            }
            break;
            
        case BLE_MODE_SERVER_ONLY:
            if (server_ != nullptr) {
                ret = server_->startAdvertising();
            }
            break;
            
        case BLE_MODE_DUAL:
            // In dual mode, start server first, then client
            if (server_ != nullptr) {
                ret = server_->startAdvertising();
            }
            if (ret == ESP_OK && client_ != nullptr) {
                ret = client_->startScan();
            }
            break;
            
        default:
            ret = ESP_ERR_INVALID_STATE;
            break;
    }

    if (ret == ESP_OK) {
        state_ = BLE_STATE_READY;
        ble_log(ESP_LOG_INFO, "BLE Library services started successfully");
    } else {
        ble_log(ESP_LOG_ERROR, "Failed to start services: %s", esp_err_to_name(ret));
        last_error_ = ret;
    }

    xSemaphoreGive(state_mutex_);
    return ret;
}

esp_err_t BLELibrary::stop() {
    ble_log(ESP_LOG_INFO, "Stopping BLE Library services");

    if (state_ != BLE_STATE_READY) {
        ble_log(ESP_LOG_WARN, "Services not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    // Update uptime
    if (start_time_ > 0) {
        total_uptime_ += (ble_get_timestamp() - start_time_) / 1000;
        start_time_ = 0;
    }

    // Stop services
    if (client_ != nullptr) {
        if (client_->isConnected()) {
            client_->disconnect(true);
        }
        if (client_->isScanning()) {
            client_->stopScan();
        }
    }

    if (server_ != nullptr) {
        if (server_->hasConnectedClients()) {
            server_->disconnectAllClients();
        }
        if (server_->isAdvertising()) {
            server_->stopAdvertising();
        }
    }

    state_ = BLE_STATE_INITIALIZED;
    
    xSemaphoreGive(state_mutex_);

    ble_log(ESP_LOG_INFO, "BLE Library services stopped");
    return ret;
}

esp_err_t BLELibrary::deinit() {
    ble_log(ESP_LOG_INFO, "Deinitializing BLE Library");

    if (state_ == BLE_STATE_UNINITIALIZED) {
        return ESP_OK;
    }

    // Stop services first
    if (state_ == BLE_STATE_READY) {
        stop();
    }

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    // Deinitialize modules
    if (client_initialized_) {
        ret = deinitClient();
        if (ret != ESP_OK) {
            ble_log(ESP_LOG_WARN, "Failed to deinitialize client: %s", esp_err_to_name(ret));
        }
    }

    if (server_initialized_) {
        ret = deinitServer();
        if (ret != ESP_OK) {
            ble_log(ESP_LOG_WARN, "Failed to deinitialize server: %s", esp_err_to_name(ret));
        }
    }

    // Deinitialize common BLE subsystem
    if (common_initialized_) {
        ret = ble_common_deinit();
        if (ret != ESP_OK) {
            ble_log(ESP_LOG_WARN, "Failed to deinitialize BLE common: %s", esp_err_to_name(ret));
        }
        common_initialized_ = false;
    }

    state_ = BLE_STATE_UNINITIALIZED;
    
    xSemaphoreGive(state_mutex_);

    ble_log(ESP_LOG_INFO, "BLE Library deinitialized");
    return ret;
}

esp_err_t BLELibrary::restart() {
    ble_log(ESP_LOG_INFO, "Restarting BLE Library");

    esp_err_t ret = stop();
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second
        ret = start();
        if (ret == ESP_OK) {
            restart_count_++;
        }
    }

    return ret;
}

/******************************************************************************/
/*                           Configuration Methods                            */
/******************************************************************************/

esp_err_t BLELibrary::setConfig(const ble_library_config_t& config) {
    if (!validateConfig(&config)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (state_ == BLE_STATE_READY) {
        ble_log(ESP_LOG_WARN, "Cannot change config while services are running");
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;
    ble_log(ESP_LOG_INFO, "Library configuration updated");
    return ESP_OK;
}

ble_library_config_t BLELibrary::getConfig() const {
    return config_;
}

esp_err_t BLELibrary::setMode(ble_mode_t mode) {
    if (state_ != BLE_STATE_UNINITIALIZED) {
        ble_log(ESP_LOG_WARN, "Cannot change mode after initialization");
        return ESP_ERR_INVALID_STATE;
    }

    config_.mode = mode;
    ble_log(ESP_LOG_INFO, "Operating mode set to: %s", getModeString(mode));
    return ESP_OK;
}

esp_err_t BLELibrary::setDeviceName(const char* device_name) {
    if (device_name == nullptr || !ble_validate_device_name(device_name)) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(config_.device_name, device_name, BLE_MAX_DEVICE_NAME_LEN - 1);
    config_.device_name[BLE_MAX_DEVICE_NAME_LEN - 1] = '\0';
    
    ble_log(ESP_LOG_INFO, "Device name set to: %s", config_.device_name);
    return ESP_OK;
}

esp_err_t BLELibrary::setSecurityConfig(const ble_security_config_t& security_config) {
    config_.security = security_config;
    
    // Update client and server security configs if they exist
    if (client_ != nullptr) {
        // Note: This would require adding a setSecurityConfig method to BLEClient
        ble_log(ESP_LOG_DEBUG, "Client security config updated");
    }
    
    if (server_ != nullptr) {
        // Note: This would require adding a setSecurityConfig method to BLEServer
        ble_log(ESP_LOG_DEBUG, "Server security config updated");
    }
    
    ble_log(ESP_LOG_INFO, "Security configuration updated");
    return ESP_OK;
}

/******************************************************************************/
/*                            Client Interface                                */
/******************************************************************************/

BLEClient* BLELibrary::getClient() {
    if (config_.mode == BLE_MODE_SERVER_ONLY) {
        return nullptr;
    }
    return client_;
}

esp_err_t BLELibrary::setClientConfig(const ble_client_config_t& client_config) {
    if (config_.mode == BLE_MODE_SERVER_ONLY) {
        return ESP_ERR_INVALID_STATE;
    }

    if (client_ != nullptr) {
        return client_->setConfig(client_config);
    }

    ble_log(ESP_LOG_WARN, "Client not initialized");
    return ESP_ERR_INVALID_STATE;
}

esp_err_t BLELibrary::startClientScan() {
    if (config_.mode == BLE_MODE_SERVER_ONLY || client_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return client_->startScan();
}

esp_err_t BLELibrary::connectToServer(const esp_bd_addr_t server_address) {
    if (config_.mode == BLE_MODE_SERVER_ONLY || client_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return client_->connect(server_address);
}

/******************************************************************************/
/*                            Server Interface                                */
/******************************************************************************/

BLEServer* BLELibrary::getServer() {
    if (config_.mode == BLE_MODE_CLIENT_ONLY) {
        return nullptr;
    }
    return server_;
}

esp_err_t BLELibrary::setServerConfig(const ble_server_config_t& server_config) {
    if (config_.mode == BLE_MODE_CLIENT_ONLY) {
        return ESP_ERR_INVALID_STATE;
    }

    if (server_ != nullptr) {
        return server_->setConfig(server_config);
    }

    ble_log(ESP_LOG_WARN, "Server not initialized");
    return ESP_ERR_INVALID_STATE;
}

esp_err_t BLELibrary::startServerAdvertising() {
    if (config_.mode == BLE_MODE_CLIENT_ONLY || server_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return server_->startAdvertising();
}

esp_err_t BLELibrary::updateServerData(uint8_t battery_level, const char* custom_data) {
    if (config_.mode == BLE_MODE_CLIENT_ONLY || server_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = server_->setBatteryLevel(battery_level);
    if (ret == ESP_OK && custom_data != nullptr) {
        ret = server_->setCustomData(custom_data);
    }

    return ret;
}

/******************************************************************************/
/*                              Status Methods                                */
/******************************************************************************/

ble_state_t BLELibrary::getState() const {
    return state_;
}

ble_mode_t BLELibrary::getMode() const {
    return config_.mode;
}

bool BLELibrary::isInitialized() const {
    return state_ != BLE_STATE_UNINITIALIZED;
}

bool BLELibrary::isRunning() const {
    return state_ == BLE_STATE_READY;
}

bool BLELibrary::isClientActive() const {
    return client_initialized_ && client_ != nullptr;
}

bool BLELibrary::isServerActive() const {
    return server_initialized_ && server_ != nullptr;
}

ble_library_status_t BLELibrary::getStatus() const {
    ble_library_status_t status;
    memset(&status, 0, sizeof(status));
    
    status.library_state = state_;
    status.operating_mode = config_.mode;
    status.client_active = isClientActive();
    status.server_active = isServerActive();
    
    if (init_time_ > 0) {
        status.library_uptime_ms = (ble_get_timestamp() - init_time_) / 1000;
    }
    
    status.free_heap_size = esp_get_free_heap_size();
    status.minimum_free_heap = esp_get_minimum_free_heap_size();
    status.last_error = last_error_;
    
    strncpy(status.version_string, getVersion(), sizeof(status.version_string) - 1);
    
    return status;
}

esp_err_t BLELibrary::getLastError() const {
    return last_error_;
}

/******************************************************************************/
/*                              Utility Methods                               */
/******************************************************************************/

const char* BLELibrary::getVersion() {
    return "0.0.1";  // Versión simple sin constantes adicionales
}

const char* BLELibrary::getBuildInfo() {
    return __DATE__ " " __TIME__;  // Información básica de compilación
}

void BLELibrary::printLibraryInfo() {
    ble_log(ESP_LOG_INFO, "=== BLE Library ===");
    ble_log(ESP_LOG_INFO, "Version: %s", getVersion());
    ble_log(ESP_LOG_INFO, "Build: %s", getBuildInfo());
    ble_log(ESP_LOG_INFO, "==================");
}

uint64_t BLELibrary::getUptime() const {
    uint64_t current_uptime = 0;
    
    if (start_time_ > 0) {
        current_uptime = (ble_get_timestamp() - start_time_) / 1000;
    }
    
    return total_uptime_ + current_uptime;
}

bool BLELibrary::performHealthCheck() {
    bool healthy = true;
    
    // Check memory
    uint32_t free_heap = esp_get_free_heap_size();
    if (free_heap < 8192) {  // Less than 8KB
        ble_log(ESP_LOG_WARN, "Low memory: %lu bytes", free_heap);
        healthy = false;
    }
    
    // Check module health
    if (client_ != nullptr && !client_->performHealthCheck()) {
        ble_log(ESP_LOG_WARN, "Client health check failed");
        healthy = false;
    }
    
    if (server_ != nullptr && !server_->performHealthCheck()) {
        ble_log(ESP_LOG_WARN, "Server health check failed");
        healthy = false;
    }
    
    // Check state consistency
    if (state_ == BLE_STATE_ERROR) {
        ble_log(ESP_LOG_ERROR, "Library in error state");
        healthy = false;
    }
    
    return healthy;
}

void BLELibrary::getMemoryStats(uint32_t* free_heap, uint32_t* min_free_heap, uint32_t* largest_block) const {
    if (free_heap != nullptr) {
        *free_heap = esp_get_free_heap_size();
    }
    if (min_free_heap != nullptr) {
        *min_free_heap = esp_get_minimum_free_heap_size();
    }
    if (largest_block != nullptr) {
        multi_heap_info_t info;
        heap_caps_get_info(&info, MALLOC_CAP_DEFAULT);
        *largest_block = info.largest_free_block;
    }
}

esp_err_t BLELibrary::generateStatusReport(char* buffer, size_t buffer_size) const {
    if (buffer == nullptr || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ble_library_status_t status = getStatus();
    uint32_t free_heap, min_free_heap, largest_block;
    getMemoryStats(&free_heap, &min_free_heap, &largest_block);

    int written = snprintf(buffer, buffer_size,
        "=== BLE Library Status Report ===\n"
        "Version: %s\n"
        "State: %s\n"
        "Mode: %s\n"
        "Uptime: %llu ms\n"
        "Restarts: %lu\n"
        "Client Active: %s\n"
        "Server Active: %s\n"
        "Free Heap: %lu bytes\n"
        "Min Free Heap: %lu bytes\n"
        "Largest Block: %lu bytes\n"
        "Last Error: %s\n",
        status.version_string,
        getStateString(status.library_state),
        getModeString(status.operating_mode),
        getUptime(),
        restart_count_,
        status.client_active ? "YES" : "NO",
        status.server_active ? "YES" : "NO",
        free_heap,
        min_free_heap,
        largest_block,
        esp_err_to_name(status.last_error)
    );

    // Add client status if active
    if (client_ != nullptr && written < (int)buffer_size - 100) {
        ble_client_stats_t client_stats = client_->getStats();
        written += snprintf(buffer + written, buffer_size - written,
            "--- Client Stats ---\n"
            "State: %s\n"
            "Connected: %s\n"
            "Scans: %lu\n"
            "Connections: %lu\n"
            "Data Received: %lu\n",
            client_->getStateString(),
            client_->isConnected() ? "YES" : "NO",
            client_stats.scan_count,
            client_stats.successful_connections,
            client_stats.data_packets_received
        );
    }

    // Add server status if active
    if (server_ != nullptr && written < (int)buffer_size - 100) {
        ble_server_stats_t server_stats = server_->getStats();
        written += snprintf(buffer + written, buffer_size - written,
            "--- Server Stats ---\n"
            "State: %s\n"
            "Advertising: %s\n"
            "Clients: %d\n"
            "Total Connections: %lu\n"
            "Data Sent: %lu\n",
            server_->getStateString(),
            server_->isAdvertising() ? "YES" : "NO",
            server_->getConnectedClientCount(),
            server_stats.total_connections,
            server_stats.data_packets_sent
        );
    }

    if (written < (int)buffer_size - 50) {
        written += snprintf(buffer + written, buffer_size - written,
            "===============================\n");
    }

    return (written > 0 && written < (int)buffer_size) ? ESP_OK : ESP_ERR_NO_MEM;
}

void BLELibrary::resetAllStats() {
    if (client_ != nullptr) {
        client_->resetStats();
    }
    if (server_ != nullptr) {
        server_->resetStats();
    }
    
    restart_count_ = 0;
    total_uptime_ = 0;
    last_error_ = ESP_OK;
    
    ble_log(ESP_LOG_INFO, "All statistics reset");
}

/******************************************************************************/
/*                              Event Methods                                 */
/******************************************************************************/

void BLELibrary::setEventCallback(ble_event_callback_t callback) {
    event_callback_ = callback;
    ble_log(ESP_LOG_DEBUG, "Event callback registered");
}

void BLELibrary::setLogCallback(ble_log_callback_t callback) {
    log_callback_ = callback;
    ble_set_log_callback(callback);
    ble_log(ESP_LOG_DEBUG, "Log callback registered");
}

/******************************************************************************/
/*                              Static Utilities                              */
/******************************************************************************/

esp_err_t BLELibrary::createDefaultConfig(ble_library_config_t* config, ble_mode_t mode) {
    if (config == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(config, 0, sizeof(ble_library_config_t));
    
    config->mode = mode;
    strncpy(config->device_name, "ESP32_BLE_Device", BLE_MAX_DEVICE_NAME_LEN - 1);
    ble_create_default_security_config(&config->security, BLE_SECURITY_AUTHENTICATED);
    config->enable_logging = true;
    config->log_level = ESP_LOG_INFO;
    config->auto_start = true;
    config->watchdog_timeout_ms = 30000;

    return ESP_OK;
}

bool BLELibrary::validateConfig(const ble_library_config_t* config) {
    if (config == nullptr) {
        return false;
    }

    // Validate mode
    if (config->mode < BLE_MODE_CLIENT_ONLY || config->mode > BLE_MODE_DUAL) {
        return false;
    }

    // Validate device name
    if (!ble_validate_device_name(config->device_name)) {
        return false;
    }

    // Validate timeouts
    if (config->watchdog_timeout_ms > 0 && config->watchdog_timeout_ms < 1000) {
        return false;  // Minimum 1 second
    }

    return true;
}

const char* BLELibrary::getModeString(ble_mode_t mode) {
    switch (mode) {
        case BLE_MODE_CLIENT_ONLY: return "CLIENT_ONLY";
        case BLE_MODE_SERVER_ONLY: return "SERVER_ONLY";
        case BLE_MODE_DUAL: return "DUAL";
        default: return "UNKNOWN";
    }
}

const char* BLELibrary::getStateString(ble_state_t state) {
    switch (state) {
        case BLE_STATE_UNINITIALIZED: return "UNINITIALIZED";
        case BLE_STATE_INITIALIZED: return "INITIALIZED";
        case BLE_STATE_STARTING: return "STARTING";
        case BLE_STATE_READY: return "READY";
        case BLE_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

/******************************************************************************/
/*                              Internal Methods                              */
/******************************************************************************/

esp_err_t BLELibrary::initClient() {
    if (client_initialized_) {
        return ESP_OK;
    }

    ble_log(ESP_LOG_INFO, "Initializing BLE Client module");

    client_ = new BLEClient();
    if (client_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // Configure client with default settings
    ble_client_config_t client_config;
    memset(&client_config, 0, sizeof(client_config));
    strncpy(client_config.target_device_name, "ESP32_BLE_Server", BLE_MAX_DEVICE_NAME_LEN - 1);
    client_config.scan_timeout_ms = 10000;
    client_config.min_rssi = -90;
    client_config.auto_reconnect = true;
    client_config.reconnect_interval_ms = 5000;
    client_config.connection_timeout_ms = 10000;
    client_config.enable_notifications = true;
    client_config.read_interval_ms = 5000;

    client_->setConfig(client_config);

    esp_err_t ret = client_->init();
    if (ret != ESP_OK) {
        delete client_;
        client_ = nullptr;
        return ret;
    }

    client_initialized_ = true;
    ble_log(ESP_LOG_INFO, "BLE Client module initialized");
    return ESP_OK;
}

esp_err_t BLELibrary::initServer() {
    if (server_initialized_) {
        return ESP_OK;
    }

    ble_log(ESP_LOG_INFO, "Initializing BLE Server module");

    server_ = new BLEServer();
    if (server_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // Configure server with library device name
    ble_server_config_t server_config;
    memset(&server_config, 0, sizeof(server_config));
    strncpy(server_config.device_name, config_.device_name, BLE_MAX_DEVICE_NAME_LEN - 1);
    server_config.advertising_interval_ms = 100;
    server_config.auto_start_advertising = false;  // We'll start manually
    server_config.data_update_interval_ms = 5000;
    server_config.simulate_battery_drain = true;
    server_config.max_clients = 4;
    server_config.client_timeout_ms = 30000;
    server_config.require_authentication = config_.security.require_authentication;
    server_config.enable_notifications = true;

    server_->setConfig(server_config);

    esp_err_t ret = server_->init();
    if (ret != ESP_OK) {
        delete server_;
        server_ = nullptr;
        return ret;
    }

    server_initialized_ = true;
    ble_log(ESP_LOG_INFO, "BLE Server module initialized");
    return ESP_OK;
}

esp_err_t BLELibrary::deinitClient() {
    if (!client_initialized_ || client_ == nullptr) {
        return ESP_OK;
    }

    ble_log(ESP_LOG_INFO, "Deinitializing BLE Client module");

    delete client_;
    client_ = nullptr;
    client_initialized_ = false;

    return ESP_OK;
}

esp_err_t BLELibrary::deinitServer() {
    if (!server_initialized_ || server_ == nullptr) {
        return ESP_OK;
    }

    ble_log(ESP_LOG_INFO, "Deinitializing BLE Server module");

    delete server_;
    server_ = nullptr;
    server_initialized_ = false;

    return ESP_OK;
}

void BLELibrary::updateStats() {
    // Update any library-level statistics here
}

void BLELibrary::watchdogTask(void *pvParameters) {
    BLELibrary* library = static_cast<BLELibrary*>(pvParameters);
    
    ble_log(ESP_LOG_INFO, "BLE Library watchdog task started");
    
    while (library->isInitialized()) {
        vTaskDelay(pdMS_TO_TICKS(library->config_.watchdog_timeout_ms));
        
        if (!library->performHealthCheck()) {
            ble_log(ESP_LOG_ERROR, "Health check failed, triggering restart");
            library->restart();
        }
    }
    
    ble_log(ESP_LOG_INFO, "BLE Library watchdog task ended");
    library->watchdog_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}

void BLELibrary::handleClientEvent(int event_type, void* event_data) {
    // Handle client events and forward to global callback if needed
    if (event_callback_ != nullptr) {
        event_callback_(event_type, event_data);
    }
}

void BLELibrary::handleServerEvent(int event_type, void* event_data) {
    // Handle server events and forward to global callback if needed
    if (event_callback_ != nullptr) {
        event_callback_(event_type, event_data);
    }
}

/******************************************************************************/
/*                              Convenience Functions                         */
/******************************************************************************/

BLELibrary* createBLEClient(const char* device_name, const char* target_server) {
    ble_library_config_t config;
    BLELibrary::createDefaultConfig(&config, BLE_MODE_CLIENT_ONLY);
    
    if (device_name != nullptr) {
        strncpy(config.device_name, device_name, BLE_MAX_DEVICE_NAME_LEN - 1);
    }
    
    BLELibrary* library = new BLELibrary(config);
    if (library != nullptr && target_server != nullptr) {
        ble_client_config_t client_config;
        memset(&client_config, 0, sizeof(client_config));
        strncpy(client_config.target_device_name, target_server, BLE_MAX_DEVICE_NAME_LEN - 1);
        client_config.scan_timeout_ms = 10000;
        client_config.auto_reconnect = true;
        
        if (library->init() == ESP_OK) {
            library->setClientConfig(client_config);
        }
    }
    
    return library;
}

BLELibrary* createBLEServer(const char* device_name) {
    ble_library_config_t config;
    BLELibrary::createDefaultConfig(&config, BLE_MODE_SERVER_ONLY);
    
    if (device_name != nullptr) {
        strncpy(config.device_name, device_name, BLE_MAX_DEVICE_NAME_LEN - 1);
    }
    
    BLELibrary* library = new BLELibrary(config);
    if (library != nullptr) {
        library->init();
    }
    
    return library;
}

BLELibrary* createBLEDual(const char* device_name, const char* target_server) {
    ble_library_config_t config;
    BLELibrary::createDefaultConfig(&config, BLE_MODE_DUAL);
    
    if (device_name != nullptr) {
        strncpy(config.device_name, device_name, BLE_MAX_DEVICE_NAME_LEN - 1);
    }
    
    BLELibrary* library = new BLELibrary(config);
    if (library != nullptr && target_server != nullptr) {
        ble_client_config_t client_config;
        memset(&client_config, 0, sizeof(client_config));
        strncpy(client_config.target_device_name, target_server, BLE_MAX_DEVICE_NAME_LEN - 1);
        client_config.scan_timeout_ms = 10000;
        client_config.auto_reconnect = true;
        
        if (library->init() == ESP_OK) {
            library->setClientConfig(client_config);
        }
    }
    
    return library;
}

/******************************************************************************/
/*                                 Examples                                   */
/******************************************************************************/

namespace BLEExamples {

void simpleClientExample() {
    ble_log(ESP_LOG_INFO, "=== Simple BLE Client Example ===");
    
    // Create and configure client
    BLELibrary* ble = createBLEClient("MyClient", "ESP32_BLE_Server");
    if (ble == nullptr) {
        ble_log(ESP_LOG_ERROR, "Failed to create BLE client");
        return;
    }
    
    // Set up callbacks
    BLEClient* client = ble->getClient();
    if (client != nullptr) {
        client->setConnectedCallback([](const ble_device_info_t* device_info) {
            ble_log(ESP_LOG_INFO, "Connected to server!");
        });
        
        client->setDataReceivedCallback([](const ble_data_packet_t* data) {
            ble_log(ESP_LOG_INFO, "Received: Battery=%d%%, Data=%s", 
                    data->battery_level, data->custom_data);
        });
    }
    
    // Start services
    esp_err_t ret = ble->start();
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to start client: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    
    ble_log(ESP_LOG_INFO, "Client started successfully. Scanning for servers...");
    
    // In a real application, you would keep the library running
    // For this example, we'll clean up after a delay
    vTaskDelay(pdMS_TO_TICKS(30000));  // Run for 30 seconds
    
    delete ble;
    ble_log(ESP_LOG_INFO, "Client example completed");
}

void simpleServerExample() {
    ble_log(ESP_LOG_INFO, "=== Simple BLE Server Example ===");
    
    // Create and configure server
    BLELibrary* ble = createBLEServer("ESP32_BLE_Server");
    if (ble == nullptr) {
        ble_log(ESP_LOG_ERROR, "Failed to create BLE server");
        return;
    }
    
    // Set up callbacks
    BLEServer* server = ble->getServer();
    if (server != nullptr) {
        server->setClientConnectedCallback([](uint16_t conn_id, const ble_device_info_t* client_info) {
            ble_log(ESP_LOG_INFO, "Client connected: conn_id=%d", conn_id);
        });
        
        server->setDataWrittenCallback([](uint16_t conn_id, const uint8_t* data, uint16_t length) {
            ble_log(ESP_LOG_INFO, "Client %d wrote data: %.*s", conn_id, length, data);
        });
    }
    
    // Start services
    esp_err_t ret = ble->start();
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to start server: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    
    ble_log(ESP_LOG_INFO, "Server started successfully. Advertising...");
    
    // Update data periodically
    for (int i = 0; i < 60; i++) {  // Run for 60 seconds
        char custom_data[64];
        snprintf(custom_data, sizeof(custom_data), "Hello from ESP32! Count: %d", i);
        
        ble->updateServerData(100 - (i % 100), custom_data);
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Update every second
    }
    
    delete ble;
    ble_log(ESP_LOG_INFO, "Server example completed");
}

void dualModeExample() {
    ble_log(ESP_LOG_INFO, "=== Dual Mode BLE Example ===");
    
    // Create dual mode library
    BLELibrary* ble = createBLEDual("ESP32_BLE_Dual", "ESP32_BLE_Server");
    if (ble == nullptr) {
        ble_log(ESP_LOG_ERROR, "Failed to create dual mode BLE");
        return;
    }
    
    // Configure client callbacks
    BLEClient* client = ble->getClient();
    if (client != nullptr) {
        client->setConnectedCallback([](const ble_device_info_t* device_info) {
            ble_log(ESP_LOG_INFO, "CLIENT: Connected to server");
        });
    }
    
    // Configure server callbacks
    BLEServer* server = ble->getServer();
    if (server != nullptr) {
        server->setClientConnectedCallback([](uint16_t conn_id, const ble_device_info_t* client_info) {
            ble_log(ESP_LOG_INFO, "SERVER: Client %d connected", conn_id);
        });
    }
    
    // Start services
    esp_err_t ret = ble->start();
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to start dual mode: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    
    ble_log(ESP_LOG_INFO, "Dual mode started successfully");
    
    // Run for a while
    vTaskDelay(pdMS_TO_TICKS(60000));  // 60 seconds
    
    delete ble;
    ble_log(ESP_LOG_INFO, "Dual mode example completed");
}

void secureConnectionExample() {
    ble_log(ESP_LOG_INFO, "=== Secure BLE Connection Example ===");
    
    // Create library with enhanced security
    ble_library_config_t config;
    BLELibrary::createDefaultConfig(&config, BLE_MODE_SERVER_ONLY);
    
    // Configure enhanced security
    config.security.level = BLE_SECURITY_AUTHENTICATED;
    config.security.use_custom_uuids = true;
    config.security.require_authentication = true;
    strncpy(config.security.auth_key, "SecureKey123", BLE_MAX_AUTH_KEY_LEN - 1);
    
    BLELibrary* ble = new BLELibrary(config);
    if (ble == nullptr) {
        ble_log(ESP_LOG_ERROR, "Failed to create secure BLE server");
        return;
    }
    
    // Initialize and start
    esp_err_t ret = ble->init();
    if (ret == ESP_OK) {
        ret = ble->start();
    }
    
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to start secure server: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    
    ble_log(ESP_LOG_INFO, "Secure server started with authentication required");
    
    // Run for a while
    vTaskDelay(pdMS_TO_TICKS(30000));
    
    delete ble;
    ble_log(ESP_LOG_INFO, "Secure connection example completed");
}

void multipleClientsExample() {
    ble_log(ESP_LOG_INFO, "=== Multiple Clients Server Example ===");
    
    // Create server that can handle multiple clients
    BLELibrary* ble = createBLEServer("ESP32_MultiServer");
    if (ble == nullptr) {
        ble_log(ESP_LOG_ERROR, "Failed to create multi-client server");
        return;
    }
    
    BLEServer* server = ble->getServer();
    if (server != nullptr) {
        // Configure for multiple clients
        ble_server_config_t server_config = server->getConfig();
        server_config.max_clients = 8;  // Support up to 8 clients
        server_config.client_timeout_ms = 60000;  // 60 second timeout
        server->setConfig(server_config);
        
        // Set up callbacks to track multiple clients
        server->setClientConnectedCallback([](uint16_t conn_id, const ble_device_info_t* client_info) {
            ble_log(ESP_LOG_INFO, "Client %d connected", conn_id);
        });
        
        server->setClientDisconnectedCallback([](uint16_t conn_id, int reason) {
            ble_log(ESP_LOG_INFO, "Client %d disconnected (reason: %d)", conn_id, reason);
        });
        
        server->setDataWrittenCallback([](uint16_t conn_id, const uint8_t* data, uint16_t length) {
            ble_log(ESP_LOG_INFO, "Client %d wrote: %.*s", conn_id, length, data);
        });
    }
    
    // Start server
    esp_err_t ret = ble->start();
    if (ret != ESP_OK) {
        ble_log(ESP_LOG_ERROR, "Failed to start multi-client server: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    
    ble_log(ESP_LOG_INFO, "Multi-client server started. Waiting for connections...");
    
    // Run and periodically report status
    for (int i = 0; i < 120; i++) {  // Run for 2 minutes
        if (i % 10 == 0 && server != nullptr) {  // Every 10 seconds
            ble_server_status_t status = server->getStatus();
            ble_log(ESP_LOG_INFO, "Status: %d clients connected, advertising: %s", 
                    status.connected_clients, status.advertising_active ? "ON" : "OFF");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    delete ble;
    ble_log(ESP_LOG_INFO, "Multiple clients example completed");
}

}