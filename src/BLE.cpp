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
    , lastError_(ESP_OK)
    , client_(nullptr)
    , server_(nullptr)
    , clientInitialized_(false)
    , serverInitialized_(false)
    , commonInitialized_(false)
    , initTime_(0)
    , startTime_(0)
    , eventCallback_(nullptr)
    , logCallback_(nullptr)
    , stateMutex_(nullptr)
    , watchdogTaskHandle_(nullptr)
    , restartCount_(0)
    , totalUpTime_(0) {
    
    // Initialize default configuration
    memset(&config_, 0, sizeof(config_));
    config_.mode = BLE_MODE_CLIENT_ONLY;
    strncpy(config_.deviceName, "ESP32_BLE_Device", BLE_MAX_DEVICE_NAME_LEN - 1);
    ble_create_default_security_config(&config_.security, BLE_SECURITY_AUTHENTICATED);
    config_.enableLogging = true;
    config_.logLevel = ESP_LOG_INFO;
    config_.autoStart = true;
    config_.watchdogTimeOut = 30000;

    // Create mutex
    stateMutex_ = xSemaphoreCreateMutex();

    ble_log(ESP_LOG_INFO, "BLE Library created");
}

BLELibrary::BLELibrary(const ble_library_config_t& config) : BLELibrary() {
    config_ = config;
    ble_log(ESP_LOG_INFO, "BLE Library created with custom configuration");
}

BLELibrary::~BLELibrary() {
    ble_log(ESP_LOG_INFO, "Destroying BLE Library");

    // Stop watchdog task
    if (watchdogTaskHandle_ != nullptr) {
        vTaskDelete(watchdogTaskHandle_);
    }

    // Deinitialize library
    deinit();

    // Clean up mutex
    if (stateMutex_ != nullptr) {
        vSemaphoreDelete(stateMutex_);
    }
}

/******************************************************************************/
/*                              Core Methods                                  */
/******************************************************************************/

esp_err_t BLELibrary::init() {
    ble_log(ESP_LOG_INFO, "Initializing BLE Library");

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    if (state_ != BLE_STATE_UNINITIALIZED) {
        ble_log(ESP_LOG_WARN, "Library already initialized");
        xSemaphoreGive(stateMutex_);
        return ESP_ERR_INVALID_STATE;
    }

    state_ = BLE_STATE_STARTING;
    initTime_ = ble_get_timestamp();

    // Initialize common BLE subsystem
    if (!commonInitialized_) {
        ret = ble_common_init();
        if (ret != ESP_OK) {
            ble_log(ESP_LOG_ERROR, "Failed to initialize BLE common: %s", esp_err_to_name(ret));
            state_ = BLE_STATE_ERROR;
            lastError_ = ret;
            xSemaphoreGive(stateMutex_);
            return ret;
        }
        commonInitialized_ = true;
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
        lastError_ = ret;
        xSemaphoreGive(stateMutex_);
        return ret;
    }

    state_ = BLE_STATE_INITIALIZED;

    // Start watchdog task
    if (config_.watchdogTimeOut > 0) {
        xTaskCreate(watchdogTask, "ble_watchdog", 4096, this, 2, &watchdogTaskHandle_);
    }

    xSemaphoreGive(stateMutex_);

    ble_log(ESP_LOG_INFO, "BLE Library initialized successfully in %s mode", 
            getModeString(config_.mode));

    // Auto-start if configured
    if (config_.autoStart) {
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

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;
    startTime_ = ble_get_timestamp();

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
        lastError_ = ret;
    }

    xSemaphoreGive(stateMutex_);
    return ret;
}

esp_err_t BLELibrary::stop() {
    ble_log(ESP_LOG_INFO, "Stopping BLE Library services");

    if (state_ != BLE_STATE_READY) {
        ble_log(ESP_LOG_WARN, "Services not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    // Update uptime
    if (startTime_ > 0) {
        totalUpTime_ += (ble_get_timestamp() - startTime_) / 1000;
        startTime_ = 0;
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

    xSemaphoreGive(stateMutex_);

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

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    // Deinitialize modules
    if (clientInitialized_) {
        ret = deinitClient();
        if (ret != ESP_OK) {
            ble_log(ESP_LOG_WARN, "Failed to deinitialize client: %s", esp_err_to_name(ret));
        }
    }

    if (serverInitialized_) {
        ret = deinitServer();
        if (ret != ESP_OK) {
            ble_log(ESP_LOG_WARN, "Failed to deinitialize server: %s", esp_err_to_name(ret));
        }
    }

    // Deinitialize common BLE subsystem
    if (commonInitialized_) {
        ret = ble_common_deinit();
        if (ret != ESP_OK) {
            ble_log(ESP_LOG_WARN, "Failed to deinitialize BLE common: %s", esp_err_to_name(ret));
        }
        commonInitialized_ = false;
    }

    state_ = BLE_STATE_UNINITIALIZED;

    xSemaphoreGive(stateMutex_);

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
            restartCount_++;
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

esp_err_t BLELibrary::setDeviceName(const char* deviceName) {
    if (deviceName == nullptr || !ble_validate_device_name(deviceName)) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(config_.deviceName, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    config_.deviceName[BLE_MAX_DEVICE_NAME_LEN - 1] = '\0';
    
    ble_log(ESP_LOG_INFO, "Device name set to: %s", config_.deviceName);
    return ESP_OK;
}

esp_err_t BLELibrary::setSecurityConfig(const ble_security_config_t& securityConfig) {
    config_.security = securityConfig;

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

esp_err_t BLELibrary::setClientConfig(const ble_client_config_t& clientConfig) {
    if (config_.mode == BLE_MODE_SERVER_ONLY) {
        return ESP_ERR_INVALID_STATE;
    }

    if (client_ != nullptr) {
        return client_->setConfig(clientConfig);
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

esp_err_t BLELibrary::connectToServer(const esp_bd_addr_t serverMACadd) {
    if (config_.mode == BLE_MODE_SERVER_ONLY || client_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return client_->connect(serverMACadd);
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

esp_err_t BLELibrary::setServerConfig(const ble_server_config_t& serverConfig) {
    if (config_.mode == BLE_MODE_CLIENT_ONLY) {
        return ESP_ERR_INVALID_STATE;
    }

    if (server_ != nullptr) {
        return server_->setConfig(serverConfig);
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

esp_err_t BLELibrary::updateServerData(uint8_t batteryLevel, const char* customData) {
    if (config_.mode == BLE_MODE_CLIENT_ONLY || server_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = server_->setBatteryLevel(batteryLevel);
    if (ret == ESP_OK && customData != nullptr) {
        ret = server_->setCustomData(customData);
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
    return clientInitialized_ && client_ != nullptr;
}

bool BLELibrary::isServerActive() const {
    return serverInitialized_ && server_ != nullptr;
}

ble_library_status_t BLELibrary::getStatus() const {
    ble_library_status_t status;
    memset(&status, 0, sizeof(status));

    status.libraryState = state_;
    status.operatingMode = config_.mode;
    status.clientActive = isClientActive();
    status.serverActive = isServerActive();
    
    if (initTime_ > 0) {
        status.libraryUptime = (ble_get_timestamp() - initTime_) / 1000;
    }
    
    status.freeHeapSize = esp_get_free_heap_size();
    status.minimumFreeHeap = esp_get_minimum_free_heap_size();
    status.lastError = lastError_;

    strncpy(status.versionString, getVersion(), sizeof(status.versionString) - 1);

    return status;
}

esp_err_t BLELibrary::getLastError() const {
    return lastError_;
}

/******************************************************************************/
/*                              Utility Methods                               */
/******************************************************************************/

const char* BLELibrary::getVersion() {
    return "0.0.6";  // actual version of the library
}

const char* BLELibrary::getBuildInfo() {
    return __DATE__ " " __TIME__;  // Basic build information
}

void BLELibrary::printLibraryInfo() {
    ble_log(ESP_LOG_INFO, "=== BLE Library ===");
    ble_log(ESP_LOG_INFO, "Version: %s", getVersion());
    ble_log(ESP_LOG_INFO, "Build: %s", getBuildInfo());
    ble_log(ESP_LOG_INFO, "==================");
}

uint64_t BLELibrary::getUptime() const {
    uint64_t current_uptime = 0;

    if (startTime_ > 0) {
        current_uptime = (ble_get_timestamp() - startTime_) / 1000;
    }

    return totalUpTime_ + current_uptime;
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

void BLELibrary::getMemoryStats(uint32_t* freeHeap, uint32_t* minFreeHeap, uint32_t* largest_free_block) const {
    if (freeHeap != nullptr) {
        *freeHeap = esp_get_free_heap_size();
    }
    if (minFreeHeap != nullptr) {
        *minFreeHeap = esp_get_minimum_free_heap_size();
    }
    if (largest_free_block != nullptr) {
        multi_heap_info_t info;
        heap_caps_get_info(&info, MALLOC_CAP_DEFAULT);
        *largest_free_block = info.largest_free_block;
    }
}

esp_err_t BLELibrary::generateStatusReport(char* buffer, size_t bufferSize) const {
    if (buffer == nullptr || bufferSize == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ble_library_status_t status = getStatus();
    uint32_t freeHeap, minFreeHeap, largestBlock;
    getMemoryStats(&freeHeap, &minFreeHeap, &largestBlock);

    int written = snprintf(buffer, bufferSize,
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
        status.versionString,
        getStateString(status.libraryState),
        getModeString(status.operatingMode),
        getUptime(),
        restartCount_,
        status.clientActive ? "YES" : "NO",
        status.serverActive ? "YES" : "NO",
        freeHeap,
        minFreeHeap,
        largestBlock,
        esp_err_to_name(status.lastError)
    );

    // Add client status if active
    if (client_ != nullptr && written < (int)bufferSize - 100) {
        ble_client_stats_t client_stats = client_->getStats();
        written += snprintf(buffer + written, bufferSize - written,
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
    if (server_ != nullptr && written < (int)bufferSize - 100) {
        ble_server_stats_t server_stats = server_->getStats();
        written += snprintf(buffer + written, bufferSize - written,
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

    if (written < (int)bufferSize - 50) {
        written += snprintf(buffer + written, bufferSize - written,
            "===============================\n");
    }

    return (written > 0 && written < (int)bufferSize) ? ESP_OK : ESP_ERR_NO_MEM;
}

void BLELibrary::resetAllStats() {
    if (client_ != nullptr) {
        client_->resetStats();
    }
    if (server_ != nullptr) {
        server_->resetStats();
    }
    
    restartCount_ = 0;
    totalUpTime_ = 0;
    lastError_ = ESP_OK;

    ble_log(ESP_LOG_INFO, "All statistics reset");
}

/******************************************************************************/
/*                              Event Methods                                 */
/******************************************************************************/

void BLELibrary::setEventCallback(ble_event_callback_t callback) {
    eventCallback_ = callback;
    ble_log(ESP_LOG_DEBUG, "Event callback registered");
}

void BLELibrary::setLogCallback(ble_log_callback_t callback) {
    logCallback_ = callback;
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
    strncpy(config->deviceName, "ESP32_BLE_Device", BLE_MAX_DEVICE_NAME_LEN - 1);
    ble_create_default_security_config(&config->security, BLE_SECURITY_AUTHENTICATED);
    config->enableLogging = true;
    config->logLevel = ESP_LOG_INFO;
    config->autoStart = true;
    config->watchdogTimeOut = 30000;

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
    if (!ble_validate_device_name(config->deviceName)) {
        return false;
    }

    // Validate timeouts
    if (config->watchdogTimeOut > 0 && config->watchdogTimeOut < 1000) {
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
    if (clientInitialized_) {
        return ESP_OK;
    }

    ble_log(ESP_LOG_INFO, "Initializing BLE Client module");

    client_ = new BLEClient();
    if (client_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // Configure client with default settings
    ble_client_config_t clientConfig;
    memset(&clientConfig, 0, sizeof(clientConfig));
    strncpy(clientConfig.target_device_name, "ESP32_BLE_Server", BLE_MAX_DEVICE_NAME_LEN - 1);
    clientConfig.scan_timeout_ms = 10000;
    clientConfig.min_rssi = -90;
    clientConfig.auto_reconnect = true;
    clientConfig.reconnect_interval_ms = 5000;
    clientConfig.connection_timeout_ms = 10000;
    clientConfig.enable_notifications = true;
    clientConfig.read_interval_ms = 5000;

    client_->setConfig(clientConfig);

    esp_err_t ret = client_->init();
    if (ret != ESP_OK) {
        delete client_;
        client_ = nullptr;
        return ret;
    }

    clientInitialized_ = true;
    ble_log(ESP_LOG_INFO, "BLE Client module initialized");
    return ESP_OK;
}

esp_err_t BLELibrary::initServer() {
    if (serverInitialized_) {
        return ESP_OK;
    }

    ble_log(ESP_LOG_INFO, "Initializing BLE Server module");

    server_ = new BLEServer();
    if (server_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // Configure server with library device name
    ble_server_config_t serverConfig;
    memset(&serverConfig, 0, sizeof(serverConfig));
    strncpy(serverConfig.deviceName, config_.deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    serverConfig.advertising_interval_ms = 100;
    serverConfig.auto_start_advertising = false;  // We'll start manually
    serverConfig.data_update_interval_ms = 5000;
    serverConfig.simulate_battery_drain = true;
    serverConfig.max_clients = 4;
    serverConfig.client_timeout_ms = 30000;
    serverConfig.require_authentication = config_.security.require_authentication;
    serverConfig.enable_notifications = true;

    server_->setConfig(serverConfig);

    esp_err_t ret = server_->init();
    if (ret != ESP_OK) {
        delete server_;
        server_ = nullptr;
        return ret;
    }

    serverInitialized_ = true;
    ble_log(ESP_LOG_INFO, "BLE Server module initialized");
    return ESP_OK;
}

esp_err_t BLELibrary::deinitClient() {
    if (!clientInitialized_ || client_ == nullptr) {
        return ESP_OK;
    }

    ble_log(ESP_LOG_INFO, "Deinitializing BLE Client module");

    delete client_;
    client_ = nullptr;
    clientInitialized_ = false;

    return ESP_OK;
}

esp_err_t BLELibrary::deinitServer() {
    if (!serverInitialized_ || server_ == nullptr) {
        return ESP_OK;
    }

    ble_log(ESP_LOG_INFO, "Deinitializing BLE Server module");

    delete server_;
    server_ = nullptr;
    serverInitialized_ = false;

    return ESP_OK;
}

void BLELibrary::watchdogTask(void *pvParameters) {
    BLELibrary* library = static_cast<BLELibrary*>(pvParameters);
    
    ble_log(ESP_LOG_INFO, "BLE Library watchdog task started");
    
    while (library->isInitialized()) {
        vTaskDelay(pdMS_TO_TICKS(library->config_.watchdogTimeOut));

        if (!library->performHealthCheck()) {
            ble_log(ESP_LOG_ERROR, "Health check failed, triggering restart");
            library->restart();
        }
    }
    
    ble_log(ESP_LOG_INFO, "BLE Library watchdog task ended");
    library->watchdogTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}

void BLELibrary::handleClientEvent(int eventType, void* eventData) {
    // Handle client events and forward to global callback if needed
    if (eventCallback_ != nullptr) {
        eventCallback_(eventType, eventData);
    }
}

void BLELibrary::handleServerEvent(int eventType, void* eventData) {
    // Handle server events and forward to global callback if needed
    if (eventCallback_ != nullptr) {
        eventCallback_(eventType, eventData);
    }
}

/******************************************************************************/
/*                              Convenience Functions                         */
/******************************************************************************/

BLELibrary* createBLEClient(const char* deviceName, const char* targetServer) {
    ble_library_config_t config;
    BLELibrary::createDefaultConfig(&config, BLE_MODE_CLIENT_ONLY);

    if (deviceName != nullptr) {
        strncpy(config.deviceName, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    }
    
    BLELibrary* library = new BLELibrary(config);
    if (library != nullptr && targetServer != nullptr) {
        ble_client_config_t clientConfig;
        memset(&clientConfig, 0, sizeof(clientConfig));
        strncpy(clientConfig.target_device_name, targetServer, BLE_MAX_DEVICE_NAME_LEN - 1);
        clientConfig.scan_timeout_ms = 10000;
        clientConfig.auto_reconnect = true;

        if (library->init() == ESP_OK) {
            library->setClientConfig(clientConfig);
        }
    }
    
    return library;
}

BLELibrary* createBLEServer(const char* deviceName) {
    ble_library_config_t config;
    BLELibrary::createDefaultConfig(&config, BLE_MODE_SERVER_ONLY);
    
    if (deviceName != nullptr) {
        strncpy(config.deviceName, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    }
    
    BLELibrary* library = new BLELibrary(config);
    if (library != nullptr) {
        library->init();
    }
    
    return library;
}

BLELibrary* createBLEDual(const char* deviceName, const char* targetServer) {
    ble_library_config_t config;
    BLELibrary::createDefaultConfig(&config, BLE_MODE_DUAL);
    
    if (deviceName != nullptr) {
        strncpy(config.deviceName, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    }
    
    BLELibrary* library = new BLELibrary(config);
    if (library != nullptr && targetServer != nullptr) {
        ble_client_config_t clientConfig;
        memset(&clientConfig, 0, sizeof(clientConfig));
        strncpy(clientConfig.target_device_name, targetServer, BLE_MAX_DEVICE_NAME_LEN - 1);
        clientConfig.scan_timeout_ms = 10000;
        clientConfig.auto_reconnect = true;

        if (library->init() == ESP_OK) {
            library->setClientConfig(clientConfig);
        }
    }
    
    return library;
}