/*******************************************************************************
 * @file ble.cpp
 * @brief Implementation of the main BLE library providing unified interface
 * for both client and server functionality with flexible operating modes.
 *
 * @version 0.0.6
 * @date 2025-08-29
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "ble.hpp"

static const char* TAG = "BLE_LIBRARY";

/******************************************************************************/
/*                         Constructor/Destructor                            */
/******************************************************************************/

/**
 * @brief Default constructor for BLELibrary
 * @details Initializes the BLE library with default configuration in CLIENT_ONLY mode
 */
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
    , stateMutex_(nullptr)
    , watchdogTaskHandle_(nullptr)
    , restartCount_(0)
    , totalUpTime_(0) {
    
    // Initialize default configuration
    memset(&config_, 0, sizeof(config_));
    config_.mode = BLE_MODE_CLIENT_ONLY;
    strncpy(config_.deviceName, "SenseAI_BLE", BLE_MAX_DEVICE_NAME_LEN - 1);
    bleCreateDefaultSecurityConfig(&config_.security, BLE_SECURITY_BASIC);
    config_.enableLogging = true;
    config_.logLevel = ESP_LOG_INFO;
    config_.autoStart = true;
    config_.watchdogTimeOut = 30000;

    stateMutex_ = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "BLE Library created");
}

/**
 * @brief Constructor with custom configuration
 * @param config Custom configuration structure for the BLE library
 */
BLELibrary::BLELibrary(const bleLibraryConfig_t& config) : BLELibrary() {
    config_ = config;
    ESP_LOGI(TAG, "BLE Library created with custom configuration");
}

/**
 * @brief Destructor for BLELibrary
 * @details Performs emergency cleanup to ensure proper resource deallocation
 */
BLELibrary::~BLELibrary() {
    ESP_LOGW(TAG, "BLELibrary destructor called - performing emergency cleanup");
    
    // Emergency cleanup to avoid spinlock
    if (state_ != BLE_STATE_UNINITIALIZED) {
        ESP_LOGW(TAG, "Library not properly stopped before destruction");
        
        // Force stop without waiting for long operations
        if (server_ != nullptr && serverInitialized_) {
            server_->stopAdvertising();
            vTaskDelay(pdMS_TO_TICKS(100));
            server_->disconnectAllClients();
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        if (client_ != nullptr && clientInitialized_) {
            // Force disconnect without callbacks
            client_->disconnect();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // Quick deinit attempt
        esp_err_t ret = deinit();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Deinit failed in destructor: %s", esp_err_to_name(ret));
        }
    }
    
    // Stop watchdog task first
    if (watchdogTaskHandle_ != nullptr) {
        vTaskDelete(watchdogTaskHandle_);
        watchdogTaskHandle_ = nullptr;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Clean pointers safely
    if (server_ != nullptr) {
        delete server_;
        server_ = nullptr;
    }
    
    if (client_ != nullptr) {
        delete client_;
        client_ = nullptr;
    }
    
    // Clean mutex last and safely
    if (stateMutex_ != nullptr) {
        // Try to take mutex with short timeout before deleting
        if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
            xSemaphoreGive(stateMutex_);
        }
        vTaskDelay(pdMS_TO_TICKS(25));
        vSemaphoreDelete(stateMutex_);
        stateMutex_ = nullptr;
    }
    
    ESP_LOGI(TAG, "BLELibrary destructor completed");
}

/******************************************************************************/
/*                              Core Methods                                 */
/******************************************************************************/

/**
 * @brief Initialize the BLE library
 * @details Initializes BLE subsystem and components based on configured mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t BLELibrary::init() {
    ESP_LOGI(TAG, "Initializing BLE Library");

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    if (state_ != BLE_STATE_UNINITIALIZED) {
        ESP_LOGW(TAG, "Library already initialized");
        xSemaphoreGive(stateMutex_);
        return ESP_ERR_INVALID_STATE;
    }

    state_ = BLE_STATE_STARTING;
    initTime_ = bleGetTimestamp();

    if (!commonInitialized_) {
        ret = bleCommonInit();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize BLE common: %s", esp_err_to_name(ret));
            state_ = BLE_STATE_ERROR;
            lastError_ = ret;
            xSemaphoreGive(stateMutex_);
            return ret;
        }
        commonInitialized_ = true;
    }

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
        ESP_LOGE(TAG, "Failed to initialize modules: %s", esp_err_to_name(ret));
        state_ = BLE_STATE_ERROR;
        lastError_ = ret;
        xSemaphoreGive(stateMutex_);
        return ret;
    }

    state_ = BLE_STATE_INITIALIZED;

    if (config_.watchdogTimeOut > 0) {
        xTaskCreate(watchdogTask, "ble_watchdog", 4096, this, 2, &watchdogTaskHandle_);
    }

    xSemaphoreGive(stateMutex_);

    ESP_LOGI(TAG, "BLE Library initialized successfully in %s mode", 
            kgetModeString(config_.mode));

    if (config_.autoStart) {
        ret = start();
    }

    return ret;
}

/**
 * @brief Start BLE library services
 * @details Starts BLE services (advertising for server, scanning for client) based on mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t BLELibrary::start() {
    ESP_LOGI(TAG, "Starting BLE Library services");

    if (state_ != BLE_STATE_INITIALIZED) {
        ESP_LOGE(TAG, "Library not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;
    startTime_ = bleGetTimestamp();

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
        ESP_LOGI(TAG, "BLE Library services started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start services: %s", esp_err_to_name(ret));
        lastError_ = ret;
    }

    xSemaphoreGive(stateMutex_);
    return ret;
}

/**
 * @brief Stop BLE library services
 * @details Stops all active BLE services with enhanced safety measures
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t BLELibrary::stop() {
    ESP_LOGI(TAG, "Stopping BLE Library services");

    if (state_ != BLE_STATE_READY) {
        ESP_LOGW(TAG, "Services not running, state: %s", kgetStateString(state_));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for stop operation");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;
    state_ = BLE_STATE_STARTING;

    if (startTime_ > 0) {
        totalUpTime_ += (bleGetTimestamp() - startTime_) / 1000;
        startTime_ = 0;
    }

    switch (config_.mode) {
        case BLE_MODE_CLIENT_ONLY:
            if (client_ != nullptr && clientInitialized_) {
                ESP_LOGI(TAG, "Stopping client services...");
                if (client_->isConnected()) {
                    client_->disconnect(true);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                if (client_->isScanning()) {
                    client_->stopScan();
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            }
            break;
            
        case BLE_MODE_SERVER_ONLY:
            if (server_ != nullptr && serverInitialized_) {
                ESP_LOGI(TAG, "Stopping server services...");
                
                if (server_->isAdvertising()) {
                    esp_err_t adv_ret = server_->stopAdvertising();
                    if (adv_ret != ESP_OK) {
                        ESP_LOGW(TAG, "Stop advertising failed: %s", esp_err_to_name(adv_ret));
                    }
                    vTaskDelay(pdMS_TO_TICKS(400));
                }
                
                if (server_->hasConnectedClients()) {
                    esp_err_t disc_ret = server_->disconnectAllClients();
                    if (disc_ret != ESP_OK) {
                        ESP_LOGW(TAG, "Disconnect clients failed: %s", esp_err_to_name(disc_ret));
                    }
                    vTaskDelay(pdMS_TO_TICKS(1200));
                }
            }
            break;
            
        case BLE_MODE_DUAL:
            ESP_LOGI(TAG, "Stopping dual mode services...");
            if (server_ != nullptr && serverInitialized_) {
                ESP_LOGI(TAG, "Stopping server in dual mode...");
                if (server_->isAdvertising()) {
                    server_->stopAdvertising();
                    vTaskDelay(pdMS_TO_TICKS(400));
                }
                if (server_->hasConnectedClients()) {
                    server_->disconnectAllClients();
                    vTaskDelay(pdMS_TO_TICKS(1200));
                }
            }
            
            if (client_ != nullptr && clientInitialized_) {
                ESP_LOGI(TAG, "Stopping client in dual mode...");
                if (client_->isConnected()) {
                    client_->disconnect(true);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                if (client_->isScanning()) {
                    client_->stopScan();
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            }
            break;
            
        default:
            ESP_LOGE(TAG, "Invalid mode during stop: %d", config_.mode);
            ret = ESP_ERR_INVALID_ARG;
            break;
    }

    if (watchdogTaskHandle_ != nullptr) {
        ESP_LOGI(TAG, "Stopping watchdog task...");
        vTaskDelete(watchdogTaskHandle_);
        watchdogTaskHandle_ = nullptr;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    state_ = BLE_STATE_INITIALIZED;

    xSemaphoreGive(stateMutex_);

    ESP_LOGI(TAG, "BLE Library services stopped successfully");
    return ret;
}

esp_err_t BLELibrary::deinit() {
    ESP_LOGI(TAG, "Deinitializing BLE Library");

    if (state_ == BLE_STATE_UNINITIALIZED) {
        ESP_LOGW(TAG, "Library already deinitialized");
        return ESP_OK;
    }

    // Stop services first if still running
    if (state_ == BLE_STATE_READY) {
        ESP_LOGI(TAG, "Stopping library before deinit");
        esp_err_t ret = stop();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Stop failed during deinit: %s", esp_err_to_name(ret));
        }
        // Force continue with deinit even if stop failed
    }

    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for deinit - forcing deinit anyway");
        // Continue without mutex in emergency situation
    } else {
        ESP_LOGI(TAG, "Mutex acquired for deinit");
    }

    esp_err_t final_ret = ESP_OK;

    // CRITICAL: Add delays between each deinit step to avoid spinlock
    
    // Deinitialize server first with extended delay
    if (server_ != nullptr && serverInitialized_) {
        ESP_LOGI(TAG, "Deinitializing server...");
        esp_err_t ret = deinitServer();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Server deinit failed: %s", esp_err_to_name(ret));
            final_ret = ret;
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Critical delay after server deinit
    }
    
    // Deinitialize client with extended delay
    if (client_ != nullptr && clientInitialized_) {
        ESP_LOGI(TAG, "Deinitializing client...");
        esp_err_t ret = deinitClient();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Client deinit failed: %s", esp_err_to_name(ret));
            final_ret = ret;
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Critical delay after client deinit
    }
    
    // Deinitialize common BLE stack LAST with longest delay
    if (commonInitialized_) {
        ESP_LOGI(TAG, "Deinitializing common BLE stack...");
        esp_err_t ret = bleCommonDeinit();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Common deinit failed: %s", esp_err_to_name(ret));
            final_ret = ret;
        }
        commonInitialized_ = false;
        vTaskDelay(pdMS_TO_TICKS(1000)); // Longest delay for BT stack cleanup
    }

    state_ = BLE_STATE_UNINITIALIZED;

    // Release mutex if we acquired it
    if (stateMutex_ != nullptr) {
        xSemaphoreGive(stateMutex_);
    }

    ESP_LOGI(TAG, "BLE Library deinitialized successfully");
    return final_ret;
}

esp_err_t BLELibrary::restart() {
    ESP_LOGI(TAG, "Restarting BLE Library");

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
/*                         Configuration Methods                             */
/******************************************************************************/

/**
 * @brief Set the configuration for the BLE library
 * @param config New configuration structure
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if config is invalid,
 *         ESP_ERR_INVALID_STATE if services are running
 */
esp_err_t BLELibrary::setConfig(const bleLibraryConfig_t& config) {
    if (!validateConfig(&config)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (state_ == BLE_STATE_READY) {
        ESP_LOGW(TAG, "Cannot change config while services are running");
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;
    ESP_LOGI(TAG, "Library configuration updated");
    return ESP_OK;
}

/**
 * @brief Get the current configuration of the BLE library
 * @return Current configuration structure
 */
bleLibraryConfig_t BLELibrary::getConfig() const {
    return config_;
}

/**
 * @brief Set the operating mode of the BLE library
 * @param mode New operating mode (CLIENT_ONLY, SERVER_ONLY, or DUAL)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t BLELibrary::setMode(bleMode_t mode) {
    if (state_ != BLE_STATE_UNINITIALIZED) {
        ESP_LOGW(TAG, "Cannot change mode after initialization");
        return ESP_ERR_INVALID_STATE;
    }

    config_.mode = mode;
    ESP_LOGI(TAG, "Operating mode set to: %s", kgetModeString(mode));
    return ESP_OK;
}

esp_err_t BLELibrary::setDeviceName(const char* deviceName) {
    if (deviceName == nullptr || !bleValidateDeviceName(deviceName)) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(config_.deviceName, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    config_.deviceName[BLE_MAX_DEVICE_NAME_LEN - 1] = '\0';
    
    ESP_LOGI(TAG, "Device name set to: %s", config_.deviceName);
    return ESP_OK;
}

esp_err_t BLELibrary::setSecurityConfig(const bleSecurityConfig_t& securityConfig) {
    config_.security = securityConfig;

                                                // Update client and server security configs
    if (client_ != nullptr) {
                                                // Note: This would require adding a setSecurityConfig method to BLEClient
        ESP_LOGD(TAG, "Client security config updated");
    }
    
    if (server_ != nullptr) {
                                                // Note: This would require adding a setSecurityConfig method to BLEServer
        ESP_LOGD(TAG, "Server security config updated");
    }
    
    ESP_LOGI(TAG, "Security configuration updated");
    return ESP_OK;
}

BLEClient* BLELibrary::getClient() {
    if (config_.mode == BLE_MODE_SERVER_ONLY) {
        return nullptr;
    }
    return client_;
}

esp_err_t BLELibrary::setClientConfig(const bleClientConfig_t& clientConfig) {
    if (config_.mode == BLE_MODE_SERVER_ONLY) {
        return ESP_ERR_INVALID_STATE;
    }

    if (client_ != nullptr) {
        return client_->setConfig(clientConfig);
    }

    ESP_LOGW(TAG, "Client not initialized");
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

BLEServer* BLELibrary::getServer() {
    if (config_.mode == BLE_MODE_CLIENT_ONLY) {
        return nullptr;
    }
    return server_;
}

esp_err_t BLELibrary::setServerConfig(const bleServerConfig_t& serverConfig) {
    if (config_.mode == BLE_MODE_CLIENT_ONLY) {
        return ESP_ERR_INVALID_STATE;
    }

    if (server_ != nullptr) {
        return server_->setConfig(serverConfig);
    }

    ESP_LOGW(TAG, "Server not initialized");
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

bleState_t BLELibrary::getState() const {
    return state_;
}

bleMode_t BLELibrary::getMode() const {
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

/******************************************************************************/
/*                            Status Methods                                  */
/******************************************************************************/

/**
 * @brief Get comprehensive status information about the BLE library
 * @return Complete status structure with current state and statistics
 */
bleLibraryStatus_t BLELibrary::getStatus() const {
    bleLibraryStatus_t status;
    memset(&status, 0, sizeof(status));

    status.libraryState = state_;
    status.operatingMode = config_.mode;
    status.clientActive = isClientActive();
    status.serverActive = isServerActive();
    
    if (initTime_ > 0) {
        status.libraryUptime = (bleGetTimestamp() - initTime_) / 1000;
    }
    
    status.freeHeapSize = esp_get_free_heap_size();
    status.minimumFreeHeap = esp_get_minimum_free_heap_size();
    status.lastError = lastError_;

    strncpy(status.versionString, kgetVersion(), sizeof(status.versionString) - 1);

    return status;
}

/**
 * @brief Get the last error code from the BLE library
 * @return Last error code that occurred
 */
esp_err_t BLELibrary::getLastError() const {
    return lastError_;
}

/******************************************************************************/
/*                           Information Methods                              */
/******************************************************************************/

/**
 * @brief Get the current version of the BLE library
 * @return Version string in format "x.y.z"
 */
const char* BLELibrary::kgetVersion() {
    return "0.0.6";  // Current version of the library
}

/**
 * @brief Get build information for the library
 * @return Build date and time string
 */
const char* BLELibrary::kgetBuildInfo() {
    return __DATE__ " " __TIME__;  // Basic build information
}

/**
 * @brief Print comprehensive library information to logs
 * @details Displays version, build info, and other library details
 */
void BLELibrary::printLibraryInfo() {
    ESP_LOGI(TAG, "=== BLE Library ===");
    ESP_LOGI(TAG, "Version: %s", kgetVersion());
    ESP_LOGI(TAG, "Build: %s", kgetBuildInfo());
    ESP_LOGI(TAG, "==================");
}

uint64_t BLELibrary::getUptime() const {
    uint64_t current_uptime = 0;

    if (startTime_ > 0) {
        current_uptime = (bleGetTimestamp() - startTime_) / 1000;
    }

    return totalUpTime_ + current_uptime;
}

bool BLELibrary::performHealthCheck() {
    bool healthy = true;
    
    // Check memory
    uint32_t free_heap = esp_get_free_heap_size();
    if (free_heap < 8192) {  // Less than 8KB
        ESP_LOGW(TAG, "Low memory: %lu bytes", free_heap);
        healthy = false;
    }
    
    // Check module health
    if (client_ != nullptr && !client_->performHealthCheck()) {
        ESP_LOGW(TAG, "Client health check failed");
        healthy = false;
    }
    
    if (server_ != nullptr && !server_->performHealthCheck()) {
        ESP_LOGW(TAG, "Server health check failed");
        healthy = false;
    }
    
    // Check state consistency
    if (state_ == BLE_STATE_ERROR) {
        ESP_LOGE(TAG, "Library in error state");
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

    bleLibraryStatus_t status = getStatus();
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
        kgetStateString(status.libraryState),
        kgetModeString(status.operatingMode),
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
        bleClientStats_t client_stats = client_->getStats();
        written += snprintf(buffer + written, bufferSize - written,
            "--- Client Stats ---\n"
            "State: %s\n"
            "Connected: %s\n"
            "Scans: %lu\n"
            "Connections: %lu\n"
            "Data Received: %lu\n",
            client_->kgetStateString(),
            client_->isConnected() ? "YES" : "NO",
            client_stats.scanCount,
            client_stats.successfulConnections,
            client_stats.dataReceived
        );
    }

    // Add server status if active
    if (server_ != nullptr && written < (int)bufferSize - 100) {
        bleServerStats_t server_stats = server_->getStats();
        written += snprintf(buffer + written, bufferSize - written,
            "--- Server Stats ---\n"
            "State: %s\n"
            "Advertising: %s\n"
            "Clients: %d\n"
            "Total Connections: %lu\n"
            "Data Sent: %lu\n",
            server_->kgetStateString(),
            server_->isAdvertising() ? "YES" : "NO",
            server_->getConnectedClientCount(),
            server_stats.totalConnections,
            server_stats.dataSent
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

    ESP_LOGI(TAG, "All statistics reset");
}

void BLELibrary::setEventCallback(bleEventCB_t callback) {
    eventCallback_ = callback;
    ESP_LOGD(TAG, "Event callback registered");
}

esp_err_t BLELibrary::createDefaultConfig(bleLibraryConfig_t* config, bleMode_t mode) {
    if (config == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(config, 0, sizeof(bleLibraryConfig_t));
    
    config->mode = mode;
    strncpy(config->deviceName, "SenseAI_BLE", BLE_MAX_DEVICE_NAME_LEN - 1);
    bleCreateDefaultSecurityConfig(&config->security, BLE_SECURITY_BASIC);
    config->enableLogging = true;
    config->logLevel = ESP_LOG_INFO;
    config->autoStart = true;
    config->watchdogTimeOut = 30000;

    return ESP_OK;
}

bool BLELibrary::validateConfig(const bleLibraryConfig_t* config) {
    if (config == nullptr) {
        return false;
    }

    // Validate mode
    if (config->mode < BLE_MODE_CLIENT_ONLY || config->mode > BLE_MODE_DUAL) {
        return false;
    }

    // Validate device name
    if (!bleValidateDeviceName(config->deviceName)) {
        return false;
    }

    // Validate timeouts
    if (config->watchdogTimeOut > 0 && config->watchdogTimeOut < 1000) {
        return false;  // Minimum 1 second
    }

    return true;
}

const char* BLELibrary::kgetModeString(bleMode_t mode) {
    switch (mode) {
        case BLE_MODE_CLIENT_ONLY: return "CLIENT_ONLY";
        case BLE_MODE_SERVER_ONLY: return "SERVER_ONLY";
        case BLE_MODE_DUAL: return "DUAL";
        default: return "UNKNOWN";
    }
}

const char* BLELibrary::kgetStateString(bleState_t state) {
    switch (state) {
        case BLE_STATE_UNINITIALIZED: return "UNINITIALIZED";
        case BLE_STATE_INITIALIZED: return "INITIALIZED";
        case BLE_STATE_STARTING: return "STARTING";
        case BLE_STATE_READY: return "READY";
        case BLE_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

esp_err_t BLELibrary::initClient() {
    if (clientInitialized_) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing BLE Client module");

    client_ = new BLEClient();
    if (client_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // Configure client with default settings
    bleClientConfig_t clientConfig;
    memset(&clientConfig, 0, sizeof(clientConfig));
    strncpy(clientConfig.targetDeviceName, "SenseAI_BLE", BLE_MAX_DEVICE_NAME_LEN - 1);
    clientConfig.scanTimeout = 10000;
    clientConfig.autoReconnect = true;
    clientConfig.reconnectInterval = 5000;
    clientConfig.connectionTimeout = 10000;
    clientConfig.enableNotifications = true;
    clientConfig.readInterval = 5000;

    client_->setConfig(clientConfig);

    esp_err_t ret = client_->init();
    if (ret != ESP_OK) {
        delete client_;
        client_ = nullptr;
        return ret;
    }

    clientInitialized_ = true;
    ESP_LOGI(TAG, "BLE Client module initialized");
    return ESP_OK;
}

esp_err_t BLELibrary::initServer() {
    if (serverInitialized_) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing BLE Server module");

    server_ = new BLEServer();
    if (server_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // Configure server with library device name
    bleServerConfig_t serverConfig;
    memset(&serverConfig, 0, sizeof(serverConfig));
    strncpy(serverConfig.deviceName, config_.deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    serverConfig.advertisingInterval = 100;
    serverConfig.autoStartAdvertising = false;  // We'll start manually
    serverConfig.dataUpdateInterval = 5000;
    serverConfig.maxClients = 4;
    serverConfig.clientTimeout = 30000;
    serverConfig.requireAuthentication = config_.security.requireAuthentication;
    serverConfig.enableNotifications = true;

    server_->setConfig(serverConfig);

    esp_err_t ret = server_->init();
    if (ret != ESP_OK) {
        delete server_;
        server_ = nullptr;
        return ret;
    }

    serverInitialized_ = true;
    ESP_LOGI(TAG, "BLE Server module initialized");
    return ESP_OK;
}

esp_err_t BLELibrary::deinitClient() {
    if (!clientInitialized_ || client_ == nullptr) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing BLE Client module");

    delete client_;
    client_ = nullptr;
    clientInitialized_ = false;

    return ESP_OK;
}

esp_err_t BLELibrary::deinitServer() {
    if (!serverInitialized_ || server_ == nullptr) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing BLE Server module");

    delete server_;
    server_ = nullptr;
    serverInitialized_ = false;

    return ESP_OK;
}

void BLELibrary::watchdogTask(void *pvParameters) {
    BLELibrary* library = static_cast<BLELibrary*>(pvParameters);
    
    ESP_LOGI(TAG, "BLE Library watchdog task started");
    
    while (library->isInitialized()) {
        vTaskDelay(pdMS_TO_TICKS(library->config_.watchdogTimeOut));

        if (!library->performHealthCheck()) {
            ESP_LOGE(TAG, "Health check failed, triggering restart");
            library->restart();
        }
    }
    
    ESP_LOGI(TAG, "BLE Library watchdog task ended");
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

BLELibrary* createBLEClient(const char* deviceName, const char* targetServer) {
    bleLibraryConfig_t config;
    BLELibrary::createDefaultConfig(&config, BLE_MODE_CLIENT_ONLY);

    if (deviceName != nullptr) {
        strncpy(config.deviceName, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    }
    
    BLELibrary* library = new BLELibrary(config);
    if (library != nullptr && targetServer != nullptr) {
        bleClientConfig_t clientConfig;
        memset(&clientConfig, 0, sizeof(clientConfig));
        strncpy(clientConfig.targetDeviceName, targetServer, BLE_MAX_DEVICE_NAME_LEN - 1);
        clientConfig.scanTimeout = 10000;
        clientConfig.autoReconnect = true;

        if (library->init() == ESP_OK) {
            library->setClientConfig(clientConfig);
        }
    }
    
    return library;
}

BLELibrary* createBLEServer(const char* deviceName) {
    bleLibraryConfig_t config;
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
    bleLibraryConfig_t config;
    BLELibrary::createDefaultConfig(&config, BLE_MODE_DUAL);
    
    if (deviceName != nullptr) {
        strncpy(config.deviceName, deviceName, BLE_MAX_DEVICE_NAME_LEN - 1);
    }
    
    BLELibrary* library = new BLELibrary(config);
    if (library != nullptr && targetServer != nullptr) {
        bleClientConfig_t clientConfig;
        memset(&clientConfig, 0, sizeof(clientConfig));
        strncpy(clientConfig.targetDeviceName, targetServer, BLE_MAX_DEVICE_NAME_LEN - 1);
        clientConfig.scanTimeout = 10000;
        clientConfig.autoReconnect = true;

        if (library->init() == ESP_OK) {
            library->setClientConfig(clientConfig);
        }
    }
    
    return library;
}
