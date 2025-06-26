/*******************************************************************************
 * @file BLE.cpp
 * @brief Implementation of the main BLE library providing unified interface
 * for both client and server functionality with flexible operating modes.
 *
 * @version 0.0.5
 * @date 2025-06-24
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "BLE.hpp"

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
            ble_log(ESP_LOG_ERROR, "Failed to initialize BLE common: %s", 
                    esp_err_to_name(ret));
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
        ble_log(ESP_LOG_ERROR, "Failed to initialize modules: %s", 
                esp_err_to_name(ret));
        state_ = BLE_STATE_ERROR;
        lastError_ = ret;
        xSemaphoreGive(stateMutex_);
        return ret;
    }

    state_ = BLE_STATE_INITIALIZED;

    // Start watchdog task
    if (config_.watchdogTimeOut > 0) {
        xTaskCreate(watchdogTask, "ble_watchdog", 4096, this, 2, 
                    &watchdogTaskHandle_);
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
        ble_log(ESP_LOG_ERROR, "Failed to start services: %s", 
                esp_err_to_name(ret));
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
            ble_log(ESP_LOG_WARN, "Failed to deinitialize client: %s", 
                    esp_err_to_name(ret));
        }
    }

    if (serverInitialized_) {
        ret = deinitServer();
        if (ret != ESP_OK) {
            ble_log(ESP_LOG_WARN, "Failed to deinitialize server: %s", 
                    esp_err_to_name(ret));
        }
    }

    // Deinitialize common BLE subsystem
    if (commonInitialized_) {
        ret = ble_common_deinit();
        if (ret != ESP_OK) {
            ble_log(ESP_LOG_WARN, "Failed to deinitialize BLE common: %s", 
                    esp_err_to_name(ret));
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
        ble_log(ESP_LOG_DEBUG, "Client security config updated");
    }
    
    if (server_ != nullptr) {
        ble_log(ESP_LOG_DEBUG, "Server security config updated");
    }
    
    ble_log(ESP_LOG_INFO, "Security configuration updated");
    return ESP_OK;
}