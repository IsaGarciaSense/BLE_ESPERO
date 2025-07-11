/*******************************************************************************
 * @file BLE.hpp
 * @brief Main header file for the BLE library providing unified interface
 * for both client and server functionality with flexible operating modes.
 *
 * @version 0.0.6
 * @date 2025-06-26
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include "BLEConfigs.hpp"
#include "BLEClient.hpp"
#include "BLEServer.hpp"

/**
 * @struct ble_library_config_t
 * @brief Main configuration structure for the BLE library
 */
typedef struct {
    ble_mode_t mode;                                    ///< Operating mode (client, server, or dual)
    char deviceName[BLE_MAX_DEVICE_NAME_LEN];          ///< Device name for BLE operations
    ble_security_config_t security;                    ///< Security configuration
    bool enableLogging;                                ///< Enable internal logging
    esp_log_level_t logLevel;                          ///< Log level for internal messages
    bool autoStart;                                    ///< Auto-start services after init
    uint32_t watchdogTimeOut;                         ///< Watchdog timeout for health monitoring ms
} ble_library_config_t;

/**
 * @struct ble_library_status_t
 * @brief Current status of the BLE library
 */
typedef struct {
    ble_state_t libraryState;                         ///< Overall library state
    ble_mode_t operatingMode;                         ///< Current operating mode
    bool clientActive;                                ///< Client module active status
    bool serverActive;                                ///< Server module active status
    uint64_t libraryUptime;                          ///< Library uptime in milliseconds
    uint32_t freeHeapSize;                           ///< Current free heap size
    uint32_t minimumFreeHeap;                        ///< Minimum free heap since startup
    esp_err_t lastError;                             ///< Last error code
    char versionString[32];                          ///< Library version string
} ble_library_status_t;

/**
 * @brief Main BLE Library class providing unified interface for BLE operations
 * 
 * This class serves as the main entry point for the BLE library, providing
 * a unified interface that can operate in client-only, server-only, or dual
 * mode. It manages the lifecycle of both client and server instances and
 * provides centralized configuration and monitoring capabilities.
 */
class BLELibrary {
public:
    /**
     * @brief Constructs a new BLELibrary object with default settings
     */
    BLELibrary();

    /**
     * @brief Constructs a new BLELibrary object with custom configuration
     * @param config Initial configuration for the library
     */
    BLELibrary(const ble_library_config_t& config);

    /**
     * @brief Destroys the BLELibrary object and cleans up all resources
     */
    virtual ~BLELibrary();

    /**
     * @brief Initializes the BLE library with current configuration
     * 
     * Sets up the BLE stack, initializes selected modules (client/server),
     * and prepares the library for operation.
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t init();

    /**
     * @brief Starts the BLE library services
     * 
     * Starts the configured services (client scanning, server advertising, etc.)
     * based on the operating mode.
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t start();

    /**
     * @brief Stops the BLE library services
     * 
     * Gracefully stops all active services while maintaining connections.
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t stop();

    /**
     * @brief Completely deinitializes the BLE library
     * 
     * Stops all services, disconnects all clients, and deinitializes the BLE stack.
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t deinit();

    /**
     * @brief Restarts the BLE library with current configuration
     * 
     * Equivalent to calling stop() followed by start().
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t restart();

    /**
     * @brief Sets the library configuration
     * 
     * @param config New configuration to apply
     * @return esp_err_t ESP_OK on success, error code otherwise
     * @note Call init() after changing configuration
     */
    esp_err_t setConfig(const ble_library_config_t& config);

    /**
     * @brief Gets the current library configuration
     * 
     * @return ble_library_config_t Current configuration
     */
    ble_library_config_t getConfig() const;

    /**
     * @brief Sets the operating mode
     * 
     * @param mode New operating mode (client, server, or dual)
     * @return esp_err_t ESP_OK on success, error code otherwise
     * @note Requires restart to take effect
     */
    esp_err_t setMode(ble_mode_t mode);

    /**
     * @brief Sets the device name for BLE operations
     * 
     * @param deviceName New device name
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t setDeviceName(const char* deviceName);

    /**
     * @brief Configures security settings
     * 
     * @param securityConfig Security configuration to apply
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t setSecurityConfig(const ble_security_config_t& securityConfig);

    /**
     * @brief Gets the BLE client instance
     * 
     * @return BLEClient* Pointer to client instance, nullptr if not available
     * @note Only available in CLIENT_ONLY or DUAL mode
     */
    BLEClient* getClient();

    /**
     * @brief Configures the BLE client
     * 
     * @param clientConfig Client configuration to apply
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t setClientConfig(const ble_client_config_t& clientConfig);

    /**
     * @brief Starts client scanning for servers
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t startClientScan();

    /**
     * @brief Connects client to a specific server
     * 
     * @param _serverMACadd MAC address of server to connect to
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t connectToServer(const esp_bd_addr_t _serverMACadd);

    /**
     * @brief Gets the BLE server instance
     * 
     * @return BLEServer* Pointer to server instance, nullptr if not available
     * @note Only available in SERVER_ONLY or DUAL mode
     */
    BLEServer* getServer();

    /**
     * @brief Configures the BLE server
     * 
     * @param serverConfig Server configuration to apply
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t setServerConfig(const ble_server_config_t& serverConfig);

    /**
     * @brief Starts server advertising
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t startServerAdvertising();

    /**
     * @brief Updates server data
     * 
     * @param batteryLevel New battery level (0-100%) for when we want to update the battery level
     * @param customData New custom data string
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t updateServerData(uint8_t batteryLevel, const char* customData);

    /**
     * @brief Gets the current library state
     * 
     * @return ble_state_t Current state
     */
    ble_state_t getState() const;

    /**
     * @brief Gets the current operating mode
     * 
     * @return ble_mode_t Current mode
     */
    ble_mode_t getMode() const;

    /**
     * @brief Checks if the library is initialized
     * 
     * @return bool True if initialized, false otherwise
     */
    bool isInitialized() const;

    /**
     * @brief Checks if services are running
     * 
     * @return bool True if services are running, false otherwise
     */
    bool isRunning() const;

    /**
     * @brief Checks if client module is active
     * 
     * @return bool True if client is active, false otherwise
     */
    bool isClientActive() const;

    /**
     * @brief Checks if server module is active
     * 
     * @return bool True if server is active, false otherwise
     */
    bool isServerActive() const;

    /**
     * @brief Gets comprehensive library status
     * 
     * @return ble_library_status_t Current library status
     */
    ble_library_status_t getStatus() const;

    /**
     * @brief Gets the last error that occurred
     * 
     * @return esp_err_t Last error code
     */
    esp_err_t getLastError() const;

    /**
     * @brief Gets library version information
     * 
     * @return const char* Version string
     */
    static const char* getVersion();

    /**
     * @brief Gets library build information
     * 
     * @return const char* Build information string
     */
    static const char* getBuildInfo();

    /**
     * @brief Prints library information to console
     */
    static void printLibraryInfo();

    /**
     * @brief Gets library uptime in milliseconds
     * 
     * @return uint64_t Uptime in milliseconds
     */
    uint64_t getUptime() const;

    /**
     * @brief Performs comprehensive health check
     * 
     * Checks memory usage, task status, and module health.
     * 
     * @return bool True if all systems are healthy, false otherwise
     */
    bool performHealthCheck();

    /**
     * @brief Gets memory usage statistics
     * 
     * @param _freeHeap Current free heap size
     * @param _minFreeHeap Minimum free heap since startup
     * @param _largestBlock Largest contiguous free block
     */
    void getMemoryStats(uint32_t* _freeHeap, uint32_t* _minFreeHeap, 
                    uint32_t* _largestBlock) const;

    /**
     * @brief Generates a comprehensive status report
     * 
     * @param _buffer Output buffer for the report
     * @param bufferSize Size of the output buffer
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t generateStatusReport(char* _buffer, size_t bufferSize) const;

    /**
     * @brief Resets all statistics counters
     */
    void resetAllStats();

    /**
     * @brief Sets global event callback for library events
     * 
     * @param callback Function to call for library events
     */
    void setEventCallback(ble_event_callback_t callback);

    /**
     * @brief Sets custom log callback
     * 
     * @param callback Function to call for log messages
     */
    void setLogCallback(ble_log_callback_t callback);

    /**
     * @brief Creates a default library configuration
     * 
     * @param _config Output configuration structure
     * @param mode Operating mode to configure for
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    static esp_err_t createDefaultConfig(ble_library_config_t* _config, 
                                         ble_mode_t mode);

    /**
     * @brief Validates a library configuration
     * 
     * @param config Configuration to validate
     * @return bool True if configuration is valid, false otherwise
     */
    static bool validateConfig(const ble_library_config_t* config);

    /**
     * @brief Gets string representation of operating mode
     * 
     * @param mode Operating mode
     * @return const char* Mode string
     */
    static const char* getModeString(ble_mode_t mode);

    /**
     * @brief Gets string representation of library state
     * 
     * @param state Library state
     * @return const char* State string
     */
    static const char* getStateString(ble_state_t state);

protected:
    /**
     * @brief Initializes the client module
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t initClient();

    /**
     * @brief Initializes the server module
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t initServer();

    /**
     * @brief Deinitializes the client module
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t deinitClient();

    /**
     * @brief Deinitializes the server module
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t deinitServer();

    /**
     * @brief Internal watchdog task for health monitoring
     * 
     * @param pvParameters Task parameters
     */
    static void watchdogTask(void *pvParameters);

    /**
     * @brief Internal event handler for client events
     * 
     * @param eventType Event type
     * @param eventData Event data
     */
    void handleClientEvent(int eventType, void* eventData);

    /**
     * @brief Internal event handler for server events
     * 
     * @param eventType Event type
     * @param eventData Event data
     */
    void handleServerEvent(int eventType, void* eventData);

private:
    ble_library_config_t config_;              ///< Library configuration
    ble_state_t state_;                        ///< Current library state
    esp_err_t lastError_;                     ///< Last error code
    
    BLEClient* client_;                        ///< BLE client instance
    BLEServer* server_;                        ///< BLE server instance
    
    bool clientInitialized_;                  ///< Client initialization status
    bool serverInitialized_;                  ///< Server initialization status
    bool commonInitialized_;                  ///< Common BLE stack initialization status
    
    uint64_t initTime_;                       ///< Library initialization timestamp
    uint64_t startTime_;                      ///< Library start timestamp
    
    // Callbacks
    ble_event_callback_t eventCallback_;      ///< Global event callback
    ble_log_callback_t logCallback_;          ///< Custom log callback
    
    // Synchronization
    SemaphoreHandle_t stateMutex_;            ///< State protection mutex

    // Tasks
    TaskHandle_t watchdogTaskHandle_;         ///< Watchdog task handle

    // Statistics
    uint32_t restartCount_;                   ///< Number of restarts
    uint64_t totalUpTime_;                    ///< Total uptime across restarts
};

/**
 * @brief Creates a client-only BLE library instance
 * 
 * @param deviceName Name of this device
 * @param targetServer Name of server to connect to
 * @return BLELibrary* Configured library instance
 */
BLELibrary* createBLEClient(const char* deviceName, const char* targetServer);

/**
 * @brief Creates a server-only BLE library instance
 * 
 * @param deviceName Name of this device for advertising
 * @return BLELibrary* Configured library instance
 */
BLELibrary* createBLEServer(const char* deviceName);

/**
 * @brief Creates a dual-mode BLE library instance
 * 
 * @param deviceName Name of this device
 * @param targetServer Name of server to connect to (for client mode)
 * @return BLELibrary* Configured library instance
 */
BLELibrary* createBLEDual(const char* deviceName, const char* targetServer);