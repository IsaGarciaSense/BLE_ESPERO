/*******************************************************************************
 * @file BLE.hpp
 * @brief Main header file for the BLE library providing unified interface
 * for both client and server functionality with flexible operating modes.
 *
 * @version 0.0.1
 * @date 2025-06-15
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include "BLEConfigs.hpp"
#include "BLEClient.hpp"
#include "BLEServer.hpp"

/******************************************************************************/
/*                                 Structures                                 */
/******************************************************************************/

/**
 * @struct ble_library_config_t
 * @brief Main configuration structure for the BLE library
 */
typedef struct {
    ble_mode_t mode;                                    ///< Operating mode (client, server, or dual)
    char device_name[BLE_MAX_DEVICE_NAME_LEN];         ///< Device name for BLE operations
    ble_security_config_t security;                    ///< Security configuration
    bool enable_logging;                               ///< Enable internal logging
    esp_log_level_t log_level;                         ///< Log level for internal messages
    bool auto_start;                                   ///< Auto-start services after init
    uint32_t watchdog_timeout_ms;                      ///< Watchdog timeout for health monitoring
} ble_library_config_t;

/**
 * @struct ble_library_status_t
 * @brief Current status of the BLE library
 */
typedef struct {
    ble_state_t library_state;                         ///< Overall library state
    ble_mode_t operating_mode;                         ///< Current operating mode
    bool client_active;                                ///< Client module active status
    bool server_active;                                ///< Server module active status
    uint64_t library_uptime_ms;                        ///< Library uptime in milliseconds
    uint32_t free_heap_size;                          ///< Current free heap size
    uint32_t minimum_free_heap;                        ///< Minimum free heap since startup
    esp_err_t last_error;                             ///< Last error code
    char version_string[32];                          ///< Library version string
} ble_library_status_t;

/******************************************************************************/
/*                                BLE Library Class                          */
/******************************************************************************/

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

    /**************************************************************************/
    /*                              Core Methods                              */
    /**************************************************************************/

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

    /**************************************************************************/
    /*                           Configuration Methods                        */
    /**************************************************************************/

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
     * @param device_name New device name
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t setDeviceName(const char* device_name);

    /**
     * @brief Configures security settings
     * 
     * @param security_config Security configuration to apply
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t setSecurityConfig(const ble_security_config_t& security_config);

    /**************************************************************************/
    /*                            Client Interface                            */
    /**************************************************************************/

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
     * @param client_config Client configuration to apply
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t setClientConfig(const ble_client_config_t& client_config);

    /**
     * @brief Starts client scanning for servers
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t startClientScan();

    /**
     * @brief Connects client to a specific server
     * 
     * @param server_address MAC address of server to connect to
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t connectToServer(const esp_bd_addr_t server_address);

    /**************************************************************************/
    /*                            Server Interface                            */
    /**************************************************************************/

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
     * @param server_config Server configuration to apply
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t setServerConfig(const ble_server_config_t& server_config);

    /**
     * @brief Starts server advertising
     * 
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t startServerAdvertising();

    /**
     * @brief Updates server data
     * 
     * @param battery_level New battery level (0-100%)
     * @param custom_data New custom data string
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t updateServerData(uint8_t battery_level, const char* custom_data);

    /**************************************************************************/
    /*                              Status Methods                            */
    /**************************************************************************/

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

    /**************************************************************************/
    /*                              Utility Methods                           */
    /**************************************************************************/

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
     * @param free_heap Current free heap size
     * @param min_free_heap Minimum free heap since startup
     * @param largest_block Largest contiguous free block
     */
    void getMemoryStats(uint32_t* free_heap, uint32_t* min_free_heap, uint32_t* largest_block) const;

    /**
     * @brief Generates a comprehensive status report
     * 
     * @param buffer Output buffer for the report
     * @param buffer_size Size of the output buffer
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t generateStatusReport(char* buffer, size_t buffer_size) const;

    /**
     * @brief Resets all statistics counters
     */
    void resetAllStats();

    /**************************************************************************/
    /*                              Event Methods                             */
    /**************************************************************************/

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

    /**************************************************************************/
    /*                              Static Utilities                          */
    /**************************************************************************/

    /**
     * @brief Creates a default library configuration
     * 
     * @param config Output configuration structure
     * @param mode Operating mode to configure for
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    static esp_err_t createDefaultConfig(ble_library_config_t* config, ble_mode_t mode);

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
    /**************************************************************************/
    /*                              Internal Methods                          */
    /**************************************************************************/

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
     * @brief Updates internal statistics
     */
    void updateStats();

    /**
     * @brief Internal watchdog task for health monitoring
     * 
     * @param pvParameters Task parameters
     */
    static void watchdogTask(void *pvParameters);

    /**
     * @brief Internal event handler for client events
     * 
     * @param event_type Event type
     * @param event_data Event data
     */
    void handleClientEvent(int event_type, void* event_data);

    /**
     * @brief Internal event handler for server events
     * 
     * @param event_type Event type
     * @param event_data Event data
     */
    void handleServerEvent(int event_type, void* event_data);

private:
    /**************************************************************************/
    /*                              Member Variables                          */
    /**************************************************************************/

    ble_library_config_t config_;              ///< Library configuration
    ble_state_t state_;                        ///< Current library state
    esp_err_t last_error_;                     ///< Last error code
    
    BLEClient* client_;                        ///< BLE client instance
    BLEServer* server_;                        ///< BLE server instance
    
    bool client_initialized_;                  ///< Client initialization status
    bool server_initialized_;                  ///< Server initialization status
    bool common_initialized_;                  ///< Common BLE stack initialization status
    
    uint64_t init_time_;                       ///< Library initialization timestamp
    uint64_t start_time_;                      ///< Library start timestamp
    
    // Callbacks
    ble_event_callback_t event_callback_;      ///< Global event callback
    ble_log_callback_t log_callback_;          ///< Custom log callback
    
    // Synchronization
    SemaphoreHandle_t state_mutex_;            ///< State protection mutex
    
    // Tasks
    TaskHandle_t watchdog_task_handle_;        ///< Watchdog task handle
    
    // Statistics
    uint32_t restart_count_;                   ///< Number of restarts
    uint64_t total_uptime_;                    ///< Total uptime across restarts
};

/******************************************************************************/
/*                              Convenience Functions                         */
/******************************************************************************/

/**
 * @brief Creates a client-only BLE library instance
 * 
 * @param device_name Name of this device
 * @param target_server Name of server to connect to
 * @return BLELibrary* Configured library instance
 */
BLELibrary* createBLEClient(const char* device_name, const char* target_server);

/**
 * @brief Creates a server-only BLE library instance
 * 
 * @param device_name Name of this device for advertising
 * @return BLELibrary* Configured library instance
 */
BLELibrary* createBLEServer(const char* device_name);

/**
 * @brief Creates a dual-mode BLE library instance
 * 
 * @param device_name Name of this device
 * @param target_server Name of server to connect to (for client mode)
 * @return BLELibrary* Configured library instance
 */
BLELibrary* createBLEDual(const char* device_name, const char* target_server);

/******************************************************************************/
/*                                 Examples                                   */
/******************************************************************************/

namespace BLEExamples {

/**
 * @brief Simple client example
 * 
 * Shows how to create a basic BLE client that connects to a server
 * and reads data periodically.
 */
void simpleClientExample();

/**
 * @brief Simple server example
 * 
 * Shows how to create a basic BLE server that advertises services
 * and serves data to connected clients.
 */
void simpleServerExample();

/**
 * @brief Dual mode example
 * 
 * Shows how to create a device that acts as both client and server
 * simultaneously, connecting to one device while serving others.
 */
void dualModeExample();

/**
 * @brief Secure connection example
 * 
 * Demonstrates advanced security features including custom UUIDs,
 * authentication, and encrypted connections.
 */
void secureConnectionExample();

/**
 * @brief Multiple clients example
 * 
 * Shows how to create a server that handles multiple simultaneous
 * client connections with individual session management.
 */
void multipleClientsExample();

} // namespace BLEExamples