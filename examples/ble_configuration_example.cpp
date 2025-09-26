/*******************************************************************************
 * @file ble_configuration_example.cpp
 * @brief Advanced BLE configuration example demonstrating custom settings
 * @details Shows how to create custom configurations for different use cases
 *          including security settings, performance tuning, and mode selection.
 * 
 * @version 0.0.1
 * @date 2025-08-29
 * @author SenseAI Team
 *******************************************************************************
 *******************************************************************************/

#include "ble.hpp"
#include "esp_log.h"
#include "nvs_flash.h"

/******************************************************************************/
/*                              Constants                                     */
/******************************************************************************/

static const char* TAG = "BLE_CONFIG_EXAMPLE";

/******************************************************************************/
/*                           Helper Functions                                 */
/******************************************************************************/

/**
 * @brief Create a high-security server configuration
 * @param config Pointer to configuration structure to fill
 */
void createSecureServerConfig(bleLibraryConfig_t* config) {
    // Start with default configuration
    BLELibrary::createDefaultConfig(config, BLE_MODE_SERVER_ONLY);
    
    // Customize for high security
    strncpy(config->deviceName, "SecureDevice", BLE_MAX_DEVICE_NAME_LEN - 1);
    config->security.level = BLE_SECURITY_ENCRYPTED;
    config->security.requireAuthentication = true;
    config->security.useCustomUUIDS = true;
    
    // Performance and reliability settings
    config->enableLogging = true;
    config->logLevel = ESP_LOG_INFO;
    config->watchdogTimeOut = 45000; // 45 seconds
    config->autoStart = false; // Manual control
}

/**
 * @brief Create a low-power client configuration
 * @param config Pointer to configuration structure to fill
 */
void createLowPowerClientConfig(bleLibraryConfig_t* config) {
    // Start with default configuration
    BLELibrary::createDefaultConfig(config, BLE_MODE_CLIENT_ONLY);
    
    // Optimize for low power consumption
    strncpy(config->deviceName, "LowPowerClient", BLE_MAX_DEVICE_NAME_LEN - 1);
    config->security.level = BLE_SECURITY_BASIC;
    
    // Logging settings for power saving
    config->enableLogging = true;
    config->logLevel = ESP_LOG_WARN; // Reduce log output
    config->watchdogTimeOut = 60000; // Longer timeout
}

/******************************************************************************/
/*                              Main Function                                 */
/******************************************************************************/

extern "C" void app_main() {
    ESP_LOGI(TAG, "=== BLE Configuration Example ===");
    
    // Initialize NVS (required for BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Example 1: High-Security Server Configuration
    ESP_LOGI(TAG, "\n--- Example 1: High-Security Server ---");
    
    bleLibraryConfig_t secureConfig;
    createSecureServerConfig(&secureConfig);
    
    BLELibrary* secureServer = new BLELibrary(secureConfig);
    if (secureServer != nullptr) {
        ret = secureServer->init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Secure server initialized successfully");
            
            // Configure server-specific settings
            BLEServer* server = secureServer->getServer();
            if (server != nullptr) {
                bleServerConfig_t serverConfig = server->getConfig();
                serverConfig.maxClients = 2; // Limit connections for security
                serverConfig.requireAuthentication = true;
                serverConfig.enableJsonCommands = false; // Disable for security
                server->setConfig(serverConfig);
                
                ESP_LOGI(TAG, "Server configured for maximum security");
            }
            
            // Manual start for controlled initialization
            ret = secureServer->start();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Secure server started and advertising");
            }
        }
    }

    // Wait and demonstrate status monitoring
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    if (secureServer != nullptr) {
        bleLibraryStatus_t status = secureServer->getStatus();
        ESP_LOGI(TAG, "Server Status:");
        ESP_LOGI(TAG, "  State: %d", status.libraryState);
        ESP_LOGI(TAG, "  Uptime: %llu ms", status.libraryUptime);
        ESP_LOGI(TAG, "  Free Heap: %lu bytes", status.freeHeapSize);
        ESP_LOGI(TAG, "  Server Active: %s", status.serverActive ? "YES" : "NO");
    }

    // Example 2: Low-Power Client Configuration
    ESP_LOGI(TAG, "\n--- Example 2: Low-Power Client ---");
    
    bleLibraryConfig_t lowPowerConfig;
    createLowPowerClientConfig(&lowPowerConfig);
    
    BLELibrary* lowPowerClient = new BLELibrary(lowPowerConfig);
    if (lowPowerClient != nullptr) {
        ret = lowPowerClient->init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Low-power client initialized successfully");
            
            // Configure client for power efficiency
            BLEClient* client = lowPowerClient->getClient();
            if (client != nullptr) {
                bleClientConfig_t clientConfig = client->getConfig();
                clientConfig.scanTimeout = 30000; // Longer scan timeout
                clientConfig.reconnectInterval = 10000; // Less frequent reconnects
                clientConfig.autoReconnect = true;
                client->setConfig(clientConfig);
                
                ESP_LOGI(TAG, "Client configured for low power consumption");
            }
        }
    }

    // Main operation loop
    uint32_t loopCount = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10 seconds
        loopCount++;
        
        ESP_LOGI(TAG, "\n--- Status Report (Loop %lu) ---", loopCount);
        
        // Report on secure server
        if (secureServer != nullptr) {
            ESP_LOGI(TAG, "Secure Server running for %llu ms", 
                     secureServer->getUptime());
        }
        
        // Report on low-power client
        if (lowPowerClient != nullptr) {
            ESP_LOGI(TAG, "Low-Power Client running for %llu ms", 
                     lowPowerClient->getUptime());
        }
        
        // Memory monitoring
        ESP_LOGI(TAG, "System free heap: %lu bytes", esp_get_free_heap_size());
        
        if (loopCount >= 6) { // Run for 1 minute then restart
            ESP_LOGI(TAG, "Restarting configuration demo...");
            loopCount = 0;
        }
    }

    // Cleanup (never reached in this example)
    delete secureServer;
    delete lowPowerClient;
}
