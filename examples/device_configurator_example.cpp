/*******************************************************************************
 * @file device_configurator_example.cpp
 * @brief Simple usage example of DeviceConfigurator library
 * 
 * This example demonstrates the complete usage of the DeviceConfigurator library,
 * showing both successful configuration flows and error handling scenarios.
 *
 * @version 0.0.1
 * @date 2025-08-25
 * @author Isabella Garcia <isa@sense-ai.co>
 *******************************************************************************
 *******************************************************************************/

// System includes
#include <cstring>

// ESP-IDF includes
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project includes
#include "device_configurator.hpp"

// Application tag for logging
static const char* TAG = "MAIN";

/******************************************************************************/
/*                              Function Prototypes                          */
/******************************************************************************/

static void handleConfigurationResult(device_config_result_t result);
static void handleTimeoutError(void);
static void handleGeneralError(void);
static void handleMemoryError(void);
static void handleNVSError(void);
static void handleUnknownError(void);
static void startMainApplication(device_config_result_t result);
static void displayMemoryStatus(void);
static void runConnectedMode(void);
static void runOfflineMode(void);

/******************************************************************************/
/*                              Main Application                             */
/******************************************************************************/

extern "C" void app_main() 
{
    ESP_LOGI(TAG, "=== DeviceConfigurator Complete Resource Cleanup Example ===");
    
    // Clear credentials for testing BLE configuration flow
    esp_err_t clear_result = DeviceConfigurator::clearCredentials();
    if (clear_result == ESP_OK) {
        ESP_LOGI(TAG, "Credentials cleared - will use BLE configuration");
    } else {
        ESP_LOGW(TAG, "Could not clear credentials: %s", esp_err_to_name(clear_result));
    }
    
    // ONE LINE CONFIGURATION - ALL resources (WiFi + BLE) automatically cleaned up after
    device_config_result_t result = DeviceConfigurator::init();

    // Handle configuration result
    handleConfigurationResult(result);
    
    // Start main application with clean system state
    startMainApplication(result);
}

/******************************************************************************/
/*                              Helper Functions                             */
/******************************************************************************/

/**
 * @brief Handle the configuration result and perform appropriate cleanup
 * @param result The configuration result to handle
 */
static void handleConfigurationResult(device_config_result_t result)
{
    switch (result) {
        case DEVICE_CONFIG_SUCCESS:
            ESP_LOGI(TAG, "SUCCESS: Generic configuration success");
            ESP_LOGI(TAG, "Configuration completed, all networking resources now freed");
            break;
            
        case DEVICE_CONFIG_WIFI_CONNECTED:
            ESP_LOGI(TAG, "SUCCESS: WiFi connected with saved credentials");
            ESP_LOGI(TAG, "Configuration confirmed, all networking resources now freed");
            break;
            
        case DEVICE_CONFIG_BLE_SUCCESS:
            ESP_LOGI(TAG, "SUCCESS: WiFi connected with BLE-provided credentials");
            ESP_LOGI(TAG, "Configuration complete, all networking resources now freed");
            break;
            
        case DEVICE_CONFIG_TIMEOUT:
            handleTimeoutError();
            break;
            
        case DEVICE_CONFIG_ERROR:
            handleGeneralError();
            break;
            
        case DEVICE_CONFIG_MEMORY_ERROR:
            handleMemoryError();
            break;
            
        case DEVICE_CONFIG_NVS_ERROR:
            handleNVSError();
            break;
            
        default:
            handleUnknownError();
            break;
    }
}

/**
 * @brief Handle timeout error scenario
 */
static void handleTimeoutError(void)
{
    ESP_LOGW(TAG, "TIMEOUT: No BLE client connected or no credentials received");
    ESP_LOGW(TAG, "Attempting emergency cleanup...");
    
    esp_err_t cleanup_result = DeviceConfigurator::forceCleanupAll();
    if (cleanup_result == ESP_OK) {
        ESP_LOGW(TAG, "Emergency cleanup completed successfully");
    } else {
        ESP_LOGE(TAG, "Emergency cleanup failed - system may need restart");
    }
}

/**
 * @brief Handle general error scenario
 */
static void handleGeneralError(void)
{
    ESP_LOGE(TAG, "ERROR: General configuration error - attempting force cleanup");
    DeviceConfigurator::forceCleanupAll();
    ESP_LOGE(TAG, "Force cleanup attempted");
}

/**
 * @brief Handle memory error scenario
 */
static void handleMemoryError(void)
{
    ESP_LOGE(TAG, "ERROR: Memory allocation error - attempting force cleanup");
    DeviceConfigurator::forceCleanupAll();
    ESP_LOGE(TAG, "Force cleanup attempted");
}

/**
 * @brief Handle NVS error scenario
 */
static void handleNVSError(void)
{
    ESP_LOGE(TAG, "ERROR: NVS storage error - attempting force cleanup");
    DeviceConfigurator::forceCleanupAll();
    ESP_LOGE(TAG, "Force cleanup attempted");
}

/**
 * @brief Handle unknown error scenario
 */
static void handleUnknownError(void)
{
    ESP_LOGE(TAG, "ERROR: Unknown configuration result - attempting force cleanup");
    DeviceConfigurator::forceCleanupAll();
    ESP_LOGE(TAG, "Force cleanup attempted");
}

/**
 * @brief Start the main application based on configuration result
 * @param result The configuration result
 */
static void startMainApplication(device_config_result_t result)
{
    // Application starts here - NO WiFi or BLE resources active
    ESP_LOGI(TAG, "=== MAIN APPLICATION START ===");
    ESP_LOGI(TAG, "Starting with completely clean system state");
    
    // Display available memory after complete cleanup
    displayMemoryStatus();
    
    if ((result == DEVICE_CONFIG_WIFI_CONNECTED) || (result == DEVICE_CONFIG_BLE_SUCCESS)) {
        runConnectedMode();
    } else {
        runOfflineMode();
    }
}

/**
 * @brief Display current memory status
 */
static void displayMemoryStatus(void)
{
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_heap = esp_get_minimum_free_heap_size();
    
    ESP_LOGI(TAG, "Available memory: %lu bytes free, minimum was %lu", free_heap, min_heap);
    ESP_LOGI(TAG, "Maximum memory available for your application");
}

/**
 * @brief Run application in connected mode (WiFi credentials available)
 */
static void runConnectedMode(void)
{
    ESP_LOGI(TAG, "=== CREDENTIALS CONFIRMED - APPLICATION READY ===");
    ESP_LOGI(TAG, "WiFi credentials validated and saved to NVS");
    ESP_LOGI(TAG, "Application can now initialize its own networking if needed");
    
    // Main application loop - completely clean system
    int counter = 0;
    const int MEMORY_CHECK_INTERVAL = 10;
    const TickType_t TASK_DELAY_MS = pdMS_TO_TICKS(30000); // 30 seconds
    
    while (true) {
        ESP_LOGI(TAG, "Application running - counter: %d (clean system)", counter);
        counter++;
        
        // Periodic memory status check
        if (counter % MEMORY_CHECK_INTERVAL == 0) {
            uint32_t current_heap = esp_get_free_heap_size();
            ESP_LOGI(TAG, "Memory: %lu bytes free (no WiFi/BLE overhead)", current_heap);
            ESP_LOGI(TAG, "Device ready for any networking/sensor/application logic");
        }
        
        vTaskDelay(TASK_DELAY_MS);
    }
}

/**
 * @brief Run application in offline mode (no WiFi credentials)
 */
static void runOfflineMode(void)
{
    ESP_LOGW(TAG, "=== NO WIFI CREDENTIALS - OFFLINE MODE ===");
    ESP_LOGW(TAG, "No credentials available, running in offline mode");
    ESP_LOGW(TAG, "All resources still cleaned up - maximum memory available");
    
    const TickType_t OFFLINE_DELAY_MS = pdMS_TO_TICKS(60000); // 1 minute
    
    // Handle non-connected state - still with clean system
    while (true) {
        ESP_LOGI(TAG, "Running in offline mode (clean system, max memory)...");
        
        vTaskDelay(OFFLINE_DELAY_MS);
    }
}
