/*******************************************************************************
 * @file simple_server_example.cpp
 * @brief Simple example of a BLE server using all default settings
 * @version 0.0.6
 * @date 2025-08-29
 * @author Isabella Garcia <isa@sense-ai.co>
 *******************************************************************************
 *******************************************************************************/

#include "ble.hpp"
#include "esp_log.h"
#include "nvs_flash.h"

static const char* TAG = "SIMPLE_BLE_SERVER";

extern "C" void app_main() {
    ESP_LOGI(TAG, "=== Simple BLE server (all defaults) ===");

    // Initialize NVS (required for BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS init correctly");

    // Show library information
    BLELibrary::printLibraryInfo();

    // Create server with all defaults (just pass nullptr to use default name)
    BLELibrary* ble = createBLEServer(nullptr);  // nullptr = use "SenseAI_BLE"

    if (ble == nullptr) {
        ESP_LOGE(TAG, "Fails creating BLE server");
        return;
    }

    // Get server for optional callbacks
    BLEServer* server = ble->getServer();
    if (server != nullptr) {
        // Optional callbacks (you can comment these out if you don't need them)
        server->setClientConnectedCallback([](uint16_t connID, const bleDeviceInfo_t* clientInfo) {
            ESP_LOGI(TAG, " Client connected - ID: %d", connID);
        });

        server->setClientDisconnectedCallback([](uint16_t connID, int reason) {
            ESP_LOGI(TAG, " Client disconnected - ID: %d", connID);
        });

        server->setDataWrittenCallback([](uint16_t connID, const uint8_t* data, uint16_t length) {
            ESP_LOGI(TAG, " Data received from client %d: %.*s", connID, length, data);
        });

        // Show current configuration
        bleServerConfig_t config = server->getConfig();
        ESP_LOGI(TAG, "\n=== CURRENT CONFIGURATION ===");
        ESP_LOGI(TAG, "Device Name: %s", config.deviceName);
        ESP_LOGI(TAG, "Advertising Interval: %lu ms", config.advertisingInterval);
        ESP_LOGI(TAG, "Max Clients: %d", config.maxClients);
        ESP_LOGI(TAG, "Notifications: %s", config.enableNotifications ? "YES" : "NO");
        ESP_LOGI(TAG, "JSON Commands: %s", config.enableJsonCommands ? "YES" : "NO");
        ESP_LOGI(TAG, "============================");
    }

    ESP_LOGI(TAG, "\n BLE server running...");
    ESP_LOGI(TAG, " You can connect from a BLE app using the name: SenseAI_BLE");
    ESP_LOGI(TAG, " Showing statistics every 15 seconds...");

    // Simple main loop
    uint32_t loopCounter = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second
        loopCounter++;

        // Show statistics every 15 seconds
        if (loopCounter % 15 == 0) {
            if (server != nullptr) {
                bleServerStatus_t status = server->getStatus();
                bleServerStats_t stats = server->getStats();
                
                ESP_LOGI(TAG, "\n === Stadistics ===");
                ESP_LOGI(TAG, " Active Time: %llu minutes", ble->getUptime() / 60000);
                ESP_LOGI(TAG, " Connected Clients: %d", status.connectedClients);
                ESP_LOGI(TAG, " Advertising Active: %s", status.advertisingActive ? "YES" : "NO");
                ESP_LOGI(TAG, " Total Connections: %lu", stats.totalConnections);
                ESP_LOGI(TAG, " Data Sent: %lu bytes", stats.dataSent);
                ESP_LOGI(TAG, " Data Received: %lu bytes", stats.dataReceived);
                ESP_LOGI(TAG, " Free Memory: %lu bytes", esp_get_free_heap_size());
                ESP_LOGI(TAG, "======================\n");

                // Update custom data automatically
                char customData[50];
                snprintf(customData, sizeof(customData), "Uptime:%llu Loop:%lu", 
                        ble->getUptime()/1000, loopCounter);
                server->setCustomData(customData);
            }
        }
    }

    // Cleanup (never reached in this example)
    delete ble;
}
