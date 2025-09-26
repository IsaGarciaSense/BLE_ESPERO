/*******************************************************************************
 * @file simple_client_example.cpp
 * @brief Basic example to BLE client using all default settings
 * @version 0.0.1
 * @date 2025-08-27
 * @author SenseAI Team
 *******************************************************************************
 *******************************************************************************/

#include "ble.hpp"

#include "esp_log.h"

#include "nvs_flash.h"

static const char* TAG = "SIMPLE_BLE_CLIENT";

extern "C" void app_main() {
    ESP_LOGI(TAG, "<<< Simple BLE Client (Default Settings) >>>");

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

    // Create client with all defaults
    // First parameter: client name (nullptr = use "SenseAI_BLE")
    // Second parameter: server name to search (nullptr = use "SenseAI_BLE")
    BLELibrary* ble = createBLEClient(nullptr, nullptr);
    
    if (ble == nullptr) {
        ESP_LOGE(TAG, "Fail creating BLE client");
        return;
    }

    // Get the client for optional callbacks
    BLEClient* client = ble->getClient();
    if (client != nullptr) {
        // Optional callbacks (you can comment these out if you don't need them)
        client->setConnectedCallback([](const bleDeviceInfo_t* deviceInfo) {
            ESP_LOGI(TAG, " Connected to server: %s", deviceInfo->name);
            ESP_LOGI(TAG, "   RSSI: %d dBm", deviceInfo->rssi);
        });

        client->setDisconnectedCallback([](int reason, bool wasPlanned) {
            ESP_LOGI(TAG, " Disconnected from server");
            ESP_LOGI(TAG, "   Reason: %d, Planned: %s", reason, wasPlanned ? "YES" : "NO");
        });

        client->setDataReceivedCallback([](const bleDataPacket_t* data) {
            ESP_LOGI(TAG, " Data received:");
            ESP_LOGI(TAG, "   Battery: %d%%", data->batteryLevel);
            ESP_LOGI(TAG, "   Custom data: %s", data->customData);
        });

        // Callback para cualquier dispositivo encontrado durante el escaneo
        client->setAnyDeviceFoundCallback([](const bleDeviceInfo_t* deviceInfo, bool isTarget) {
            ESP_LOGI(TAG, " Device found: %s (RSSI: %d dBm) - %s", 
                     deviceInfo->name, deviceInfo->rssi, 
                     isTarget ? " TARGET" : " Other");
        });

        client->setScanStartedCallback([](uint32_t duration) {
            ESP_LOGI(TAG, " Scanning started for %lu ms", duration);
        });

        client->setScanCompletedCallback([](uint32_t devicesFound, bool targetFound) {
            ESP_LOGI(TAG, " Scanning completed:");
            ESP_LOGI(TAG, "   Devices found: %lu", devicesFound);
            ESP_LOGI(TAG, "   Target found: %s", targetFound ? "YES" : "NO");
        });

        // Show current configuration
        bleClientConfig_t config = client->getConfig();
        ESP_LOGI(TAG, "\n=== CURRENT CONFIGURATION ===");
        ESP_LOGI(TAG, "Target device: %s", config.targetDeviceName);
        ESP_LOGI(TAG, "Scan timeout: %lu ms", config.scanTimeout);
        ESP_LOGI(TAG, "Auto-reconnect: %s", config.autoReconnect ? "YES" : "NO");
        ESP_LOGI(TAG, "Reconnect interval: %lu ms", config.reconnectInterval);
        ESP_LOGI(TAG, "Notifications: %s", config.enableNotifications ? "YES" : "NO");
        ESP_LOGI(TAG, "============================");
    }

    // Loop principal simple
    uint32_t loopCounter = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Esperar 1 segundo
        loopCounter++;

        // Mostrar estadísticas cada 10 segundos
        if (loopCounter % 10 == 0) {
            if (client != nullptr) {
                ESP_LOGI(TAG, "\n === Actual data ===");
                ESP_LOGI(TAG, " Active time: %llu minutes", ble->getUptime() / 60000);
                ESP_LOGI(TAG, " Current state: %s", client->kgetStateString());
                ESP_LOGI(TAG, " Connected: %s", client->isConnected() ? "YES" : "NO");
                ESP_LOGI(TAG, " Scanning: %s", client->isScanning() ? "YES" : "NO");
                ESP_LOGI(TAG, " Free memory: %lu bytes", esp_get_free_heap_size());

                if (client->isConnected()) {
                    const bleDeviceInfo_t* device = client->kgetConnectedDevice();
                    if (device != nullptr) {
                        ESP_LOGI(TAG, " Connected server: %s (RSSI: %d dBm)", 
                                device->name, device->rssi);
                        ESP_LOGI(TAG, "  Connection time: %llu seconds", 
                                client->getConnectionUptime() / 1000000);
                    }

                    const bleDataPacket_t* lastData = client->kgetLastData();
                    if (lastData != nullptr && lastData->valid) {
                        ESP_LOGI(TAG, " Last data: Battery=%d%%, Custom=%s", 
                                lastData->batteryLevel, lastData->customData);
                    }
                }

                bleClientStats_t stats = client->getStats();
                ESP_LOGI(TAG, " Connection attempts: %lu", stats.connectionAttempts);
                ESP_LOGI(TAG, " Successful connections: %lu", stats.successfulConnections);
                ESP_LOGI(TAG, " Data received: %lu bytes", stats.dataReceived);
                ESP_LOGI(TAG, "======================\n");
            }
        }
    }

    // Cleanup (nunca se alcanza en este ejemplo)
    delete ble;
}
