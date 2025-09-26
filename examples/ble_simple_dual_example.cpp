/*******************************************************************************
 * @file simple_dual_example.cpp
 * @brief basic example of dual BLE mode (client + server) using default settings
 * @version 0.0.1
 * @date 2025-08-27
 * @author SenseAI Team
 *******************************************************************************
 *******************************************************************************/

#include "ble.hpp"

#include "esp_log.h"

#include "nvs_flash.h"

static const char* TAG = "SIMPLE_BLE_DUAL";

extern "C" void app_main() {
    ESP_LOGI(TAG, "<<< Basic BLE Dual mode (Modo Dual BLE Simple) >>>");

    // Init NVS necessary to BLE
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Show library information
    BLELibrary::printLibraryInfo();

    // Create dual device with all defaults
    // First parameter: device name (nullptr = use "SenseAI_BLE")
    // Second parameter: target name to search as client (nullptr = use "SenseAI_BLE")
    BLELibrary* ble = createBLEDual(nullptr, nullptr);
    
    if (ble == nullptr) {
        ESP_LOGE(TAG, "Error creating dual BLE device");
        return;
    }

    // Configure callbacks for the server
    BLEServer* server = ble->getServer();
    if (server != nullptr) {
        server->setClientConnectedCallback([](uint16_t connID, const bleDeviceInfo_t* clientInfo) {
            ESP_LOGI(TAG, "[SERVER] Client connected - ID: %d", connID);
        });

        server->setClientDisconnectedCallback([](uint16_t connID, int reason) {
            ESP_LOGI(TAG, "[SERVER] Client disconnected - ID: %d", connID);
        });

        server->setDataWrittenCallback([](uint16_t connID, const uint8_t* data, uint16_t length) {
            ESP_LOGI(TAG, "[SERVER] Data from client %d: %.*s", connID, length, data);
        });
    }

    // Configure callbacks for the client
    BLEClient* client = ble->getClient();
    if (client != nullptr) {
        client->setConnectedCallback([](const bleDeviceInfo_t* deviceInfo) {
            ESP_LOGI(TAG, "[CLIENT] Connected to server: %s (RSSI: %d dBm)", 
                     deviceInfo->name, deviceInfo->rssi);
        });

        client->setDisconnectedCallback([](int reason, bool wasPlanned) {
            ESP_LOGI(TAG, "[CLIENT] Disconnected from server (reason: %d)", reason);
        });

        client->setDataReceivedCallback([](const bleDataPacket_t* data) {
            ESP_LOGI(TAG, "[CLIENT] Data from server - Battery: %d%%, Custom: %s", 
                     data->batteryLevel, data->customData);
        });

        client->setAnyDeviceFoundCallback([](const bleDeviceInfo_t* deviceInfo, bool isTarget) {
            ESP_LOGI(TAG, "[CLIENT] Device: %s (RSSI: %d) - %s", 
                     deviceInfo->name, deviceInfo->rssi, 
                     isTarget ? "Target" : " NO");
        });
    }

    // Show actual configurations
    ESP_LOGI(TAG, "\n<<< APPLIED CONFIGURATIONS >>>");
    if (server != nullptr) {
        bleServerConfig_t serverConfig = server->getConfig();
        ESP_LOGI(TAG, "[SERVER]");
        ESP_LOGI(TAG, "   Name: %s", serverConfig.deviceName);
        ESP_LOGI(TAG, "   Max clients: %d", serverConfig.maxClients);
        ESP_LOGI(TAG, "   JSON commands: %s", serverConfig.enableJsonCommands ? "YES" : "NO");
    }
    
    if (client != nullptr) {
        bleClientConfig_t clientConfig = client->getConfig();
        ESP_LOGI(TAG, "[CLIENT]");
        ESP_LOGI(TAG, "   Target: %s", clientConfig.targetDeviceName);
        ESP_LOGI(TAG, "   Auto-reconnect: %s", clientConfig.autoReconnect ? "YES" : "NO");
        ESP_LOGI(TAG, "   Scan timeout: %lu ms", clientConfig.scanTimeout);
    }

    // Loop principal
    uint32_t loopCounter = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // wait 1 second
        loopCounter++;

        // Mostrar estadísticas cada 12 segundos
        if (loopCounter % 12 == 0) {
            ESP_LOGI(TAG, "\n === DUAL STATISTICS ===");
            ESP_LOGI(TAG, " Active time: %llu minutes", ble->getUptime() / 60000);
            ESP_LOGI(TAG, " General status: %s", BLELibrary::kgetStateString(ble->getState()));
            ESP_LOGI(TAG, "  Server active: %s", ble->isServerActive() ? "YES" : "NO");
            ESP_LOGI(TAG, " Client active: %s", ble->isClientActive() ? "YES" : "NO");
            ESP_LOGI(TAG, " Free memory: %lu bytes", esp_get_free_heap_size());

            // Estadísticas del servidor
            if (server != nullptr) {
                bleServerStatus_t serverStatus = server->getStatus();
                bleServerStats_t serverStats = server->getStats();
                ESP_LOGI(TAG, "\n  [SERVER DATA]");
                ESP_LOGI(TAG, "   State: %s", server->kgetStateString());
                ESP_LOGI(TAG, "   Connected clients: %d", serverStatus.connectedClients);
                ESP_LOGI(TAG, "   Advertising active: %s", serverStatus.advertisingActive ? "YES" : "NO");
                ESP_LOGI(TAG, "   Total connections: %lu", serverStats.totalConnections);
                ESP_LOGI(TAG, "   Data sent: %lu bytes", serverStats.dataSent);
                ESP_LOGI(TAG, "   Data received: %lu bytes", serverStats.dataReceived);

                // Actualizar datos personalizados del servidor
                char customData[50];
                snprintf(customData, sizeof(customData), "Dual-Uptime:%llu", ble->getUptime()/1000);
                server->setCustomData(customData);
            }

            // Estadísticas del cliente
            if (client != nullptr) {
                bleClientStats_t clientStats = client->getStats();
                ESP_LOGI(TAG, "\n [CLIENT]");
                ESP_LOGI(TAG, "   State: %s", client->kgetStateString());
                ESP_LOGI(TAG, "   Connected: %s", client->isConnected() ? "YES" : "NO");
                ESP_LOGI(TAG, "   Scanning: %s", client->isScanning() ? "YES" : "NO");
                ESP_LOGI(TAG, "   Connection attempts: %lu", clientStats.connectionAttempts);
                ESP_LOGI(TAG, "   Successful connections: %lu", clientStats.successfulConnections);
                ESP_LOGI(TAG, "   Data received: %lu bytes", clientStats.dataReceived);

                if (client->isConnected()) {
                    const bleDeviceInfo_t* connectedDevice = client->kgetConnectedDevice();
                    if (connectedDevice != nullptr) {
                        ESP_LOGI(TAG, "   Connected to: %s (RSSI: %d dBm)", 
                                connectedDevice->name, connectedDevice->rssi);
                    }
                }
            }
            ESP_LOGI(TAG, "<<<<<<<<<<<>>>>>>>\n");
        }
    }

    // Cleanup (nunca se alcanza en este ejemplo)
    delete ble;
}
