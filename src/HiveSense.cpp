/*******************************************************************************
 * @file main.cpp
 * @brief Server_example to use BLE library
 * @version 0.0.5
 * @date 2025-06-19
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "ble_sense.hpp"
#include "wifi_sense.hpp"
#include "ble_server.hpp"
// #include "device_configurator.hpp"
#include "HiveConfig.hpp"
#include "aws_sense.hpp"

#include "esp_log.h"
#include <inttypes.h>

#include "nvs_flash.h"

WifiHandler* wifiHandler = nullptr;
awsHandler* awsHelper = nullptr;

extern "C" void app_main() {
    ESP_LOGI(TAG, "=== HIVE FIRST STEPS ===");

    wifiHandler = new WifiHandler(userSSID, userPassword);

    if (wifiHandler->connectWifi() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi.");
        return;
    }

    ESP_LOGI(TAG, "Connected to WiFi!");
    awsHelper = new awsHandler(*wifiHandler);

    if (awsHelper->init(AWS_IOT_ENDPOINT, THINGNAME, AWS_ServerCA, AWS_ClientCertificate, AWS_ClientKey) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AWS helper");
        return;
    }
    
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "NVS flash erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS init corretly");

    // Mostrar datos de la librería
    BLELibrary::printLibraryInfo();

    // Crear configuración personalizada
    bleLibraryConfig_t config;
    ret = BLELibrary::createDefaultConfig(&config, BLE_MODE_SERVER_ONLY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fail creating configuration: %s", esp_err_to_name(ret));
        return;
    }
    
    // Personalizar configuración
    strncpy(config.deviceName, "SenseAI_BLE", BLE_MAX_DEVICE_NAME_LEN - 1);
    config.autoStart = false; // Controlaremos manualmente
    config.enableLogging = true;
    config.logLevel = ESP_LOG_INFO;

    BLELibrary* ble = new BLELibrary(config);
    if (ble == nullptr) {
        ESP_LOGE(TAG, "Fail creating ble_library");
        return;
    }

    ret = ble->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fail initializating ble_library: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }

    BLEServer* server = ble->getServer();
    if (server != nullptr) {
        ESP_LOGI(TAG, "Settings server");
        
        bleServerConfig_t server_config = server->getConfig();
        server_config.dataUpdateInterval = 0;    
        server_config.clientTimeout = 60000;     // 60 segundos timeout en lugar de 0
        server_config.maxClients = 4;
        server_config.enableNotifications = true;
        server_config.autoStartAdvertising = false;
        server_config.enableJsonCommands = true;

        ret = server->setConfig(server_config);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Warnings in settings server: %s", esp_err_to_name(ret));
        }

        // Configurar callbacks básicos con más logging
        server->setClientConnectedCallback([](uint16_t connID, const bleDeviceInfo_t* clientInfo) {
            ESP_LOGI(TAG, " Client connected - ID: %d, MAC: %02x:%02x:%02x:%02x:%02x:%02x", 
                    connID,
                    clientInfo->address[0], clientInfo->address[1], clientInfo->address[2],
                    clientInfo->address[3], clientInfo->address[4], clientInfo->address[5]);
        });

        server->setClientDisconnectedCallback([](uint16_t connID, int reason) {
            ESP_LOGI(TAG, " Client disconnected - ID: %d, Reason: %d", connID, reason);
        });

        server->setAdvertisingCallback([](bool isStarted, esp_err_t result) {
            if (isStarted && result == ESP_OK) {
                ESP_LOGI(TAG, " Advertising STARTED successfully");
            } else if (!isStarted || result != ESP_OK) {
                ESP_LOGE(TAG, " Advertising STOPPED or FAILED: %s", esp_err_to_name(result));
            }
        });

        server->setDataWrittenCallback([](uint16_t connID, const uint8_t* data, uint16_t length) {
            ESP_LOGI(TAG, " Client %d data received: %.*s", connID, length, data);
            // Echo de vuelta los datos recibidos para testing
            if (length > 0) {
                char response[128];
                snprintf(response, sizeof(response), "Server received: %.*s", length, data);
                // Connect to AWS IoT Core via MQTT over TLS.
                if (awsHelper->connect() != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to connect to AWS IoT Core");
                    return;
                }

                // Publish the current time to an AWS IoT Core topic (change the topic as needed).
                if (awsHelper->publish(mqttDataTopic, response) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to publish data to AWS");
                } else {
                    ESP_LOGI(TAG, "Data published to AWS successfully");
                }

                // Disconnect from AWS IoT Core.
                awsHelper->disconnect();
            }
        });

        // Manual Advertising
        ESP_LOGI(TAG, "Init advertising...");
        ret = server->startAdvertising();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Fail init advertising: %s", esp_err_to_name(ret));
            delete ble;
            return;
        }
        
        ESP_LOGI(TAG, "loop basic");
        
        int loopCount = 0;
        int lastConnectedClients = 0;
        while (true) {
            loopCount++;

            // Check for connection changes every loop
            bleServerStatus_t status = server->getStatus();
            if ((int)status.connectedClients != lastConnectedClients) {
                ESP_LOGI(TAG, " Connection count changed: %d -> %d", 
                        lastConnectedClients, (int)status.connectedClients);
                lastConnectedClients = (int)status.connectedClients;
            }
            
            // Verificar y forzar advertising si no está activo y no hay clientes máximos
            if (!status.advertisingActive && status.connectedClients < 4) {
                ESP_LOGW(TAG, "  Advertising not active with %d clients - forcing restart...", 
                        (int)status.connectedClients);
                esp_err_t ret = server->startAdvertising();
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, " Advertising restarted successfully");
                } else {
                    ESP_LOGE(TAG, " Failed to restart advertising: %s", esp_err_to_name(ret));
                }
            }

            if (loopCount % 30 == 0) {
                bleServerStats_t stats = server->getStats();
                
                ESP_LOGI(TAG, "===============================================");
                ESP_LOGI(TAG, " SERVER STATUS REPORT - Loop %d", loopCount);
                ESP_LOGI(TAG, "===============================================");
                ESP_LOGI(TAG, "   State: %s", server->kgetStateString());
                ESP_LOGI(TAG, "   Clients connected: %d/%d", (int)status.connectedClients, 4);
                ESP_LOGI(TAG, "    Advertising: %s", status.advertisingActive ? " ACTIVE" : " INACTIVE");
                ESP_LOGI(TAG, "   Total connections: %lu", stats.totalConnections);
                ESP_LOGI(TAG, "   Data sent: %lu | Data received: %lu", stats.dataSent, stats.dataReceived);
                ESP_LOGI(TAG, "   Uptime: %llu seconds", ble->getUptime()/1000000);
                ESP_LOGI(TAG, "   Free memory: %lu bytes", esp_get_free_heap_size());
                ESP_LOGI(TAG, "===============================================");
                
                char status_data[64];
                snprintf(status_data, sizeof(status_data), "Uptime:%llu Loop:%d Clients:%d ADV:%s", 
                        ble->getUptime()/1000000, loopCount, (int)status.connectedClients,
                        status.advertisingActive ? "ON" : "OFF");
                server->setCustomData(status_data);
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
    } else {
        ESP_LOGE(TAG, "Get server not found");
        delete ble;
        return;
    }
    delete ble;
}
