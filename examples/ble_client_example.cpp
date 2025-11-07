/*******************************************************************************
 * @file ble_client_example.cpp
 * @brief BLE client example targeting device by name (SenseAI_BLE)
 * @details Demonstrates connection to device by name instead of MAC address
 *
 * @version 0.0.7
 * @date 2025-10-22
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "ble_sense.hpp"
#include "esp_log.h"
#include "nvs_flash.h"

/******************************************************************************/
/*                              Constants                                     */
/******************************************************************************/

static const char* TAG = "BLE_NAME_TARGET";

// Nombre del dispositivo target - ESTE ES EL PRINCIPAL
const char* TARGET_DEVICE_NAME = "SenseAI_BLE";

/******************************************************************************/
/*                         Global Variables                                   */
/******************************************************************************/

uint32_t counter = 0;
static BLEClient* g_client = nullptr; // Variable global para acceder desde callbacks

/******************************************************************************/
/*                         Callback Functions                                */
/******************************************************************************/

// Callback cuando se encuentra cualquier dispositivo
void onAnyDeviceFound(const bleDeviceInfo_t* deviceInfo, bool isTarget) {
    // Solo mostrar dispositivos con nombre conocido
    if (strlen(deviceInfo->name) > 0) {
        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                deviceInfo->address[0], deviceInfo->address[1],
                deviceInfo->address[2], deviceInfo->address[3],
                deviceInfo->address[4], deviceInfo->address[5]);

        ESP_LOGI(TAG, " Device Found: '%s' | MAC: %s | RSSI: %d dBm %s", 
                 deviceInfo->name, macStr, deviceInfo->rssi,
                 isTarget ? "[TARGET]" : "");

        // Usar el parámetro isTarget en lugar de verificar nombre nuevamente
        if (isTarget && g_client != nullptr && !g_client->isConnected()) {
            ESP_LOGI(TAG, " TARGET VERIFIED! Stopping scan and connecting...");
            
            // Solo detener scan - la conexión se manejará en el callback de scan completed
            g_client->stopScan();
        }
    }
}

// Callback cuando se conecta exitosamente
void onConnected(const bleDeviceInfo_t* deviceInfo) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
            deviceInfo->address[0], deviceInfo->address[1], deviceInfo->address[2],
            deviceInfo->address[3], deviceInfo->address[4], deviceInfo->address[5]);

    ESP_LOGI(TAG, "\n🎉 CONNECTION SUCCESSFUL! 🎉");
    ESP_LOGI(TAG, "   Device: %s", deviceInfo->name);
    ESP_LOGI(TAG, "   MAC: %s", macStr);
    ESP_LOGI(TAG, "   RSSI: %d dBm\n", deviceInfo->rssi);
}

// Callback cuando se desconecta
void onDisconnected(int reason, bool wasPlanned) {
    ESP_LOGW(TAG, " DISCONNECTED - Reason: %d, Planned: %s", reason,
             wasPlanned ? "YES" : "NO");
    
    if (!wasPlanned && g_client != nullptr) {
        ESP_LOGI(TAG, " Unexpected disconnection, restarting search...");
        // Reiniciar búsqueda después de desconexión inesperada
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (!g_client->isScanning()) {
            g_client->startDiscoveryScan(10000);
        }
    }
}

// Callback cuando inicia el scan
void onScanStarted(uint32_t duration) {
    ESP_LOGI(TAG, " 🔍 Scanning for %lu ms...", duration);
}

// Callback cuando termina el scan
void onScanCompleted(uint32_t found, bool targetFound) {
    ESP_LOGI(TAG, " Scan completed: %lu devices found", found);
    if (!targetFound && g_client != nullptr && !g_client->isConnected()) {
        ESP_LOGI(TAG, " Target not found, restarting scan...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        g_client->startDiscoveryScan(10000);
    }
}

/******************************************************************************/
/*                              Main Function                                 */
/******************************************************************************/

extern "C" void app_main() {
    ESP_LOGI(TAG, "\n=== BLE CLIENT NAME TARGETING EXAMPLE ===");
    ESP_LOGI(TAG, "Target Device Name: %s", TARGET_DEVICE_NAME);
    ESP_LOGI(TAG, "========================================\n");

    // Step 1: Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, " NVS initialized");

    // Step 2: Create BLE Library configuration with autoStart DISABLED
    bleLibraryConfig_t bleConfig;
    BLELibrary::createDefaultConfig(&bleConfig, BLE_MODE_CLIENT_ONLY);
    strncpy(bleConfig.deviceName, "NameTargetClient",
            BLE_MAX_DEVICE_NAME_LEN - 1);

    // CRITICAL: Disable auto-start for manual configuration
    bleConfig.autoStart = false;

    ESP_LOGI(TAG, "Creating BLE Library...");
    ESP_LOGI(TAG, "Auto-start disabled for manual configuration");
    BLELibrary* ble = new BLELibrary(bleConfig);
    if (ble == nullptr) {
        ESP_LOGE(TAG, " Failed to create BLE Library");
        return;
    }
    ESP_LOGI(TAG, " BLE Library created");

    // Step 3: Initialize the library
    ESP_LOGI(TAG, "\n--- Initializing BLE Library ---");
    ret = ble->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Failed to initialize BLE: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, " BLE Library initialized");

    // Step 4: Get the client
    BLEClient* client = ble->getClient();
    if (client == nullptr) {
        ESP_LOGE(TAG, " Failed to get BLE Client");
        delete ble;
        return;
    }
    ESP_LOGI(TAG, " BLE Client obtained");

    // Set global client pointer for callbacks
    g_client = client;

    // Step 5: Configure client for NAME-based search
    ESP_LOGI(TAG, "\n--- Configuring Client for Name-Based Search ---");

    bleClientConfig_t clientConfig;
    memset(&clientConfig, 0, sizeof(clientConfig));

    // Configure target device NAME (not MAC)
    strncpy(clientConfig.targetDeviceName, TARGET_DEVICE_NAME,
            BLE_MAX_DEVICE_NAME_LEN - 1);

    // Set MAC to all zeros (no específico MAC targeting)
    memset(clientConfig.targetServerMACadd, 0x00, ESP_BD_ADDR_LEN);

    // Configure timeouts
    clientConfig.scanTimeout = 10000;
    clientConfig.connectionTimeout = 10000;
    clientConfig.autoReconnect = true;
    clientConfig.reconnectInterval = 5000;
    clientConfig.enableNotifications = true;
    clientConfig.readInterval = 5000;

    ret = client->setConfig(clientConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Failed to set client config: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, " Client configuration applied");

    // Step 6: Configure security
    ESP_LOGI(TAG, "\n--- Configuring Security ---");
    bleSecurityConfig_t secConfig;
    ret = bleCreateDefaultSecurityConfig(&secConfig, BLE_SECURITY_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Failed to create security config: %s",
                 esp_err_to_name(ret));
        delete ble;
        return;
    }

    ret = client->setSecurityConfig(secConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Failed to apply security config: %s",
                 esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, " Security config applied successfully");

    // Step 7: Setup callbacks BEFORE starting
    ESP_LOGI(TAG, "\n--- Setting up Callbacks ---");

    // Set all callbacks using function pointers
    client->setAnyDeviceFoundCallback(onAnyDeviceFound);
    client->setConnectedCallback(onConnected);
    client->setDisconnectedCallback(onDisconnected);
    client->setScanStartedCallback(onScanStarted);
    client->setScanCompletedCallback(onScanCompleted);

    ESP_LOGI(TAG, " Callbacks configured");

    // Step 8: Start BLE services
    ESP_LOGI(TAG, "\n--- Starting BLE Services ---");
    ret = ble->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Failed to start BLE services: %s",
                 esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, "BLE Services started");

    // Step 9: Main monitoring loop (MUY SIMPLIFICADO)
    ESP_LOGI(TAG, "\n🔎 Starting search for device: %s\n", TARGET_DEVICE_NAME);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        counter++;

        // Si está conectado, solo mostrar datos
        if (client->isConnected()) {
            // Mostrar datos cada 5 segundos
            if (counter % 5 == 0) {
                const bleDataPacket_t* data = client->kgetLastData();
                if (data != nullptr && data->valid) {
                    ESP_LOGI(TAG, " 📊 Data: Battery=%d%%, Custom=%s",
                             data->batteryLevel, data->customData);
                } else {
                    ESP_LOGD(TAG, " Requesting fresh data...");
                    client->readBatteryLevel();
                    client->readCustomData();
                }
            }
            
            // Status menos frecuente cuando conectado
            if (counter % 30 == 0) {
                ESP_LOGI(TAG, "\n=== ✅ Connected Status (t=%lu s) ===", counter);
                ESP_LOGI(TAG, "State: %s", client->kgetStateString());
                
                bleClientStats_t stats = client->getStats();
                ESP_LOGI(TAG, "Stats: %lu scans, %lu/%lu connections", 
                         stats.scanCount, stats.successfulConnections, stats.connectionAttempts);
                ESP_LOGI(TAG, "=====================================\n");
            }
        } 
        // Si NO está conectado, verificar que esté escaneando
        else {
            // Status cada 10 segundos cuando desconectado
            if (counter % 10 == 0) {
                ESP_LOGI(TAG, "\n=== 🔍 Searching Status (t=%lu s) ===", counter);
                ESP_LOGI(TAG, "State: %s", client->kgetStateString());
                ESP_LOGI(TAG, "Scanning: %s", client->isScanning() ? "YES" : "NO");
                
                bleClientStats_t stats = client->getStats();
                ESP_LOGI(TAG, "Stats: %lu scans, %lu/%lu connections", 
                         stats.scanCount, stats.successfulConnections, stats.connectionAttempts);
                ESP_LOGI(TAG, "=====================================\n");

                // Reiniciar scan si no está escaneando y no está conectado
                if (!client->isScanning()) {
                    ESP_LOGW(TAG, " Restarting discovery scan...");
                    client->startDiscoveryScan(10000);
                }
            }
        }
    }

    // Cleanup (never reached)
    delete ble;
}