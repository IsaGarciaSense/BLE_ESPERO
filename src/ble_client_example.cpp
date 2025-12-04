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
const char* TARGET_DEVICE_NAME = "HiveSense";

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
    // El callback ahora solo se llama para dispositivos con nombre real
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            deviceInfo->address[0], deviceInfo->address[1],
            deviceInfo->address[2], deviceInfo->address[3],
            deviceInfo->address[4], deviceInfo->address[5]);

    if (isTarget) {
        ESP_LOGI(TAG, "*** TARGET FOUND: '%s' | MAC: %s | RSSI: %d dBm ***", 
                 deviceInfo->name, macStr, deviceInfo->rssi);
        // La librería se encarga de detener el scan y conectar automáticamente
    } else {
        // Log menos frecuente para otros dispositivos (solo debug)
        ESP_LOGD(TAG, "Device: '%s' | MAC: %s | RSSI: %d dBm", 
                 deviceInfo->name, macStr, deviceInfo->rssi);
    }
}

// Callback cuando se conecta exitosamente
void onConnected(const bleDeviceInfo_t* deviceInfo) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
            deviceInfo->address[0], deviceInfo->address[1], deviceInfo->address[2],
            deviceInfo->address[3], deviceInfo->address[4], deviceInfo->address[5]);

    ESP_LOGI(TAG, "\n CONNECTION SUCCESSFUL! 🎉");
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
    ESP_LOGI(TAG, "  Scanning for %lu ms...", duration);
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

// Enhanced callback for data received with acknowledgment options
void onDataReceivedWithAck(const bleDataPacket_t* data, bool shouldAck, bool shouldDisconnect) {
    if (!data || !data->valid) return;
    
    ESP_LOGI(TAG, "\n DATA RECEIVED WITH ACKNOWLEDGMENT OPTIONS:");
    ESP_LOGI(TAG, "   Battery: %d%%", data->batteryLevel);
    ESP_LOGI(TAG, "   Custom: %s", data->customData);
    ESP_LOGI(TAG, "   Requires ACK: %s", shouldAck ? "YES" : "NO");
    ESP_LOGI(TAG, "   Should Disconnect: %s\n", shouldDisconnect ? "YES" : "NO");
    
    // Manual acknowledgment demonstration (if auto-ack is disabled)
    if (shouldAck && g_client != nullptr) {
        ESP_LOGI(TAG, " Manually sending acknowledgment...");
        esp_err_t ret = g_client->sendDataAcknowledgment(0); // Use 0 for general ack
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, " Acknowledgment sent successfully");
        } else {
            ESP_LOGE(TAG, " Failed to send acknowledgment: %s", esp_err_to_name(ret));
        }
    }
}

/******************************************************************************/
/*                              Main Function                                 */
/******************************************************************************/

extern "C" void app_main() {
    ESP_LOGI(TAG, "\n=== BLE CLIENT ACKNOWLEDGMENT DEMONSTRATION ===");
    ESP_LOGI(TAG, "Target Device Name: %s", TARGET_DEVICE_NAME);
    ESP_LOGI(TAG, "Features Demonstrated:");
    ESP_LOGI(TAG, "  • Manual acknowledgment handling");
    ESP_LOGI(TAG, "  • Automatic acknowledgment mode");
    ESP_LOGI(TAG, "  • Auto-disconnect after acknowledgment");
    ESP_LOGI(TAG, "  • Enhanced data received callbacks");
    ESP_LOGI(TAG, "=============================================\n");

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
    
    // 🎯 NEW: Configure acknowledgment settings
    clientConfig.autoSendAcknowledgments = false; // Manual ack for demonstration
    clientConfig.disconnectAfterAck = false;      // Stay connected after ack

    ret = client->setConfig(clientConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Failed to set client config: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, " Client configuration applied");
    ESP_LOGI(TAG, " 🔧 Acknowledgment mode: MANUAL (auto=false, disconnect=false)");

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
    
    // 🎯 NEW: Register enhanced data received callback with acknowledgment support
    client->setDataReceivedWithAckCallback(onDataReceivedWithAck);

    ESP_LOGI(TAG, " Callbacks configured (including enhanced data callback)");
    
    // 🎯 OPTIONAL: Demonstrate auto-acknowledgment mode switch after 30 seconds
    // client->setAutoAcknowledgment(true, false); // Enable auto-ack, stay connected

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
    ESP_LOGI(TAG, "\n Starting search for device: %s\n", TARGET_DEVICE_NAME);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        counter++;

        // Si está conectado, solo mostrar datos
        if (client->isConnected()) {
            // Mostrar datos cada 5 segundos
            if (counter % 5 == 0) {
                const bleDataPacket_t* data = client->kgetLastData();
                if (data != nullptr && data->valid) {
                    ESP_LOGI(TAG, "  Data: Battery=%d%%, Custom=%s",
                             data->batteryLevel, data->customData);
                } else {
                    ESP_LOGD(TAG, " Requesting fresh data...");
                    client->readBatteryLevel();
                    client->readCustomData();
                }
            }
            
            // Enviar datos al servidor cada 10 segundos
            if (counter % 10 == 0) {
                char sendBuffer[128];
                snprintf(sendBuffer, sizeof(sendBuffer), 
                         "{\"client\":\"ESP32\",\"counter\":%lu,\"ack_demo\":true,\"timestamp\":%llu}", 
                         counter, esp_timer_get_time()/1000);
                esp_err_t writeRet = client->writeCustomData(sendBuffer);
                if (writeRet == ESP_OK) {
                    ESP_LOGI(TAG, "📤 Sent to server: %s", sendBuffer);
                } else {
                    ESP_LOGW(TAG, "❌ Failed to send data: %s", esp_err_to_name(writeRet));
                }
            }
            
            // 🎯 NEW: Switch to auto-acknowledgment mode after 60 seconds
            if (counter == 60) {
                ESP_LOGI(TAG, "\n🔄 SWITCHING TO AUTO-ACKNOWLEDGMENT MODE...");
                client->setAutoAcknowledgment(true, false); // Auto-ack, stay connected
                ESP_LOGI(TAG, "✅ Auto-acknowledgment enabled - will automatically ack received data\n");
            }
            
            // 🎯 NEW: Switch to auto-disconnect mode after 120 seconds
            if (counter == 120) {
                ESP_LOGI(TAG, "\n🔄 SWITCHING TO AUTO-DISCONNECT MODE...");
                client->setAutoAcknowledgment(true, true); // Auto-ack and disconnect
                ESP_LOGI(TAG, "✅ Auto-disconnect enabled - will disconnect after ack\n");
            }
            
            // Status menos frecuente cuando conectado
            if (counter % 30 == 0) {
                ESP_LOGI(TAG, "\n===  Connected Status (t=%lu s) ===", counter);
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
                ESP_LOGI(TAG, "\n===  Searching Status (t=%lu s) ===", counter);
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