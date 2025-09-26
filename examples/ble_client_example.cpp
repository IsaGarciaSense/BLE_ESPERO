/*******************************************************************************
 * @file ble_client_example.cpp
 * @brief BLE client example targeting specific device by MAC address
 * @details Demonstrates proper configuration order for MAC-based targeting
 * 
 * @version 0.0.6
 * @date 2025-09-01
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "ble.hpp"
#include "esp_log.h"
#include "nvs_flash.h"

/******************************************************************************/
/*                              Constants                                     */
/******************************************************************************/

static const char* TAG = "BLE_MAC_TARGET";

// MAC address del dispositivo target (cambiar por el tuyo)
const esp_bd_addr_t TARGET_MAC = {0x48, 0xCA, 0x43, 0x18, 0x5D, 0x1E};

// Nombre del dispositivo target (opcional, se puede usar solo MAC)
const char* TARGET_DEVICE_NAME = "SenseAI_BLE";

/******************************************************************************/
/*                              Main Function                                 */
/******************************************************************************/

extern "C" void app_main() {
    ESP_LOGI(TAG, "\n=== BLE CLIENT MAC TARGETING EXAMPLE ===");
    ESP_LOGI(TAG, "Target MAC: 48:CA:43:18:5D:1E");
    ESP_LOGI(TAG, "Target Name: %s", TARGET_DEVICE_NAME);
    ESP_LOGI(TAG, "==========================================\n");
    
    // Step 1: Initialize NVS (requerido para BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✓ NVS initialized");

    // Step 2: Crear la configuración de BLE Library con autoStart DESHABILITADO
    bleLibraryConfig_t bleConfig;
    BLELibrary::createDefaultConfig(&bleConfig, BLE_MODE_CLIENT_ONLY);
    strncpy(bleConfig.deviceName, "MacTargetClient", BLE_MAX_DEVICE_NAME_LEN - 1);
    
    // CRÍTICO: Deshabilitar auto-start para poder configurar antes
    bleConfig.autoStart = false;
    
    ESP_LOGI(TAG, "Creating BLE Library...");
    ESP_LOGI(TAG, "Auto-start disabled for manual configuration");
    BLELibrary* ble = new BLELibrary(bleConfig);
    if (ble == nullptr) {
        ESP_LOGE(TAG, "✗ Failed to create BLE Library");
        return;
    }
    ESP_LOGI(TAG, "✓ BLE Library created");

    // Step 3: Inicializar la librería para crear el cliente interno
    ESP_LOGI(TAG, "\n--- Initializing BLE Library ---");
    ret = ble->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to initialize BLE: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, "✓ BLE Library initialized");

    // Step 4: Obtener el cliente (ya inicializado internamente)
    BLEClient* client = ble->getClient();
    if (client == nullptr) {
        ESP_LOGE(TAG, "✗ Failed to get BLE Client");
        delete ble;
        return;
    }
    ESP_LOGI(TAG, "✓ BLE Client obtained");

    // Step 5: Configurar TODO usando setClientConfig COMPLETO
    ESP_LOGI(TAG, "\n--- Configuring Client with Complete Config ---");
    
    // Crear configuración completa del cliente
    bleClientConfig_t clientConfig;
    memset(&clientConfig, 0, sizeof(clientConfig));
    
    // Configurar MAC target
    memcpy(clientConfig.targetServerMACadd, TARGET_MAC, ESP_BD_ADDR_LEN);
    
    // Configurar nombre target
    strncpy(clientConfig.targetDeviceName, TARGET_DEVICE_NAME, BLE_MAX_DEVICE_NAME_LEN - 1);
    
    // Configurar timeouts
    clientConfig.scanTimeout = 10000;
    clientConfig.connectionTimeout = 10000;
    clientConfig.autoReconnect = true;
    clientConfig.reconnectInterval = 5000;
    clientConfig.enableNotifications = true;
    clientConfig.readInterval = 5000;
    
    // Aplicar configuración completa
    ret = client->setConfig(clientConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to set client config: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, "✓ Client configuration applied");

    // Step 6: Configurar seguridad por separado
    ESP_LOGI(TAG, "\n--- Configuring Security ---");
    bleSecurityConfig_t secConfig;
    ret = bleCreateDefaultSecurityConfig(&secConfig, BLE_SECURITY_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to create security config: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, "✓ Security config created - Level: %d", secConfig.level);

    ret = client->setSecurityConfig(secConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to apply security config: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, "✓ Security config applied successfully");

    // Step 7: VERIFICAR configuración ANTES de start()
    ESP_LOGI(TAG, "\n=== PRE-START CONFIGURATION VERIFICATION ===");
    bleClientConfig_t currentConfig = client->getConfig();
    
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            currentConfig.targetServerMACadd[0], currentConfig.targetServerMACadd[1],
            currentConfig.targetServerMACadd[2], currentConfig.targetServerMACadd[3],
            currentConfig.targetServerMACadd[4], currentConfig.targetServerMACadd[5]);
    
    ESP_LOGI(TAG, "PRE-START Target Device Name: %s", currentConfig.targetDeviceName);
    ESP_LOGI(TAG, "PRE-START Target MAC Address: %s", macStr);
    ESP_LOGI(TAG, "PRE-START Scan Timeout: %lu ms", currentConfig.scanTimeout);
    ESP_LOGI(TAG, "PRE-START Auto Reconnect: %s", currentConfig.autoReconnect ? "YES" : "NO");
    ESP_LOGI(TAG, "===================================================\n");

    // Step 8: Configurar callbacks para monitoreo ANTES de start()
    ESP_LOGI(TAG, "Setting up callbacks...");
    
    client->setDeviceFoundCallback([](const bleDeviceInfo_t* deviceInfo, bool* shouldConnect) {
        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                deviceInfo->address[0], deviceInfo->address[1], 
                deviceInfo->address[2], deviceInfo->address[3], 
                deviceInfo->address[4], deviceInfo->address[5]);
                
        ESP_LOGI(TAG, "🎯 TARGET FOUND! Connecting to:");
        ESP_LOGI(TAG, "   Name: %s", deviceInfo->name);
        ESP_LOGI(TAG, "   MAC: %s", macStr);
        ESP_LOGI(TAG, "   RSSI: %d dBm", deviceInfo->rssi);
        *shouldConnect = true;
    });

    client->setConnectedCallback([](const bleDeviceInfo_t* deviceInfo) {
        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                deviceInfo->address[0], deviceInfo->address[1], 
                deviceInfo->address[2], deviceInfo->address[3], 
                deviceInfo->address[4], deviceInfo->address[5]);
                
        ESP_LOGI(TAG, "✅ CONEXIÓN EXITOSA!");
        ESP_LOGI(TAG, "   Dispositivo: %s", deviceInfo->name);
        ESP_LOGI(TAG, "   MAC: %s", macStr);
        ESP_LOGI(TAG, "   RSSI: %d dBm", deviceInfo->rssi);
    });

    client->setDisconnectedCallback([](int reason, bool wasPlanned) {
        ESP_LOGW(TAG, "❌ Desconectado - Razón: %d, Planeado: %s", 
                reason, wasPlanned ? "SÍ" : "NO");
    });

    client->setScanStartedCallback([](uint32_t duration) {
        ESP_LOGI(TAG, "🔍 Iniciando escaneo por %lu ms...", duration);
    });

    client->setScanCompletedCallback([](uint32_t found, bool targetFound) {
        ESP_LOGI(TAG, "📋 Escaneo completado: %lu dispositivos, Target: %s", 
                found, targetFound ? "✅ ENCONTRADO" : "❌ NO ENCONTRADO");
    });

    ESP_LOGI(TAG, "✓ Callbacks configured");

    // Step 9: AHORA SÍ iniciar servicios BLE (esto iniciará el escaneo con la config correcta)
    ESP_LOGI(TAG, "\n--- Starting BLE Services ---");
    ret = ble->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to start BLE services: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, "✓ BLE Services started");

    // Step 10: VERIFICACIÓN FINAL después de start()
    ESP_LOGI(TAG, "\n=== FINAL VERIFICATION AFTER START ===");
    bleClientConfig_t finalConfig = client->getConfig();
    
    char finalMacStr[18];
    sprintf(finalMacStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            finalConfig.targetServerMACadd[0], finalConfig.targetServerMACadd[1],
            finalConfig.targetServerMACadd[2], finalConfig.targetServerMACadd[3],
            finalConfig.targetServerMACadd[4], finalConfig.targetServerMACadd[5]);
    
    ESP_LOGI(TAG, "FINAL Target MAC: %s", finalMacStr);
    ESP_LOGI(TAG, "FINAL Target Name: %s", finalConfig.targetDeviceName);
    ESP_LOGI(TAG, "=======================================\n");

    // Step 11: Loop principal de monitoreo
    ESP_LOGI(TAG, "🚀 Iniciando búsqueda del dispositivo target...\n");
    
    uint32_t counter = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        counter++;

        // Status cada 15 segundos
        if (counter % 15 == 0) {
            ESP_LOGI(TAG, "\n📊 === Status (t=%lu s) ===", counter);
            ESP_LOGI(TAG, "Estado: %s", client->kgetStateString());
            ESP_LOGI(TAG, "Conectado: %s", client->isConnected() ? "✅ SÍ" : "❌ NO");
            ESP_LOGI(TAG, "Escaneando: %s", client->isScanning() ? "✅ SÍ" : "❌ NO");
            
            bleClientStats_t stats = client->getStats();
            ESP_LOGI(TAG, "Estadísticas:");
            ESP_LOGI(TAG, "  Escaneos: %lu", stats.scanCount);
            ESP_LOGI(TAG, "  Conexiones exitosas: %lu/%lu", 
                    stats.successfulConnections, stats.connectionAttempts);
            ESP_LOGI(TAG, "=========================\n");
            
            // Si no está escaneando ni conectado, reiniciar escaneo
            if (!client->isConnected() && !client->isScanning()) {
                ESP_LOGW(TAG, "⚠️ Reiniciando escaneo...");
                client->startTargetedScan();
            }
        }

        // Si está conectado, mostrar datos cada 5 segundos
        if (client->isConnected() && (counter % 5 == 0)) {
            const bleDataPacket_t* data = client->kgetLastData();
            if (data != nullptr && data->valid) {
                ESP_LOGI(TAG, "📊 Datos: Batería=%d%%, Custom=%s", 
                        data->batteryLevel, data->customData);
            }
        }
    }

    // Cleanup (nunca se alcanza)
    delete ble;
}