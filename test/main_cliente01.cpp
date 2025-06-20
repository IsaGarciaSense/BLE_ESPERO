#include "BLE.hpp"
#include "esp_log.h"
#include "nvs_flash.h"

static const char* TAG = "BLE_EXTENDED_DEMO";
const esp_bd_addr_t MAC_TARGET = {0x24, 0xEC, 0x4A, 0x36, 0x86, 0x42};

extern "C" void app_main() {
    ESP_LOGI(TAG, "=== Demo BLE Client Extendido ===");
    
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Crear cliente BLE
    BLELibrary* ble = createBLEClient("ScannClient", "154_BLE_Server");
    if (ble == nullptr) {
        ESP_LOGE(TAG, "Error creando cliente BLE");
        return;
    }

    BLEClient* client = ble->getClient();
    if (client != nullptr) {
        ble_client_config_t config = client->getConfig();

        config.min_rssi = -90; // Ajustar RSSI mínimo
        // config.
        client->setConfig(config);
        client->setMacTarget(MAC_TARGET);

        // NUEVO: Callback para CUALQUIER dispositivo encontrado
        client->setAnyDeviceFoundCallback([](const ble_device_info_t* device_info, bool is_target) {
            ESP_LOGI(TAG, " Device detected:");
            ESP_LOGI(TAG, "   Name: %s", device_info->name);
            ESP_LOGI(TAG, "  MAC: %02x:%02x:%02x:%02x:%02x:%02x", 
                     device_info->address[0], device_info->address[1], device_info->address[2],
                     device_info->address[3], device_info->address[4], device_info->address[5]);
            ESP_LOGI(TAG, "   RSSI: %d dBm", device_info->rssi);
            ESP_LOGI(TAG, "   target: %s", is_target ? "yes" : "no");
            ESP_LOGI(TAG, "  ────────────────────────────");
        });

        // NUEVO: Callback cuando inicia escaneo
        client->setScanStartedCallback([](uint32_t scan_duration_ms) {
            ESP_LOGI(TAG, "🔍 Init Scan - time duration: %lu ms", scan_duration_ms);
        });

        // NUEVO: Callback cuando termina escaneo
        client->setScanCompletedCallback([](uint32_t devices_found, bool target_found) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, " ESCANEO COMPLETADO");
            ESP_LOGI(TAG, "   Dispositivos únicos encontrados: %lu", devices_found);
            ESP_LOGI(TAG, "   Target '154_BLE_Server' encontrado: %s", target_found ? " SÍ" : " NO");
            ESP_LOGI(TAG, "════════════════════════════");
            ESP_LOGI(TAG, "");
        });

        // Callbacks tradicionales
        client->setConnectedCallback([](const ble_device_info_t* device_info) {
            ESP_LOGI(TAG, "🔗 CONECTADO al servidor: %s", device_info->name);
        });

        client->setDataReceivedCallback([](const ble_data_packet_t* data) {
            ESP_LOGI(TAG, "📊 DATOS: Batería=%d%%, Data=%s", data->battery_level, data->custom_data);
        });
    }

    ESP_LOGI(TAG, "🚀 Cliente BLE extendido listo");
    ESP_LOGI(TAG, "📡 Objetivo: 154_BLE_Server");
    ESP_LOGI(TAG, "========================");

    // Demo de las nuevas funcionalidades
    uint32_t demo_step = 1;
    
    while (true) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "🎯 === DEMO PASO %lu ===", demo_step);
        
        switch (demo_step) {
            case 1:
                ESP_LOGI(TAG, "🔍 Paso 1: Escaneo de descubrimiento (TODOS los dispositivos BLE)");
                client->startDiscoveryScan(15000); // 15 segundos
                vTaskDelay(pdMS_TO_TICKS(17000)); // Esperar a que termine
                
                // Mostrar lista de dispositivos encontrados
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "📋 RESUMEN DE DISPOSITIVOS ENCONTRADOS:");
                for (uint32_t i = 0; i < client->getUniqueDevicesFound(); i++) {
                    const ble_device_info_t* device = client->getFoundDevice(i);
                    if (device != nullptr) {
                        ESP_LOGI(TAG, "  %lu. %s (RSSI: %d dBm)", i+1, device->name, device->rssi);
                    }
                }
                ESP_LOGI(TAG, "Total: %lu dispositivos únicos", client->getUniqueDevicesFound());
                break;

            case 2:
                ESP_LOGI(TAG, "🎯 Paso 2: Escaneo dirigido (solo buscar: 154_BLE_Server)");
                client->startTargetedScan();
                vTaskDelay(pdMS_TO_TICKS(20000)); // Esperar conexión o timeout
                break;

            case 3:
                ESP_LOGI(TAG, "🔄 Paso 3: Otro escaneo de descubrimiento después de limpiar lista");
                client->clearDeviceList(); // Limpiar lista anterior
                ESP_LOGI(TAG, "🧹 Lista limpiada. Reescaneando...");
                client->startDiscoveryScan(12000); // 12 segundos
                vTaskDelay(pdMS_TO_TICKS(14000));
                
                ESP_LOGI(TAG, "📊 RESUMEN FINAL:");
                ESP_LOGI(TAG, "  Dispositivos únicos después de limpiar: %lu", client->getUniqueDevicesFound());
                break;

            default:
                ESP_LOGI(TAG, "🔁 Reiniciando demo...");
                demo_step = 0;
                vTaskDelay(pdMS_TO_TICKS(5000));
                break;
        }

        demo_step++;
        vTaskDelay(pdMS_TO_TICKS(3000)); // Pausa entre pasos
    }
}