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
            ESP_LOGI(TAG, "Init Scan - time duration: %lu ms", scan_duration_ms);
        });

        // NUEVO: Callback cuando termina escaneo
        client->setScanCompletedCallback([](uint32_t devices_found, bool target_found) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, " Scann completed");
            ESP_LOGI(TAG, "   Founds Device: %lu", devices_found);
            ESP_LOGI(TAG, "   Is Target '154_BLE_Server': %s", target_found ? " yes" : " no");
            ESP_LOGI(TAG, "════════════════════════════");
            ESP_LOGI(TAG, "");
        });
    }

    // Demo de las nuevas funcionalidades
    uint32_t demo_step = 1;
    
    while (true) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== DEMO PASO %lu ===", demo_step);
        
        switch (demo_step) {
            case 1:
                ESP_LOGI(TAG, "All ble devices");
                client->startDiscoveryScan(15000); // 15 segundos
                vTaskDelay(pdMS_TO_TICKS(17000)); // Esperar a que termine
                
                // Mostrar lista de dispositivos encontrados
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "Devices founds:");
                for (uint32_t i = 0; i < client->getUniqueDevicesFound(); i++) {
                    const ble_device_info_t* device = client->getFoundDevice(i);
                    if (device != nullptr) {
                        ESP_LOGI(TAG, "  %lu. %s (RSSI: %d dBm)", i+1, device->name, device->rssi);
                    }
                }
                ESP_LOGI(TAG, "Total: %lu unique devices", client->getUniqueDevicesFound());
                break;

            case 2:
                ESP_LOGI(TAG, "Scann for a specific target (154_BLE_Server)");
                client->startTargetedScan();
                vTaskDelay(pdMS_TO_TICKS(20000)); // Esperar conexión o timeout
                break;

            case 3:
                ESP_LOGI(TAG, "Second round for scanning devices");
                client->clearDeviceList(); // Limpiar lista anterior
                client->startDiscoveryScan(12000); // 12 segundos
                vTaskDelay(pdMS_TO_TICKS(14000));
                
                ESP_LOGI(TAG, "You got:");
                ESP_LOGI(TAG, "  Devices unique after cleaning: %lu", client->getUniqueDevicesFound());
                break;

            default:
                ESP_LOGI(TAG, "Restart demo...");
                demo_step = 0;
                vTaskDelay(pdMS_TO_TICKS(5000));
                break;
        }

        demo_step++;
        vTaskDelay(pdMS_TO_TICKS(3000)); // Pausa entre pasos
    }
}