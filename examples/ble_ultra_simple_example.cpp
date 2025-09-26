/*******************************************************************************
 * @file ble_ultra_simple_example.cpp
 * @brief Ultra simple BLE server example with minimal configuration
 * @details This example demonstrates the simplest way to create a BLE server:
 * 
 * Features enabled by default:
 * - Automatic name: "SenseAI_BLE"
 * - Automatic advertising enabled
 * - Accepts up to 4 simultaneous clients
 * - JSON commands enabled
 * - Notifications enabled
 * - Basic security enabled
 * - Automatic reconnection
 * - Automatic memory management
 * - Informative logs
 *
 * To use Client mode, change line: 
 *   BLELibrary* ble = createBLEClient(nullptr, nullptr);
 *
 * To use Dual mode, change line:
 *   BLELibrary* ble = createBLEDual(nullptr, nullptr);
 *
 * @version 0.0.2
 * @date 2025-08-29
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "ble.hpp"
#include "esp_log.h"
#include "nvs_flash.h"

/******************************************************************************/
/*                              Constants                                     */
/******************************************************************************/

static const char* TAG = "ULTRA_SIMPLE";

/******************************************************************************/
/*                              Main Function                                 */
/******************************************************************************/

extern "C" void app_main() {
    // 1. Initialize NVS (required for BLE)
    nvs_flash_init();

    // 2. Create BLE server with all defaults
    BLELibrary* ble = createBLEServer(nullptr); 

    // 3. Check if created successfully
    if (ble == nullptr) {
        ESP_LOGE(TAG, "Error creating server");
        return;
    }

    // 4. It's working! Just show message
    ESP_LOGI(TAG, " BLE server 'SenseAI_BLE' running with automatic configuration");
    ESP_LOGI(TAG, " Connect from a BLE app to the device: SenseAI_BLE");

    // 5. Simple loop - the server works automatically
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 5 seconds
        ESP_LOGI(TAG, " Server active - Time: %llu minutes", ble->getUptime() / 60000);
    }
}
