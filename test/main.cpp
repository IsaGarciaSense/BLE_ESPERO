#include "MyBLEClient.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "MAIN";
BLE_Lib ble;

void client_callback(const BLE_Device& dev) {
    ESP_LOGI(TAG, "Dispositivo detectado: %s, RSSI: %d", dev.name.c_str(), dev.rssi);
    char addr_str[18];
    snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             dev.address[0], dev.address[1], dev.address[2],
             dev.address[3], dev.address[4], dev.address[5]);
    ESP_LOGI(TAG, "Dirección MAC: %s", addr_str);
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Iniciando cliente BLE...");
    ble.initClient(client_callback);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ble.startScan(30);  // Escanear por 30 segundos
    
    // Mantener la tarea activa
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}