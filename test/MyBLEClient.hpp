#pragma once
#include "esp_bt.h"
#include "esp_gap_ble_api.h"

class MyBLEClient {
public:
    static void init();                          // Inicializa BLE
    static void startScan(unsigned long duration);    // Inicia escaneo BLE
};
