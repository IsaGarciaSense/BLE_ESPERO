#pragma once

#include <functional>
#include <vector>
#include <string>
#include "esp_gap_ble_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_log.h"

struct BLE_Device {
    esp_bd_addr_t address;
    int rssi;
    std::string name;
};

using BleClientEventCallback = std::function<void(const BLE_Device&)>;

class BLE_Lib {
public:
    BLE_Lib();
    ~BLE_Lib();

    void initClient(BleClientEventCallback callback);
    void startScan(uint32_t scan_duration_sec = 30);

private:
    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
    static void printAddress(const esp_bd_addr_t addr);

    static BleClientEventCallback user_callback;
    static std::vector<BLE_Device> scanned_devices;
};