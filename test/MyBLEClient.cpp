#include "MyBLEClient.hpp"
#include <cstring>

static const char* TAG = "BLE_LIB";

// Inicialización de miembros estáticos
BleClientEventCallback BLE_Lib::user_callback = nullptr;
std::vector<BLE_Device> BLE_Lib::scanned_devices;

BLE_Lib::BLE_Lib() {}

BLE_Lib::~BLE_Lib() {
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
    }
}

void BLE_Lib::initClient(BleClientEventCallback callback) {
    user_callback = callback;

    // Inicializar controlador BT
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Inicializar Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Registrar callback
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gapEventHandler));
}

void BLE_Lib::startScan(uint32_t scan_duration_sec) {
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };
    
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));
    ESP_ERROR_CHECK(esp_ble_gap_start_scanning(scan_duration_sec));
}

void BLE_Lib::gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            auto scan_result = param->scan_rst;
            
            if (scan_result.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                BLE_Device dev;
                memcpy(dev.address, scan_result.bda, sizeof(esp_bd_addr_t));
                dev.rssi = scan_result.rssi;
                
                // Obtener nombre del dispositivo
                uint8_t len = 0;
                uint8_t* data = esp_ble_resolve_adv_data(scan_result.ble_adv, 
                                                        ESP_BLE_AD_TYPE_NAME_CMPL, &len);
                if (data && len > 0) {
                    dev.name.assign(reinterpret_cast<char*>(data), len);
                } else {
                    dev.name = "Unknown";
                }
                
                scanned_devices.push_back(dev);
                
                if (user_callback) {
                    user_callback(dev);
                }
            }
            break;
        }
        
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Parámetros de escaneo configurados");
            break;
            
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Escaneo iniciado");
            } else {
                ESP_LOGE(TAG, "Error al iniciar escaneo");
            }
            break;
            
        default:
            break;
    }
}

void BLE_Lib::printAddress(const esp_bd_addr_t addr) {
    char bda_str[18];
    snprintf(bda_str, sizeof(bda_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    ESP_LOGI(TAG, "MAC: %s", bda_str);
}