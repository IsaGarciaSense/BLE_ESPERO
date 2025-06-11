// #include "MyBLEClient.hpp"

// extern "C" void app_main(void) {
//     MyBLEClient::init();         // Inicializa BLE
//     MyBLEClient::startScan(10);  // Escanea por 10 segundos
// }
// BLE GATT Client para ESP-IDF v5.4
#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_log.h"

#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_A_APP_ID           0

static esp_gattc_char_elem_t *char_elem_result = NULL;
static bool connect = false;
static bool get_server = false;
static char remote_device_name[ESP_BLE_ADV_DATA_LEN_MAX] = "ESP_GATTS_DEMO";

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static struct {
    uint16_t gattc_if;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
} profile;

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            esp_ble_gap_start_scanning(10);
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = param;
            uint8_t adv_name_len;
            uint8_t *adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);

            if (adv_name && adv_name_len > 0 &&
                strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {

                if (!connect) {
                    connect = true;
                    esp_ble_gap_stop_scanning();
                    esp_ble_gattc_open(profile.gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                }
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(GATTC_TAG, "Scan stopped");
            break;
        default:
            break;
    }
}

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                     esp_ble_gattc_cb_param_t *param) {
    switch (event) {
        case ESP_GATTC_REG_EVT: {
            profile.gattc_if = gattc_if;
            esp_ble_gap_set_scan_params(&ble_scan_params);
            break;
        }
        case ESP_GATTC_CONNECT_EVT: {
            ESP_LOGI(GATTC_TAG, "Connected");
            profile.conn_id = param->connect.conn_id;
            memcpy(profile.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            esp_ble_gattc_search_service(gattc_if, param->connect.conn_id, NULL);
            break;
        }
        case ESP_GATTC_SEARCH_RES_EVT: {
            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
                param->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
                get_server = true;
                profile.service_start_handle = param->search_res.start_handle;
                profile.service_end_handle = param->search_res.end_handle;
            }
            break;
        }
        case ESP_GATTC_SEARCH_CMPL_EVT: {
            if (get_server) {
                uint16_t count = 0;
                esp_ble_gattc_get_attr_count(gattc_if, profile.conn_id, ESP_GATT_DB_CHARACTERISTIC,
                    profile.service_start_handle, profile.service_end_handle, ESP_GATT_INVALID_HANDLE, &count);

                if (count) {
                    char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);

                    esp_bt_uuid_t notify_uuid;
                    notify_uuid.len = ESP_UUID_LEN_16;
                    notify_uuid.uuid.uuid16 = REMOTE_NOTIFY_CHAR_UUID;

                    esp_ble_gattc_get_char_by_uuid(gattc_if, profile.conn_id,
                        profile.service_start_handle, profile.service_end_handle,
                        notify_uuid,
                        char_elem_result, &count);

                    if (char_elem_result && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
                        profile.char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if, profile.remote_bda, profile.char_handle);
                    }
                    free(char_elem_result);
                }
            }
            break;
        }
        case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
            uint16_t notify_en = 1;
            esp_ble_gattc_write_char_descr(gattc_if, profile.conn_id, profile.char_handle + 1,
                                           sizeof(notify_en), (uint8_t *)&notify_en,
                                           ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            break;
        }
        case ESP_GATTC_NOTIFY_EVT:
            ESP_LOGI(GATTC_TAG, "Notification received");
            ESP_LOG_BUFFER_HEX(GATTC_TAG, param->notify.value, param->notify.value_len);
            break;
        default:
            break;
    }
}

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(PROFILE_A_APP_ID));
}