#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"

#define TAG "BLE_WIFI"
#define DEVICE_NAME "154_Server_Ask"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define MAX_RETRY 3

// Globals
static uint16_t handle_table[6];
static bool notify_enabled = false;
static uint16_t conn_id_global = 0;
static esp_gatt_if_t gatts_if_global = ESP_GATT_IF_NONE;
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool wifi_connected = false;
static char wifi_ssid[32] = "";
static char wifi_password[64] = "";
static uint8_t adv_config_done = 0;

// UUIDs
static uint8_t service_uuid[16] = {0x92, 0xb9, 0x08, 0x31, 0xa8, 0x46, 0x85, 0xaf, 0x1a, 0x4e, 0x3a, 0x93, 0x7c, 0x5f, 0x2c, 0xcf};
static uint8_t char_uuid_rx[16] = {0x00, 0x4d, 0x3f, 0x3a, 0xc3, 0x70, 0x4b, 0xb9, 0x5a, 0x4a, 0xa8, 0xaa, 0x6a, 0x28, 0x9c, 0x08};
static uint8_t char_uuid_tx[16] = {0xc1, 0xe0, 0x3a, 0x47, 0x6b, 0x2c, 0xe1, 0x98, 0x0a, 0x47, 0x3d, 0x84, 0x60, 0xc4, 0x3a, 0xd0};

// BLE configs
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false, .include_name = true, .include_txpower = true,
    .min_interval = 0x0006, .max_interval = 0x0010, .appearance = 0x00,
    .manufacturer_len = 0, .p_manufacturer_data = NULL,
    .service_data_len = 0, .p_service_data = NULL,
    .service_uuid_len = 16, .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20, .adv_int_max = 0x40, .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC, .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GATT DB
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t ccc_val[2] = {0x00, 0x00};
static const uint8_t char_val[4] = {0x11, 0x22, 0x33, 0x44};

static const esp_gatts_attr_db_t gatt_db[6] = {
    [0] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ, 16, 16, (uint8_t*)service_uuid}},
    [1] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ, 1, 1, (uint8_t*)&char_prop_write}},
    [2] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, char_uuid_rx, ESP_GATT_PERM_WRITE, 500, 4, (uint8_t*)char_val}},
    [3] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ, 1, 1, (uint8_t*)&char_prop_read_write_notify}},
    [4] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, char_uuid_tx, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 500, 4, (uint8_t*)char_val}},
    [5] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 2, 2, (uint8_t*)ccc_val}},
};

// WiFi event handler
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        wifi_connected = false;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi connected!");
    }
}

void wifi_init() {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
}

bool connect_wifi(const char* ssid, const char* password) {
    if (!ssid || strlen(ssid) == 0) return false;
    
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password) strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

void send_response(const char* msg) {
    if (notify_enabled && gatts_if_global != ESP_GATT_IF_NONE) {
        esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global, handle_table[4], strlen(msg), (uint8_t*)msg, false);
    }
}

// Parsing simple JSON (sin cJSON para ahorrar espacio)
void process_simple_json(const char* json_str) {
    ESP_LOGI(TAG, "RX: %s", json_str);
    
    // Buscar ssid
    char* ssid_start = strstr(json_str, "\"ssid\":");
    char* pwd_start = strstr(json_str, "\"password\":");
    
    if (ssid_start) {
        ssid_start = strchr(ssid_start, '"');
        if (ssid_start) {
            ssid_start = strchr(ssid_start + 1, '"') + 1;
            char* ssid_end = strchr(ssid_start, '"');
            if (ssid_end) {
                int len = ssid_end - ssid_start;
                if (len < sizeof(wifi_ssid)) {
                    strncpy(wifi_ssid, ssid_start, len);
                    wifi_ssid[len] = '\0';
                    
                    // Buscar password
                    wifi_password[0] = '\0';
                    if (pwd_start) {
                        pwd_start = strchr(pwd_start, '"');
                        if (pwd_start) {
                            pwd_start = strchr(pwd_start + 1, '"') + 1;
                            char* pwd_end = strchr(pwd_start, '"');
                            if (pwd_end) {
                                int pwd_len = pwd_end - pwd_start;
                                if (pwd_len < sizeof(wifi_password)) {
                                    strncpy(wifi_password, pwd_start, pwd_len);
                                    wifi_password[pwd_len] = '\0';
                                }
                            }
                        }
                    }
                    
                    ESP_LOGI(TAG, "Connecting to: %s", wifi_ssid);
                    if (connect_wifi(wifi_ssid, wifi_password)) {
                        send_response("{\"status\":\"wifi_connected\"}");
                    } else {
                        send_response("{\"status\":\"wifi_failed\"}");
                    }
                    return;
                }
            }
        }
    }
    
    // Comando simple
    if (strstr(json_str, "pullVars")) {
        send_response("{\"status\":\"ok\",\"wifi_connected\":true}");
    } else {
        send_response("{\"status\":\"received\"}");
    }
}

// BLE handlers
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~1);
        if (adv_config_done == 0) esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~2);
        if (adv_config_done == 0) esp_ble_gap_start_advertising(&adv_params);
        break;
    default: break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= 1;
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= 2;
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, 6, 0);
        break;
        
    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == handle_table[2]) {
            char* json_str = (char*)malloc(param->write.len + 1);
            if (json_str) {
                memcpy(json_str, param->write.value, param->write.len);
                json_str[param->write.len] = '\0';
                process_simple_json(json_str);
                free(json_str);
            }
        } else if (param->write.handle == handle_table[5]) {
            uint16_t val = param->write.value[1] << 8 | param->write.value[0];
            notify_enabled = (val == 0x0001);
        }
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        }
        break;
        
    case ESP_GATTS_CONNECT_EVT:
        conn_id_global = param->connect.conn_id;
        gatts_if_global = gatts_if;
        break;
        
    case ESP_GATTS_DISCONNECT_EVT:
        notify_enabled = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;
        
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
            esp_ble_gatts_start_service(handle_table[0]);
        }
        break;
        
    default: break;
    }
}

extern "C" void app_main(void) {
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Init WiFi
    wifi_init();
    
    // Init BT
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0x55));
    
    ESP_LOGI(TAG, "BLE server started - ready for WiFi config!");
}