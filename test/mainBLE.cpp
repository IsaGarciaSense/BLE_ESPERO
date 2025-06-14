#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"

static const char* TAG = "BLE_SCANNER";

// Parámetros de escaneo BLE
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

// Función para convertir dirección MAC a string
static char* bda2str(esp_bd_addr_t bda, char *str, size_t size) {
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return str;
}

// Función para obtener el nombre del dispositivo desde los datos de advertising
static void parse_adv_data(uint8_t* adv_data, uint8_t adv_data_len, char* device_name, size_t name_len) {
    for (int i = 0; i < adv_data_len; ) {
        uint8_t length = adv_data[i];
        if (length == 0) break;
        
        uint8_t type = adv_data[i + 1];
        
        // Buscar nombre del dispositivo (tipo 0x09 = nombre completo, 0x08 = nombre corto)
        if ((type == 0x09 || type == 0x08) && length > 1) {
            size_t copy_len = (length - 1 < name_len - 1) ? length - 1 : name_len - 1;
            memcpy(device_name, &adv_data[i + 2], copy_len);
            device_name[copy_len] = '\0';
            return;
        }
        
        i += length + 1;
    }
    strcpy(device_name, "Sin nombre");
}

// Callback para eventos GAP
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        ESP_LOGI(TAG, "Parámetros de escaneo configurados, iniciando escaneo...");
        esp_ble_gap_start_scanning(0); // Escaneo continuo
        break;
    }
    
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Error al iniciar escaneo: %d", param->scan_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, " Escaneo BLE iniciado correctamente");
        }
        break;
    }
    
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = param;
        
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            char mac_str[18];
            char device_name[32] = {0};
            
            bda2str(scan_result->scan_rst.bda, mac_str, sizeof(mac_str));
            
            // Extraer nombre del dispositivo si está disponible
            if (scan_result->scan_rst.adv_data_len > 0) {
                parse_adv_data(scan_result->scan_rst.ble_adv, 
                              scan_result->scan_rst.adv_data_len, 
                              device_name, sizeof(device_name));
            } else {
                strcpy(device_name, "Sin datos ADV");
            }
            
            // Mostrar información del dispositivo encontrado
            ESP_LOGI(TAG, "   DISPOSITIVO: %s", device_name);
            ESP_LOGI(TAG, "   MAC: %s", mac_str);
            ESP_LOGI(TAG, "   RSSI: %d dBm", scan_result->scan_rst.rssi);
            ESP_LOGI(TAG, "   Tipo: %s", 
                     scan_result->scan_rst.ble_addr_type == BLE_ADDR_TYPE_PUBLIC ? "PUBLIC" : "RANDOM");
            ESP_LOGI(TAG, "   Datos ADV: %d bytes", scan_result->scan_rst.adv_data_len);
            ESP_LOGI(TAG, "   ────────────────────────────────");
            
            break;
        }
        
        case ESP_GAP_SEARCH_INQ_CMPL_EVT: {
            ESP_LOGI(TAG, "Escaneo completado, reiniciando...");
            break;
        }
        
        default:
            break;
        }
        break;
    }
    
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Error al detener escaneo: %d", param->scan_stop_cmpl.status);
        } else {
            ESP_LOGI(TAG, "Escaneo detenido");
        }
        break;
    }
    
    default:
        break;
    }
}

// Inicialización del sistema BLE
static esp_err_t ble_init(void) {
    esp_err_t ret;
    
    // Inicializar NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Liberar memoria clásica de Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    // Configurar e inicializar controlador BT
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Error inicializando controlador BT: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Error habilitando controlador BT: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Inicializar stack Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Error inicializando bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Error habilitando bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Registrar callback GAP
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "Error registrando callback GAP: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE inicializado correctamente");
    return ESP_OK;
}

// Función principal
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Iniciando BLE Scanner para ESP32-S3");
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    
    // Inicializar BLE
    if (ble_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error en inicialización BLE");
        return;
    }
    
    // Configurar y comenzar escaneo
    esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (scan_ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando parámetros: %s", esp_err_to_name(scan_ret));
        return;
    }
    
    ESP_LOGI(TAG, "Buscando dispositivos BLE...");
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    
    // Loop principal - reinicia escaneo cada 30 segundos
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        
        ESP_LOGI(TAG, "Reiniciando escaneo...");
        esp_ble_gap_stop_scanning();
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_ble_gap_start_scanning(0);
    }
}