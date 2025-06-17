#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "BLE_CLIENT";

#define MAX_DEVICE_NAME_LEN 64

// Estructura para almacenar información de dispositivos escaneados
typedef struct {
    esp_bd_addr_t bda;
    char name[MAX_DEVICE_NAME_LEN + 1];
    int rssi;
    bool name_complete;
} scanned_device_t;

// Array para almacenar dispositivos encontrados
static scanned_device_t scanned_devices[50];
static int device_count = 0;

// Parámetros de escaneo
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,    // 0x50*0.625ms = 50ms
    .scan_window            = 0x30,    // 0x30*0.625ms = 30ms
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

// Función para convertir dirección MAC a string
static void bda_to_string(esp_bd_addr_t bda, char *str) {
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

// Función para buscar si un dispositivo ya está en la lista
static int find_device_by_bda(esp_bd_addr_t bda) {
    for (int i = 0; i < device_count; i++) {
        if (memcmp(scanned_devices[i].bda, bda, ESP_BD_ADDR_LEN) == 0) {
            return i;
        }
    }
    return -1;
}

// Función para agregar o actualizar un dispositivo en la lista
static void add_or_update_device(esp_bd_addr_t bda, char *name, int rssi, bool name_complete) {
    int index = find_device_by_bda(bda);
    
    if (index == -1 && device_count < 50) {
        // Nuevo dispositivo
        index = device_count++;
        memcpy(scanned_devices[index].bda, bda, ESP_BD_ADDR_LEN);
        scanned_devices[index].name[0] = '\0';
        scanned_devices[index].name_complete = false;
    }
    
    if (index != -1) {
        // Actualizar RSSI
        scanned_devices[index].rssi = rssi;
        
        // Actualizar nombre si es necesario
        if (name && strlen(name) > 0) {
            if (!scanned_devices[index].name_complete || strlen(scanned_devices[index].name) == 0) {
                strncpy(scanned_devices[index].name, name, MAX_DEVICE_NAME_LEN);
                scanned_devices[index].name[MAX_DEVICE_NAME_LEN] = '\0';
                scanned_devices[index].name_complete = name_complete;
            }
        }
    }
}

// Función para mostrar todos los dispositivos encontrados
static void print_discovered_devices(void) {
    ESP_LOGI(TAG, "\n=== DISPOSITIVOS BLE ENCONTRADOS ===");
    ESP_LOGI(TAG, "Total: %d dispositivos", device_count);
    
    for (int i = 0; i < device_count; i++) {
        char bda_str[18];
        bda_to_string(scanned_devices[i].bda, bda_str);
        
        if (strlen(scanned_devices[i].name) > 0) {
            ESP_LOGI(TAG, "[%d] %s | %s | RSSI: %d dBm", 
                     i + 1, bda_str, scanned_devices[i].name, scanned_devices[i].rssi);
        } else {
            ESP_LOGI(TAG, "[%d] %s | <Sin nombre> | RSSI: %d dBm", 
                     i + 1, bda_str, scanned_devices[i].rssi);
        }
    }
    ESP_LOGI(TAG, "=====================================\n");
}

// Callback para eventos GAP (Generic Access Profile)
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        ESP_LOGI(TAG, "Parámetros de escaneo configurados");
        // Iniciar escaneo por 30 segundos
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Error al iniciar escaneo: %d", param->scan_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "Escaneo BLE iniciado... Buscando dispositivos por 30 segundos");
            device_count = 0; // Resetear contador
        }
        break;
        
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            char bda_str[18];
            bda_to_string(scan_result->scan_rst.bda, bda_str);
            
            // Extraer nombre del dispositivo de los datos de advertising
            char device_name[MAX_DEVICE_NAME_LEN + 1] = {0};
            bool name_complete = false;
            
            if (scan_result->scan_rst.adv_data_len > 0) {
                uint8_t *adv_data = scan_result->scan_rst.ble_adv;
                uint8_t adv_data_len = scan_result->scan_rst.adv_data_len;
                
                // Parsear datos de advertising para encontrar el nombre
                for (int i = 0; i < adv_data_len; ) {
                    uint8_t length = adv_data[i];
                    if (length == 0) break;
                    
                    uint8_t type = adv_data[i + 1];
                    
                    // Nombre completo (0x09) o nombre corto (0x08)
                    if (type == 0x09 || type == 0x08) {
                        uint8_t name_len = length - 1;
                        if (name_len > MAX_DEVICE_NAME_LEN) {
                            name_len = MAX_DEVICE_NAME_LEN;
                        }
                        
                        memcpy(device_name, &adv_data[i + 2], name_len);
                        device_name[name_len] = '\0';
                        name_complete = (type == 0x09);
                        break;
                    }
                    
                    i += length + 1;
                }
            }
            
            // Agregar o actualizar dispositivo
            add_or_update_device(scan_result->scan_rst.bda, 
                                device_name, 
                                scan_result->scan_rst.rssi, 
                                name_complete);
            
            // Mostrar dispositivo encontrado en tiempo real
            if (strlen(device_name) > 0) {
                ESP_LOGI(TAG, "Encontrado: %s | %s | RSSI: %d dBm", 
                         bda_str, device_name, scan_result->scan_rst.rssi);
            } else {
                ESP_LOGI(TAG, "Encontrado: %s | <Sin nombre> | RSSI: %d dBm", 
                         bda_str, scan_result->scan_rst.rssi);
            }
            break;
        }
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(TAG, "Escaneo BLE completado");
            print_discovered_devices();
            
            // Reiniciar escaneo después de 5 segundos
            ESP_LOGI(TAG, "Reiniciando escaneo en 5 segundos...");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            esp_ble_gap_start_scanning(30);
            break;
        default:
            break;
        }
        break;
    }
    
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Error al detener escaneo: %d", param->scan_stop_cmpl.status);
        } else {
            ESP_LOGI(TAG, "Escaneo BLE detenido");
        }
        break;
        
    default:
        break;
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== Iniciando Cliente BLE Scanner ===");
    
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS inicializado correctamente");
    
    // Liberar memoria del controlador Bluetooth clásico
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    // Configurar controlador Bluetooth
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Error al inicializar controlador BT: %s", esp_err_to_name(ret));
        return;
    }
    
    // Habilitar controlador en modo BLE únicamente
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Error al habilitar controlador BT: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Controlador Bluetooth habilitado");
    
    // Inicializar Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Error al inicializar Bluedroid: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Error al habilitar Bluedroid: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Bluedroid inicializado y habilitado");
    
    // Registrar callback para eventos GAP
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "Error al registrar callback GAP: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Callback GAP registrado");
    
    // Configurar parámetros de escaneo
    ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (ret) {
        ESP_LOGE(TAG, "Error al configurar parámetros de escaneo: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "=== Cliente BLE listo para escanear ===");
}