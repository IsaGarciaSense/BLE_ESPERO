#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "BLE_SERVER";

// Configuración del servidor
#define DEVICE_NAME             "ISA_BLE_Server"
static uint16_t service_uuid = 0x180F;  // Battery Service UUID
#define CHAR_UUID_BATTERY       0x2A19  // Battery Level Characteristic
#define CHAR_UUID_CUSTOM        0xFF01  // Custom Characteristic

// Configuración de advertising
#define ADV_CONFIG_FLAG         (1 << 0)
#define SCAN_RSP_CONFIG_FLAG    (1 << 1)

// Handles para el servicio y características
static uint16_t service_handle = 0;
static uint16_t battery_char_handle = 0;
static uint16_t custom_char_handle = 0;

// Variables de estado
static bool connected = false;
static uint16_t conn_id = 0;
static esp_gatt_if_t gatts_if = 0;
static uint8_t battery_level = 85;  // Nivel de batería simulado
static uint8_t custom_data[20] = "Hola desde ESP32!";

// IDs de la aplicación GATT
#define GATTS_APP_ID            0x55
#define GATTS_NUM_HANDLE        10

// Configuración de advertising
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp           = false,
    .include_name           = true,
    .include_txpower        = false,
    .min_interval           = 0x0006,
    .max_interval           = 0x0010,
    .appearance             = 0x00,
    .manufacturer_len       = 0,
    .p_manufacturer_data    = NULL,
    .service_data_len       = 0,
    .p_service_data         = NULL,
    .service_uuid_len       = 0,
    .p_service_uuid         = NULL,
    .flag                   = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Configuración de scan response
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp           = true,
    .include_name           = true,
    .include_txpower        = true,
    .min_interval           = 0,
    .max_interval           = 0,
    .appearance             = 0x00,
    .manufacturer_len       = 0,
    .p_manufacturer_data    = NULL,
    .service_data_len       = 0,
    .p_service_data         = NULL,
    .service_uuid_len       = 0,
    .p_service_uuid         = NULL,
    .flag                   = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Parámetros de advertising
static esp_ble_adv_params_t adv_params = {
    .adv_int_min            = 0x20,
    .adv_int_max            = 0x40,
    .adv_type               = ADV_TYPE_IND,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr              = {0},
    .peer_addr_type         = BLE_ADDR_TYPE_PUBLIC,
    .channel_map            = ADV_CHNL_ALL,
    .adv_filter_policy      = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Tarea para actualizar datos periódicamente
static void update_data_task(void *pvParameters) {
    while (1) {
        // Simular cambio en nivel de batería
        battery_level = (battery_level > 5) ? battery_level - 1 : 100;
        
        // Actualizar datos custom
        snprintf((char*)custom_data, sizeof(custom_data), "Tiempo: %llu", esp_timer_get_time() / 1000000);
        
        // Si hay cliente conectado, enviar notificación
        if (connected && gatts_if != 0) {
            // Notificar nivel de batería
            esp_ble_gatts_send_indicate(gatts_if, conn_id, battery_char_handle,
                                      sizeof(battery_level), &battery_level, false);
            
            // Notificar datos custom
            esp_ble_gatts_send_indicate(gatts_if, conn_id, custom_char_handle,
                                      strlen((char*)custom_data), custom_data, false);
        }
        
        ESP_LOGI(TAG, "Datos actualizados - Batería: %d%%, Custom: %s", battery_level, custom_data);
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Actualizar cada 5 segundos
    }
}

// Callback para eventos GAP
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising data configurado");
        esp_ble_gap_start_advertising(&adv_params);
        break;
        
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan response data configurado");
        break;
        
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Error al iniciar advertising");
        } else {
            ESP_LOGI(TAG, "📡 Advertising iniciado - Dispositivo visible como: %s", DEVICE_NAME);
        }
        break;
        
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Error al detener advertising");
        } else {
            ESP_LOGI(TAG, "Advertising detenido");
        }
        break;
        
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "Parámetros de conexión actualizados - latencia: %d, timeout: %d",
                param->update_conn_params.latency,
                param->update_conn_params.timeout);
        break;
        
    default:
        break;
    }
}

// Callback para eventos GATTS (GATT Server)
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if_param, esp_ble_gatts_cb_param_t *param) {
    esp_err_t ret;
    esp_err_t set_dev_name_ret;
    esp_gatt_srvc_id_t service_id;
    esp_bt_uuid_t char_uuid_battery;
    esp_bt_uuid_t char_uuid_custom;
    esp_attr_value_t battery_attr_value;
    esp_attr_value_t custom_attr_value;
    
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATT Server registrado - app_id: %04x, status: %d",
                param->reg.app_id, param->reg.status);
        
        gatts_if = gatts_if_param; // Guardar el gatts_if
        
        // Configurar nombre del dispositivo
        set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
        if (set_dev_name_ret) {
            ESP_LOGE(TAG, "Error al configurar nombre del dispositivo: %s", esp_err_to_name(set_dev_name_ret));
        }
        
        // Configurar advertising data
        ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret) {
            ESP_LOGE(TAG, "Error al configurar advertising data: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Advertising data configurado correctamente");
        }
        
        // Configurar scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret) {
            ESP_LOGE(TAG, "Error al configurar scan response data: %s", esp_err_to_name(ret));
        }
        
        // Crear servicio
        service_id.is_primary = true;
        service_id.id.inst_id = 0x00;
        service_id.id.uuid.len = ESP_UUID_LEN_16;
        service_id.id.uuid.uuid.uuid16 = service_uuid;
        
        esp_ble_gatts_create_service(gatts_if_param, &service_id, GATTS_NUM_HANDLE);
        break;
        
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "Servicio creado - service_handle: %d", param->create.service_handle);
        service_handle = param->create.service_handle;
        
        // Iniciar servicio
        esp_ble_gatts_start_service(service_handle);
        
        // Agregar característica de batería
        char_uuid_battery.len = ESP_UUID_LEN_16;
        char_uuid_battery.uuid.uuid16 = CHAR_UUID_BATTERY;
        
        battery_attr_value.attr_max_len = sizeof(battery_level);
        battery_attr_value.attr_len = sizeof(battery_level);
        battery_attr_value.attr_value = &battery_level;
        
        esp_ble_gatts_add_char(service_handle, &char_uuid_battery,
                              ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                              ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                              &battery_attr_value, NULL);
        break;
        
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(TAG, "Característica agregada - attr_handle: %d, service_handle: %d",
                param->add_char.attr_handle, param->add_char.service_handle);
        
        if (param->add_char.char_uuid.uuid.uuid16 == CHAR_UUID_BATTERY) {
            battery_char_handle = param->add_char.attr_handle;
            
            // Agregar característica custom
            char_uuid_custom.len = ESP_UUID_LEN_16;
            char_uuid_custom.uuid.uuid16 = CHAR_UUID_CUSTOM;
            
            custom_attr_value.attr_max_len = sizeof(custom_data);
            custom_attr_value.attr_len = (uint16_t)strlen((char*)custom_data);
            custom_attr_value.attr_value = custom_data;
            
            esp_ble_gatts_add_char(service_handle, &char_uuid_custom,
                                  ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                  ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                  &custom_attr_value, NULL);
        } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_UUID_CUSTOM) {
            custom_char_handle = param->add_char.attr_handle;
        }
        break;
        
    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "🚀 Servicio iniciado - service_handle: %d", param->start.service_handle);
        break;
        
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "🔗 Cliente conectado - conn_id: %d", param->connect.conn_id);
        connected = true;
        conn_id = param->connect.conn_id;
        
        // Detener advertising
        esp_ble_gap_stop_advertising();
        break;
        
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Cliente desconectado - conn_id: %d, reason: %d", 
                param->disconnect.conn_id, param->disconnect.reason);
        connected = false;
        conn_id = 0;
        
        // Reiniciar advertising
        esp_ble_gap_start_advertising(&adv_params);
        break;
        
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "Lectura solicitada - conn_id: %d, handle: %d", 
                param->read.conn_id, param->read.handle);
        
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        
        if (param->read.handle == battery_char_handle) {
            rsp.attr_value.len = sizeof(battery_level);
            rsp.attr_value.value[0] = battery_level;
            ESP_LOGI(TAG, "Enviando nivel de batería: %d%%", battery_level);
        } else if (param->read.handle == custom_char_handle) {
            rsp.attr_value.len = strlen((char*)custom_data);
            memcpy(rsp.attr_value.value, custom_data, rsp.attr_value.len);
            ESP_LOGI(TAG, "Enviando datos custom: %s", custom_data);
        }
        
        esp_ble_gatts_send_response(gatts_if_param, param->read.conn_id, param->read.trans_id,
                                   ESP_GATT_OK, &rsp);
        break;
        
    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(TAG, "Escritura recibida - conn_id: %d, handle: %d, len: %d", 
                param->write.conn_id, param->write.handle, param->write.len);
        
        if (param->write.handle == custom_char_handle && param->write.len > 0) {
            memcpy(custom_data, param->write.value, param->write.len);
            custom_data[param->write.len] = '\0';
            ESP_LOGI(TAG, "Nuevos datos custom recibidos: %s", custom_data);
        }
        
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if_param, param->write.conn_id, param->write.trans_id,
                                       ESP_GATT_OK, NULL);
        }
        break;
        
    default:
        break;
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== Iniciando Servidor BLE ===");
    
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
    
    // Registrar callbacks
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "Error al registrar callback GAP: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "Error al registrar callback GATTS: %s", esp_err_to_name(ret));
        return;
    }
    
    // Registrar aplicación GATT
    ret = esp_ble_gatts_app_register(GATTS_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "Error al registrar aplicación GATT: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Callbacks registrados correctamente");
    
    // Crear tarea para actualizar datos
    xTaskCreate(update_data_task, "update_data", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "=== Servidor BLE listo - Esperando conexiones ===");
}