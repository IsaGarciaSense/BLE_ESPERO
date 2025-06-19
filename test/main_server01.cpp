#include "BLE.hpp"
#include "esp_log.h"
#include "nvs_flash.h"

static const char* TAG = "MAIN";

extern "C" void app_main() {
    ESP_LOGI(TAG, "=== Iniciando Servidor BLE Seguro ===");
    
    // Inicializar NVS (requerido por BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Borrando NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ NVS inicializado correctamente");

    // Mostrar datos de la librería
    BLELibrary::printLibraryInfo();

    // Crear configuración personalizada para evitar las tareas problemáticas
    ble_library_config_t config;
    ret = BLELibrary::createDefaultConfig(&config, BLE_MODE_SERVER_ONLY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error creando configuración: %s", esp_err_to_name(ret));
        return;
    }
    
    // Personalizar configuración
    strncpy(config.device_name, "154_BLE_Server", BLE_MAX_DEVICE_NAME_LEN - 1);
    config.auto_start = false; // Controlaremos manualmente
    config.enable_logging = true;
    config.log_level = ESP_LOG_INFO;

    // Crear la instancia BLE
    ESP_LOGI(TAG, "Creando BLE Library...");
    BLELibrary* ble = new BLELibrary(config);
    if (ble == nullptr) {
        ESP_LOGE(TAG, "❌ Error creando BLELibrary");
        return;
    }

    // Inicializar
    ESP_LOGI(TAG, "Inicializando BLE Library...");
    ret = ble->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error inicializando: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }

    // Obtener servidor y configurarlo para evitar las tareas problemáticas
    BLEServer* server = ble->getServer();
    if (server != nullptr) {
        ESP_LOGI(TAG, "Configurando servidor...");
        
        // Configurar servidor sin las tareas automáticas que causan problemas
        ble_server_config_t server_config = server->getConfig();
        server_config.data_update_interval_ms = 0;    // ❌ Desactivar tarea automática
        server_config.client_timeout_ms = 0;          // ❌ Desactivar tarea de timeout
        server_config.simulate_battery_drain = false; // ❌ Desactivar simulación
        server_config.max_clients = 4;
        server_config.enable_notifications = true;
        server_config.auto_start_advertising = false;
        
        ret = server->setConfig(server_config);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Advertencia al configurar servidor: %s", esp_err_to_name(ret));
        }

        // Configurar callbacks
        server->setClientConnectedCallback([](uint16_t conn_id, const ble_device_info_t* client_info) {
            ESP_LOGI(TAG, "🔗 Cliente conectado - ID: %d", conn_id);
        });

        server->setClientDisconnectedCallback([](uint16_t conn_id, int reason) {
            ESP_LOGI(TAG, "❌ Cliente desconectado - ID: %d, Razón: %d", conn_id, reason);
        });

        server->setDataWrittenCallback([](uint16_t conn_id, const uint8_t* data, uint16_t length) {
            ESP_LOGI(TAG, "📝 Cliente %d escribió: %.*s", conn_id, length, data);
        });

        server->setAdvertisingCallback([](bool started, esp_err_t error_code) {
            if (started && error_code == ESP_OK) {
                ESP_LOGI(TAG, "📡 ✅ Advertising iniciado");
            } else if (!started) {
                ESP_LOGI(TAG, "📡 ⏹️ Advertising detenido");
            } else {
                ESP_LOGE(TAG, "📡 ❌ Error en advertising: %s", esp_err_to_name(error_code));
            }
        });

        // Iniciar advertising manualmente
        ESP_LOGI(TAG, "Iniciando advertising...");
        ret = server->startAdvertising();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Error iniciando advertising: %s", esp_err_to_name(ret));
            delete ble;
            return;
        }
    } else {
        ESP_LOGE(TAG, "❌ No se pudo obtener el servidor");
        delete ble;
        return;
    }

    ESP_LOGI(TAG, "🚀 ✅ Servidor BLE iniciado correctamente");
    ESP_LOGI(TAG, "📡 Anunciándose como: 154_BLE_Server");
    ESP_LOGI(TAG, "🔧 Modo seguro: Sin tareas automáticas");

    // Bucle principal manual (reemplaza las tareas automáticas)
    uint32_t loop_count = 0;
    uint8_t battery_level = 100;
    
    ESP_LOGI(TAG, "🔄 Iniciando bucle principal...");
    
    while (true) {
        loop_count++;
        
        // Actualizar datos cada 15 segundos (manualmente)
        if (loop_count % 15 == 0) {
            // Simular cambio de batería
            if (battery_level > 0) {
                battery_level--;
            } else {
                battery_level = 100;
            }
            
            // Crear datos personalizados
            char custom_data[64];
            uint64_t uptime_sec = ble->getUptime() / 1000;
            snprintf(custom_data, sizeof(custom_data), "Loop:%lu Up:%llus", loop_count, uptime_sec);
            
            // Actualizar datos
            ret = server->setBatteryLevel(battery_level);
            if (ret == ESP_OK) {
                ret = server->setCustomData(custom_data);
            }
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "📊 Datos actualizados - Bat:%d%% Data:%s", battery_level, custom_data);
                
                // Notificar a clientes conectados
                if (server->hasConnectedClients()) {
                    server->notifyAllClients();
                }
            } else {
                ESP_LOGW(TAG, "⚠️ Error actualizando datos: %s", esp_err_to_name(ret));
            }
        }
        
        // Mostrar estado cada 30 segundos
        if (loop_count % 30 == 0) {
            ble_server_status_t status = server->getStatus();
            ble_server_stats_t stats = server->getStats();
            
            ESP_LOGI(TAG, "📈 === ESTADO SERVIDOR ===");
            ESP_LOGI(TAG, "   Estado: %s", server->getStateString());
            ESP_LOGI(TAG, "   Clientes conectados: %d", status.connected_clients);
            ESP_LOGI(TAG, "   Advertising activo: %s", status.advertising_active ? "✅ SÍ" : "❌ NO");
            ESP_LOGI(TAG, "   Conexiones totales: %lu", stats.total_connections);
            ESP_LOGI(TAG, "   Paquetes enviados: %lu", stats.data_packets_sent);
            ESP_LOGI(TAG, "   Tiempo activo: %llu ms", ble->getUptime());
            ESP_LOGI(TAG, "   Memoria libre: %lu bytes", esp_get_free_heap_size());
            ESP_LOGI(TAG, "========================");
        }
        
        // Verificar salud cada minuto
        if (loop_count % 60 == 0) {
            bool healthy = server->performHealthCheck();
            ESP_LOGI(TAG, "🏥 Check salud: %s", healthy ? "✅ BUENO" : "❌ PROBLEMA");
            
            uint32_t free_heap = esp_get_free_heap_size();
            if (free_heap < 20000) {
                ESP_LOGW(TAG, "⚠️ Memoria baja: %lu bytes", free_heap);
            }
        }
        
        // Dormir 1 segundo
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Cleanup (nunca se ejecuta en este ejemplo)
    ESP_LOGI(TAG, "Finalizando...");
    delete ble;
}