/*******************************************************************************
 * @file main.cpp
 * @brief Servidor BLE compatible con apps Serial Monitor
 * Configurado para trabajar con apps que buscan perfil serial
 * @version 0.0.5
 * @date 2025-06-19
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "BLE.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include <cJSON.h>

static const char* TAG = "SERIAL_BLE";

// ✅ UUIDs COMPATIBLES CON SERIAL MONITOR APPS
// Estos UUIDs son reconocidos por muchas apps de serial
#define SERIAL_SERVICE_UUID     "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // Nordic UART Service
#define SERIAL_RX_CHAR_UUID     "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // RX (Write)
#define SERIAL_TX_CHAR_UUID     "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // TX (Notify)

// Variables globales
static BLEServer* g_server = nullptr;
static bool deviceConnected = false;

// Función para convertir UUID string a bytes (orden correcto para BLE)
void uuid_string_to_bytes_ble(const char* uuid_str, uint8_t* uuid_bytes) {
    // Remover guiones y convertir, pero en orden reverso para BLE
    char clean_uuid[33];
    int clean_idx = 0;
    
    // Limpiar UUID (remover guiones)
    for (int i = 0; i < strlen(uuid_str); i++) {
        if (uuid_str[i] != '-') {
            clean_uuid[clean_idx++] = uuid_str[i];
        }
    }
    clean_uuid[clean_idx] = '\0';
    
    // Convertir en orden reverso (BLE usa little-endian)
    for (int i = 0; i < 16; i++) {
        char hex_pair[3] = {clean_uuid[30 - (i*2)], clean_uuid[31 - (i*2)], '\0'};
        uuid_bytes[i] = (uint8_t)strtol(hex_pair, NULL, 16);
    }
}

// Función para generar temperatura aleatoria
int generateRandomTemperature() {
    return 20; // Temperatura entre 20-35°C
}

// Función para enviar respuesta JSON
bool sendJsonResponse(uint16_t conn_id, const char* jsonString) {
    if (!g_server || !jsonString) {
        ESP_LOGE(TAG, "❌ Error: server o JSON nulo");
        return false;
    }

    esp_err_t ret = g_server->sendJsonResponse(conn_id, jsonString);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "📤 JSON enviado: %s", jsonString);
        return true;
    } else {
        ESP_LOGE(TAG, "❌ Error enviando JSON: %s", esp_err_to_name(ret));
        return false;
    }
}

// Procesar comando JSON recibido
bool processJsonCommand(const ble_json_command_t* command) {
    if (!command) {
        return false;
    }
    
    if (strlen(command->command_json) == 0) {
        return false;
    }
    
    uint16_t conn_id = command->conn_id;
    const char* jsonStr = command->command_json;
    
    ESP_LOGI(TAG, "📥 Comando recibido: %s", jsonStr);
    
    // Verificar si es JSON
    if (jsonStr[0] == '{') {
        // Procesar como JSON
        cJSON *json = cJSON_Parse(jsonStr);
        if (!json) {
            ESP_LOGE(TAG, "❌ Error parseando JSON");
            return false;
        }
        
        bool processed = false;
        
        // Comando pullVars
        cJSON *pullVars = cJSON_GetObjectItem(json, "pullVars");
        if (cJSON_IsNumber(pullVars) && pullVars->valueint == 1) {
            ESP_LOGI(TAG, "🔍 Comando JSON pullVars");
            
            cJSON *response = cJSON_CreateObject();
            cJSON_AddNumberToObject(response, "Temperatura", generateRandomTemperature());
            cJSON_AddStringToObject(response, "Status", "OK");
            
            char *responseString = cJSON_Print(response);
            if (responseString) {
                sendJsonResponse(conn_id, responseString);
                free(responseString);
            }
            cJSON_Delete(response);
            processed = true;
        }
        
        cJSON_Delete(json);
        return processed;
    } else {
        // Procesar como comando de texto simple
        ESP_LOGI(TAG, "📝 Comando de texto: %s", jsonStr);
        
        char response[200];
        
        if (strstr(jsonStr, "temp") || strstr(jsonStr, "TEMP")) {
            snprintf(response, sizeof(response), "Temperatura: %d°C\n", generateRandomTemperature());
        } else if (strstr(jsonStr, "ping") || strstr(jsonStr, "PING")) {
            snprintf(response, sizeof(response), "PONG from ESP32\n");
        } else if (strstr(jsonStr, "status") || strstr(jsonStr, "STATUS")) {
            snprintf(response, sizeof(response), "ESP32 Online - Memoria: %lu bytes\n", esp_get_free_heap_size());
        } else if (strstr(jsonStr, "help") || strstr(jsonStr, "HELP")) {
            snprintf(response, sizeof(response), 
                "Comandos disponibles:\n"
                "- temp: Obtener temperatura\n"
                "- ping: Test de conexión\n"
                "- status: Estado del sistema\n"
                "- {\"pullVars\":1}: Comando JSON\n");
        }
        
        sendJsonResponse(conn_id, response);
        return true;
    }
}

// Callback cuando cliente se conecta
void onClientConnected(uint16_t conn_id, const ble_device_info_t* client_info) {
    ESP_LOGI(TAG, "🔗 Cliente conectado - ID: %d", conn_id);
    deviceConnected = true;
    
    // Enviar mensaje de bienvenida
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    const char* welcome = 
        "=== ESP32 Serial BLE ===\n"
        "Dispositivo conectado exitosamente\n"
        "Escribe 'help' para ver comandos\n"
        "Ejemplo JSON: {\"pullVars\":1}\n"
        "========================\n";
    
    sendJsonResponse(conn_id, welcome);
}

// Callback cuando cliente se desconecta
void onClientDisconnected(uint16_t conn_id, int reason) {
    ESP_LOGI(TAG, "❌ Cliente desconectado - ID: %d, razón: %d", conn_id, reason);
    deviceConnected = false;
}

// Función principal
extern "C" void app_main() {
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "=== SERVIDOR BLE PARA SERIAL MONITOR APPS ===");
    ESP_LOGI(TAG, "================================================");
    
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ NVS inicializado");

    // Crear configuración BLE con UUIDs de Nordic UART
    ble_library_config_t config;
    BLELibrary::createDefaultConfig(&config, BLE_MODE_SERVER_ONLY);
    strcpy(config.device_name, "ESP32_Serial");
    config.auto_start = false;
    
    // ✅ CONFIGURAR CON UUIDs DE NORDIC UART SERVICE
    config.security.level = BLE_SECURITY_BASIC;
    config.security.use_custom_uuids = true;
    config.security.require_authentication = false;
    
    // Convertir UUIDs de Nordic UART Service
    uuid_string_to_bytes_ble(SERIAL_SERVICE_UUID, config.security.service_uuid);
    uuid_string_to_bytes_ble(SERIAL_RX_CHAR_UUID, config.security.custom_char_uuid);    // RX (Write)
    uuid_string_to_bytes_ble(SERIAL_TX_CHAR_UUID, config.security.battery_char_uuid);   // TX (Notify)

    ESP_LOGI(TAG, "📋 === CONFIGURACIÓN SERIAL BLE ===");
    ESP_LOGI(TAG, "   Dispositivo: ESP32_Serial");
    ESP_LOGI(TAG, "   Service: Nordic UART Service");
    ESP_LOGI(TAG, "   UUID Service: %s", SERIAL_SERVICE_UUID);
    ESP_LOGI(TAG, "   UUID RX: %s", SERIAL_RX_CHAR_UUID);
    ESP_LOGI(TAG, "   UUID TX: %s", SERIAL_TX_CHAR_UUID);
    ESP_LOGI(TAG, "===================================");

    // Crear y configurar servidor BLE
    BLELibrary* ble = new BLELibrary(config);
    if (!ble) {
        ESP_LOGE(TAG, "❌ Error creando BLE");
        return;
    }

    ret = ble->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error inicializando BLE: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }

    // Configurar servidor
    BLEServer* server = ble->getServer();
    if (server) {
        g_server = server;
        
        // Configuración optimizada para Serial Monitor
        ble_server_config_t server_config = server->getConfig();
        server_config.enable_json_commands = true;
        server_config.data_update_interval_ms = 0;
        server_config.client_timeout_ms = 0;
        server_config.simulate_battery_drain = false;
        server_config.auto_start_advertising = false;
        server_config.enable_notifications = true;
        server_config.max_clients = 1;
        server_config.advertising_interval_ms = 100;
        server->setConfig(server_config);

        // Configurar callbacks
        server->setClientConnectedCallback(onClientConnected);
        server->setClientDisconnectedCallback(onClientDisconnected);
        server->setJsonCommandCallback(processJsonCommand);

        // Esperar para estabilizar
        ESP_LOGI(TAG, "⏳ Inicializando servicios...");
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Iniciar advertising
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

    ESP_LOGI(TAG, "🚀 Servidor BLE Serial iniciado");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📱 === INSTRUCCIONES PARA SERIAL MONITOR ===");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📋 OPCIÓN 1: Perfil personalizado en tu app");
    ESP_LOGI(TAG, "   1. Mantén presionado 'ESP32_Serial'");
    ESP_LOGI(TAG, "   2. 'Define custom profile'");
    ESP_LOGI(TAG, "   3. Service UUID: %s", SERIAL_SERVICE_UUID);
    ESP_LOGI(TAG, "   4. RX UUID: %s", SERIAL_RX_CHAR_UUID);
    ESP_LOGI(TAG, "   5. TX UUID: %s", SERIAL_TX_CHAR_UUID);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📋 OPCIÓN 2: Apps recomendadas");
    ESP_LOGI(TAG, "   - 'Serial Bluetooth Terminal'");
    ESP_LOGI(TAG, "   - 'nRF Connect'");
    ESP_LOGI(TAG, "   - 'BLE Terminal' ");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🔧 === COMANDOS DISPONIBLES ===");
    ESP_LOGI(TAG, "   Texto simple:");
    ESP_LOGI(TAG, "   - temp        -> Obtener temperatura");
    ESP_LOGI(TAG, "   - ping        -> Test conexión");
    ESP_LOGI(TAG, "   - status      -> Estado sistema");
    ESP_LOGI(TAG, "   - help        -> Mostrar ayuda");
    ESP_LOGI(TAG, "   ");
    ESP_LOGI(TAG, "   JSON:");
    ESP_LOGI(TAG, "   - {\"pullVars\":1}  -> Temperatura JSON");
    ESP_LOGI(TAG, "==========================================");

    // Bucle principal
    uint32_t counter = 0;
    
    while (true) {
        counter++;
        
        // Mostrar estado cada 30 segundos
        if (counter % 30 == 0) {
            ble_server_stats_t stats = server->getStats();
            ESP_LOGI(TAG, "📊 Estado: %s | Conexiones: %lu | Comandos: %lu", 
                     deviceConnected ? "🟢 CONECTADO" : "🔵 ESPERANDO",
                     stats.total_connections,
                     stats.json_commands_processed);
        }
        
        // Enviar datos automáticos cada 60 segundos si conectado
        if (counter % 60 == 0 && deviceConnected) {
            char autoData[150];
            snprintf(autoData, sizeof(autoData), 
                "Auto-update: Temp=%d°C | Uptime=%lum | Mem=%lu bytes\n",
                generateRandomTemperature(), 
                counter / 60,
                esp_get_free_heap_size());
            
            sendJsonResponse(1, autoData);
        }
        
        // Verificar advertising
        if (!deviceConnected && !server->isAdvertising() && counter % 120 == 0) {
            ESP_LOGW(TAG, "⚠️ Reiniciando advertising...");
            server->startAdvertising();
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    delete ble;
}