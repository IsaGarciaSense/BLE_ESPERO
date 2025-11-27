/*******************************************************************************
 * @file main.cpp
 * @brief Server_example to use BLE library
 * @version 0.0.5
 * @date 2025-06-19
 * @author isa@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "ble_sense.hpp"
#include "wifi_sense.hpp"
#include "ble_server.hpp"
// #include "device_configurator.hpp"
#include "HiveConfig.hpp"
#include "aws_sense.hpp"
#include "actuators_sense.hpp"

#include "esp_log.h"
#include <inttypes.h>

#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"

// Estructura para pasar datos entre tareas
struct DataMessage {
    char data[256];
    size_t length;
    uint64_t timestamp;
};

WifiHandler* wifiHandler = nullptr;
awsHandler* awsHelper = nullptr;
BLEServer* globalServer = nullptr;
QueueHandle_t dataQueue = nullptr;
RGB* brainLED = nullptr;

// Función para obtener el nombre del archivo basado en la fecha actual
std::string getDateBasedFilename(const char* prefix) {
    time_t now;
    struct tm timeinfo;
    char buffer[64];
    
    time(&now);
    localtime_r(&now, &timeinfo);
    
    snprintf(buffer, sizeof(buffer), "%s_%04d-%02d-%02d.txt", 
             prefix,
             timeinfo.tm_year + 1900,
             timeinfo.tm_mon + 1,
             timeinfo.tm_mday);
    
    return std::string(buffer);
}

// Función auxiliar para escribir datos en SD
bool writeToSD(const char* filename, const char* data) {
    if (!sdCard) return false;
    
    FRESULT error = sdCard->openFile(filename, SD::OpenMode::kOpenAppend);
    if (error) {
        ESP_LOGE(TAG, "SD error at openFile: %s", sdCard->getFastFsErrName(error));
        return false;
    }
    
    error = sdCard->fileWrite(data);
    if (error) {
        ESP_LOGE(TAG, "SD error at fileWrite: %s", sdCard->getFastFsErrName(error));
        sdCard->closeFile();
        return false;
    }
    
    sdCard->closeFile();
    return true;
}

// Tarea WiFi/AWS - maneja conexión y envío de datos
void wifiAwsTask(void* pvParameters) {
    ESP_LOGI(TAG, "WiFi/AWS Task started");
    
    DataMessage msg;
    char timestampedData[512];
    
    while (true) {
        // Esperar por datos en la cola
        if (xQueueReceive(dataQueue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Processing: %.*s", msg.length, msg.data);
            
            // Primero guardar TODOS los datos en SD (archivo diario)
            std::string dailyFilename = getDateBasedFilename("data");
            FRESULT error = sdCard->createFile(dailyFilename);
            if (error && error != FR_EXIST) {
                ESP_LOGE(TAG, "Error creating daily file: %s", sdCard->getFastFsErrName(error));
            }
            //TODO: Corregir el timeStamp que se está creando
            snprintf(timestampedData, sizeof(timestampedData), "[%llu] %s\n", msg.timestamp, msg.data);
            
            if (writeToSD(dailyFilename.c_str(), timestampedData)) {
                ESP_LOGI(TAG, "Saved to: %s", dailyFilename.c_str());
            } else {
                ESP_LOGE(TAG, "Failed to save to SD");
            }
            
            // Intentar conectar a AWS y enviar
            bool mqttSuccess = false;
            
            if (awsHelper->connect() == ESP_OK) {
                if (awsHelper->publish(mqttDataTopic, msg.data) == ESP_OK) {
                    ESP_LOGI(TAG, "Published to AWS");
                    mqttSuccess = true;
                } else {
                    ESP_LOGE(TAG, "Failed to publish to AWS");
                }
                awsHelper->disconnect();
            } else {
                ESP_LOGE(TAG, "Failed to connect to AWS");
            }
            
            // Si falló el envío MQTT, guardar en archivo de fallidos
            if (!mqttSuccess) {
                std::string failedFilename = getDateBasedFilename("failed");
                error = sdCard->createFile(failedFilename);
                if (error && error != FR_EXIST) {
                    ESP_LOGE(TAG, "Error creating failed file");
                }
                
                if (writeToSD(failedFilename.c_str(), timestampedData)) {
                    ESP_LOGI(TAG, "Saved to failed: %s", failedFilename.c_str());
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

// Tarea BLE - maneja el servidor BLE y advertising
void bleTask(void* pvParameters) {
    ESP_LOGI(TAG, "BLE Task started");
    
    BLELibrary* ble = (BLELibrary*)pvParameters;
    
    if (!globalServer) {
        ESP_LOGE(TAG, "Global server is NULL");
        vTaskDelete(NULL);
        return;
    }
    
    int loopCount = 0;
    int lastConnectedClients = 0;
    
    while (true) {
        loopCount++;

        // Check for connection changes every loop
        bleServerStatus_t status = globalServer->getStatus();
        if ((int)status.connectedClients != lastConnectedClients) {
            ESP_LOGI(TAG, " Connection count changed: %d -> %d", 
                    lastConnectedClients, (int)status.connectedClients);
            lastConnectedClients = (int)status.connectedClients;
        }
        
        // Verificar y forzar advertising si no está activo y no hay clientes máximos
        if (!status.advertisingActive && status.connectedClients < 4) {
            ESP_LOGW(TAG, "  Advertising not active with %d clients - forcing restart...", 
                    (int)status.connectedClients);
            esp_err_t ret = globalServer->startAdvertising();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, " Advertising restarted successfully");
            } else {
                ESP_LOGE(TAG, " Failed to restart advertising: %s", esp_err_to_name(ret));
            }
        }

        if (loopCount % 30 == 0) {
            bleServerStats_t stats = globalServer->getStats();
            
            ESP_LOGI(TAG, "===============================================");
            ESP_LOGI(TAG, " SERVER STATUS REPORT - Loop %d", loopCount);
            ESP_LOGI(TAG, "===============================================");
            ESP_LOGI(TAG, "   State: %s", globalServer->kgetStateString());
            ESP_LOGI(TAG, "   Clients connected: %d/%d", (int)status.connectedClients, 4);
            ESP_LOGI(TAG, "    Advertising: %s", status.advertisingActive ? " ACTIVE" : " INACTIVE");
            ESP_LOGI(TAG, "   Total connections: %lu", stats.totalConnections);
            ESP_LOGI(TAG, "   Data sent: %lu | Data received: %lu", stats.dataSent, stats.dataReceived);
            ESP_LOGI(TAG, "   Uptime: %llu seconds", ble->getUptime()/1000);
            ESP_LOGI(TAG, "   Free memory: %lu bytes", esp_get_free_heap_size());
            ESP_LOGI(TAG, "===============================================");
            
            // REDUCIR EL TAMAÑO PARA EVITAR EL WARNING DE BLE
            char status_data[20]; // 20 bytes máximo para BLE
            snprintf(status_data, sizeof(status_data), "UP:%d C:%d", 
                    (int)(ble->getUptime()/1000000), (int)status.connectedClients);
            globalServer->setCustomData(status_data);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main() {

    brainLED = new RGB(255, 255, 255);
    esp_err_t err = brainLED->init();
    if (err) {
        printf("LED couldn't be initialized.\n");
        printf("%s\n", esp_err_to_name(err));
        // while(1);
    }

    ESP_LOGI(TAG, "=== HIVE FIRST STEPS ===");

    esp_err_t mtu_ret = esp_ble_gatt_set_local_mtu(512);
    if (mtu_ret == ESP_OK) {
        ESP_LOGI(TAG, " Server local MTU set to 512 bytes");
    } else {
        ESP_LOGW(TAG, "Failed to set local MTU: %s", esp_err_to_name(mtu_ret));
    }

    wifiHandler = new WifiHandler(userSSID, userPassword);

    if (wifiHandler->connectWifi() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi.");
        return;
    }

    ESP_LOGI(TAG, "Connected to WiFi!");
    
    // Configurar SNTP para sincronización de tiempo
    ESP_LOGI(TAG, "Initializing SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    
    // Esperar a que el tiempo se sincronice
    int retry = 0;
    const int retry_count = 10;
    
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    time_t now;
    time(&now);
    struct tm *timeinfo_ptr = localtime(&now);
    struct tm timeinfo = *timeinfo_ptr;
    
    if (timeinfo.tm_year > (2023 - 1900)) {
        ESP_LOGI(TAG, "Time synchronized: %04d-%02d-%02d %02d:%02d:%02d",
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        ESP_LOGW(TAG, "Time not synchronized, using default time");
    }
    
    awsHelper = new awsHandler(*wifiHandler);

    if (awsHelper->init(AWS_IOT_ENDPOINT, THINGNAME, AWS_ServerCA, AWS_ClientCertificate, AWS_ClientKey) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AWS helper");
        return;
    }
    
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "NVS flash erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS init corretly");

    SPI spiMaster(SPI::SpiMode::kMaster,SPI2_HOST, kMosiPin,kMisoPin,kSclPin);
    err = spiMaster.init();
    if (err) {
        printf("SPI error while init: %s\n", esp_err_to_name(err));
    }

    sdCard = new SD(spiMaster, kCsPin);  
    err = sdCard->init();
    //TODO: Hacer task para la sd, si sale algún error que la reinicio o algo así

    if (err) {
        printf("SD error while init: %s\n", esp_err_to_name(err));
    } else {
        printf("SD was initialized!\n");
    }

    // 2. Mount SD card
    FRESULT error = sdCard->mountCard();
    if (error) {
        printf("SD error while mounting: %s\n", sdCard->getFastFsErrName(error));
    } else {
        std::string currentPath = sdCard->getCurrentDir();
        printf(
            ("Card mounted, here is the root path - " + currentPath).c_str());
        printf("\n");
    }
    // std::string fileName ="testHive.txt";


    error = sdCard->createFile(fileName);
    if (error) {
        printf("SD error at createFile: %s\n", sdCard->getFastFsErrName(error));
    } else {
        printf("File created at: %s\n", sdCard->getCurrentDir().c_str());
    }

    // Mostrar datos de la librería
    BLELibrary::printLibraryInfo();

    // Crear configuración personalizada
    bleLibraryConfig_t config;
    ret = BLELibrary::createDefaultConfig(&config, BLE_MODE_SERVER_ONLY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fail creating configuration: %s", esp_err_to_name(ret));
        return;
    }
    
    // Personalizar configuración
    strncpy(config.deviceName, "HiveSense", BLE_MAX_DEVICE_NAME_LEN - 1);
    config.autoStart = false; // Controlaremos manualmente
    config.enableLogging = true;
    config.logLevel = ESP_LOG_INFO;

    BLELibrary* ble = new BLELibrary(config);
    if (ble == nullptr) {
        ESP_LOGE(TAG, "Fail creating ble_library");
        return;
    }

    ret = ble->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fail initializating ble_library: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }

    BLEServer* server = ble->getServer();
    if (server != nullptr) {
        ESP_LOGI(TAG, "Settings server");

        esp_err_t name_ret = server->setDeviceName("HiveSense");
        if (name_ret == ESP_OK) {
            ESP_LOGI(TAG, "Device name successfully set to HiveSense");
        } else {
            ESP_LOGW(TAG, "Failed to set device name: %s", esp_err_to_name(name_ret));
        }
        
        bleServerConfig_t server_config = server->getConfig();
        server_config.dataUpdateInterval = 0;    
        server_config.clientTimeout = 60000;     // 60 segundos timeout en lugar de 0
        server_config.maxClients = 4;
        server_config.enableNotifications = true;
        server_config.autoStartAdvertising = true;
        server_config.enableJsonCommands = true;
        strncpy(server_config.deviceName, "HiveSense", BLE_MAX_DEVICE_NAME_LEN - 1);

        ret = server->setConfig(server_config);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Warnings in settings server: %s", esp_err_to_name(ret));
        }

        // Configurar callbacks básicos con más logging
        server->setClientConnectedCallback([](uint16_t connID, const bleDeviceInfo_t* clientInfo) {
            ESP_LOGI(TAG, " Client connected - ID: %d, MAC: %02x:%02x:%02x:%02x:%02x:%02x", 
                    connID,
                    clientInfo->address[0], clientInfo->address[1], clientInfo->address[2],
                    clientInfo->address[3], clientInfo->address[4], clientInfo->address[5]);
        });

        server->setClientDisconnectedCallback([](uint16_t connID, int reason) {
            ESP_LOGI(TAG, " Client disconnected - ID: %d, Reason: %d", connID, reason);
        });

        server->setAdvertisingCallback([](bool isStarted, esp_err_t result) {
            if (isStarted && result == ESP_OK) {
                ESP_LOGI(TAG, " Advertising STARTED successfully");
            } else if (!isStarted || result != ESP_OK) {
                ESP_LOGE(TAG, " Advertising STOPPED or FAILED: %s", esp_err_to_name(result));
            }
        });

        server->setDataWrittenCallback([](uint16_t connID, const uint8_t* data, uint16_t length) {
            ESP_LOGI(TAG, " Client %d data received (%d bytes)", connID, length);
            
            if (length > 0 && length < 256) {
                // Solo preparar el mensaje y enviarlo a la cola
                // La escritura en SD se hace en la tarea WiFi/AWS para no bloquear el callback
                if(brainLED != nullptr){
                    brainLED->setColor(0, 255, 0); // Verde al recibir datos
                    brainLED->pulse(200); // Pulso breve
                }
                // vTaskDelay(pdMS_TO_TICKS(100));
                DataMessage msg;
                memcpy(msg.data, data, length);
                msg.data[length] = '\0';
                msg.length = length;
                
                struct timeval tv;
                gettimeofday(&tv, NULL);
                msg.timestamp = (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
                
                if (xQueueSend(dataQueue, &msg, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Queue full, data dropped");
                }
            }
        });


        // Manual Advertising
        ESP_LOGI(TAG, "Init advertising...");
        ret = server->startAdvertising();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Fail init advertising: %s", esp_err_to_name(ret));
            delete ble;
            return;
        }
        
        // Guardar referencia global al servidor
        globalServer = server;
        
        // Crear cola para comunicación entre tareas
        dataQueue = xQueueCreate(20, sizeof(DataMessage));
        if (dataQueue == NULL) {
            ESP_LOGE(TAG, "Failed to create data queue");
            delete ble;
            return;
        }
        ESP_LOGI(TAG, "Data queue created successfully");
        
        // Crear tarea WiFi/AWS con más stack
        BaseType_t wifiTaskCreated = xTaskCreatePinnedToCore(
            wifiAwsTask,
            "WiFi_AWS_Task",
            12288,  // 12KB de stack
            NULL,
            5,
            NULL,
            1  // Core 1
        );
        
        if (wifiTaskCreated != pdPASS) {
            ESP_LOGE(TAG, "Failed to create WiFi/AWS task");
            delete ble;
            return;
        }
        ESP_LOGI(TAG, "WiFi/AWS task created on core 1");
        
        // Crear tarea BLE con más stack
        BaseType_t bleTaskCreated = xTaskCreatePinnedToCore(
            bleTask,
            "BLE_Task",
            6144,  // 6KB de stack
            (void*)ble,
            4,
            NULL,
            0  // Core 0
        );
        
        if (bleTaskCreated != pdPASS) {
            ESP_LOGE(TAG, "Failed to create BLE task");
            delete ble;
            return;
        }
        ESP_LOGI(TAG, "BLE task created on core 0");
        
        ESP_LOGI(TAG, "=== System running with separate tasks ===");
        ESP_LOGI(TAG, "BLE Task: Handling connections and data reception");
        ESP_LOGI(TAG, "WiFi/AWS Task: Publishing data to MQTT and saving failed attempts");
        
    } else {
        ESP_LOGE(TAG, "Get server not found");
        delete ble;
        return;
    }
}
