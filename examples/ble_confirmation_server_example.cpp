/*******************************************************************************
 * @file ble_confirmation_server_example.cpp
 * @brief BLE server example demonstrating the new acknowledgment confirmation system
 * @details Shows how to send data with acknowledgment requirements and handle
 *          automatic disconnection after confirmation
 *
 * @version 1.0.0
 * @date 2025-12-04
 * @author GitHub Copilot (Enhanced Confirmation System)
 *******************************************************************************
 *******************************************************************************/

#include "ble_sense.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/******************************************************************************/
/*                              Constants                                     */
/******************************************************************************/

static const char* TAG = "BLE_CONFIRM_SERVER";
const char* SERVER_DEVICE_NAME = "HiveSense";

/******************************************************************************/
/*                         Global Variables                                   */
/******************************************************************************/

uint32_t loopCount = 0;
static BLEServer* g_server = nullptr;

/******************************************************************************/
/*                         Callback Functions                                */
/******************************************************************************/

// Callback when a client connects
void onClientConnected(uint16_t connID, const bleDeviceInfo_t* clientInfo) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            clientInfo->address[0], clientInfo->address[1], clientInfo->address[2],
            clientInfo->address[3], clientInfo->address[4], clientInfo->address[5]);

    ESP_LOGI(TAG, "\n🎉 CLIENT CONNECTED!");
    ESP_LOGI(TAG, "   Connection ID: %d", connID);
    ESP_LOGI(TAG, "   MAC: %s", macStr);
    ESP_LOGI(TAG, "   RSSI: %d dBm\n", clientInfo->rssi);
    
    // Configure this client for confirmation tracking with auto-disconnect after 30 seconds
    if (g_server != nullptr) {
        g_server->setClientConfirmationMode(connID, true, 10000); // Auto-disconnect after 10 second timeout
        ESP_LOGI(TAG, "🔧 Client %d configured for auto-disconnect after acknowledgment", connID);
    }
}

// Callback when a client disconnects
void onClientDisconnected(uint16_t connID, int reason) {
    ESP_LOGI(TAG, "\n👋 CLIENT DISCONNECTED");
    ESP_LOGI(TAG, "   Connection ID: %d", connID);
    ESP_LOGI(TAG, "   Reason: %d\n", reason);
}

// Callback when data is written by a client
void onDataWritten(uint16_t connID, const uint8_t* data, uint16_t length) {
    ESP_LOGI(TAG, "\n📨 DATA RECEIVED from client %d (%d bytes):", connID, length);
    ESP_LOGI(TAG, "   Content: %.*s\n", length, data);
}

// Callback for advertising events
void onAdvertising(bool started, esp_err_t result) {
    if (started && result == ESP_OK) {
        ESP_LOGI(TAG, "📡 ADVERTISING STARTED successfully");
    } else if (!started || result != ESP_OK) {
        ESP_LOGE(TAG, "❌ ADVERTISING FAILED or STOPPED: %s", esp_err_to_name(result));
    }
}

/******************************************************************************/
/*                              Main Function                                 */
/******************************************************************************/

extern "C" void app_main() {
    ESP_LOGI(TAG, "\n=== BLE SERVER CONFIRMATION DEMONSTRATION ===");
    ESP_LOGI(TAG, "Server Device Name: %s", SERVER_DEVICE_NAME);
    ESP_LOGI(TAG, "Features Demonstrated:");
    ESP_LOGI(TAG, "  • Send data with acknowledgment requirements");
    ESP_LOGI(TAG, "  • Track client confirmation status");
    ESP_LOGI(TAG, "  • Automatic disconnection after acknowledgment");
    ESP_LOGI(TAG, "  • Confirmation timeout handling");
    ESP_LOGI(TAG, "============================================\n");

    // Step 1: Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ NVS initialized");

    // Step 2: Create BLE Library in server mode
    bleLibraryConfig_t bleConfig;
    BLELibrary::createDefaultConfig(&bleConfig, BLE_MODE_SERVER_ONLY);
    strncpy(bleConfig.deviceName, SERVER_DEVICE_NAME, BLE_MAX_DEVICE_NAME_LEN - 1);
    
    BLELibrary* ble = new BLELibrary(bleConfig);
    if (ble == nullptr) {
        ESP_LOGE(TAG, "❌ Failed to create BLE Library");
        return;
    }
    ESP_LOGI(TAG, "✅ BLE Library created");

    // Step 3: Initialize BLE
    ret = ble->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize BLE: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, "✅ BLE Library initialized");

    // Step 4: Get the server and configure callbacks
    BLEServer* server = ble->getServer();
    if (server == nullptr) {
        ESP_LOGE(TAG, "❌ Failed to get BLE Server");
        delete ble;
        return;
    }
    g_server = server; // Set global reference
    ESP_LOGI(TAG, "✅ BLE Server obtained");

    // Step 5: Register callbacks
    ESP_LOGI(TAG, "\n--- Setting up Server Callbacks ---");
    server->setClientConnectedCallback(onClientConnected);
    server->setClientDisconnectedCallback(onClientDisconnected);
    server->setDataWrittenCallback(onDataWritten);
    server->setAdvertisingCallback(onAdvertising);
    ESP_LOGI(TAG, "✅ Callbacks configured");

    // Step 6: Start BLE services
    ESP_LOGI(TAG, "\n--- Starting BLE Server ---");
    ret = ble->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start BLE services: %s", esp_err_to_name(ret));
        delete ble;
        return;
    }
    ESP_LOGI(TAG, "✅ BLE Server started and advertising");

    // Step 7: Main demonstration loop
    ESP_LOGI(TAG, "\n🚀 Starting confirmation system demonstration...\n");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        loopCount++;

        if (server->hasConnectedClients()) {
            // Send different types of data based on loop count
            if (loopCount % 15 == 0) {
                // Send data that requires acknowledgment with auto-disconnect
                char ackData[128];
                snprintf(ackData, sizeof(ackData), 
                         "CRITICAL_DATA_%lu_REQUIRES_ACK", loopCount);
                
                ESP_LOGI(TAG, "\n🎯 Sending data with ACK requirement and auto-disconnect:");
                ESP_LOGI(TAG, "   Data: %s", ackData);
                
                // Get first connected client
                bleClientSession_t sessions[4];
                uint8_t sessionCount = server->getAllClientSessions(sessions, 4);
                if (sessionCount > 0) {
                    esp_err_t sendRet = server->sendDataWithConfirmation(
                        sessions[0].connID, ackData, true, true); // Require ack, auto-disconnect
                    
                    if (sendRet == ESP_OK) {
                        ESP_LOGI(TAG, "✅ Data sent successfully, waiting for acknowledgment...");
                        
                        // Wait for acknowledgment with timeout
                        esp_err_t waitRet = server->waitForClientAck(sessions[0].connID, 8000);
                        if (waitRet == ESP_OK) {
                            ESP_LOGI(TAG, "✅ Acknowledgment received! Client will auto-disconnect.");
                        } else {
                            ESP_LOGI(TAG, "⏰ Acknowledgment timeout - client may disconnect anyway");
                        }
                    } else {
                        ESP_LOGE(TAG, "❌ Failed to send data: %s", esp_err_to_name(sendRet));
                    }
                }
            }
            else if (loopCount % 8 == 0) {
                // Send data that requires acknowledgment but no auto-disconnect
                char normalAckData[128];
                snprintf(normalAckData, sizeof(normalAckData), 
                         "DATA_%lu_NEEDS_ACK_NO_DISCONNECT", loopCount);
                
                ESP_LOGI(TAG, "\n📤 Sending data with ACK requirement (no disconnect):");
                ESP_LOGI(TAG, "   Data: %s", normalAckData);
                
                bleClientSession_t sessions[4];
                uint8_t sessionCount = server->getAllClientSessions(sessions, 4);
                if (sessionCount > 0) {
                    esp_err_t sendRet = server->sendDataWithConfirmation(
                        sessions[0].connID, normalAckData, true, false); // Require ack, stay connected
                    
                    if (sendRet == ESP_OK) {
                        ESP_LOGI(TAG, "✅ Data sent successfully");
                    }
                }
            }
            else if (loopCount % 5 == 0) {
                // Send regular data without acknowledgment requirement
                char regularData[128];
                snprintf(regularData, sizeof(regularData), 
                         "Regular_Update_%lu", loopCount);
                
                ESP_LOGD(TAG, "📊 Sending regular update: %s", regularData);
                server->setCustomData(regularData);
            }
            
            // Show connection status every 10 seconds
            if (loopCount % 10 == 0) {
                bleServerStatus_t status = server->getStatus();
                ESP_LOGI(TAG, "\n📈 SERVER STATUS (t=%lu s):", loopCount);
                ESP_LOGI(TAG, "   Connected clients: %d", status.connectedClients);
                ESP_LOGI(TAG, "   State: %s", server->kgetStateString());
                ESP_LOGI(TAG, "   Battery: %d%%, Data: %s", status.batteryLevel, status.customData);
                
                // Show confirmation status for each client
                bleClientSession_t sessions[4];
                uint8_t sessionCount = server->getAllClientSessions(sessions, 4);
                for (int i = 0; i < sessionCount; i++) {
                    bleConfirmationStatus_t confStatus = server->getClientConfirmationStatus(sessions[i].connID);
                    const char* statusStr = "UNKNOWN";
                    switch (confStatus) {
                        case BLE_CONFIRM_NOT_REQUIRED: statusStr = "NOT_REQUIRED"; break;
                        case BLE_CONFIRM_PENDING: statusStr = "PENDING"; break;
                        case BLE_CONFIRM_ACKNOWLEDGED: statusStr = "ACKNOWLEDGED"; break;
                        case BLE_CONFIRM_TIMEOUT: statusStr = "TIMEOUT"; break;
                    }
                    ESP_LOGI(TAG, "   Client %d confirmation: %s", sessions[i].connID, statusStr);
                }
                ESP_LOGI(TAG, "=======================================\n");
            }
        } else {
            // No clients connected
            if (loopCount % 20 == 0) {
                ESP_LOGI(TAG, "\n⏳ Waiting for clients to connect... (t=%lu s)", loopCount);
                ESP_LOGI(TAG, "   Advertising: %s", server->isAdvertising() ? "ACTIVE" : "INACTIVE");
                ESP_LOGI(TAG, "   Device name: %s\n", SERVER_DEVICE_NAME);
            }
        }
    }

    // Cleanup (never reached)
    delete ble;
}