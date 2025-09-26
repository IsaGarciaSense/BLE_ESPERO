/*******************************************************************************
 * @file esp_now_server.cpp
 * @brief File to test the ESP-NOW class.
 * 
 * @version 0.4.0
 * @date 2024-10-28
 * @author valeria@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "espnow_sense.hpp"

#define TAG "ESP_NOW_SERVER"


EspNow Hub(false); 

uint8_t incomingMac [6] = {0};
uint8_t macClient [6] = {0};
esp_err_t err;
bool waitingMessageLogged = false; 
bool continueReceiving = true;

struct EssityConsumptionReport {
    uint32_t productCount; 
    uint16_t userCount;    
    uint32_t deviceStatus; 
}; EssityConsumptionReport report;

struct ReportMessage {
    uint32_t packedData;   
}; ReportMessage msg;

size_t receivedLength = 0;
uint8_t MessageReport[32];

enum class State {
    kWaitingForMessage,
    kAddPeer,
    kProcessMessage,
    kCheckStatus,
    kReceiveUplinkData,
    kSendMillis
};
State currentState = State::kWaitingForMessage;

bool isFlagSet(uint32_t status, uint32_t flag) {
    return (status & flag) != 0;
}

void unpackUserLogData(uint32_t formattedData, uint8_t* hour, uint8_t* minutes, 
            uint8_t* productCount, uint16_t* userCount) {
   
    *hour = (formattedData >> 27) & 0x1F;   // 5 bits 
    *minutes = (formattedData >> 21) & 0x3F;   // 6 bits 
    *productCount = (formattedData >> 15) & 0x3F;   // 6 bits 
    *userCount = formattedData & 0x7FFF;   // 15 bits 
}

extern "C" void app_main() {

    
    if (!Hub.initialize()) {
        ESP_LOGE(TAG, "Error initializing the server.");
    }

   
    if (!Hub.receiveData()) {
        ESP_LOGE(TAG, "Error registering the reception callback.");
    } 

    while (true) {

        switch (currentState) {
            
            case State::kWaitingForMessage:
                
                if (!waitingMessageLogged) {
                    ESP_LOGI(TAG, "Server is active. Waiting for messages...");
                    waitingMessageLogged = true;
                }
                
               
                if (Hub.getClientMac(incomingMac) == ESP_OK) {    
                    
                    if (!esp_now_is_peer_exist(incomingMac)) {
                        Hub.saveMacToNvs(incomingMac);
                        currentState = State::kAddPeer;
                        waitingMessageLogged = false; 
                    } else {
                        currentState = State::kProcessMessage;
                    }
                }
                
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;

            case State::kAddPeer:
                ESP_LOGI(TAG, "Adding new PAR...");

                if (Hub.addPeer(incomingMac) == ESP_OK) {
                    Hub.savePeerToNvs(incomingMac);
                    currentState = State::kSendMillis;
                } else {
                    currentState = State::kWaitingForMessage;
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;

            case State::kSendMillis:
                
                ESP_LOGI(TAG, "Sending Timestamp to client...");
                Hub.sendMillis(incomingMac);
                currentState = State::kProcessMessage;
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;


            case State::kProcessMessage:

                if (Hub.getReceivedData(MessageReport, &receivedLength) 
                                        == ESP_OK) {

                    if (receivedLength == sizeof(EssityConsumptionReport)) {
                        memcpy(&report, MessageReport, 
                            sizeof(EssityConsumptionReport));

                        ESP_LOGI(TAG, "Product Count: %lu", report.productCount);
                        ESP_LOGI(TAG, "User Count: %u", report.userCount);
                        ESP_LOGI(TAG, "Device Status: %lX", report.deviceStatus);
                        
                        bool isReportTime = isFlagSet(report.deviceStatus, 
                                            kReportFlag);
                        
                        if (isReportTime) {
                            printf("THE REPORT FLAG IS SET!!!!!!\n");
                            currentState = State::kCheckStatus;
                        }

                    } else {
                        ESP_LOGE(TAG, "Received message has an unexpected size.");
                        currentState = State::kWaitingForMessage;
                    }
                }
               
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;

            case State::kCheckStatus:
                err = Hub.readMacFromNvs(macClient); 
                
                if (err == ESP_OK) {
            
                    if (Hub.checkUplinkMode(macClient) == ESP_OK){
                        currentState = State::kReceiveUplinkData;
                    }
                    
                } else {
                    
                    ESP_LOGE(TAG, "Error reading MAC from NVS: %s", 
                            esp_err_to_name(err));
                    currentState = State::kWaitingForMessage;
                }

                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;
            
            case State::kReceiveUplinkData: 

                while (continueReceiving) {
                    
                    if (Hub.getReceivedData(MessageReport, &receivedLength) 
                                            == ESP_OK) {

                        if (receivedLength == sizeof(ReportMessage)) { 

                           memcpy(&msg, MessageReport, sizeof(ReportMessage));
                            
                            uint8_t hour = 0;       
                            uint8_t minutes = 0;   
                            uint8_t productCount = 0;     
                            uint16_t userCount = 0;

                            if (msg.packedData != 0) {
                                unpackUserLogData(msg.packedData, &hour, &minutes, 
                                                &productCount, &userCount);
                             
                                ESP_LOGI(TAG, "Hour: %u", hour); 
                                ESP_LOGI(TAG, "Minutes: %u", minutes); 
                                ESP_LOGI(TAG, "Product: %u", productCount);
                                ESP_LOGI(TAG, "User: %u", userCount);
                            }
                        } 
                        
                        if (receivedLength == sizeof(EssityConsumptionReport)) {

                            memcpy(&report, MessageReport, 
                                    sizeof(EssityConsumptionReport));
                            ESP_LOGI(TAG, "Device Status: %lX", 
                                    report.deviceStatus);
                                    
                            bool isReportTime = isFlagSet(report.deviceStatus,
                                                 kReportFlag);
                                
                            if (!isReportTime) { 
                                printf("The Mode Uplink Closed!!!!!!\n");
                                continueReceiving = false; 
                                Hub.sendMillis(macClient);
                                currentState = State::kWaitingForMessage;
                                
                            }
                        } 

                    } 
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);  
                break;

            default:
                ESP_LOGE(TAG, "Unknown status.");
                break;
                vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}      
   