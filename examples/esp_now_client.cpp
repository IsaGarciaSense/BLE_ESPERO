/*******************************************************************************
 * @file esp_now_client.cpp
 * @brief File to test the ESP-NOW class.
 * 
 * @version 0.4.0
 * @date 2024-10-28
 * @author valeria@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "espnow_sense.hpp"

#define TAG "ESP_NOW_CLIENT"

uint8_t server_mac[6] = {0xCC, 0x8D, 0xA2, 0x0C, 0x6D, 0xC9};
uint8_t message[251];

EspNow Node(true, server_mac);

struct EssityConsumptionReport {
    uint32_t productCount; 
    uint16_t userCount;    
    uint32_t deviceStatus; 
}; EssityConsumptionReport report;

struct ReportMessage {
    uint32_t formattedData;   
}; ReportMessage msg;

size_t receivedLength = 0;
uint8_t MessageReport[32];


bool isFlagSet(uint32_t status, uint32_t flag) {
    return (status & flag) != 0;
}

uint32_t formatUserLogData(uint8_t hour, uint8_t minutes,
                           uint8_t productUsed, uint16_t userID) {

    if (productUsed <= 0) {
        return 0xFFFF;  // Error: --> No product detected
    }

    // Avoid overflow
    if (hour > 23) hour = 23;
    if (minutes > 59) minutes = 59;
    if (productUsed > 63) productUsed = 63;
    if (userID > 32767) userID = 32767;

    uint32_t formattedData = 0;

    formattedData |= (hour & 0x1F) << 27;         // 31-27 (5 bits)
    formattedData |= (minutes & 0x3F) << 21;      // 26-21 (6 bits)
    formattedData |= (productUsed & 0x3F) << 15;  // 20-15 (6 bits)
    formattedData |= (userID & 0x7FFF);           //  14-0 (15 bits)

    return formattedData;
}


extern "C" void app_main() {
  
    if (!Node.initialize()) {
        ESP_LOGE(TAG, "Error initializing client.");
        
    }

    if (!Node.connect()) {
        ESP_LOGE(TAG, "Error connecting to server.");
        
    }

    report.productCount = 0xAABBCCDD;
    report.userCount = 0xF333;
    report.deviceStatus = kReportFlag;

    bool isReportTime = isFlagSet(report.deviceStatus, kReportFlag);

    Node.setMessage(report);

    printf("Pruduct: %lu\nUsers: %u\nStatus: 0x%lX\n",
            report.productCount, report.userCount, report.deviceStatus);

    if (!Node.sendData()) {
        ESP_LOGE(TAG, "Error sending message.");
    }

    if (!Node.receiveData()) {
        ESP_LOGE(TAG, "Error registering reception callback.");
        
    }

    while (true) {
        
        if (isReportTime) {
        
            printf("THE REPORT FLAG IS SET!!!!!!\n");

            if (Node.getReceivedData(MessageReport, &receivedLength) == ESP_OK) {
                
                if (MessageReport[0] == MESSAGE_TYPE_CONFIRMATION) { 

                    printf("Message type confirmation!!!!!!\n");

                    uint8_t hour = 13;       
                    uint8_t minutes = 45;   
                    uint8_t productCount = 20;     
                    uint16_t userCount = 123;       

                    msg.formattedData = formatUserLogData(hour, minutes, 
                                        productCount,userCount);

                    for (int i = 0; i < 5; ++i) {

                        Node.setMessage(msg);
                        ESP_LOGI(TAG, "Sending message: Hour %u, Minutes %u, Products %u,  User %u",
                                hour, minutes, productCount,userCount);
                        
                        if (!Node.sendData()) {
                                ESP_LOGE(TAG, "Error sending message.");
                        }

                    vTaskDelay(1000 / portTICK_PERIOD_MS); 
                    }

                    report.deviceStatus &= ~kReportFlag; 
                    memcpy(&report, MessageReport, 
                        sizeof(EssityConsumptionReport));
                        
                    Node.setMessage(report);
                    bool isReportTime = isFlagSet(report.deviceStatus, 
                                        kReportFlag);
                    
                    if (!Node.sendData()) {
                        ESP_LOGE(TAG, "Error sending message.");
                    }

                    if (!isReportTime) {
                        ESP_LOGE(TAG, "Uplink mode disabled");
                        break;
                    }
                }
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }     
}
