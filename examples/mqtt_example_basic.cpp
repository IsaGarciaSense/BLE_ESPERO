/*******************************************************************************
 * @file wifiApis_example.cpp
 * @brief File to test wifi connection and basic fetch time and send data to MQTT.
 * 
 * @version 0.1.0
 * @date 2025-3-05
 * author daniel@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

 #include "esp_log.h"
 #include "esp_err.h"
 #include "wifi_sense.hpp"
 #include "wifi_apis.hpp"
 
 extern "C" void app_main() {
    static const char* TAG = "WiFiExample";

    ESP_LOGI(TAG, "Initializing WiFi...");
    const char* userSSID = "SENSE 2.4";
    const char* userPassword = "901798716SA";

    WifiHandler wifiHandler(userSSID, userPassword);
    if (wifiHandler.connectWifi() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi.");
        return;
    }

    ESP_LOGI(TAG, "Connected to WiFi!");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 5 seconds before sending requests

    // Synchronize time
    if (wifiHandler.syncTime() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to synchronize time");
        return;
    }

    // Use the sendData method to send data to the desired MQTT topic
    const char* brokerHost = "192.168.1.3";
    uint32_t brokerPort = 1883;
    const char* mqttTopic = "SenseAI_dev";

    while (true){
        // Get current time
        char timeBuffer[32];
        if (wifiHandler.getCurrentTime(timeBuffer, sizeof(timeBuffer)) == ESP_OK) {
            ESP_LOGI(TAG, "Current Time: %s", timeBuffer);
        } else {
            ESP_LOGE(TAG, "Failed to get current time");
        }
        
        const char* mqttData = timeBuffer;

        ESP_LOGI(TAG, "Sending data to MQTT...");
        if (wifiHandler.sendData(brokerHost, brokerPort, mqttTopic, mqttData) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send data to MQTT.");
        } else {
            ESP_LOGI(TAG, "Data sent to MQTT successfully.");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 5 seconds before sending requests
    }
 }