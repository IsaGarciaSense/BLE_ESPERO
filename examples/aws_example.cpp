/*******************************************************************************
 * @file awsExample.cpp
 * @brief Example application to connect to Wi-Fi, synchronize time, and publish
 *        messages to AWS IoT Core using MQTT over TLS via the awsHandler object.
 *
 * @version 0.1.0
 * @date 2025-03-05
 * 
 ******************************************************************************/

#include "esp_log.h"
#include "esp_err.h"
#include "aws_sense.hpp"
#include <string>

#define THINGNAME "InnovaKit"

// Secrets for In-Ova AWS Server
const char AWS_IOT_ENDPOINT[] = "a2qie2s2us6mqo-ats.iot.us-west-2.amazonaws.com";

 
extern "C" void app_main(void) {
    static const char* TAG = "AWSExample";

    ESP_LOGI(TAG, "Initializing Wi-Fi...");
    const char* userSSID = "SENSE 2.4";
    const char* userPassword = "901798716SA";

    // Create and initialize the WifiHandler object.
    WifiHandler wifiHandler(userSSID, userPassword);
    if (wifiHandler.connectWifi() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi.");
        return;
    }
    ESP_LOGI(TAG, "Connected to Wi-Fi!");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Small delay after connecting

    // Synchronize time with SNTP server
    if (wifiHandler.syncTime() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to synchronize time");
        return;
    }
    ESP_LOGI(TAG, "Time synchronized");

    // Create and initialize the AWS handler using the WiFiHandler.
    awsHandler awsHelper(wifiHandler);
    if (awsHelper.init(AWS_IOT_ENDPOINT, THINGNAME, "AmazonRootCA1.pem", "inova_secrets_iot.pem", "private.pem") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AWS helper");
        return;
    }

    // Main loop: publish data (e.g., current time) to an AWS IoT topic.
    while (true) {
        char timeBuffer[32] = {0};
        if (wifiHandler.getCurrentTime(timeBuffer, sizeof(timeBuffer)) == ESP_OK) {
            ESP_LOGI(TAG, "Current Time: %s", timeBuffer);
        } else {
            ESP_LOGE(TAG, "Failed to get current time");
        }
        
        // Connect to AWS IoT Core via MQTT over TLS.
        if (awsHelper.connect() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to connect to AWS IoT Core");
            return;
        }

        // Publish the current time to an AWS IoT Core topic (change the topic as needed).
        if (awsHelper.publish("alpha/pub", timeBuffer, 1, 0) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to publish data to AWS");
        } else {
            ESP_LOGI(TAG, "Data published to AWS successfully");
        }

        // Disconnect from AWS IoT Core.
        awsHelper.disconnect();
        
        vTaskDelay(pdMS_TO_TICKS(60000));  // Wait 60 seconds before the next publish
    }
}
