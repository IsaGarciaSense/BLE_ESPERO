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
 
// Use the sendData method to send data to the desired MQTT topic
//const char* userSSID = "SENSE 2.4";
//const char* userPassword = "901798716SA";

const char* userSSID = "SenseAI";
const char* userPassword = "felicidad";

const char* brokerHost = "192.168.226.161";
uint32_t brokerPort = 1883;
const char* mqttTopic = "SenseAI_dev";


extern "C" void app_main() {
   static const char* TAG = "WiFiExample";

   ESP_LOGI(TAG, "Initializing WiFi...");

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
       //return;
   }

   esp_err_t err = wifiHandler.mqttInit(brokerHost, brokerPort);
   if (err != ESP_OK) {
       ESP_LOGE(TAG, "Failed to initialize MQTT");
   } else {
       ESP_LOGI(TAG, "MQTT initialized successfully");
   }

   uint16_t waitCounter = 0;
   while (wifiHandler.isMQTTClientConnected() == false){
       vTaskDelay(100);
       waitCounter++;
       if (waitCounter > 20){
           ESP_LOGE(TAG, "MQTT connection failed");
           return;
       }
   }

   // Subscribe to the MQTT topic
   err = wifiHandler.mqttSubscribe(mqttTopic, 1);
   if (err != ESP_OK) {
       ESP_LOGE(TAG, "Failed to subscribe to MQTT topic");
   } else {
       ESP_LOGI(TAG, "Subscribed to MQTT topic");
   }

   while (true){
       if (wifiHandler.mqttResponseFlag()) {
           ESP_LOGI(TAG, "Received MQTT message: %s", wifiHandler.getResponse().c_str());
           wifiHandler.resetResponseFlag();
       }
       
       vTaskDelay(pdMS_TO_TICKS(100));  // Wait 100ms before checking again
   }
}

