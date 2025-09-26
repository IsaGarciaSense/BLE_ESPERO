/*******************************************************************************
 * @file wifiApis_example.cpp
 * @brief File to test wifi connection and different basic APIs.
 * 
 * @version 0.1.0
 * @date 2025-3-01
 * @author daniel@sense-ai.co
 * 
 * build_flags = 
    -D CONFIG_MBEDTLS_CERTIFICATE_BUNDLE  # Enable cert bundle
    -D CONFIG_ESP_TLS_USING_MBEDTLS  # Ensure ESP-TLS is using mbedTLS
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
    //  const char* userSSID = "CLARO-055C";
    //  const char* userPassword = "Cl4r0@B6055C";
     const char* userPassword = "901798716SA";
 
 
     WifiHandler wifiHandler(userSSID, userPassword);
     if (wifiHandler.connectWifi() != ESP_OK) {
         ESP_LOGE(TAG, "Failed to connect to WiFi.");
         return;
     }
     
     ESP_LOGI(TAG, "Connected to WiFi!");
     vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 5 seconds before sending HTTP request
 
     wifiApiHandler apiHandler(wifiHandler);
     
     ESP_LOGI(TAG, "Fetching IP...");
     if (apiHandler.getIP() != ESP_OK) {
         ESP_LOGE(TAG, "Failed to retrieve IP.");
     } else {
         ESP_LOGI(TAG, "IP Address: %s", apiHandler.getStoredIP().c_str());
     }
 
     ESP_LOGI(TAG, "Fetching city...");
     if (apiHandler.getCity() != ESP_OK) {
         ESP_LOGE(TAG, "Failed to retrieve city.");
     } else {
         ESP_LOGI(TAG, "Location: %s, %s", apiHandler.getStoredCity().c_str(), apiHandler.getStoredCountry().c_str());
     }
 
     ESP_LOGI(TAG, "Fetching weather...");
     std::string weatherMain, weatherDesc;
     if (apiHandler.getGeolocation(weatherMain, weatherDesc) != ESP_OK) {
         ESP_LOGE(TAG, "Failed to retrieve weather.");
     } else {
         ESP_LOGI(TAG, "Weather: %s (%s)", weatherMain.c_str(), weatherDesc.c_str());
     }
 
     ESP_LOGI(TAG, "All API requests completed. Exiting.");
 }
  