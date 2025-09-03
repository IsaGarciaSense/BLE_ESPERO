#include "wifi_apis.hpp"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"

static const char* TAG = "WiFiApiHandler";

wifiApiHandler::wifiApiHandler(WifiHandler& wifiHandler) : wifi_(wifiHandler) {}

esp_err_t wifiApiHandler::getIP() {
    std::string response;
    esp_err_t err = wifi_.httpGet("http://api.my-ip.io/v1/ip", response);

    if (err == ESP_OK) {
        ipAddress = response;
        ESP_LOGI(TAG, "IP Address: %s", ipAddress.c_str());
    } else {
        ESP_LOGE(TAG, "Failed to get IP");
    }
    return err;
}

esp_err_t wifiApiHandler::getCity() {
    if (ipAddress.empty()) {
        ESP_LOGE(TAG, "IP is not available, cannot fetch city");
        return ESP_FAIL;
    }

    std::string response;
    std::string apiUrl = "http://ip-api.com/json/" + ipAddress;
    esp_err_t err = wifi_.httpGet(apiUrl.c_str(), response);

    if (err == ESP_OK) {
        cJSON* root = cJSON_Parse(response.c_str());
        if (root == nullptr) {
            ESP_LOGE(TAG, "JSON Parsing failed");
            return ESP_FAIL;
        }

        cJSON* cityJson = cJSON_GetObjectItem(root, "city");
        cJSON* countryJson = cJSON_GetObjectItem(root, "countryCode");
        cJSON* latJson = cJSON_GetObjectItem(root, "lat");
        cJSON* lonJson = cJSON_GetObjectItem(root, "lon");

        if (cityJson && countryJson && latJson && lonJson) {
            city = cityJson->valuestring;
            country = countryJson->valuestring;
            latitude = (float)latJson->valuedouble;
            longitude = (float)lonJson->valuedouble;

            ESP_LOGI(TAG, "Location: %s, %s (Lat: %f, Lon: %f)", city.c_str(), country.c_str(), latitude, longitude);
        }

        cJSON_Delete(root);
    } else {
        ESP_LOGE(TAG, "Failed to get city");
    }
    return err;
}

esp_err_t wifiApiHandler::getGeolocation(std::string& weatherMain, std::string& weatherDesc) {
    if (city.empty() || country.empty()) {
        ESP_LOGE(TAG, "City & country are required before fetching weather");
        return ESP_FAIL;
    }

    std::string response;
    std::string weatherUrl = "http://api.openweathermap.org/data/2.5/weather"
    "?lat=" + std::to_string(latitude) +
    "&lon=" + std::to_string(longitude) +
    "&appid=" + std::string("&APPID=de226e12382406a1d5ceced47bdcd1d4");


    esp_err_t err = wifi_.httpGet(weatherUrl.c_str(), response);
    if (err == ESP_OK) {
        cJSON* root = cJSON_Parse(response.c_str());
        if (root == nullptr) {
            ESP_LOGE(TAG, "JSON Parsing failed");
            return ESP_FAIL;
        }

        cJSON* weatherArray = cJSON_GetObjectItem(root, "weather");
        if (cJSON_IsArray(weatherArray) && cJSON_GetArraySize(weatherArray) > 0) {
            cJSON* weatherObject = cJSON_GetArrayItem(weatherArray, 0);
            cJSON* mainJson = cJSON_GetObjectItem(weatherObject, "main");
            cJSON* descriptionJson = cJSON_GetObjectItem(weatherObject, "description");

            if (mainJson && descriptionJson) {
                weatherMain = mainJson->valuestring;
                weatherDesc = descriptionJson->valuestring;
            }
        }

        cJSON* timezoneJson = cJSON_GetObjectItem(root, "timezone");
        if (timezoneJson) {
            gmtOffset = timezoneJson->valueint;
        }

        ESP_LOGI(TAG, "Weather: %s (%s), GMT Offset: %d", weatherMain.c_str(), weatherDesc.c_str(), gmtOffset);

        cJSON_Delete(root);
    } else {
        ESP_LOGE(TAG, "Failed to get geolocation");
    }
    return err;
}

esp_err_t wifiApiHandler::testConnection() {
    std::string response;
    esp_err_t err = wifi_.httpGet("http://httpbin.org/ip", response);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Connection successful: %s", response.c_str());
    } else {
        ESP_LOGE(TAG, "Failed to test connection");
    }
    return err;
}
