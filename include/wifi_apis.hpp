/*******************************************************************************
 * @file wifi_apis.cpp
 * @brief Wi-Fi handler class to use multiple APIs in a single class.
 * 
 * @version 0.1.0
 * @date 2025-2-26
 * @author daniel@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_http_client.h"
#include "wifi_sense.hpp"
#include <string>


class wifiApiHandler {
public:
    explicit wifiApiHandler(WifiHandler& wifiHandler);

    esp_err_t getIP();
    esp_err_t getCity();
    esp_err_t getGeolocation(std::string& weatherMain, std::string& weatherDesc);
    esp_err_t testConnection();

    std::string getStoredIP() const { return ipAddress; }
    std::string getStoredCity() const { return city; }
    std::string getStoredCountry() const { return country; }

private:
    WifiHandler& wifi_;
    std::string ipAddress;
    std::string city;
    std::string country;
    float latitude = 0.0f;
    float longitude = 0.0f;
    int gmtOffset = 0;

};
