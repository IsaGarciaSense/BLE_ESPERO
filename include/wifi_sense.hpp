/*******************************************************************************
 * @file wifi_sense.hpp
 * @brief Wi-Fi handler class to manage Wi-Fi connection, time synchronization,
 * HTTP client functions (GET/POST), and MQTT communication.
 *
 * @version 0.2.0
 * @date 2025-2-26
 * @author daniel@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "mqtt_client.h"
#include "nvs_flash.h"

class WifiHandler {
public:
    WifiHandler(const char* ssid, const char* password);
    ~WifiHandler();

    // Wi-Fi Connection Methods
    esp_err_t connectWifi();
    esp_err_t reconnect();
    esp_err_t disconnect();

    esp_err_t wifiThread(void* pvParameters);
    bool isWifiConnected();

    // Time Synchronization
    esp_err_t syncTime();
    esp_err_t getCurrentTime(char* buffer, size_t bufferSize);

    bool isHTTPClientConnected();
    bool isMQTTClientConnected();

    // HTTP Requests
    esp_err_t config(const char* param, const char* value);
    esp_err_t httpGet(const char* url, std::string& response);
    esp_err_t httpPost(const char* url, const char* data, char* responseBuffer,
                       size_t bufferSize);

    // MQTT Methods
    esp_err_t mqttInit(const char* brokerHost, uint32_t brokerPort);
    esp_err_t mqttPublish(const char* topic, const char* data, int qos = 0,
                          int retain = 0);
    esp_err_t mqttSubscribe(const char* topic, int qos = 0);
    esp_err_t mqttUnsubscribe(const char* topic);
    esp_err_t sendData(const char* brokerHost, uint32_t brokerPort,
                       const char* topic, const char* data);
    std::string getResponse() {
        return mqttResponseBody_;
    }
    bool mqttResponseFlag() {
        return mqttResponse_;
    }
    void resetResponseFlag() {
        mqttResponse_ = false;
    }
    void mqttStop();

private:
    // Wi-Fi Configs
    const char* ssid_;
    const char* password_;

    uint16_t rebootT_ = 20000;
    uint16_t retryTime_ = 500;
    uint16_t loopTime_ = 500;
    uint16_t rebootTimer_ = 0;

    // NTP Configs
    int16_t gmtOffset_sec_ = -3600 * 5;
    const char* ntpServer_ = "pool.ntp.org";
    const char* ntpServer2_ = "0.pool.ntp.org";
    int16_t daylightOffset_sec_ = 0;

    static bool wifiConnected_;
    static bool httpClientConnected_;
    static bool mqttClientConnected_;
    static std::string httpResponseBody_;
    static std::string mqttResponseBody_;
    static bool mqttResponse_;

    // HTTP Utilities
    static esp_err_t _http_event_handler(esp_http_client_event_t* evt);
    static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                   int32_t event_id, void* event_data);

    // MQTT Utilities
    static void mqtt_event_handler(void* handler_args, esp_event_base_t base,
                                   int32_t event_id, void* event_data);
    esp_mqtt_client_handle_t client_ = nullptr;
};
