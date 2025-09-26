/*******************************************************************************
 * @file wifi_sense.cpp
 * @brief Wi-Fi handler class to manage Wi-Fi connection and time synchronization. 
 * Includes basic HTTP client functions for GET and POST requests.
 * 
 * @version 0.1.0
 * @date 2025-2-26
 * @author daniel@sense-ai.co
 *******************************************************************************
 *******************************************************************************/


#include "wifi_sense.hpp"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_http_client.h"
#include "esp_system.h" // Add this line to include the header for esp_read_mac
#include "esp_wifi_types.h" // Add this line to include the header for ESP_MAC_WIFI_STA
#include "esp_efuse.h"
#include "esp_mac.h"
#include "esp_crt_bundle.h"

extern "C" esp_err_t esp_crt_bundle_attach(void *conf);

static const char* TAG = "WifiHandler";

bool WifiHandler::wifiConnected_ = false;  // Define static variable

WifiHandler::WifiHandler(const char* ssid, const char* password) 
    : ssid_(ssid), password_(password) {

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP Stack (check if already initialized)
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "TCP/IP stack already initialized, continuing...");
    }

    // Create default event loop (check if it already exists)
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Event loop already exists, continuing...");
    }

    // Initialize WiFi (create network interface if not exists)
    esp_netif_t* sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif == NULL) {
        ESP_LOGI(TAG, "Creating WiFi STA network interface...");
        sta_netif = esp_netif_create_default_wifi_sta();
        if (sta_netif == NULL) {
            ESP_LOGE(TAG, "Failed to create WiFi STA network interface");
            return;
        }
    } else {
        ESP_LOGI(TAG, "WiFi STA network interface already exists, reusing...");
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "WiFi already initialized, continuing...");
    }

    ESP_LOGI(TAG, "WiFiHandler initialized");
}


WifiHandler::~WifiHandler() {
    esp_wifi_stop();
    esp_wifi_deinit();
    ESP_LOGI(TAG, "WiFiHandler destroyed");
}


esp_err_t WifiHandler::config(const char* param, const char* value) {
    if (strcmp(param, "ntp_server") == 0) {
        ntpServer_ = value;
        ESP_LOGI(TAG, "NTP server updated to: %s", ntpServer_);
        return ESP_OK;
    } else if (strcmp(param, "gmt_offset") == 0) {
        gmtOffset_sec_ = atoi(value);
        ESP_LOGI(TAG, "GMT Offset updated to: %d", gmtOffset_sec_);
        return ESP_OK;
    } else if (strcmp(param, "daylight_offset") == 0) {
        daylightOffset_sec_ = atoi(value);
        ESP_LOGI(TAG, "Daylight Offset updated to: %d", daylightOffset_sec_);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Invalid config param: %s", param);
    return ESP_ERR_INVALID_ARG;
}


void WifiHandler::wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOGI("WiFiHandler", " Event received: base=%s, id=%ld", event_base, event_id);
    
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t* disconn_event = (wifi_event_sta_disconnected_t*) event_data;
                wifiConnected_ = false;
                
                //  FIXED: Check disconnect reason to avoid infinite reconnect with bad credentials
                ESP_LOGW("WiFiHandler", "Wi-Fi disconnected, reason: %d", disconn_event->reason);
                
                switch (disconn_event->reason) {
                    case WIFI_REASON_AUTH_EXPIRE:
                    case WIFI_REASON_AUTH_FAIL:
                    case WIFI_REASON_ASSOC_FAIL:
                    case WIFI_REASON_HANDSHAKE_TIMEOUT:
                    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
                        ESP_LOGE("WiFiHandler", " Authentication failed - credentials likely incorrect");
                        ESP_LOGE("WiFiHandler", "   Reason: %d (auth/handshake failure)", disconn_event->reason);
                        // Don't auto-reconnect on authentication failures
                        break;
                        
                    case WIFI_REASON_NO_AP_FOUND:
                        ESP_LOGE("WiFiHandler", " SSID not found - network may be out of range");
                        // Don't auto-reconnect if network not found
                        break;
                        
                    default:
                        ESP_LOGW("WiFiHandler", " Connection lost, attempting reconnect...");
                        esp_wifi_connect(); // Auto-reconnect only for transient issues
                        break;
                }
                break;
            }
            case WIFI_EVENT_STA_START:
                ESP_LOGI("WiFiHandler", "Wi-Fi started. Connecting...");
                esp_wifi_connect();
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WiFiHandler", " IP_EVENT_STA_GOT_IP received! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI("WiFiHandler", " Setting wifiConnected_ = true");
        wifiConnected_ = true;
    } else {
        ESP_LOGW("WiFiHandler", " Unhandled event: base=%s, id=%ld", event_base, event_id);
    }
}


esp_err_t WifiHandler::connectWifi() {
    ESP_LOGI(TAG, " Starting WiFi connection process...");
    
    // Reset connection state
    wifiConnected_ = false;
    
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid_, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password_, sizeof(wifi_config.sta.password));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;

    ESP_LOGI(TAG, " Registering WiFi event handlers...");
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    ESP_LOGI(TAG, "  Configuring WiFi mode and credentials...");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi...");

    int wifiCounter = 0;
    bool authenticationStarted = false;
    while (wifiCounter < 40) {  //  INCREASED: 20 seconds for IP acquisition
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // PRIMARY: Check if we got IP via event handler
        if (wifiConnected_) {
            esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (netif != nullptr) {
                esp_netif_ip_info_t ip_info;
                if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
                    ESP_LOGI(TAG, " Connected to Wi-Fi: %s with IP: " IPSTR, ssid_, IP2STR(&ip_info.ip));
                    return ESP_OK;
                }
            }
        }
        
        // SECONDARY: Manual IP check in case event handler missed
        if (authenticationStarted && wifiCounter > 10) {  // After 5 seconds of auth
            esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (netif != nullptr) {
                esp_netif_ip_info_t ip_info;
                if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
                    ESP_LOGW(TAG, " Manual IP check: Found IP without event - " IPSTR, IP2STR(&ip_info.ip));
                    ESP_LOGW(TAG, " Connected to Wi-Fi: %s (event handler issue bypassed)", ssid_);
                    wifiConnected_ = true;  // Set flag for consistency
                    return ESP_OK;
                }
            }
        }
        
        // Check authentication progress
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            if (!authenticationStarted) {
                ESP_LOGI(TAG, " WiFi authentication successful, waiting for IP...");
                authenticationStarted = true;
            }
        } else {
            ESP_LOGI(TAG, "."); // Trying to connect
        }
        wifiCounter++;
    }

    ESP_LOGW(TAG, " Could not connect to Wi-Fi: '%s'", ssid_);
    ESP_LOGI(TAG, " Scanning for available networks to help diagnose...");
    
    // Quick scan to show available networks for debugging
    wifi_scan_config_t scan_config = {};
    scan_config.ssid = NULL;
    scan_config.bssid = NULL;
    scan_config.channel = 0;
    scan_config.show_hidden = false;
    scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_config.scan_time.active.min = 120;
    scan_config.scan_time.active.max = 150;
    
    if (esp_wifi_scan_start(&scan_config, true) == ESP_OK) {
        uint16_t number = 10;  // Limit to 10 networks
        wifi_ap_record_t ap_info[10];
        if (esp_wifi_scan_get_ap_records(&number, ap_info) == ESP_OK) {
            ESP_LOGI(TAG, " Found %d networks:", number);
            for (int i = 0; i < number; i++) {
                ESP_LOGI(TAG, "   %d: '%s' (RSSI: %d, Ch: %d)", 
                    i+1, ap_info[i].ssid, ap_info[i].rssi, ap_info[i].primary);
                
                // Check if our target SSID is in the list
                if (strcmp((char*)ap_info[i].ssid, ssid_) == 0) {
                    ESP_LOGW(TAG, "     Target SSID '%s' found but connection failed!", ssid_);
                    ESP_LOGW(TAG, "      This suggests password or security configuration issue");
                }
            }
        }
    } else {
        ESP_LOGW(TAG, "   Scan failed - can't show available networks");
    }
    
    wifiConnected_ = false;
    return ESP_FAIL;
}


esp_err_t WifiHandler::reconnect() {
  ESP_LOGI(TAG, "Reconnecting to Wi-Fi...");

  TickType_t startTick = xTaskGetTickCount();
  TickType_t rebootTimeout = pdMS_TO_TICKS(rebootT_);
  TickType_t delayTicks = pdMS_TO_TICKS(retryTime_);

  while (true) {
      wifi_ap_record_t ap_info;
      if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
          ESP_LOGI(TAG, "Reconnected to Wi-Fi: %s", ssid_);
          return ESP_OK;
      }

      ESP_LOGW(TAG, "Reconnection attempt failed. Retrying in %d ms...", retryTime_);
      vTaskDelay(delayTicks);

      // Check for reboot timeout
      if ((xTaskGetTickCount() - startTick) > rebootTimeout) {
          ESP_LOGE(TAG, "Rebooting system after failed reconnection attempts...");
          return ESP_FAIL;
      }

      // Attempt reconnection
      esp_err_t err = esp_wifi_connect();
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "esp_wifi_connect() failed: %s", esp_err_to_name(err));
      }
  }
}

esp_err_t WifiHandler::disconnect() {
    ESP_LOGI(TAG, "Disconnecting from Wi-Fi...");

    // Disconnect from the current AP
    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_disconnect() failed: %s", esp_err_to_name(err));
        return err;
    }

    // Stop the Wi-Fi driver to free resources
    err = esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_stop() failed: %s", esp_err_to_name(err));
        return err;
    }

    wifiConnected_ = false;
    ESP_LOGI(TAG, "Wi-Fi disconnected successfully");
    return ESP_OK;
}

bool WifiHandler::isWifiConnected() { 
    return wifiConnected_; 
};

// ---------------- SNTP  ----------------------//
esp_err_t WifiHandler::syncTime() {
    // Set default values for NTP server and time zone
    if (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET) {
        ESP_LOGI(TAG, "Initializing SNTP");
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, ntpServer_);
        esp_sntp_setservername(1, ntpServer2_);
        esp_sntp_init();
    }

    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo;
    int retry = 0;
    const int retry_count = 10;

    while (retry < retry_count) {
        time(&now);
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year > (2016 - 1900)) {
            ESP_LOGI(TAG, "Time synchronized successfully");
            return ESP_OK;
        }
        ESP_LOGI(TAG, "Waiting for time sync...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        retry++;
    }

    ESP_LOGE(TAG, "Failed to synchronize time");
    return ESP_FAIL;
}


esp_err_t WifiHandler::getCurrentTime(char* buffer, size_t bufferSize) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGE(TAG, "Time not synchronized");
        return ESP_FAIL;
    }

    strftime(buffer, bufferSize, "%d/%m/%Y-%H:%M:%S", &timeinfo);
    //ESP_LOGI(TAG, "Current Time: %s", buffer);
    return ESP_OK;
}


esp_err_t WifiHandler::wifiThread(void *pvParameters) {
  ESP_LOGI(TAG, "Starting Wi-Fi thread");

  // Initial Wi-Fi connection
  if (connectWifi() != ESP_OK) {
      ESP_LOGE(TAG, "Initial Wi-Fi connection failed");
  }

  vTaskDelay(pdMS_TO_TICKS(100));  // Small delay before entering the loop

  while (true) {
      wifi_ap_record_t ap_info;
      // Check if connected to Wi-Fi
      if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
          ESP_LOGI(TAG, "Wi-Fi is connected to SSID: %s", ssid_);
      } else {
          ESP_LOGW(TAG, "Wi-Fi disconnected, attempting to reconnect...");
          esp_err_t err = esp_wifi_connect();
          if (err != ESP_OK) {
              ESP_LOGE(TAG, "Wi-Fi reconnection failed: %s", esp_err_to_name(err));
          }
      }

      // Placeholder for future MQTT client handling:
      // e.g., if (mqttClient && !mqttClient->connected()) { reconnect(); }

      vTaskDelay(pdMS_TO_TICKS(loopTime_));  // Check connection every 1s
  }

  return ESP_OK;
}


//////////////////////// HTTP  ///////////////////////////////
std::string WifiHandler::httpResponseBody_ = "";  // Define static variable
bool WifiHandler::httpClientConnected_ = false;  // Define static variable

esp_http_client_config_t defaultHTTPConfigs() {
    esp_http_client_config_t defaultConfig = {
        .url = NULL,
        .host = NULL,
        .port = 0,
        .username = NULL,
        .password = NULL,
        .auth_type = HTTP_AUTH_TYPE_NONE,
        .path = NULL,
        .query = NULL,
        .cert_pem = NULL,
        .cert_len = 0,
        .client_cert_pem = NULL,
        .client_cert_len = 0,
        .client_key_pem = NULL,
        .client_key_len = 0,
        .client_key_password = NULL,
        .client_key_password_len = 0,
        .tls_version = ESP_HTTP_CLIENT_TLS_VER_ANY,
        .user_agent = NULL,
        .method = HTTP_METHOD_GET,  // Default method
        .timeout_ms = 5000,  // Default timeout (adjust as needed)
        .disable_auto_redirect = false,
        .max_redirection_count = 5,
        .max_authorization_retries = 3,
        .event_handler = NULL,
        .transport_type = HTTP_TRANSPORT_OVER_SSL, // Default transport type
        .buffer_size = 512,  // Default buffer size
        .buffer_size_tx = 512,
        .user_data = NULL,
        .is_async = false,
        .use_global_ca_store = false,
        .skip_cert_common_name_check = true,
        .common_name = NULL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = false,
        .keep_alive_idle = 0,
        .keep_alive_interval = 0,
        .keep_alive_count = 0,
        .if_name = NULL,
        .ds_data = NULL
    };
    return defaultConfig;
}


esp_err_t WifiHandler::_http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            httpClientConnected_ = true;
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            if (evt->data_len > 0) {
                httpResponseBody_.append((char*)evt->data, evt->data_len);
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH, Response:\n%s", httpResponseBody_.c_str());

            // Ensure response only contains useful data
            if (httpResponseBody_.find("<html>") != std::string::npos) {
                ESP_LOGW(TAG, "Detected HTML response, clearing buffer!");
                httpResponseBody_.clear();
            }
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            httpClientConnected_ = false;
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGI(TAG, "HTTP_EVENT_REDIRECT");
            httpResponseBody_.clear();  // Clear previous response
            break;
    }
    return ESP_OK;
}


esp_err_t WifiHandler::httpGet(const char* url, std::string& response) {
    httpResponseBody_.clear();  // Clear previous response

    esp_http_client_config_t config = defaultHTTPConfigs();
    config.url = url;
    config.method = HTTP_METHOD_GET;
    config.event_handler = _http_event_handler;  // Set event handler

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "HTTP Status Code: %d", status_code);
        response = httpResponseBody_;  // Assign response
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}


esp_err_t WifiHandler::httpPost(const char* url, const char* data, char* responseBuffer, size_t bufferSize) {
    if (!responseBuffer || bufferSize == 0) return ESP_ERR_INVALID_ARG; // Safety check

    esp_http_client_config_t config = defaultHTTPConfigs(); // Get default values
    config.url = url;
    config.method = HTTP_METHOD_POST;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return ESP_FAIL;

    // Set POST data
    esp_http_client_set_post_field(client, data, strlen(data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int content_length = esp_http_client_get_content_length(client);
        if (content_length < 0 || content_length >= bufferSize) content_length = bufferSize - 1; // Safety

        int read_len = esp_http_client_read(client, responseBuffer, content_length);
        if (read_len > 0) {
            responseBuffer[read_len] = '\0'; // Null-terminate response
        } else {
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

// ========================= MQTT Event Handler =========================
std::string WifiHandler::mqttResponseBody_ = "";  // Define static variable
bool WifiHandler::mqttResponse_ = false;  // Define static variable
bool WifiHandler::mqttClientConnected_ = false;  // Define static variable


static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG, "%s: 0x%x", message, error_code);
    }
}

esp_mqtt_client_config_t defaultMQTTConfigs() {
    esp_mqtt_client_config_t config = {
        .broker = {
            .address = {
                .uri = NULL,               // Complete MQTT broker URI (set later)
                .hostname = NULL,          // Broker hostname (if URI not used)
                .transport = MQTT_TRANSPORT_UNKNOWN,  // Default transport (unencrypted TCP)
                .path = NULL,              // URI path (if any)
                .port = 1883,                 // Broker port (set later)
            },
            .verification = {
                .use_global_ca_store = false,  // Don't use global CA store by default
                .crt_bundle_attach = NULL,       // No certificate bundle attached by default
                .certificate = NULL,             // No broker certificate provided
                .certificate_len = 0,            // Length is zero
                .psk_hint_key = NULL,            // No PSK provided
                .skip_cert_common_name_check = false, // Do not skip common name check by default
                .alpn_protos = NULL,             // No ALPN protocols defined
                .common_name = NULL,             // No override for common name
            },
        },
        .credentials = {
            .username = "senseDev",           // No username by default
            .client_id = "senseDev_369",          // Use default client ID (ESP32_<chipid>) if not set
            .set_null_client_id = false, // Do not force a NULL client ID by default
            .authentication = {
                .password = "901798716",       // No password by default
                .certificate = NULL,    // No client certificate
                .certificate_len = 0,   // Zero length
                .key = NULL,            // No client private key
                .key_len = 0,           // Zero length
                .key_password = NULL,   // No key decryption password
                .key_password_len = 0,  // Zero length
                .use_secure_element = false, // Do not use secure element by default
                .ds_data = NULL,        // No digital signature data
            },
        },
        .session = {
            .last_will = {
                .topic = NULL,          // No LWT topic
                .msg = NULL,            // No LWT message
                .msg_len = 0,           // Zero length message
                .qos = 0,               // LWT QoS default 0
                .retain = 0,            // LWT retain flag off
            },
            .disable_clean_session = false, // Use clean session (i.e. not disabled)
            .keepalive = 120,           // Keepalive interval of 120 seconds
            .disable_keepalive = false, // Keepalive enabled by default
            .protocol_ver = MQTT_PROTOCOL_V_3_1_1, // Use MQTT version 3.1.1 by default
            .message_retransmit_timeout = 120,    // Retransmit timeout (example: 120 seconds)
        },
        .network = {
            .reconnect_timeout_ms = 10000,  // Reconnect timeout: 10 seconds
            .timeout_ms = 10000,            // Network operation timeout: 10 seconds
            .refresh_connection_after_ms = 0, // No forced connection refresh by default
            .disable_auto_reconnect = false,  // Auto-reconnect enabled by default
            .transport = NULL,              // No custom transport provided
            .if_name = NULL,                // Use default network interface
        },
        .task = {
            .priority = 5,    // Default MQTT task priority
            .stack_size = 4096, // Default stack size (can be adjusted)
        },
        .buffer = {
            .size = 1024,     // Default MQTT send/receive buffer size
            .out_size = 1024, // Default output buffer size (if not specified, same as size)
        },
        .outbox = {
            .limit = 0,       // No outbox limit by default
        },
    };
    return config;
}

// Function to get the device's Wi-Fi MAC address as a string
std::string getDeviceMac() {
    uint8_t mac[6];
    // Get the factory-programmed MAC address
    esp_err_t ret = esp_efuse_mac_get_default(mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get default MAC address: %s", esp_err_to_name(ret));
        return std::string();
    }
        
    char macStr[18]; // "XX:XX:XX:XX:XX:XX" + null terminator = 18 bytes
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(macStr);
}

// ========================= MQTT Initialization =========================
#include <strings.h>  // for strncasecmp

esp_err_t WifiHandler::mqttInit(const char* brokerHost, uint32_t brokerPort) {
    if (client_) {
        ESP_LOGW(TAG, "MQTT Client already initialized.");
        return ESP_FAIL;
    }
    
    // Generate a client ID using the device MAC
    std::string clientId = "senseDev" + getDeviceMac();
    
    // Get the default MQTT configuration structure
    esp_mqtt_client_config_t mqtt_cfg = defaultMQTTConfigs();
    
    // Do not use a complete URI; instead, set the broker details manually:
    mqtt_cfg.broker.address.uri = NULL;           // Do not set a URI
    mqtt_cfg.broker.address.hostname = brokerHost;  // Set the broker hostname (e.g., "192.168.1.3")
    mqtt_cfg.broker.address.port = brokerPort;      // Set the broker port (e.g., 1883)
    mqtt_cfg.broker.address.path = "";              // No specific path is needed in most cases
    mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;  // Force unencrypted TCP transport
    
    // Set the client ID in the credentials
    mqtt_cfg.credentials.client_id = clientId.c_str();
    
    // Initialize the MQTT client with the configured settings
    client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (!client_) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    
    // Register the MQTT event handler
    esp_mqtt_client_register_event(client_, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID), mqtt_event_handler, this);
    
    // Start the MQTT client
    esp_err_t start_err = esp_mqtt_client_start(client_);
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(start_err));
        return start_err;
    }
    
    ESP_LOGI(TAG, "MQTT Client started successfully with client ID: %s", clientId.c_str());
    return ESP_OK;
}


void WifiHandler::mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    ESP_LOGD(TAG, "Event dispatched: base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    //esp_mqtt_client_handle_t client = event->client;
    
    //int msg_id = 0;
    //std::string topicToUse = "senseDev/" + getDeviceMac();
    //std::string dataToSend = "deviceInit";

    switch ((esp_mqtt_event_id_t) event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            //msg_id = esp_mqtt_client_publish(client, topicToUse.c_str(), dataToSend.c_str(), 0, 1, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            mqttClientConnected_ = true;

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT Disconnected");
            mqttClientConnected_ = false;
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "Subscribed to topic, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "Unsubscribed, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "Message published, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT Data Received");
            mqttResponse_ = true;
            mqttResponseBody_ = std::string(event->data, event->data_len);
            //ESP_LOGI(TAG, "Topic: %.*s", event->topic_len, event->topic);
            //ESP_LOGI(TAG, "Data: %.*s", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG, "Unhandled Event ID: %d", event->event_id);
            break;
    }
}


// ========================= MQTT Publish =========================
esp_err_t WifiHandler::mqttPublish(const char* topic, const char* data, int qos, int retain) {
    if (!client_) {
        ESP_LOGE(TAG, "MQTT client is not initialized");
        return ESP_FAIL;
    }
    
    int msg_id = esp_mqtt_client_publish(client_, topic, data, 0, qos, retain);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "MQTT Publish failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Published to %s, msg_id=%d", topic, msg_id);
    return ESP_OK;
}


// ========================= MQTT Subscribe =========================
esp_err_t WifiHandler::mqttSubscribe(const char* topic, int qos) {
    if (!client_) {
        ESP_LOGE(TAG, "MQTT  client is not initialized");
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_subscribe(client_, topic, qos);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "MQTT Subscribe failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", topic, msg_id);
    return ESP_OK;
}


// ========================= MQTT Unsubscribe =========================
esp_err_t WifiHandler::mqttUnsubscribe(const char* topic) {
    if (!client_) {
        ESP_LOGE(TAG, "MQTT client is not initialized");
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_unsubscribe(client_, topic);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "MQTT Unsubscribe failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Unsubscribed from %s, msg_id=%d", topic, msg_id);
    return ESP_OK;
}


// ========================= MQTT Stop =========================
void WifiHandler::mqttStop() {
    if (!client_) {
        ESP_LOGW(TAG, "MQTT client is already stopped or not initialized");
        return;
    }

    esp_mqtt_client_stop(client_);
    esp_mqtt_client_destroy(client_);
    client_ = nullptr;
    ESP_LOGI(TAG, "MQTT Client stopped");
}


esp_err_t WifiHandler::sendData(const char* brokerHost, uint32_t brokerPort, const char* topic, const char* data) {
    // Initialize MQTT client
    esp_err_t err = mqttInit(brokerHost, brokerPort);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT");
        return err;
    }

    // Publish data to the specified topic
    err = mqttPublish(topic, data, 1, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to publish data to topic: %s", topic);
        mqttStop();
        return err;
    }

    ESP_LOGI(TAG, "Data published to topic: %s", topic);

    // Stop and clean up MQTT client
    mqttStop();
    return ESP_OK;
}

bool WifiHandler::isHTTPClientConnected() { 
    return httpClientConnected_; 
}

bool WifiHandler::isMQTTClientConnected() { 
    return mqttClientConnected_; 
}