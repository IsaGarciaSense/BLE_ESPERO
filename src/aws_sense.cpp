/*******************************************************************************
 * @file aws_sense.cpp
 * @brief AWS helper class to manage AWS IoT Core connection and data transfer
 *        using MQTT over TLS. This class leverages the WifiHandler object for
 *        network connectivity. AWS connection parameters (thing name, endpoint,
 *        certificates) are loaded from secrets.hpp.
 *
 * @version 0.1.0
 * @date 2025-03-05
 ******************************************************************************/

#include "aws_sense.hpp"
#include "esp_log.h"
#include "mqtt_client.h"
// #include "secrets.hpp"  // Contains AWS_THINGNAME, AWS_IOT_ENDPOINT,
// AWS_CERT_CA, AWS_CERT_CRT, AWS_CERT_PRIVATE
#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "dirent.h"
#include "esp_tls.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "mbedtls/pk.h"
#include "mbedtls/x509_crt.h"

static const char* TAG = "awsHandler";

esp_err_t init_spiffs() {
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {.base_path = "/spiffs",
                                  .partition_label = NULL,
                                  .max_files = 5,
                                  .format_if_mount_failed = true};

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted successfully");
    }
    return ret;
}

void listSPIFFSFiles() {
    ESP_LOGI("SPIFFS", "Listing files in SPIFFS...");
    struct dirent* entry;
    DIR* dir = opendir("/spiffs");
    if (dir == NULL) {
        ESP_LOGE("SPIFFS", "Failed to open directory.");
        return;
    }

    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI("SPIFFS", "File: %s", entry->d_name);
    }
    closedir(dir);
}

std::string readFile(const char* filePath) {
    std::string fullPath = std::string("/spiffs/") + filePath;
    std::ifstream file(fullPath);

    if (!file.is_open()) {
        ESP_LOGE("SPIFFS", "Failed to open file: %s", fullPath.c_str());
        return "";
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    if (content.back() != '\n') {
        content += "\n";  // Ensure a newline at the end
    }
    content += '\0';  // Ensure null termination

    // ESP_LOGI("SPIFFS", "Read %d bytes from %s", content.length(),
    // fullPath.c_str()); ESP_LOGI("SPIFFS", "First 100 chars:\n%.100s",
    // content.c_str());

    return content;
}

// Function to validate a certificate
esp_err_t validateCertificate(const std::string& pemContent,
                              const char* certType) {
    mbedtls_x509_crt cert;
    mbedtls_x509_crt_init(&cert);

    int ret =
        mbedtls_x509_crt_parse(&cert, (const unsigned char*)pemContent.c_str(),
                               pemContent.length() + 1);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to parse %s: -0x%x", certType, -ret);
        mbedtls_x509_crt_free(&cert);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "%s parsed successfully!", certType);
    mbedtls_x509_crt_free(&cert);
    return ESP_OK;
}

// Function to validate a private key
esp_err_t validatePrivateKey(const std::string& pemContent) {
    mbedtls_pk_context pk;
    mbedtls_pk_init(&pk);

    int ret =
        mbedtls_pk_parse_key(&pk, (const unsigned char*)pemContent.c_str(),
                             pemContent.length() + 1, NULL, 0, NULL, 0);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to parse private key: -0x%x", -ret);
        mbedtls_pk_free(&pk);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Private key parsed successfully!");
    mbedtls_pk_free(&pk);
    return ESP_OK;
}

// Function to validate and load a certificate
esp_err_t validateAndLoadCert(const char* certPath, std::string& certBuffer) {
    certBuffer = readFile(certPath);

    if (certBuffer.empty()) {
        ESP_LOGE(TAG, "Failed to read certificate: %s", certPath);
        return ESP_FAIL;
    }

    if (validateCertificate(certBuffer, certPath) != ESP_OK) {
        ESP_LOGE(TAG, "Certificate validation failed: %s", certPath);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Certificate %s is valid!", certPath);
    return ESP_OK;
}

// Function to validate and load a private key
esp_err_t validateAndLoadKey(const char* keyPath, std::string& keyBuffer) {
    keyBuffer = readFile(keyPath);

    if (keyBuffer.empty()) {
        ESP_LOGE(TAG, "Failed to read private key: %s", keyPath);
        return ESP_FAIL;
    }

    if (validatePrivateKey(keyBuffer) != ESP_OK) {
        ESP_LOGE(TAG, "Private key validation failed: %s", keyPath);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Private key %s is valid!", keyPath);
    return ESP_OK;
}

// ========================= awsHandler Constructor =========================
awsHandler::awsHandler(WifiHandler& wifiHandler) : wifi_(wifiHandler) {
    // Constructor initializes the AWS handler
    ESP_LOGI(TAG, "awsHandler initialized");
}

// ========================= awsHandler Destructor =========================
// awsHandler::~awsHandler() {
//     // Ensure proper cleanup of MQTT client
//     disconnect();

//     // Clear topic-specific message storage for this instance's subscriptions
//     for (const auto& subscription : activeSubscriptions_) {
//         const std::string& topic = subscription.first;
//         topicMessages_.erase(topic);
//         topicMessageFlags_.erase(topic);
//     }

//     ESP_LOGI(TAG, "awsHandler destructor completed");
// }

// ========================= init() =========================
esp_err_t awsHandler::init(const char* awsEndpoint, const char* clientId,
                           const char* caCertPath, const char* deviceCertPath,
                           const char* privateKeyPath) {
    if (awsEndpoint == NULL || clientId == NULL || caCertPath == NULL ||
        deviceCertPath == NULL || privateKeyPath == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for AWS init");
        return ESP_ERR_INVALID_ARG;
    }

    init_spiffs();
    listSPIFFSFiles();
    awsEndpoint_ = awsEndpoint;
    clientId_ = clientId;

    if (validateAndLoadCert(caCertPath, awsRootCA_) != ESP_OK ||
        validateAndLoadCert(deviceCertPath, certificatePem_) != ESP_OK ||
        validateAndLoadKey(privateKeyPath, privateKeyPem_) != ESP_OK) {
        ESP_LOGE(TAG, "Certificate or key validation failed.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "AWS init: Endpoint=%s, ClientId=%s", awsEndpoint_.c_str(),
             clientId_.c_str());
    return ESP_OK;
}

// ========================= defaultAWSMQTTConfigs() =========================
esp_mqtt_client_config_t awsHandler::defaultAWSMQTTConfigs() {
    // AWS IoT Core typically uses port 8883 for TLS connections.
    esp_mqtt_client_config_t config = {
        .broker =
            {
                .address =
                    {
                        .uri = NULL,  // Do not set URI when providing
                                      // hostname/port manually.
                        .hostname = awsEndpoint_.c_str(),  // AWS IoT endpoint
                        .transport =
                            MQTT_TRANSPORT_OVER_SSL,  // Force SSL/TLS transport
                        .path = NULL,
                        .port = 8883,  // AWS default TLS port
                    },
                .verification =
                    {
                        .use_global_ca_store =
                            false,  // Not using a global CA store here
                        .crt_bundle_attach =
                            NULL,  // Optionally attach a certificate bundle if
                                   // available
                        // Set the CA certificate from file:
                        .certificate = awsRootCA_.c_str(),
                        .certificate_len = awsRootCA_.length(),
                        .psk_hint_key = NULL,
                        .skip_cert_common_name_check = false,
                        .alpn_protos = NULL,
                        .common_name = NULL,
                    },
            },
        .credentials =
            {
                .username =
                    NULL,  // AWS IoT Core typically doesn't require username
                .client_id =
                    clientId_.c_str(),  // Set the client ID (you can
                                        // incorporate the thing name or MAC)
                .set_null_client_id = false,
                .authentication =
                    {
                        .password = NULL,  // No password; using TLS
                                           // certificates for authentication
                        .certificate =
                            certificatePem_
                                .c_str(),  // Device certificate in PEM format
                        .certificate_len = certificatePem_.length(),
                        .key = privateKeyPem_.c_str(),  // Device private key in
                                                        // PEM format
                        .key_len = privateKeyPem_.length(),
                        .key_password = NULL,
                        .key_password_len = 0,
                        .use_secure_element = false,
                        .ds_data = NULL,
                    },
            },
        .session =
            {
                .last_will =
                    {
                        .topic = NULL,
                        .msg = NULL,
                        .msg_len = 0,
                        .qos = 0,
                        .retain = 0,
                    },
                .disable_clean_session = false,
                .keepalive = 120,
                .disable_keepalive = false,
                .protocol_ver = MQTT_PROTOCOL_V_3_1_1,
                .message_retransmit_timeout = 120,
            },
        .network =
            {
                .reconnect_timeout_ms = 10000,
                .timeout_ms = 10000,
                .refresh_connection_after_ms = 0,
                .disable_auto_reconnect = false,
                .transport = NULL,
                .if_name = NULL,
            },
        .task =
            {
                .priority = 5,
                .stack_size = 16384,
            },
        .buffer =
            {
                .size = 4096,
                .out_size = 1024,
            },
        .outbox =
            {
                .limit = 0,
            },
    };
    return config;
}

// ========================= connect() =========================
esp_err_t awsHandler::connect() {
    // If already connected, no need to reconnect
    if (connectionFlag_) {
        ESP_LOGI(TAG, "AWS MQTT client already connected");
        return ESP_OK;
    }

    // Clean up existing client if it exists
    if (mqttClient_) {
        ESP_LOGW(TAG, "Cleaning up existing MQTT client before reconnection");
        esp_mqtt_client_stop(mqttClient_);
        esp_mqtt_client_destroy(mqttClient_);
        mqttClient_ = nullptr;
        connectionFlag_ = false;
    }

    uint8_t timeoutCounter = 20;  ///< 20s
    // Use the default AWS MQTT configuration and then initialize the MQTT
    // client
    esp_mqtt_client_config_t mqtt_cfg = defaultAWSMQTTConfigs();

    mqttClient_ = esp_mqtt_client_init(&mqtt_cfg);
    if (!mqttClient_) {
        ESP_LOGE(TAG, "Failed to initialize AWS MQTT client");
        return ESP_FAIL;
    }

    // Register our MQTT event handler
    esp_mqtt_client_register_event(
        mqttClient_, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
        awsHandler::mqtt_event_handler, this);

    esp_err_t start_err = esp_mqtt_client_start(mqttClient_);
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start AWS MQTT client: %s",
                 esp_err_to_name(start_err));
        esp_mqtt_client_destroy(mqttClient_);
        mqttClient_ = nullptr;
        return start_err;
    }

    // Wait for connection with timeout
    while (!connectionFlag_ && timeoutCounter) {
        // While not connected or timeout
        timeoutCounter = (timeoutCounter == 0) ? 0 : timeoutCounter - 1;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (timeoutCounter == 0) {
        ESP_LOGE(TAG, "AWS MQTT connection timeout");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "AWS MQTT Client connected successfully with client ID: %s",
             clientId_.c_str());
    return ESP_OK;
}

// ========================= MQTT Event Handler for AWS
// =========================
std::string awsHandler::aws_mqttResponseBody_ = "";
bool awsHandler::aws_mqttResponse_ = false;
std::map<std::string, std::string> awsHandler::topicMessages_;
std::map<std::string, bool> awsHandler::topicMessageFlags_;

void awsHandler::mqtt_event_handler(void* handler_args, esp_event_base_t base,
                                    int32_t event_id, void* event_data) {
    awsHandler* aws = static_cast<awsHandler*>(handler_args);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    ESP_LOGD(TAG, "AWS MQTT event dispatched: base=%s, event_id=%" PRIi32, base,
             event_id);

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "AWS MQTT Connected");
            aws->connectionFlag_ = true;
            // Automatically resubscribe to all previous subscriptions
            if (!aws->activeSubscriptions_.empty()) {
                ESP_LOGI(TAG, "Resubscribing to %zu previous subscriptions",
                         aws->activeSubscriptions_.size());
                esp_err_t resubResult = aws->resubscribeAll();
                if (resubResult != ESP_OK) {
                    ESP_LOGE(TAG, "Some resubscriptions failed");
                }
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "AWS MQTT Disconnected");
            aws->connectionFlag_ = false;
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "AWS MQTT Subscribed, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "AWS MQTT Unsubscribed, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "AWS MQTT Published, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "AWS MQTT Data Received");

            // Check if we have topic information
            if (event->topic_len > 0) {
                std::string topic(event->topic, event->topic_len);

                // Check if this topic is in our active subscriptions
                bool isSubscribed = false;
                for (const auto& subscription : aws->activeSubscriptions_) {
                    if (subscription.first == topic) {
                        isSubscribed = true;
                        break;
                    }
                }

                if (!isSubscribed) {
                    ESP_LOGW(TAG,
                             "Received message from non-subscribed topic: %s, "
                             "ignoring message",
                             topic.c_str());
                } else {
                    // Process message from subscribed topic
                    std::string payload(event->data, event->data_len);

                    // Store the latest message for this topic (overwrites previous)
                    topicMessages_[topic] = payload;
                    topicMessageFlags_[topic] = true;

                    // Also update legacy variables for backward compatibility
                    aws_mqttResponse_ = true;
                    aws_mqttResponseBody_ = payload;

                    ESP_LOGI(TAG, "Message received from topic: %s", topic.c_str());
                }
            } else {
                ESP_LOGW(TAG, "Received MQTT data without topic information");
            }

            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "AWS MQTT Event Error");
            break;
        default:
            ESP_LOGI(TAG, "AWS MQTT Unhandled Event ID: %d", event->event_id);
            break;
    }
}

// ========================= publish() =========================
esp_err_t awsHandler::publish(const char* topic, const char* payload, int qos,
                              int retain) {
    if (!mqttClient_) {
        ESP_LOGE(TAG, "AWS MQTT client not initialized");
        return ESP_FAIL;
    }

    int msg_id =
        esp_mqtt_client_publish(mqttClient_, topic, payload, 0, qos, retain);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "AWS MQTT Publish failed");
        return ESP_FAIL;
    }
    // This is done in the mqtt handler too
    // ESP_LOGI(TAG, "Published to %s, msg_id=%d", topic, msg_id);
    return ESP_OK;
}

// ========================= subscribe() =========================
esp_err_t awsHandler::subscribe(const char* topic, int qos) {
    if (!mqttClient_) {
        ESP_LOGE(TAG, "AWS MQTT client not initialized");
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_subscribe(mqttClient_, topic, qos);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "AWS MQTT Subscribe failed");
        return ESP_FAIL;
    }

    // Add to active subscriptions list (avoid duplicates)
    std::string topicStr(topic);
    auto it =
        std::find_if(activeSubscriptions_.begin(), activeSubscriptions_.end(),
                     [&topicStr](const std::pair<std::string, int>& sub) {
                         return sub.first == topicStr;
                     });

    if (it != activeSubscriptions_.end()) {
        // Update QoS if topic already exists
        it->second = qos;
    } else {
        // Add new subscription
        activeSubscriptions_.emplace_back(topicStr, qos);

        // Initialize topic message storage
        topicMessages_[topicStr] = "";
        topicMessageFlags_[topicStr] = false;
    }

    ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", topic, msg_id);
    return ESP_OK;
}

// ========================= unsubscribe() =========================
esp_err_t awsHandler::unsubscribe(const char* topic) {
    if (!mqttClient_) {
        ESP_LOGE(TAG, "AWS MQTT client not initialized");
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_unsubscribe(mqttClient_, topic);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "AWS MQTT Unsubscribe failed");
        return ESP_FAIL;
    }

    // Remove from active subscriptions list
    std::string topicStr(topic);
    activeSubscriptions_.erase(
        std::remove_if(activeSubscriptions_.begin(), activeSubscriptions_.end(),
                       [&topicStr](const std::pair<std::string, int>& sub) {
                           return sub.first == topicStr;
                       }),
        activeSubscriptions_.end());

    // Free allocated memory for this topic
    topicMessages_.erase(topicStr);
    topicMessageFlags_.erase(topicStr);

    ESP_LOGI(TAG, "Unsubscribed from %s, msg_id=%d", topic, msg_id);
    return ESP_OK;
}

// ========================= resubscribeAll() =========================
esp_err_t awsHandler::resubscribeAll() {
    if (!mqttClient_ || !connectionFlag_) {
        ESP_LOGE(TAG, "AWS MQTT client not connected for resubscription");
        return ESP_FAIL;
    }

    esp_err_t result = ESP_OK;
    for (const auto& subscription : activeSubscriptions_) {
        int msg_id = esp_mqtt_client_subscribe(
            mqttClient_, subscription.first.c_str(), subscription.second);
        if (msg_id < 0) {
            ESP_LOGE(TAG, "Failed to resubscribe to %s",
                     subscription.first.c_str());
            result = ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "Resubscribed to %s with QoS %d, msg_id=%d",
                     subscription.first.c_str(), subscription.second, msg_id);
        }
    }

    if (result == ESP_OK && !activeSubscriptions_.empty()) {
        ESP_LOGI(TAG, "Successfully resubscribed to %zu topics",
                 activeSubscriptions_.size());
    }

    return result;
}

// ========================= disconnect() =========================
void awsHandler::disconnect() {
    if (mqttClient_) {
        esp_mqtt_client_stop(mqttClient_);
        esp_mqtt_client_destroy(mqttClient_);
        mqttClient_ = nullptr;
        connectionFlag_ = false;
        // Clear active subscriptions when manually disconnecting
        activeSubscriptions_.clear();
        ESP_LOGI(TAG, "AWS MQTT Client disconnected and subscriptions cleared");
    } else {
        ESP_LOGW(TAG,
                 "AWS MQTT client already disconnected or not initialized");
    }
}

// ========================= Message Queue Methods =========================

bool awsHandler::awsGetResponse(const std::string& topic,
                                std::string& message) {
    auto messageIt = topicMessages_.find(topic);
    if (messageIt != topicMessages_.end() && !messageIt->second.empty()) {
        message = messageIt->second;
        // Clear the message after reading (consumed)
        messageIt->second.clear();
        topicMessageFlags_[topic] = false;  // Reset flag for this topic
        return true;
    }
    return false;
}

void awsHandler::awsClearMessage(const std::string& topic) {
    auto messageIt = topicMessages_.find(topic);
    if (messageIt != topicMessages_.end()) {
        messageIt->second.clear();
        topicMessageFlags_[topic] = false;  // Reset flag for this topic
        ESP_LOGI(TAG, "Cleared message for topic: %s", topic.c_str());
    } else {
        ESP_LOGW(TAG, "Topic %s not found in message storage", topic.c_str());
    }
}

bool awsHandler::awsMqttResponseFlag(const std::string& topic) {
    auto it = topicMessageFlags_.find(topic);
    return (it != topicMessageFlags_.end()) ? it->second : false;
}

void awsHandler::awsResetResponseFlag(const std::string& topic) {
    auto it = topicMessageFlags_.find(topic);
    if (it != topicMessageFlags_.end()) {
        it->second = false;
        ESP_LOGD(TAG, "Reset response flag for topic: %s", topic.c_str());
    }
}
