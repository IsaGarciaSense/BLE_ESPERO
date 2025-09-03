/*******************************************************************************
 * @file aws_sense.hpp
 * @brief AWS helper class to manage AWS IoT Core connection and data transfer
 *        using MQTT over TLS. This class leverages the WifiHandler object
 *        for network connectivity.
 *
 * @version 0.1.0
 * @date 2025-03-05
 * @author
 ******************************************************************************/

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "cJSON.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "mqtt_client.h"
#include "spiffs_config.h"
#include "wifi_sense.hpp"

class awsHandler {
public:
    /**
     * @brief Construct a new awsHandler object.
     *
     * @param wifiHandler Reference to an already initialized WifiHandler
     * object.
     */
    explicit awsHandler(WifiHandler& wifiHandler);

    // TODO: The module brokes when this is uncommented, analyze why
    // /**
    //  * @brief Destructor to properly clean up AWS resources.
    //  */
    // ~awsHandler();

    /**
     * @brief Initialize AWS IoT Core connection parameters.
     *
     * @param awsEndpoint AWS IoT endpoint (e.g.,
     * "your-endpoint.iot.region.amazonaws.com")
     * @param clientId MQTT client identifier (e.g., derived from device MAC or
     * custom string)
     * @param certificatePem Device certificate in PEM format (used for TLS)
     * @param privateKeyPem Device private key in PEM format (used for TLS)
     * @return esp_err_t ESP_OK on success, otherwise error code.
     */
    esp_err_t init(const char* awsEndpoint, const char* clientId,
                   const char* caCertPath, const char* deviceCertPath,
                   const char* privateKeyPath);

    /**
     * @brief Connect to AWS IoT Core via MQTT over TLS.
     *
     * @return esp_err_t ESP_OK on success, otherwise error code.
     */
    esp_err_t connect();

    /**
     * @brief Returns mqtt-broker status
     * @return true if broker is connected
     */
    bool isConnected(void) {
        return connectionFlag_;
    }

    /**
     * @brief Publish a message to an AWS IoT topic.
     *
     * @param topic MQTT topic string.
     * @param payload Message payload.
     * @param qos Quality of Service (default 1).
     * @param retain Retain flag (default 0).
     * @return esp_err_t ESP_OK on success, otherwise error code.
     */
    esp_err_t publish(const char* topic, const char* payload, int qos = 1,
                      int retain = 0);

    /**
     * @brief Subscribe to an AWS IoT topic.
     *
     * @param topic MQTT topic string.
     * @param qos Quality of Service (default 1).
     * @return esp_err_t ESP_OK on success, otherwise error code.
     */
    esp_err_t subscribe(const char* topic, int qos = 1);

    /**
     * @brief Unsubscribe from an AWS IoT topic.
     *
     * @param topic MQTT topic string to unsubscribe from.
     * @return esp_err_t ESP_OK on success, otherwise error code.
     */
    esp_err_t unsubscribe(const char* topic);

    /**
     * @brief Get the list of active subscriptions.
     *
     * @return std::vector<std::pair<std::string, int>> Vector of topic-QoS
     * pairs.
     */
    std::vector<std::pair<std::string, int>> getActiveSubscriptions() const {
        return activeSubscriptions_;
    }

    /**
     * @brief Disconnect from AWS IoT Core and clean up MQTT resources.
     */
    void disconnect();

    static void mqtt_event_handler(void* handler_args, esp_event_base_t base,
                                   int32_t event_id, void* event_data);

    // For MQTT message handling
    std::string awsGetResponse() {
        return aws_mqttResponseBody_;
    }

    /**
     * @brief Get the latest message from a specific topic
     *
     * @param topic The topic to get the message from
     * @param message Output parameter for the message
     * @return true if message found, false otherwise
     */
    bool awsGetResponse(const std::string& topic, std::string& message);

    /**
     * @brief Clear the message from a specific topic
     *
     * @param topic The topic to clear message from
     */
    void awsClearMessage(const std::string& topic);

    bool awsMqttResponseFlag() {
        return aws_mqttResponse_;
    }

    /**
     * @brief Check if there are messages available for a specific topic
     *
     * @param topic The topic to check
     * @return true if messages are available, false otherwise
     */
    bool awsMqttResponseFlag(const std::string& topic);

    void awsResetResponseFlag() {
        aws_mqttResponse_ = false;
    }

    /**
     * @brief Reset the response flag for a specific topic
     *
     * @param topic The topic to reset flag for
     */
    void awsResetResponseFlag(const std::string& topic);

private:
    WifiHandler& wifi_;  // Reference to the WifiHandler for network operations

    // AWS IoT configuration parameters
    std::string awsEndpoint_;     // AWS IoT endpoint
    std::string clientId_;        // MQTT client identifier
    std::string certificatePem_;  // Device certificate in PEM format
    std::string privateKeyPem_;   // Device private key in PEM format
    std::string awsRootCA_;       // AWS IoT Root CA certificate

    // MQTT client handle for AWS IoT
    esp_mqtt_client_handle_t mqttClient_ = nullptr;

    static std::string aws_mqttResponseBody_;
    static bool aws_mqttResponse_;  ///< Checks if a message was received
    bool connectionFlag_ = false;   ///< Checks connection status

    // Topic-specific message storage
    static std::map<std::string, std::string>
        topicMessages_;  ///< Per-topic message storage
    static std::map<std::string, bool>
        topicMessageFlags_;  ///< Per-topic message flags

    // Subscription tracking for automatic resubscription
    std::vector<std::pair<std::string, int>>
        activeSubscriptions_;  ///< topic, qos pairs

    /**
     * @brief Internal helper function to create a default MQTT configuration
     *        structure suitable for connecting to AWS IoT Core.
     *
     * @return esp_mqtt_client_config_t Config structure with default AWS
     * settings.
     */
    esp_mqtt_client_config_t defaultAWSMQTTConfigs();

    /**
     * @brief Resubscribe to all previously active subscriptions.
     *        Called automatically when connection is restored.
     *
     * @return esp_err_t ESP_OK on success, otherwise error code.
     */
    esp_err_t resubscribeAll();

    const char* Topic_ = "alpha";
    const char* registerTopic_ = "register";
    const char* variablesTopic_ = "pub";
    const char* responseTopic_ = "sub";
};
