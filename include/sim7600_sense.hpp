#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "driver/gpio.h"  // Include GPIO header for GPIO_NUM_x definitions
#include "esp_err.h"      // For ESP error codes
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Include UART_sense for UART communication
#include "uart_sense.hpp"

extern "C" {
#include "esp_system.h"
}

class SimCom7600 {
public:
    // Configuration structure for SimCom7600
    struct Config {
        bool debug_mode = true;
        bool pretty_output = false;
        int default_timeout_ms = 1000;  // Default timeout in milliseconds
        std::string mqtt_broker;
        int mqtt_port;
        std::string mqtt_client_id;
        std::string mqtt_username;
        std::string mqtt_password;
    };  // Constructor that takes a reference to an existing UART object
    SimCom7600(UART& uart);

    // Destructor
    ~SimCom7600();

    // Basic AT command functions
    std::string sendAT(const std::string& command, int timeout_ms = -1,
                       bool print_command = true);
    esp_err_t isResponseOK(const std::string& response);
    esp_err_t isResponseError(
        const std::string& response);  // Module status and connection
    void getModuleStatus();
    void getSimStatus();
    void getNetworkConfig();
    std::string pingTest(const std::string& host = "8.8.8.8");
    esp_err_t checkSim7600Status(int timeout_ms = 3000);
    esp_err_t checkGSMStatus(int timeout_ms = 5000);

    // Module reset functions
    esp_err_t reset();
    esp_err_t factoryReset();

    // Safe restart with configuration preservation
    esp_err_t restart(int wait_time_ms = 15000);
    esp_err_t safeRestart(bool save_config = true, int wait_time_ms = 15000);

    // Time synchronization functions
    esp_err_t syncTimeWithNTP(const std::string& ntpServer = "pool.ntp.org",
                              int timezoneOffset = 0, int timeout_ms = 10000);
    esp_err_t getNetworkTime(struct tm* timeinfo, int timeout_ms = 5000);
    bool isTimeSet();

    // Certificate management
    esp_err_t uploadCertificate(const std::string& cert_file,
                                const std::string& cert_name);
    esp_err_t uploadCertificates();
    esp_err_t deleteCertificate(const std::string& cert_name);
    esp_err_t deleteAllCertificates();  // MQTT configuration
    void configureMqttAuth();
    void setWill();
    void disableWill();
    esp_err_t configureMQTTSSL();
    esp_err_t initMqtt();
    esp_err_t checkBrokerStatus(int timeout_ms = 5000);
    esp_err_t publishMessage(const std::string& topic,
                             const std::string& message);
    void setMqttCredentials(const std::string& username,
                            const std::string& password,
                            const std::string& broker = "", int port = -1,
                            const std::string& client_id = "");
    void disconnect();

    // Debug and configuration
    void enableFullDebug();
    void showConfig();
    void configure(const Config& config);

private:
    // UART reference
    UART& uart_;

    // Module configuration
    Config config_;

    // Buffer for communication
    static const int BUFFER_SIZE = 1024;  // Utility methods
    std::string formatResponse(const std::string& response);
    esp_err_t validateCertificateFormat(const std::string& content,
                                        const std::string& cert_name);

    // Logging
    static const char* TAG;
};
