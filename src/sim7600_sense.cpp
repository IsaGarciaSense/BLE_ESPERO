#include <algorithm>
#include <cstdlib>  // For rand() and srand()
#include <ctime>    // For time()
#include <fstream>
#include <regex>
#include <sstream>

#include "../include/sim7600_sense.hpp"
#include "esp_err.h"  // For ESP error codes
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// TAG for logging
const char* SimCom7600::TAG = "SimCom7600";

SimCom7600::SimCom7600(UART& uart) : uart_(uart) {
    // Initialize default configuration
    config_.debug_mode = true;
    config_.pretty_output = false;
    config_.default_timeout_ms = 1000;

    // Initialize MQTT defaults
    config_.mqtt_broker = "";
    config_.mqtt_port = 8883;
    config_.mqtt_client_id = "";
    config_.mqtt_username = "";
    config_.mqtt_password = "";

    ESP_LOGI(TAG, "SimCom7600 initialized with external UART");

    // Allow module to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));
}

SimCom7600::~SimCom7600() {
    // No need to deinitialize the UART since it's managed externally
}

std::string SimCom7600::sendAT(const std::string& command, int timeout_ms,
                               bool print_command) {
    // Use config timeout if not specified
    if (timeout_ms < 0) {
        timeout_ms = config_.default_timeout_ms;
    }

    if (print_command && config_.debug_mode) {
        ESP_LOGI(TAG, "Sending command: %s", command.c_str());
    }

    // Add carriage return and line feed if not present
    std::string cmd = command;
    if (cmd.length() >= 2 && cmd.substr(cmd.length() - 2) != "\r\n") {
        cmd += "\r\n";
    }

    // Send command
    esp_err_t ret = uart_.write(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to UART: %d", ret);
        return "ERROR";
    }

    if (config_.debug_mode) {
        ESP_LOGI(TAG, "Command sent successfully");
    }

    // Wait for response
    if (config_.debug_mode) {
        ESP_LOGI(TAG, "Waiting %d ms for response", timeout_ms);
    }
    vTaskDelay(pdMS_TO_TICKS(timeout_ms));  // Read response
    static std::string dataRetrieved;

    // can be used only after reading Rx buffer
    int len = uart_.getDataLen();
    if (len < 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGW(TAG, "No data available in UART buffer.");
    }

    uart_.read(dataRetrieved);
    if (config_.debug_mode) {
        ESP_LOGI(TAG, "Data retrieved: %s", dataRetrieved.c_str());
    }

    std::string formatted_response =
        config_.pretty_output ? formatResponse(dataRetrieved) : dataRetrieved;

    dataRetrieved.clear();  // Clear the buffer after reading

    return formatted_response;
}

std::string SimCom7600::formatResponse(const std::string& response) {
    if (response.empty()) {
        return "";
    }

    // Clean up response
    std::string cleanedResponse = response;
    // Replace CR+LF with just LF for consistent line endings
    size_t pos = 0;
    while ((pos = cleanedResponse.find("\r\n", pos)) != std::string::npos) {
        cleanedResponse.replace(pos, 2, "\n");
        pos += 1;
    }

    // Split into lines
    std::vector<std::string> lines;
    std::istringstream iss(cleanedResponse);
    std::string line;
    while (std::getline(iss, line)) {
        if (!line.empty()) {
            lines.push_back(line);
        }
    }

    // Filter meaningful lines (remove empty lines and echoed commands)
    std::string formatted;
    for (const auto& line : lines) {
        if (!line.empty()) {
            if (formatted.length() > 0) {
                formatted += "\n";
            }
            formatted += line;
        }
    }

    return formatted;
}

esp_err_t SimCom7600::isResponseOK(const std::string& response) {
    // Convert to uppercase for case-insensitive comparison
    std::string upperResponse = response;
    std::transform(upperResponse.begin(), upperResponse.end(),
                   upperResponse.begin(), ::toupper);
    return upperResponse.find("OK") != std::string::npos ? ESP_OK : ESP_FAIL;
}

esp_err_t SimCom7600::isResponseError(const std::string& response) {
    // Convert to uppercase for case-insensitive comparison
    std::string upperResponse = response;
    std::transform(upperResponse.begin(), upperResponse.end(),
                   upperResponse.begin(), ::toupper);
    return upperResponse.find("ERROR") != std::string::npos ? ESP_OK : ESP_FAIL;
}

void SimCom7600::getModuleStatus() {
    ESP_LOGI(TAG, "=== Module Status Information ===");
    ESP_LOGI(TAG, "%s", sendAT("AT+CPIN?").c_str());
    ESP_LOGI(TAG, "%s", sendAT("ATI").c_str());
    ESP_LOGI(TAG, "%s", sendAT("AT+CGMM").c_str());
    ESP_LOGI(TAG, "%s", sendAT("AT+CGMR").c_str());
}

void SimCom7600::getSimStatus() {
    ESP_LOGI(TAG, "=== SIM Card Status ===");
    ESP_LOGI(TAG, "%s", sendAT("AT+CIMI").c_str());
    ESP_LOGI(TAG, "%s", sendAT("AT+CCID").c_str());
}

void SimCom7600::getNetworkConfig() {
    ESP_LOGI(TAG, "=== Network Configuration ===");
    ESP_LOGI(TAG, "%s", sendAT("AT+COPS?").c_str());
}

std::string SimCom7600::pingTest(const std::string& host) {
    ESP_LOGI(TAG, "=== Ping Test to %s ===", host.c_str());
    std::string cmd = "AT+CPING=\"" + host + "\",1,5,64,1000,20000,255";
    std::string result = sendAT(cmd, 10000);  // 10 second timeout for ping
    ESP_LOGI(TAG, "%s", result.c_str());
    return result;
}

esp_err_t SimCom7600::checkSim7600Status(int timeout_ms) {
    ESP_LOGI(TAG, "=== Checking SIM7600 Module Status ===");
    ESP_LOGI(TAG, "Sending simple AT command...");

    std::string response = sendAT("AT", timeout_ms);
    ESP_LOGI(TAG, "Response: %s", response.c_str());

    if (response == "ERROR") {
        ESP_LOGE(TAG, "UART communication failed");
        return ESP_FAIL;
    } else if (response.find("OK") != std::string::npos) {
        ESP_LOGI(TAG, "Module responded successfully");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "No response or unexpected response within timeout");
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t SimCom7600::checkGSMStatus(int timeout_ms) {
    ESP_LOGI(TAG, "=== GSM Network Status Check ===");

    // Check network registration status
    std::string regResponse = sendAT("AT+CREG?", timeout_ms / 3);
    ESP_LOGI(TAG, "Network registration: %s", regResponse.c_str());

    // Check signal quality
    std::string signalResponse = sendAT("AT+CSQ", timeout_ms / 3);
    ESP_LOGI(TAG, "Signal quality: %s", signalResponse.c_str());

    // Check packet domain attach
    std::string attachResponse = sendAT("AT+CGATT?", timeout_ms / 3);
    ESP_LOGI(TAG, "GPRS attachment: %s", attachResponse.c_str());

    // Check IP address
    std::string ipResponse = sendAT("AT+CGPADDR", timeout_ms / 3);
    ESP_LOGI(TAG, "IP address: %s", ipResponse.c_str());

    // If registration is successful (typically +CREG: 0,1 or +CREG: 0,5)
    if ((regResponse.find("+CREG: 0,1") != std::string::npos ||
         regResponse.find("+CREG: 0,5") != std::string::npos) &&
        attachResponse.find("+CGATT: 1") != std::string::npos) {
        ESP_LOGI(TAG, "GSM network connected successfully");
        return ESP_OK;
    } else if (regResponse == "ERROR" || signalResponse == "ERROR" ||
               attachResponse == "ERROR" || ipResponse == "ERROR") {
        ESP_LOGE(TAG, "Failed to communicate with module");
        return ESP_FAIL;
    } else {
        ESP_LOGW(TAG, "GSM network not fully connected or timeout occurred");
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t SimCom7600::restart(int wait_time_ms) {
    ESP_LOGI(TAG, "=== Restarting SIM7600 Module (Soft Reset) ===");

    // Method 1: Software restart using CFUN (recommended - preserves most
    // configurations)
    ESP_LOGI(TAG, "Sending software restart command...");
    std::string response = sendAT("AT+CFUN=1,1", 2000);
    ESP_LOGI(TAG, "Restart response: %s", response.c_str());

    if (response.find("OK") == std::string::npos &&
        response.find("ERROR") == std::string::npos) {
        ESP_LOGW(TAG, "No immediate response to restart command (expected)");
    }

    // Wait for module to restart (typically takes 10-15 seconds)
    int actual_wait = (wait_time_ms > 0) ? wait_time_ms : 15000;
    ESP_LOGI(TAG, "Waiting %d ms for module to restart...", actual_wait);
    vTaskDelay(pdMS_TO_TICKS(actual_wait));

    // Try to communicate with module to verify it's back online
    ESP_LOGI(TAG, "Verifying module is responsive...");
    int retries = 5;
    while (retries-- > 0) {
        std::string test_response = sendAT("AT", 1000);
        if (test_response.find("OK") != std::string::npos) {
            ESP_LOGI(TAG, "Module restart successful - module is responsive");
            return ESP_OK;
        }

        ESP_LOGW(TAG,
                 "Module not yet responsive, retrying... (%d attempts left)",
                 retries);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGE(TAG,
             "Module restart failed - module not responsive after restart");
    return ESP_FAIL;
}

esp_err_t SimCom7600::safeRestart(bool save_config, int wait_time_ms) {
    ESP_LOGI(TAG, "=== Safe Restart (Preserving Configurations) ===");

    if (save_config) {
        ESP_LOGI(TAG, "Saving current configuration...");
        std::string saveResponse = sendAT("AT&W", 3000);
        if (saveResponse.find("OK") != std::string::npos) {
            ESP_LOGI(TAG, "Configuration saved successfully");
        } else {
            ESP_LOGW(TAG, "Failed to save configuration: %s",
                     saveResponse.c_str());
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Perform soft restart (preserves saved configurations)
    return restart(wait_time_ms);
}

esp_err_t SimCom7600::reset() {
    ESP_LOGI(TAG, "=== Performing Hard Reset ===");

    // Use CRESET command for complete module reset
    std::string response = sendAT("AT+CRESET", 2000);
    ESP_LOGI(TAG, "Hard reset response: %s", response.c_str());

    // Wait longer for hard reset (up to 30 seconds)
    ESP_LOGI(TAG, "Waiting 20 seconds for hard reset to complete...");
    vTaskDelay(pdMS_TO_TICKS(20000));

    // Verify module is responsive
    return checkSim7600Status(5000);
}

esp_err_t SimCom7600::factoryReset() {
    ESP_LOGI(TAG, "=== Factory Reset ===");

    // Reset to factory defaults
    std::string response = sendAT("ATZ", 2000);
    ESP_LOGI(TAG, "Factory reset response: %s", response.c_str());

    if (response.find("OK") != std::string::npos) {
        ESP_LOGI(TAG, "Factory reset command accepted");
        vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for reset to complete
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Factory reset failed");
        return ESP_FAIL;
    }
}

esp_err_t SimCom7600::uploadCertificate(const std::string& cert_file,
                                        const std::string& cert_name) {
    ESP_LOGI(TAG, "=== Uploading Certificate: %s ===", cert_name.c_str());

    // Read certificate file
    std::ifstream file(cert_file);
    if (!file) {
        ESP_LOGE(TAG, "Failed to open certificate file: %s", cert_file.c_str());
        return ESP_FAIL;
    }

    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    // Validate certificate format
    esp_err_t valid = validateCertificateFormat(content, cert_name);
    if (valid != ESP_OK) {
        ESP_LOGE(TAG, "Certificate format validation failed");
        return ESP_FAIL;
    }

    // Get certificate size
    int cert_size = content.length();

    // Send command to start certificate upload
    std::string cmd =
        "AT+CCERTDOWN=\"" + cert_name + "\"," + std::to_string(cert_size);
    std::string response = sendAT(cmd, 5000);
    ESP_LOGI(TAG, "%s", response.c_str());

    if (response.find("DOWNLOAD") == std::string::npos) {
        ESP_LOGE(TAG, "Failed to start certificate upload");
        return ESP_FAIL;
    }  // Send certificate content
    esp_err_t ret = uart_.write(content);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write certificate content: %d", ret);
        return ESP_FAIL;
    }

    // Wait for response
    vTaskDelay(pdMS_TO_TICKS(1000));  // Read response
    std::string upload_response;

    if (uart_.getDataLen() > 0) {
        uart_.read(upload_response);
        ESP_LOGI(TAG, "Certificate upload response: %s",
                 upload_response.c_str());
    }

    return upload_response.find("OK") != std::string::npos ? ESP_OK : ESP_FAIL;
}

esp_err_t SimCom7600::validateCertificateFormat(const std::string& content,
                                                const std::string& cert_name) {
    // Check for begin and end markers
    bool has_begin = content.find("-----BEGIN") != std::string::npos;
    bool has_end = content.find("-----END") != std::string::npos;

    if (!has_begin || !has_end) {
        ESP_LOGE(TAG, "Certificate %s missing BEGIN/END markers",
                 cert_name.c_str());
        return ESP_FAIL;
    }

    // Check for Windows-style line endings
    bool has_windows_linefeeds = content.find("\r\n") != std::string::npos;
    if (has_windows_linefeeds) {
        ESP_LOGW(TAG, "Certificate %s has Windows-style line endings",
                 cert_name.c_str());
    }

    return ESP_OK;
}

esp_err_t SimCom7600::uploadCertificates() {
    ESP_LOGI(TAG, "=== Certificate Upload ===");

    // This implementation is simplified
    // In a real implementation, you would need to handle file system operations
    // to find and read certificate files

    ESP_LOGW(TAG, "Certificate upload not implemented for ESP-IDF yet");
    ESP_LOGW(TAG, "This would require file system operations");

    return ESP_FAIL;
}

esp_err_t SimCom7600::deleteCertificate(const std::string& cert_name) {
    ESP_LOGI(TAG, "=== Deleting Certificate: %s ===", cert_name.c_str());
    std::string cmd = "AT+CCERTDELE=\"" + cert_name + "\"";
    std::string response = sendAT(cmd);
    ESP_LOGI(TAG, "%s", response.c_str());

    if (response.find("OK") != std::string::npos) {
        ESP_LOGI(TAG, "Certificate %s deleted successfully", cert_name.c_str());
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to delete certificate %s", cert_name.c_str());
        return ESP_FAIL;
    }
}

esp_err_t SimCom7600::deleteAllCertificates() {
    ESP_LOGI(TAG, "=== Deleting All Certificates ===");

    // First list all certificates
    ESP_LOGI(TAG, "Listing current certificates...");
    std::string cert_list_response = sendAT("AT+CCERTLIST");
    ESP_LOGI(TAG, "%s", cert_list_response.c_str());

    // Parse certificate list response to extract certificate names
    std::vector<std::string> cert_names;
    std::istringstream iss(cert_list_response);
    std::string line;
    while (std::getline(iss, line)) {
        // Look for lines containing "*.pem" or other certificate names
        if (line.find(".pem") != std::string::npos ||
            line.find(".crt") != std::string::npos ||
            line.find(".key") != std::string::npos) {
            // Extract certificate name (remove any surrounding quotes)
            std::string cert_name = line;
            cert_name.erase(
                std::remove(cert_name.begin(), cert_name.end(), '"'),
                cert_name.end());
            cert_name.erase(
                std::remove(cert_name.begin(), cert_name.end(), ' '),
                cert_name.end());
            cert_names.push_back(cert_name);
        }
    }

    if (cert_names.empty()) {
        ESP_LOGW(TAG, "No certificates found to delete");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Found %d certificates to delete", cert_names.size());

    // Delete each certificate
    esp_err_t success = ESP_OK;
    for (const auto& cert_name : cert_names) {
        if (deleteCertificate(cert_name) != ESP_OK) {
            success = ESP_FAIL;
        }
    }

    // Verify all certificates are deleted
    ESP_LOGI(TAG, "Verifying all certificates are deleted...");
    std::string verification = sendAT("AT+CCERTLIST");
    ESP_LOGI(TAG, "%s", verification.c_str());

    if (success == ESP_OK) {
        ESP_LOGI(TAG, "Successfully deleted all certificates");
    } else {
        ESP_LOGE(TAG, "Failed to delete some certificates");
    }

    return success;
}

void SimCom7600::configureMqttAuth() {
    ESP_LOGI(TAG, "=== Configuring MQTT Authentication ===");

    // Set SSL version to TLS 1.2
    ESP_LOGI(TAG, "%s", sendAT("AT+CSSLCFG=\"sslversion\",0,4").c_str());

    // Set authentication mode to server+client
    ESP_LOGI(TAG, "%s", sendAT("AT+CSSLCFG=\"authmode\",0,2").c_str());

    // Configure CA certificate
    ESP_LOGI(TAG, "%s",
             sendAT("AT+CSSLCFG=\"cacert\",0,\"cacert.pem\"").c_str());

    // Configure client certificate
    ESP_LOGI(TAG, "%s",
             sendAT("AT+CSSLCFG=\"clientcert\",0,\"clientcert.pem\"").c_str());

    // Configure client key
    ESP_LOGI(TAG, "%s",
             sendAT("AT+CSSLCFG=\"clientkey\",0,\"clientkey.pem\"").c_str());

    ESP_LOGI(TAG, "MQTT Authentication configured successfully");
}

void SimCom7600::setWill() {
    ESP_LOGI(TAG, "=== Setting MQTT Will Message ===");

    // In this simplified version, we'll just set a hardcoded will message
    std::string topic = "device/status";
    int topic_length = topic.length();

    // Configure will topic
    ESP_LOGI(TAG, "Setting will topic: %s", topic.c_str());
    ESP_LOGI(TAG, "%s",
             sendAT("AT+CMQTTWILLTOPIC=0," + std::to_string(topic_length))
                 .c_str());  // Send topic content without AT command prefix
    esp_err_t ret = uart_.write(topic);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write topic: %d", ret);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));  // Read response after sending the topic
    std::string response;

    if (uart_.getDataLen() > 0) {
        uart_.read(response);
        ESP_LOGI(TAG, "Topic response: %s", response.c_str());
    }

    // Set will message
    std::string will_msg = "SIMCom Connected!";
    int msg_length =
        will_msg
            .length();  // Configure will message (setting QoS=1, retained=1)
    ESP_LOGI(TAG, "Setting will message: %s", will_msg.c_str());
    ESP_LOGI(TAG, "%s",
             sendAT("AT+CMQTTWILLMSG=0," + std::to_string(msg_length) + ",1")
                 .c_str());  // Send message content without AT command prefix
    ret = uart_.write(will_msg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write will message: %d", ret);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));  // Read response after sending the message
    response.clear();

    if (uart_.getDataLen() > 0) {
        uart_.read(response);
        ESP_LOGI(TAG, "Message response: %s", response.c_str());
    }

    ESP_LOGI(TAG, "MQTT Will message configured successfully");
}

void SimCom7600::disableWill() {
    ESP_LOGI(TAG, "=== Disabling MQTT Will Message ===");

    // Set an empty will topic (essentially disabling it)
    ESP_LOGI(TAG, "Clearing will topic...");
    ESP_LOGI(TAG, "%s", sendAT("AT+CMQTTWILLTOPIC=0,0").c_str());

    // Set an empty will message
    ESP_LOGI(TAG, "Clearing will message...");
    ESP_LOGI(TAG, "%s", sendAT("AT+CMQTTWILLMSG=0,0,0").c_str());

    // Verify with client status
    ESP_LOGI(TAG, "Verifying will message status...");
    ESP_LOGI(TAG, "%s", sendAT("AT+CMQTTACCQ?").c_str());

    ESP_LOGI(TAG, "MQTT Will message disabled successfully");
}

esp_err_t SimCom7600::configureMQTTSSL() {
    sendAT("AT+CSSLCFG=\"sslversion\",0,4");
    sendAT("AT+CSSLCFG=\"authmode\",0,2");
    sendAT("AT+CSSLCFG=\"cacert\",0,\"cacert.pem\"");
    sendAT("AT+CSSLCFG=\"clientcert\",0,\"clientcert.pem\"");
    sendAT("AT+CSSLCFG=\"clientkey\",0,\"clientkey.pem\"");
    return ESP_OK;
}

esp_err_t SimCom7600::initMqtt() {
    ESP_LOGI(TAG, "=== Initializing MQTT Connection ===");

    // First check if MQTT is already running and release any existing clients
    ESP_LOGI(TAG, "Checking MQTT status...");
    std::string status_response = sendAT("AT+CMQTTSTART?");
    ESP_LOGI(TAG, "%s", status_response.c_str());

    // Check for existing MQTT clients and release them
    std::string clients_response = sendAT("AT+CMQTTACCQ?");
    ESP_LOGI(TAG, "%s", clients_response.c_str());

    if (clients_response.find("0") != std::string::npos) {
        ESP_LOGI(TAG, "Releasing existing MQTT client...");
        ESP_LOGI(TAG, "%s", sendAT("AT+CMQTTREL=0").c_str());
    }

    // Stop MQTT service if it's running
    if (status_response.find("STARTED") != std::string::npos) {
        ESP_LOGI(TAG, "Stopping MQTT service...");
        ESP_LOGI(TAG, "%s", sendAT("AT+CMQTTSTOP").c_str());
    }

    // Start MQTT service
    ESP_LOGI(TAG, "Starting MQTT service...");
    std::string start_response = sendAT("AT+CMQTTSTART");
    ESP_LOGI(TAG, "%s", start_response.c_str());

    if (start_response.find("ERROR") != std::string::npos &&
        start_response.find("+CMQTTSTART: 23") != std::string::npos) {
        ESP_LOGW(TAG, "MQTT service already started");
    } else if (start_response.find("ERROR") != std::string::npos) {
        ESP_LOGE(TAG, "Failed to start MQTT service");
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // Give MQTT time to initialize

    // Acquire a client
    // Either use the passed client ID or generate a random one
    std::string client_id =
        config_.mqtt_client_id.empty() ? "Device_001" : config_.mqtt_client_id;
    ESP_LOGI(TAG, "Acquiring MQTT client with ID: %s...", client_id.c_str());
    std::string acquire_response =
        sendAT("AT+CMQTTACCQ=0,\"" + client_id + "\",1");
    ESP_LOGI(TAG, "%s", acquire_response.c_str());

    if (acquire_response.find("ERROR") != std::string::npos) {
        ESP_LOGE(TAG, "Failed to acquire MQTT client");
        return ESP_FAIL;
    }

    // Configure SSL for MQTT
    ESP_LOGI(TAG, "Configuring SSL for MQTT...");
    std::string ssl_response = sendAT("AT+CMQTTSSLCFG=0,0");
    ESP_LOGI(TAG, "%s", ssl_response.c_str());

    if (ssl_response.find("ERROR") != std::string::npos) {
        ESP_LOGE(TAG, "Failed to configure SSL for MQTT");
        return ESP_FAIL;
    }

    // Verify SSL configuration is correct
    ESP_LOGI(TAG, "Verifying SSL configuration...");
    std::string ssl_check = sendAT("AT+CSSLCFG?");
    ESP_LOGI(TAG, "%s", ssl_check.c_str());

    // Extract host and port from the MQTT broker URL
    std::string mqtt_url = config_.mqtt_broker;
    std::string host;
    int port = config_.mqtt_port;

    // Remove protocol prefix if present
    if (mqtt_url.find("://") != std::string::npos) {
        host = mqtt_url.substr(mqtt_url.find("://") + 3);
    } else {
        host = mqtt_url;
    }

    // Remove port if included in the URL
    if (host.find(":") != std::string::npos) {
        host = host.substr(0, host.find(":"));
    }

    ESP_LOGI(TAG, "Connecting to MQTT broker: %s on port %d", host.c_str(),
             port);

    // Construct connect command based on available credentials
    std::string connect_cmd;
    if (!config_.mqtt_username.empty() && !config_.mqtt_password.empty()) {
        connect_cmd = "AT+CMQTTCONNECT=0,\"tcp://" + host + ":" +
                      std::to_string(port) + "\",60,1,\"" +
                      config_.mqtt_username + "\",\"" + config_.mqtt_password +
                      "\"";
    } else if (!config_.mqtt_username.empty()) {
        connect_cmd = "AT+CMQTTCONNECT=0,\"tcp://" + host + ":" +
                      std::to_string(port) + "\",60,1,\"" +
                      config_.mqtt_username + "\"";
    } else {
        connect_cmd = "AT+CMQTTCONNECT=0,\"tcp://" + host + ":" +
                      std::to_string(port) + "\",60,1";
    }

    std::string connect_response = sendAT(connect_cmd, 15000);
    ESP_LOGI(TAG, "%s", connect_response.c_str());

    if (connect_response.find("0,0") != std::string::npos) {
        ESP_LOGI(TAG, "Successfully connected to MQTT broker");
        return ESP_OK;
    } else if (connect_response.find("+CMQTTCONNECT: 0,13") !=
               std::string::npos) {
        ESP_LOGE(TAG, "Failed to connect: DNS resolution failed");
        return ESP_FAIL;
    } else if (connect_response.find("+CMQTTCONNECT: 0,26") !=
               std::string::npos) {
        ESP_LOGE(TAG, "Failed to connect: SSL handshake failed");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Failed to connect to MQTT broker");
        return ESP_FAIL;
    }
}

esp_err_t SimCom7600::checkBrokerStatus(int timeout_ms) {
    ESP_LOGI(TAG, "=== Checking MQTT Broker Connection Status ===");

    // Query MQTT client acquisition and connection state
    std::string response = sendAT("AT+CMQTTACCQ?", timeout_ms);
    ESP_LOGI(TAG, "MQTT client status: %s", response.c_str());

    if (response.find("ERROR") != std::string::npos) {
        ESP_LOGE(TAG, "Failed to communicate with SIM7600 module.");
        return ESP_FAIL;
    }

    // Look for the client ID we're using
    std::string client_id =
        config_.mqtt_client_id.empty() ? "Device_001" : config_.mqtt_client_id;

    // Look for a line containing our client ID and extract connection status
    std::string search_str = "+CMQTTACCQ: 0,\"" + client_id + "\"";
    size_t pos = response.find(search_str);

    if (pos != std::string::npos) {
        // Find the position after the client ID
        size_t start_pos = pos + search_str.length();
        // Look for the connection status value (should be after the client ID,
        // separated by a comma)
        if (start_pos < response.length() && response[start_pos] == ',') {
            // The next character after the comma should be the status
            // (1=connected, 0=not connected)
            if (start_pos + 1 < response.length() &&
                response[start_pos + 1] == '1') {
                ESP_LOGI(TAG, "MQTT client is connected to the broker.");
                return ESP_OK;
            } else {
                ESP_LOGW(TAG, "MQTT client is acquired but not connected.");
                return ESP_ERR_TIMEOUT;
            }
        }
    }

    ESP_LOGW(TAG, "MQTT client not acquired or connection status unknown.");
    return ESP_ERR_TIMEOUT;
}

esp_err_t SimCom7600::publishMessage(const std::string& topic,
                                     const std::string& message) {
    ESP_LOGI(TAG, "=== Publish MQTT Message ===");

    if (topic.empty() || message.empty()) {
        ESP_LOGW(TAG, "Topic or message is empty, nothing to publish");
        return ESP_FAIL;
    }

    // Give other tasks a chance to run before starting this long operation
    taskYIELD();

    // === Send Topic ===
    int topic_length = topic.length();
    ESP_LOGI(TAG, "Setting topic: %s", topic.c_str());

    std::string response =
        sendAT("AT+CMQTTTOPIC=0," + std::to_string(topic_length));
    if (response.find(">") == std::string::npos) {
        ESP_LOGE(TAG, "Did not receive '>' prompt for topic");
        return ESP_FAIL;
    }

    // Send actual topic string - this is a fast operation, no need to yield
    uart_.write(topic);

    // Small delay but yield to other tasks while waiting
    for (int i = 0; i < 4; i++) {
        taskYIELD();
        vTaskDelay(
            pdMS_TO_TICKS(50));  // Split the 200ms delay into smaller chunks
    }

    // === Send Payload ===
    int msg_length = message.length();
    ESP_LOGI(TAG, "Setting payload (%d bytes)...", msg_length);

    response = sendAT("AT+CMQTTPAYLOAD=0," + std::to_string(msg_length));
    if (response.find(">") == std::string::npos &&
        response.find("OK") == std::string::npos) {
        ESP_LOGE(TAG, "Did not receive expected prompt/OK for payload");
        return ESP_FAIL;
    }

    // Send actual payload content - split large payloads to allow yielding
    if (msg_length > 1024) {
        // For large payloads, send in chunks to allow task switching
        const size_t CHUNK_SIZE = 512;
        for (size_t offset = 0; offset < message.length();
             offset += CHUNK_SIZE) {
            size_t chunk_len = std::min(CHUNK_SIZE, message.length() - offset);
            uart_.write(message.substr(offset, chunk_len));
            taskYIELD();
        }
    } else {
        // For small payloads, send all at once
        uart_.write(message);
    }

    // Small delay but yield to other tasks while waiting
    for (int i = 0; i < 4; i++) {
        taskYIELD();
        vTaskDelay(
            pdMS_TO_TICKS(50));  // Split the 200ms delay into smaller chunks
    }

    // === Publish ===
    ESP_LOGI(TAG, "Publishing message...");

    // The publish operation can take a while, so we'll make it non-blocking
    // by setting a shorter timeout and checking status separately
    std::string publish_response =
        sendAT("AT+CMQTTPUB=0,1,60,0", 500);  // Reduced timeout

    // Check if we got an immediate response
    if (publish_response.find("+CMQTTPUB: 0,0") != std::string::npos ||
        publish_response.find("OK") != std::string::npos) {
        ESP_LOGI(TAG, "Message published successfully (immediate response)");
        return ESP_OK;
    }

    // If we didn't get an immediate completion, poll for completion
    // This allows other tasks to run during the publish operation
    int retries = 10;
    while (retries-- > 0) {
        // Yield to other tasks while we wait
        taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(100));

        // Check if there's any data indicating publish completion
        std::string status_data;
        if (uart_.isDataAvailable()) {
            uart_.read(status_data);

            if (status_data.find("+CMQTTPUB: 0,0") != std::string::npos ||
                status_data.find("OK") != std::string::npos) {
                ESP_LOGI(TAG,
                         "Message published successfully (delayed response)");
                return ESP_OK;
            } else if (status_data.find("ERROR") != std::string::npos ||
                       status_data.find("+CMQTTPUB: 0,") != std::string::npos) {
                // Found an error response
                ESP_LOGE(TAG, "Failed to publish message: %s",
                         status_data.c_str());
                return ESP_FAIL;
            }
        }
    }

    // If we reach here, the publish operation is taking too long
    // We'll return a timeout but let it continue in the background
    ESP_LOGW(TAG,
             "Publish operation taking longer than expected, continuing in "
             "background");
    return ESP_ERR_TIMEOUT;
}

void SimCom7600::setMqttCredentials(const std::string& username,
                                    const std::string& password,
                                    const std::string& broker, int port,
                                    const std::string& client_id) {
    ESP_LOGI(TAG, "=== Setting MQTT Credentials ===");
    config_.mqtt_username = username;
    config_.mqtt_password = password;
    if (!broker.empty()) {
        config_.mqtt_broker = broker;
    }

    if (port > 0) {
        config_.mqtt_port = port;
    }

    if (!client_id.empty()) {
        config_.mqtt_client_id = client_id;
    }

    ESP_LOGI(
        TAG,
        "MQTT credentials set: Username=%s, Broker=%s, Port=%d, ClientID=%s",
        username.c_str(), config_.mqtt_broker.c_str(), config_.mqtt_port,
        config_.mqtt_client_id.c_str());
}

void SimCom7600::disconnect() {
    ESP_LOGI(TAG, "=== Disconnecting MQTT ===");
    ESP_LOGI(TAG, "%s", sendAT("AT+CMQTTDISC=0,120").c_str());
    ESP_LOGI(TAG, "%s", sendAT("AT+CMQTTREL=0").c_str());
    ESP_LOGI(TAG, "%s", sendAT("AT+CMQTTSTOP").c_str());
    ESP_LOGI(TAG, "MQTT disconnected");
}

void SimCom7600::enableFullDebug() {
    ESP_LOGI(TAG, "=== Enabling Full Debug ===");
    config_.debug_mode = true;
    ESP_LOGI(TAG, "Debug mode enabled");
}

void SimCom7600::showConfig() {
    ESP_LOGI(TAG, "=== Current Configuration ===");
    ESP_LOGI(TAG, "Debug mode: %s",
             config_.debug_mode ? "enabled" : "disabled");
    ESP_LOGI(TAG, "Pretty output: %s",
             config_.pretty_output ? "enabled" : "disabled");
    ESP_LOGI(TAG, "Default timeout: %d ms", config_.default_timeout_ms);
    ESP_LOGI(TAG, "MQTT username: %s",
             config_.mqtt_username.empty() ? "(not set)"
                                           : config_.mqtt_username.c_str());
    ESP_LOGI(TAG, "MQTT password: %s",
             config_.mqtt_password.empty() ? "(not set)" : "********");
}

void SimCom7600::configure(const Config& config) {
    ESP_LOGI(TAG, "=== Updating Configuration ===");
    config_ = config;
    ESP_LOGI(TAG, "Configuration updated");
}

/**
 * @brief Synchronize time using NTP server via the SIM7600 module
 *
 * @param ntpServer NTP server to use for time synchronization
 * @param timezone Timezone string (IANA format, e.g., "America/Bogota")
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_FAIL or ESP_ERR_TIMEOUT on failure
 */
esp_err_t SimCom7600::syncTimeWithNTP(const std::string& ntpServer,
                                      int timezoneOffset, int timeout_ms) {
    ESP_LOGI(TAG, "Configuring NTP sync (Server: %s, TZ Offset: %d)...",
             ntpServer.c_str(), timezoneOffset);

    // First ensure network connection is stable
    std::string netStatus = sendAT("AT+CREG?", 1000);
    if (netStatus.find("+CREG: 0,1") == std::string::npos &&
        netStatus.find("+CREG: 0,5") == std::string::npos) {
        ESP_LOGW(TAG, "Network not registered, time sync may fail");
    }

    // Enable time zone update with longer timeout to ensure it completes
    std::string ctzuRes = sendAT("AT+CTZU=1", 2000);
    if (ctzuRes.find("OK") == std::string::npos) {
        ESP_LOGW(TAG, "Failed to enable time zone updates");
    }

    // Wait between commands to avoid overlap
    vTaskDelay(pdMS_TO_TICKS(500));

    // Optional: Set timezone (15-min intervals)
    int tz_quarters = timezoneOffset * 4;
    std::string tzRes = sendAT("AT+CTZV=" + std::to_string(tz_quarters), 1000);
    if (tzRes.find("ERROR") != std::string::npos) {
        ESP_LOGW(TAG, "AT+CTZV not supported, skipping time zone offset");
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    // Try binding PDP context (required on many firmwares)
    std::string pdpRes = sendAT("AT+CNTPCID=1", 2000);
    if (pdpRes.find("ERROR") != std::string::npos) {
        ESP_LOGW(TAG, "PDP context binding failed, using default context");
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    // Set NTP server with longer timeout
    std::string setResponse = sendAT("AT+CNTP=\"" + ntpServer + "\",0", 5000);
    if (setResponse.find("OK") == std::string::npos) {
        ESP_LOGE(TAG, "Failed to set NTP server");
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Trigger time sync with much longer timeout
    ESP_LOGI(TAG, "Requesting time synchronization...");
    std::string syncResponse = sendAT("AT+CNTP", 20000);

    // Proper response checking
    bool syncSuccess = false;
    if (syncResponse.find("+CNTP: 0") != std::string::npos ||
        syncResponse.find("+CNTP: 1") != std::string::npos) {
        syncSuccess = true;
        ESP_LOGI(TAG, "NTP sync successful");
    } else {
        ESP_LOGW(TAG, "NTP sync response: %s", syncResponse.c_str());
        ESP_LOGW(TAG, "Time sync via CNTP failed, trying fallback");

        // Fallback: enable local time sync via GSM network
        sendAT("AT+CLTS=1", 2000);
        sendAT("AT&W", 2000);  // Save config
        ESP_LOGI(TAG, "CLTS enabled, power cycle required.");
    }

    // Allow more time for the clock to update
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Read updated time with longer timeout
    std::string clkResponse = sendAT("AT+CCLK?", 3000);
    ESP_LOGI(TAG, "Clock read: %s", clkResponse.c_str());

    size_t pos = clkResponse.find("+CCLK: \"");
    if (pos == std::string::npos ||
        clkResponse.find("01/01/01") != std::string::npos) {
        ESP_LOGE(TAG, "Time sync failed: clock not updated");
        return ESP_FAIL;
    }

    std::string timestamp = clkResponse.substr(pos + 8);
    size_t quotePos = timestamp.find("\"");
    if (quotePos != std::string::npos) {
        timestamp = timestamp.substr(0, quotePos);
    }

    // Log the extracted timestamp for debugging
    ESP_LOGI(TAG, "Extracted timestamp: %s", timestamp.c_str());

    struct tm tm_time = {};
    if (!strptime(timestamp.c_str(), "%y/%m/%d,%H:%M:%S", &tm_time)) {
        ESP_LOGE(TAG, "Failed to parse timestamp: %s", timestamp.c_str());
        return ESP_FAIL;
    }

    // Set DST flag to -1 (let system determine automatically)
    tm_time.tm_isdst = -1;

    // Apply timezone offset (converting from UTC to local time)
    time_t rawtime = mktime(&tm_time);
    rawtime += (timezoneOffset * 3600);  // Add timezone offset in seconds

    struct timeval now = {.tv_sec = rawtime, .tv_usec = 0};
    settimeofday(&now, nullptr);

    // Log parsed time components for verification
    char timeStr[64];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &tm_time);
    ESP_LOGI(TAG, "Parsed time: %s (adding offset %d hours)", timeStr,
             timezoneOffset);

    // Log final system time
    time_t currentTime = time(nullptr);
    struct tm* timeinfo = localtime(&currentTime);
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", timeinfo);
    ESP_LOGI(TAG, "ESP32 system time set to: %s", timeStr);

    return ESP_OK;
}

/**
 * @brief Get the current network time from the SIM7600 module
 *
 * @param timeinfo Pointer to a tm structure to store the time information
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t SimCom7600::getNetworkTime(struct tm* timeinfo, int timeout_ms) {
    ESP_LOGI(TAG, "=== Getting Network Time ===");

    if (timeinfo == nullptr) {
        ESP_LOGE(TAG, "Invalid timeinfo pointer");
        return ESP_FAIL;
    }

    // Request current time from the module
    std::string response = sendAT("AT+CCLK?", timeout_ms);

    // Extract time from the response
    // Format: +CCLK: "YY/MM/DD,HH:MM:SS+TZ"
    if (response.find("+CCLK:") != std::string::npos) {
        // Extract the time string
        size_t startPos = response.find("\"");
        size_t endPos = response.find("\"", startPos + 1);

        if (startPos != std::string::npos && endPos != std::string::npos) {
            std::string timeStr =
                response.substr(startPos + 1, endPos - startPos - 1);
            ESP_LOGI(TAG, "Raw time string: %s", timeStr.c_str());

            // Parse the time string
            int year, month, day, hour, minute, second;
            char timezone[10];

            if (sscanf(timeStr.c_str(), "%d/%d/%d,%d:%d:%d%s", &year, &month,
                       &day, &hour, &minute, &second, timezone) >= 6) {
                // Convert YY to YYYY (assuming 20YY)
                year += 2000;

                // Fill the tm structure
                timeinfo->tm_year = year - 1900;  // Years since 1900
                timeinfo->tm_mon = month - 1;     // Months since January [0-11]
                timeinfo->tm_mday = day;          // Day of the month [1-31]
                timeinfo->tm_hour = hour;         // Hours since midnight [0-23]
                timeinfo->tm_min = minute;  // Minutes after the hour [0-59]
                timeinfo->tm_sec = second;  // Seconds after the minute [0-59]

                // Format for logging
                char formatted[64];
                snprintf(formatted, sizeof(formatted),
                         "%04d-%02d-%02d %02d:%02d:%02d", year, month, day,
                         hour, minute, second);

                ESP_LOGI(TAG, "Parsed network time: %s", formatted);
                return ESP_OK;
            } else {
                ESP_LOGE(TAG, "Failed to parse time string: %s",
                         timeStr.c_str());
            }
        }
    }

    ESP_LOGE(TAG, "Failed to get network time: %s", response.c_str());
    return ESP_FAIL;
}

/**
 * @brief Check if the system time has been set
 *
 * @return bool true if time is set, false otherwise
 */
bool SimCom7600::isTimeSet() {
    time_t now = time(nullptr);

    // If time is not set, it will be close to the Unix epoch
    // We consider it set if it's after Jan 1, 2020
    if (now > 1577836800) {  // January 1, 2020 00:00:00 UTC
        return true;
    }

    return false;
}
