/*******************************************************************************
 * espnow_sense.cpp
 *
 * Header file with the declarations of the methods from the ESP-NOW class
 * @Sense AI
 * @version 0.5.0
 * @date 2025-04-13
 * @author daniel@sense-ai.co
 *******************************************************************************/

#include "esp_efuse.h"
#include "esp_mac.h"
#include "esp_phy_init.h"    // For ESP_PHY_CAL_FULL and related definitions
#include "esp_system.h"      // For esp_system functions, including esp_random
#include "esp_system.h"      // For esp_random function
#include "esp_wifi.h"        // For Wi-Fi functions including esp_read_mac
#include "esp_wifi_types.h"  // For ESP_MAC_WIFI_STA definition
#include "espnow_sense.hpp"

static const char* TAG = "EspNow";

EspNow* EspNow::instance = nullptr;

SemaphoreHandle_t EspNow::sendSemaphore =
    xSemaphoreCreateBinary();  // Crear semáforo

EspNow::EspNow(uint8_t maxRetries, uint8_t channel, bool setAsServer)
    : flashStorage_("peersInfo"),
      maxRetries_(maxRetries),
      channel_(channel),
      isServer_(setAsServer) {
    ESP_LOGI(TAG, "Initializing EspNow instance...");
    ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Max retries: %d", maxRetries_);
    ESP_LOGI(TAG, "ESP-NOW channel: %d", channel_);

    retries_ = 0;
    messageSent_ = false;
    newMessage_ = false;
    dataSize_ = 0;
    currentDataSize_ = 0;
    isBuffer_ = false;
    bufferComplete_ = false;
    bufferStatus_ = BufferTransferStatus::NONE;
    bufferTaskHandle_ = nullptr;
}

EspNow::~EspNow() {
}

void EspNow::on_data_sent(const uint8_t* macAddr,
                          esp_now_send_status_t status) {
    // ESP_LOGI(TAG, "Message callback: %s", status == ESP_NOW_SEND_SUCCESS ?
    // "Success" : "Failure");

    if (status == ESP_NOW_SEND_SUCCESS) {
        messageSent_ = true;
    } else {
        messageSent_ = false;
    }
    xSemaphoreGive(sendSemaphore);
}

void EspNow::on_data_recv(const esp_now_recv_info_t* recv_info,
                          const uint8_t* data, int len) {
    // Store the sender's MAC address
    for (int i = 0; i < ESP_NOW_ETH_ALEN; ++i) {
        lastPeerMac_[i] = recv_info->src_addr[i];
    }

    // Check if this is a BUFFER_REQUEST packet - just set the flag, don't
    // process
    if ((data[0] == static_cast<uint8_t>(EspNowPacketType::BUFFER_REQUEST) ||
         data[0] ==
             static_cast<uint8_t>(EspNowPacketType::SAFE_BUFFER_REQUEST)) &&
        len == sizeof(upStreamDataInfo) &&
        bufferStatus_ != BufferTransferStatus::IN_PROGRESS) {
        ESP_LOGI(TAG,
                 "Buffer request received, setting flag for user handling");
        isBuffer_ = true;
        bufferComplete_ = false;  // Reset the buffer complete flag
        bufferStatus_ =
            BufferTransferStatus::IN_PROGRESS;  // Set the status to IN_PROGRESS

        // Store the packet in the standard buffer for the user to access
        for (int i = 0; i < len; ++i) {
            data_[i] = data[i];
        }
        currentDataSize_ = len;
        newMessage_ = true;

        return;
    }

    // Check if this is a BUFFER_CONTENT packet - process automatically
    if (data[0] == static_cast<uint8_t>(EspNowPacketType::BUFFER_CONTENT) &&
        bufferStatus_ == BufferTransferStatus::IN_PROGRESS) {
        // Create a copy of the data for queueing
        uint8_t* dataCopy = (uint8_t*)malloc(len);
        if (dataCopy != nullptr) {
            memcpy(dataCopy, data, len);

            // Queue data if queue is available
            if (dataQueue_ != nullptr) {
                struct {
                    uint8_t* data;
                    size_t len;
                } queueItem;

                queueItem.data = dataCopy;
                queueItem.len = len;

                // Send to queue with timeout - if fails, free memory and
                // process directly
                if (xQueueSend(dataQueue_, &queueItem, pdMS_TO_TICKS(50)) !=
                    pdTRUE) {
                    ESP_LOGW(
                        TAG,
                        "Queue full or not available, processing directly");
                    free(dataCopy);
                    processDataTransferPacket(data, len);
                }
            } else {
                // No queue available, process directly and free memory
                ESP_LOGW(TAG, "Queue not available, processing directly");
                processDataTransferPacket(data, len);
                free(dataCopy);
            }
        } else {
            ESP_LOGE(TAG, "Failed to allocate memory for data copy");
            processDataTransferPacket(data, len);
        }

        return;
    }

    // For normal packets, continue with the existing handling
    for (int i = 0; i < len; ++i) {
        data_[i] = data[i];
    }

    currentDataSize_ = len;
    newMessage_ = true;
}

void EspNow::on_data_sent_static(const uint8_t* macAddr,
                                 esp_now_send_status_t status) {
    if (instance == nullptr) {
        ESP_LOGE(TAG, "Instance is nullptr. Cannot call on_data_sent.");
        return;
    }
    instance->on_data_sent(macAddr, status);
}

void EspNow::on_data_static(const esp_now_recv_info_t* recv_info,
                            const uint8_t* data, int len) {
    if (instance != nullptr) {
        instance->on_data_recv(recv_info, data, len);
    }
}

esp_err_t EspNow::initialize() {
    // Initialize flash storage
    esp_err_t ret = flashStorage_.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize flash storage: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // Ensure Wi-Fi event loop is initialized
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create default event loop: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // Initialize Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Wi-Fi: %s", esp_err_to_name(ret));
        return ret;
    }

    if (isServer_) {
        // Set the device as a Wi-Fi station
        ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set Wi-Fi mode: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        // Set the device as a Wi-Fi station
        ret = esp_wifi_set_mode(WIFI_MODE_STA);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set Wi-Fi mode: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set the ESP-NOW channel
    ret = esp_wifi_set_channel(channel_, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ESP-NOW channel: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // Initialize ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register callbacks
    ret = esp_now_register_send_cb(on_data_sent_static);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register send callback: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ret = esp_now_register_recv_cb(on_data_static);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register receive callback for node: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    if (wakeWindow_ > 0 && wakeInterval_ > 0) {
        ESP_ERROR_CHECK(
            esp_wifi_connectionless_module_set_wake_interval(wakeInterval_));
        ESP_ERROR_CHECK(esp_now_set_wake_window(wakeWindow_));
    }

    ESP_LOGI(TAG, "Initialization successful.");
    instance = this;  // Set the static instance pointer
    return ESP_OK;
}

esp_err_t EspNow::configPowerSaving(uint16_t wakeInterval,
                                    uint16_t wakeWindow) {
    static const char* TAG = "EspNow::configPowerSaving";

    if (wakeInterval < 1 || wakeWindow < 1) {
        ESP_LOGE(TAG,
                 "Invalid parameters: wake interval and window must be "
                 "greater than 0");
        return ESP_ERR_INVALID_ARG;
    }

    if (wakeWindow > wakeInterval) {
        ESP_LOGW(TAG,
                 "Wake window (%u) is larger than wake interval (%u), this "
                 "might cause unexpected behavior",
                 wakeWindow, wakeInterval);
    }

    // Store the values in member variables
    wakeInterval_ = wakeInterval;
    wakeWindow_ = wakeWindow;

    ESP_LOGI(TAG, "Power saving configured: wake interval=%u, wake window=%u",
             wakeInterval_, wakeWindow_);

    // If ESP-NOW is already initialized, apply settings immediately
    esp_err_t err = ESP_OK;
    if (instance == this) {
        ESP_LOGI(TAG, "Applying power saving settings to active connection");
        err = esp_wifi_connectionless_module_set_wake_interval(wakeInterval_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set wake interval: %s",
                     esp_err_to_name(err));
            return err;
        }

        err = esp_now_set_wake_window(wakeWindow_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set wake window: %s",
                     esp_err_to_name(err));
            return err;
        }
    }

    return err;
}

esp_err_t EspNow::addPeer(uint8_t* peerMac) {
    esp_now_peer_info_t peer_info = {};

    memcpy(peer_info.peer_addr, peerMac, 6);
    peer_info.channel = channel_;   // Default channel
    peer_info.encrypt = false;      // No encryption
    peer_info.ifidx = WIFI_IF_STA;  // or WIFI_IF_AP depending on your setup

    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Peer added successfully: %02X:%02X:%02X:%02X:%02X:%02X",
                 peerMac[0], peerMac[1], peerMac[2], peerMac[3], peerMac[4],
                 peerMac[5]);
    } else if (ret == ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "Peer already exists: %02X:%02X:%02X:%02X:%02X:%02X",
                 peerMac[0], peerMac[1], peerMac[2], peerMac[3], peerMac[4],
                 peerMac[5]);
        // Modify the peer if it already exists
        ret = esp_now_mod_peer(&peer_info);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to modify existing peer: %s",
                     esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "Error adding peer: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t EspNow::getReceivedMessage(uint8_t* _message,
                                     size_t* _receivedLength) {
    if (newMessage_) {
        if (_message == nullptr || _receivedLength == nullptr) {
            return ESP_ERR_INVALID_ARG;
        }

        size_t dataSize = currentDataSize_;

        if (dataSize > maxDataPacketSize_) {
            return ESP_ERR_INVALID_SIZE;
        }

        for (int i = 0; i < dataSize; ++i) {
            _message[i] = data_[i];
        }

        *_receivedLength = dataSize;
        newMessage_ = false;
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t EspNow::getLastPeerMac(uint8_t* _mac) {
    if (_mac == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t sumMac = 0;
    for (int i = 0; i < 6; ++i) {
        sumMac += lastPeerMac_[i];
    }

    if (sumMac == 0) {
        return ESP_ERR_INVALID_MAC;
    }

    for (int i = 0; i < 6; ++i) {
        _mac[i] = lastPeerMac_[i];
    }
    return ESP_OK;
}

esp_err_t EspNow::sendData() {
    return sendEspNowData(
        serverMac_);  // Call the overloaded function with serverMac_
}

esp_err_t EspNow::sendEspNowData(const uint8_t* mac) {
    esp_err_t ret;

    xSemaphoreTake(sendSemaphore, 0);

    retries_ = 0;          // Reset retries counter
    messageSent_ = false;  // Reset message sent flag

    while (retries_ < maxRetries_ && !messageSent_) {
        ++retries_;
        ret = esp_now_send(mac, dataBuffer_, dataSize_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error initiating send: %s", esp_err_to_name(ret));
            break;
        }

        if (xSemaphoreTake(sendSemaphore, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Timeout waiting for message to be sent.");
        }
    }

    if (!messageSent_) {
        ESP_LOGE(TAG, "Failed to send message after %d retries", retries_);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Message sent. Rt: %d", retries_);
    return ESP_OK;
}

esp_err_t EspNow::testConnection(const uint8_t* mac, uint64_t timeStamp,
                                 uint32_t responseTimeoutMs) {
    esp_err_t ret;

    // Define a simple struct to hold test data
    struct TestConnectionData {
        uint64_t timestamp;
        uint8_t retryCount;
    };

    TestConnectionData testData;

    xSemaphoreTake(sendSemaphore, 0);

    retries_ = 0;          // Reset retries counter
    messageSent_ = false;  // Reset message sent flag
    newMessage_ =
        false;  // Reset new message flag to ensure we detect only new responses

    while (retries_ < maxRetries_ && !messageSent_) {
        // Update the test data with current values
        testData.timestamp = timeStamp;
        testData.retryCount = retries_;

        ++retries_;

        // Send the test data structure
        ret =
            esp_now_send(mac, (uint8_t*)&testData, sizeof(TestConnectionData));

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error initiating connection test: %s",
                     esp_err_to_name(ret));
            break;
        }

        if (xSemaphoreTake(sendSemaphore, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Timeout waiting for test message to be sent.");
        }
    }

    if (!messageSent_) {
        ESP_LOGE(TAG, "Failed to test connection after %d retries", retries_);
        return ESP_FAIL;
    }

    uint32_t startTime = esp_log_timestamp();
    ESP_LOGI(TAG, "Message sent successfully. Waiting for response...");

    while ((esp_log_timestamp() - startTime) < responseTimeoutMs) {
        if (newMessage_) {
            ESP_LOGI(TAG, "Response received within timeout period");
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
    }

    ESP_LOGW(TAG, "No response received within %lu ms timeout",
             responseTimeoutMs);
    return ESP_ERR_TIMEOUT;
}

esp_err_t EspNow::receiveData() {
    return receiveData(
        receivedData_);  // Call the overloaded function with data_
}

esp_err_t EspNow::receiveData(std::string& receivedData) {
    if (newMessage_) {
        // Copy the received data into the provided std::string
        receivedData.assign(reinterpret_cast<char*>(data_), currentDataSize_);
        newMessage_ = false;  // Reset the flag after reading the data
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;  // No new message available
}

bool EspNow::isMacDuplicate(const uint8_t* _newMac) {
    for (uint8_t i = 0; i < macCount_; ++i) {
        if (memcmp(macAddresses_[i], _newMac, 6) == 0) {
            ESP_LOGD(TAG, "Duplicate MAC found: %02X:%02X:%02X:%02X:%02X:%02X",
                     _newMac[0], _newMac[1], _newMac[2], _newMac[3], _newMac[4],
                     _newMac[5]);
            return true;
        }
    }
    return false;
}

bool EspNow::isMacStoredInNvs(const char* key) {
    uint8_t tempMac[6];
    size_t size = sizeof(tempMac);
    return flashStorage_.get(key, tempMac, size) == ESP_OK;
}

esp_err_t EspNow::saveMacToNvs(const uint8_t* mac) {
    if (isMacStoredInNvs("server_mac")) {
        ESP_LOGD(TAG,
                 "MAC already stored in NVS: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return ESP_OK;  // No need to store again
    }

    esp_err_t err = flashStorage_.put("server_mac", mac, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving MAC to NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Commit changes
    err = flashStorage_.commit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(err));
    }

    return err;
}

void EspNow::resetStoredMacs() {
    ESP_LOGW(TAG, "Maximum peers reached. Resetting stored MAC addresses.");

    for (uint8_t i = 0; i < MAX_PEERS; ++i) {
        char key[10];
        snprintf(key, sizeof(key), "mac_%d", i);
        flashStorage_.remove(key);
    }

    macCount_ = 0;
    flashStorage_.put("mac_count", macCount_);
    flashStorage_.commit();
}

esp_err_t EspNow::storeMacInNvs(uint8_t index, const uint8_t* mac) {
    char key[10];
    snprintf(key, sizeof(key), "mac_%d", index);

    esp_err_t err = flashStorage_.put(key, mac, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving MAC to NVS: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t EspNow::savePeerToNvs(const uint8_t* mac) {
    if (isMacDuplicate(mac)) {
        ESP_LOGD(TAG, "MAC already exists: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return ESP_OK;
    }

    // Determine where to write:
    uint8_t index = (macCount_ < MAX_PEERS) ? macCount_ : writeIndex_;

    // Save new MAC in memory (overwrite if necessary)
    memcpy(macAddresses_[index], mac, 6);

    // Store in NVS
    esp_err_t err = storeMacInNvs(index, mac);
    if (err != ESP_OK) {
        return err;
    }

    // Increment macCount_ up to MAX_PEERS
    if (macCount_ < MAX_PEERS) {
        ++macCount_;
        err = flashStorage_.put("mac_count", macCount_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error updating MAC count in NVS: %s",
                     esp_err_to_name(err));
            return err;
        }
    }

    // Update writeIndex_ for circular overwriting
    writeIndex_ = (writeIndex_ + 1) % MAX_PEERS;
    err = flashStorage_.put("write_index", writeIndex_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error updating write index in NVS: %s",
                 esp_err_to_name(err));
        return err;
    }

    // Commit changes
    err = flashStorage_.commit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t EspNow::readMacFromNvs(uint8_t* _mac) {
    if (_mac == nullptr) {
        ESP_LOGE(TAG, "Invalid argument: MAC pointer is null.");
        return ESP_ERR_INVALID_ARG;
    }

    size_t required_size = 6;  // Size of the MAC address
    esp_err_t err = flashStorage_.get(
        "server_mac", _mac, required_size);  // Use FlashStorage to read the MAC

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "MAC not found in NVS.");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading MAC from NVS: %s", esp_err_to_name(err));
    } /*else {
        ESP_LOGI(TAG, "Server MAC from NVS:
    %02X:%02X:%02X:%02X:%02X:%02X",_mac[0], _mac[1], _mac[2], _mac[3], _mac[4],
    _mac[5]);

    }*/

    return err;
}

esp_err_t EspNow::setServer(const uint8_t* mac) {
    esp_err_t err;

    if (mac != nullptr) {
        // If a MAC is provided, save it to NVS
        memcpy(serverMac_, mac, 6);  // Copy the provided MAC to serverMac_
        err = saveMacToNvs(mac);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save provided MAC to NVS: %s",
                     esp_err_to_name(err));
            return err;
        }
    } else {
        // If no MAC is provided, read it from NVS
        err = readMacFromNvs(serverMac_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read MAC from NVS: %s",
                     esp_err_to_name(err));
            return err;
        }
    }

    // Add the server MAC as a peer using the addPeer method
    err = addPeer(serverMac_);

    return err;
}

esp_err_t EspNow::loadMacsFromNvs() {
    esp_err_t err = flashStorage_.get(
        "mac_count", macCount_);  // Retrieve the count of stored MACs
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No MAC count found in NVS. Initializing to 0.");
        macCount_ = 0;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading MAC count from NVS: %s",
                 esp_err_to_name(err));
        return err;
    }

    uint8_t uniqueMacAddresses[MAX_PEERS][6];
    uint8_t uniqueCount = 0;

    for (uint8_t i = 0; i < macCount_; ++i) {
        char key[10];
        snprintf(key, sizeof(key), "mac_%d", i);
        size_t required_size = 6;

        err = flashStorage_.get(key, macAddresses_[i],
                                required_size);  // Retrieve each MAC address
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Skipping invalid MAC at index %d: %s", i,
                     esp_err_to_name(err));
            continue;  // Skip this MAC instead of breaking the loop
        }

        // Check for duplicates before storing
        bool exists = false;
        for (uint8_t j = 0; j < uniqueCount; ++j) {
            if (memcmp(uniqueMacAddresses[j], macAddresses_[i], 6) == 0) {
                exists = true;
                break;
            }
        }

        if (!exists) {
            memcpy(uniqueMacAddresses[uniqueCount], macAddresses_[i], 6);
            ++uniqueCount;
            ESP_LOGI(TAG, "Unique MAC %d: %02X:%02X:%02X:%02X:%02X:%02X",
                     uniqueCount - 1, macAddresses_[i][0], macAddresses_[i][1],
                     macAddresses_[i][2], macAddresses_[i][3],
                     macAddresses_[i][4], macAddresses_[i][5]);
        }
    }

    // Update stored MAC addresses and count
    memcpy(macAddresses_, uniqueMacAddresses,
           uniqueCount * sizeof(uniqueMacAddresses[0]));
    macCount_ = uniqueCount;

    // Add peers to ESP-NOW
    for (uint8_t i = 0; i < macCount_; ++i) {
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, macAddresses_[i], 6);
        peer_info.channel = channel_;
        peer_info.encrypt = false;
        peer_info.ifidx = WIFI_IF_STA;

        err = esp_now_add_peer(&peer_info);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Peer added: %02X:%02X:%02X:%02X:%02X:%02X",
                     peer_info.peer_addr[0], peer_info.peer_addr[1],
                     peer_info.peer_addr[2], peer_info.peer_addr[3],
                     peer_info.peer_addr[4], peer_info.peer_addr[5]);
        } else if (err == ESP_ERR_ESPNOW_EXIST) {
            ESP_LOGW(TAG, "Peer already exists: %02X:%02X:%02X:%02X:%02X:%02X",
                     peer_info.peer_addr[0], peer_info.peer_addr[1],
                     peer_info.peer_addr[2], peer_info.peer_addr[3],
                     peer_info.peer_addr[4], peer_info.peer_addr[5]);
            esp_now_mod_peer(&peer_info);
        } else {
            ESP_LOGE(TAG, "Error adding peer: %s", esp_err_to_name(err));
        }
    }

    return ESP_OK;
}

esp_err_t EspNow::connect() {
    esp_err_t ret;

    // Initialize the ESP-NOW system
    ret = initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set the server MAC address
    ret = setServer(nullptr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set server MAC: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ESP-NOW connection established successfully.");
    return ESP_OK;
}

bool EspNow::isLikelyString(const uint8_t* data, size_t length) const {
    // Check if all bytes are printable ASCII characters or whitespace
    for (size_t i = 0; i < length; ++i) {
        if (!(isprint(data[i]) || isspace(data[i]))) {
            return false;  // Non-printable character found
        }
    }
    return true;  // All characters are printable
}

long EspNow::getUnixTime(void) {
    time_t now;
    time(&now);
    return (int64_t)now;
}

esp_err_t EspNow::sendMillis(uint8_t* _clientMillis) {
    int64_t unix_time = getUnixTime();
    uint8_t data[sizeof(unix_time)];
    memcpy(data, &unix_time, sizeof(unix_time));

    ESP_LOGI(TAG, "Sending Unix time: %lld", unix_time);

    esp_err_t ret = esp_now_send(_clientMillis, data, sizeof(data));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error sending message: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    return ESP_OK;
}

/*esp_err_t EspNow::checkUplinkMode(uint8_t* _clientUplink) {

    uint8_t confirmMessage[] = {MESSAGE_TYPE_CONFIRMATION};
    esp_err_t ret = esp_now_send(_clientUplink, confirmMessage,
                            sizeof(confirmMessage));

    if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error sending message: %s", esp_err_to_name(ret));
    return ESP_FAIL;
    }

    return ESP_OK;
}*/

esp_err_t EspNow::disconnect() {
    esp_err_t ret = esp_now_deinit();
    flashStorage_.end();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error deinitializing ESP-NOW: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "ESP-NOW successfully deinitialized");
    }
    if (instance == this) {
        instance = nullptr;  // Reset the static instance pointer
    }
    return ret;
}

bool EspNow::sendFastBuffer(const char* buffer, size_t size,
                            const uint8_t* mac) {
    static const char* TAG = "EspNow::sendFastBuffer";

    // Use serverMac_ if no MAC address provided
    const uint8_t* targetMac = (mac != nullptr) ? mac : serverMac_;

    // Calculate number of chunks needed based on a reasonable chunk size
    // We use a smaller chunk size (200) than the max packet size to ensure fit
    // with headers
    constexpr size_t maxChunkSize = 200;
    uint16_t totalChunks = (size + maxChunkSize - 1) / maxChunkSize;

    // Prepare and send upStreamDataInfo packet
    upStreamDataInfo dataInfo;
    dataInfo.packet_type = EspNowPacketType::BUFFER_REQUEST;
    dataInfo.total_chunks = totalChunks;
    dataInfo.data_size = size;

    ESP_LOGI(TAG, "Starting transfer of %zu bytes in %hu chunks", size,
             totalChunks);

    // Send initial packet and wait for acknowledgment
    esp_err_t ret = sendData(targetMac, dataInfo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send initial packet: %s",
                 esp_err_to_name(ret));
        return false;
    }

    // Wait for TRANSFER_ACK (up to 1000ms)
    uint32_t startTime = esp_log_timestamp();
    bool ackReceived = false;

    while ((esp_log_timestamp() - startTime) < 1000) {
        if (hasNewMessage()) {
            uint8_t responseBuffer[1];
            size_t responseLength;

            if (getReceivedMessage(responseBuffer, &responseLength) == ESP_OK) {
                if (responseLength == 1 &&
                    responseBuffer[0] ==
                        static_cast<uint8_t>(EspNowPacketType::TRANSFER_ACK)) {
                    ESP_LOGI(TAG, "Received initial ACK");
                    ackReceived = true;
                    break;
                } else if (responseBuffer[0] ==
                           static_cast<uint8_t>(
                               EspNowPacketType::TRANSFER_NACK)) {
                    ESP_LOGW(TAG, "Received NACK for initial packet");
                    return false;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
    }

    if (!ackReceived) {
        ESP_LOGE(TAG, "Did not receive ACK for initial packet");
        return false;
    }

    // Send data chunks with flow control
    const int chunkBatchSize = 5;  // Send batches of 5 chunks at a time
    int batchCount = 0;

    for (uint16_t i = 0; i < totalChunks; i++) {
        size_t offset = i * maxChunkSize;
        size_t chunkSize =
            (offset + maxChunkSize <= size) ? maxChunkSize : (size - offset);

        // Prepare chunk structure
        upStreamData chunkData;
        chunkData.packet_type = EspNowPacketType::BUFFER_CONTENT;
        chunkData.chunkNumber = i;
        chunkData.chunkSize = chunkSize;

        // Combine header and data
        uint8_t tempBuffer[sizeof(upStreamData) + maxChunkSize];
        memcpy(tempBuffer, &chunkData, sizeof(upStreamData));
        memcpy(tempBuffer + sizeof(upStreamData), buffer + offset, chunkSize);

        dataSize_ = sizeof(upStreamData) + chunkSize;
        memcpy(dataBuffer_, tempBuffer, dataSize_);

        // Limit logging to avoid overwhelming the console
        if (i % 10 == 0 || i == totalChunks - 1) {
            ESP_LOGI(TAG, "Sending chunk %u/%hu, size: %zu bytes", i + 1,
                     totalChunks, chunkSize);
        }

        ret = sendEspNowData(targetMac);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send chunk %u: %s", i,
                     esp_err_to_name(ret));
            return false;
        }

        // Flow control: add brief delay after each chunk batch to avoid
        // overwhelming receiver
        batchCount++;
        if (batchCount >= chunkBatchSize) {
            vTaskDelay(pdMS_TO_TICKS(5));  // 5ms delay after batch
            batchCount = 0;
        }
    }

    // Wait for final acknowledgment
    startTime = esp_log_timestamp();
    ackReceived = false;

    while ((esp_log_timestamp() - startTime) <
           1000) {  // Increased timeout to 1 second for larger transfers
        if (hasNewMessage()) {
            uint8_t responseBuffer[1];
            size_t responseLength;

            if (getReceivedMessage(responseBuffer, &responseLength) == ESP_OK) {
                if (responseLength == 1) {
                    if (responseBuffer[0] ==
                        static_cast<uint8_t>(EspNowPacketType::TRANSFER_ACK)) {
                        ESP_LOGI(TAG, "Transfer completed successfully");
                        ackReceived = true;
                        break;
                    } else if (responseBuffer[0] ==
                               static_cast<uint8_t>(
                                   EspNowPacketType::TRANSFER_NACK)) {
                        ESP_LOGW(TAG, "Transfer rejected by receiver");
                        return false;
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
    }

    if (!ackReceived) {
        ESP_LOGW(TAG, "Did not receive final ACK, but chunks were sent");
    }

    return ackReceived;
}

bool EspNow::sendFastBuffer(const std::string& data, const uint8_t* mac) {
    // Delegate to the char* version to avoid code duplication
    return sendFastBuffer(data.c_str(), data.size(), mac);
}

bool EspNow::sendSafeBuffer(const char* buffer, size_t size,
                            const uint8_t* mac) {
    static const char* TAG = "EspNow::sendSafeBuffer";

    // Use serverMac_ if no MAC address provided
    const uint8_t* targetMac = (mac != nullptr) ? mac : serverMac_;

    // Calculate number of chunks needed based on a reasonable chunk size
    constexpr size_t maxChunkSize = 150;
    uint16_t totalChunks = (size + maxChunkSize - 1) / maxChunkSize;

    // Prepare and send upStreamDataInfo packet
    upStreamDataInfo dataInfo;
    dataInfo.packet_type =
        EspNowPacketType::SAFE_BUFFER_REQUEST;  // Use safe buffer type
    dataInfo.total_chunks = totalChunks;
    dataInfo.data_size = size;

    ESP_LOGI(TAG, "Starting safe transfer of %zu bytes in %hu chunks", size,
             totalChunks);

    // Send initial packet and wait for acknowledgment
    esp_err_t ret = sendData(targetMac, dataInfo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send initial packet: %s",
                 esp_err_to_name(ret));
        return false;
    }

    // Wait for TRANSFER_ACK with a longer timeout (2000ms) for extra
    // reliability
    uint32_t startTime = esp_log_timestamp();
    bool ackReceived = false;

    while ((esp_log_timestamp() - startTime) < 2000) {
        if (hasNewMessage()) {
            uint8_t responseBuffer[1];
            size_t responseLength;

            if (getReceivedMessage(responseBuffer, &responseLength) == ESP_OK) {
                if (responseLength == 1 &&
                    responseBuffer[0] ==
                        static_cast<uint8_t>(EspNowPacketType::TRANSFER_ACK)) {
                    ESP_LOGI(TAG, "Received initial ACK for safe transfer");
                    ackReceived = true;
                    break;
                } else if (responseBuffer[0] ==
                           static_cast<uint8_t>(
                               EspNowPacketType::TRANSFER_NACK)) {
                    ESP_LOGW(TAG, "Received NACK for initial packet");
                    return false;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
    }

    if (!ackReceived) {
        ESP_LOGE(TAG, "Did not receive ACK for initial packet in safe mode");
        return false;
    }

    // Constants for safe buffer transfer
    const uint32_t CHUNK_ACK_TIMEOUT_MS = 500;  // Longer timeout per chunk
    const uint32_t MAX_RETRIES_PER_CHUNK = 5;   // Maximum retries per chunk
    const uint32_t MAX_TOTAL_TIMEOUT_MS =
        60000;  // Maximum 60 seconds for entire transfer

    uint32_t totalStartTime = esp_log_timestamp();

    // Send data chunks one at a time, waiting for ACK after each
    for (uint16_t i = 0; i < totalChunks; i++) {
        size_t offset = i * maxChunkSize;
        size_t chunkSize =
            (offset + maxChunkSize <= size) ? maxChunkSize : (size - offset);
        bool chunkSent = false;
        uint8_t retryCount = 0;

        // Try to send this chunk until successful or max retries
        while (!chunkSent && retryCount < MAX_RETRIES_PER_CHUNK) {
            // Check total transfer time
            if ((esp_log_timestamp() - totalStartTime) > MAX_TOTAL_TIMEOUT_MS) {
                ESP_LOGE(TAG, "Total transfer timeout exceeded");
                return false;
            }

            // Prepare chunk structure
            safeUpStreamData chunkData;
            chunkData.packet_type = EspNowPacketType::BUFFER_CONTENT;
            chunkData.chunkNumber = i;
            chunkData.chunkSize = chunkSize;

            // Combine header and data in a temporary buffer for checksum
            // calculation
            uint8_t tempBuffer[sizeof(safeUpStreamData) + maxChunkSize];
            memcpy(tempBuffer + sizeof(safeUpStreamData), buffer + offset,
                   chunkSize);

            // Calculate checksum for this chunk's data
            chunkData.checkSum = calculateChecksum(
                reinterpret_cast<const uint8_t*>(buffer + offset), chunkSize);

            // Copy header with checksum to temp buffer
            memcpy(tempBuffer, &chunkData, sizeof(safeUpStreamData));

            // Set data size and copy to send buffer
            dataSize_ = sizeof(safeUpStreamData) + chunkSize;
            memcpy(dataBuffer_, tempBuffer, dataSize_);

            ESP_LOGI(TAG,
                     "Sending chunk %u/%u with checksum 0x%lu, size: %zu bytes "
                     "(try %u)",
                     i + 1, totalChunks, chunkData.checkSum, chunkSize,
                     retryCount + 1);

            ret = sendEspNowData(targetMac);

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send chunk %u: %s", i,
                         esp_err_to_name(ret));
                retryCount++;
                vTaskDelay(pdMS_TO_TICKS(10));  // Short delay before retry
                continue;
            }

            // Wait for ACK for this specific chunk
            uint32_t chunkStartTime = esp_log_timestamp();
            ackReceived = false;

            while ((esp_log_timestamp() - chunkStartTime) <
                   CHUNK_ACK_TIMEOUT_MS) {
                if (hasNewMessage()) {
                    uint8_t responseBuffer[1];
                    size_t responseLength;

                    if (getReceivedMessage(responseBuffer, &responseLength) ==
                        ESP_OK) {
                        if (responseLength == 1) {
                            if (responseBuffer[0] ==
                                static_cast<uint8_t>(
                                    EspNowPacketType::TRANSFER_ACK)) {
                                ESP_LOGI(TAG, "Chunk %u ACK received", i + 1);
                                ackReceived = true;
                                chunkSent = true;
                                break;
                            } else if (responseBuffer[0] ==
                                       static_cast<uint8_t>(
                                           EspNowPacketType::TRANSFER_NACK)) {
                                ESP_LOGW(TAG,
                                         "Chunk %u NACK received, will retry",
                                         i + 1);
                                break;  // Break from ACK wait loop to retry
                            }
                        }
                    }
                }
                vTaskDelay(
                    pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
            }

            if (!ackReceived) {
                ESP_LOGW(TAG, "Timeout waiting for chunk %u ACK, will retry",
                         i + 1);
                retryCount++;
            }
        }

        // If we couldn't send this chunk after max retries, abort the entire
        // transfer
        if (!chunkSent) {
            ESP_LOGE(
                TAG,
                "Failed to send chunk %u after %lu retries, aborting transfer",
                i + 1, MAX_RETRIES_PER_CHUNK);
            return false;
        }

        // Add a small delay between chunks for receiver processing
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // All chunks sent successfully
    ESP_LOGI(TAG, "All %hu chunks successfully sent and acknowledged",
             totalChunks);

    // Wait for final acknowledgment
    uint32_t finalAckStartTime = esp_log_timestamp();
    ackReceived = false;

    while ((esp_log_timestamp() - finalAckStartTime) < 1000) {
        if (hasNewMessage()) {
            uint8_t responseBuffer[1];
            size_t responseLength;

            if (getReceivedMessage(responseBuffer, &responseLength) == ESP_OK) {
                if (responseLength == 1 &&
                    responseBuffer[0] ==
                        static_cast<uint8_t>(EspNowPacketType::TRANSFER_ACK)) {
                    ESP_LOGI(TAG, "Final transfer ACK received");
                    ackReceived = true;
                    break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
    }

    if (!ackReceived) {
        ESP_LOGW(TAG,
                 "Final ACK not received, but all chunks were acknowledged");
    }

    // Total time
    ESP_LOGI(TAG, "Total safe transfer time: %lu ms",
             esp_log_timestamp() - totalStartTime);

    return true;
}

bool EspNow::sendSafeBuffer(const std::string& data, const uint8_t* mac) {
    // Delegate to the char* version to avoid code duplication
    return sendSafeBuffer(data.c_str(), data.size(), mac);
}

bool EspNow::sendBuffer(const char* buffer, size_t size, const uint8_t* mac) {
    static const char* TAG = "EspNow::sendBuffer";

    // Use serverMac_ if no MAC address provided
    const uint8_t* targetMac = (mac != nullptr) ? mac : serverMac_;

    // Calculate number of chunks needed based on a reasonable chunk size
    // We use a smaller chunk size (200) than the max packet size to ensure fit
    // with headers
    constexpr size_t maxChunkSize = 200;
    uint16_t totalChunks = (size + maxChunkSize - 1) / maxChunkSize;

    // Prepare and send upStreamDataInfo packet
    upStreamDataInfo dataInfo;
    dataInfo.packet_type = EspNowPacketType::BUFFER_REQUEST;
    dataInfo.total_chunks = totalChunks;
    dataInfo.data_size = size;

    ESP_LOGI(TAG, "Starting reliable transfer of %zu bytes in %hu chunks", size,
             totalChunks);

    // Send initial packet and wait for acknowledgment
    esp_err_t ret = sendData(targetMac, dataInfo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send initial packet: %s",
                 esp_err_to_name(ret));
        return false;
    }

    // Wait for TRANSFER_ACK (up to 1000ms)
    uint32_t startTime = esp_log_timestamp();
    bool ackReceived = false;

    while ((esp_log_timestamp() - startTime) < 1000) {
        if (hasNewMessage()) {
            uint8_t responseBuffer[1];
            size_t responseLength;

            if (getReceivedMessage(responseBuffer, &responseLength) == ESP_OK) {
                if (responseLength == 1 &&
                    responseBuffer[0] ==
                        static_cast<uint8_t>(EspNowPacketType::TRANSFER_ACK)) {
                    ESP_LOGI(TAG, "Received initial ACK");
                    ackReceived = true;
                    break;
                } else if (responseBuffer[0] ==
                           static_cast<uint8_t>(
                               EspNowPacketType::TRANSFER_NACK)) {
                    ESP_LOGW(TAG, "Received NACK for initial packet");
                    return false;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
    }

    if (!ackReceived) {
        ESP_LOGE(TAG, "Did not receive ACK for initial packet");
        return false;
    }

    // Send data chunks with flow control
    const int chunkBatchSize = 5;  // Send batches of 5 chunks at a time
    int batchCount = 0;

    // Array to track which chunks have been sent successfully
    bool* chunkSent = (bool*)calloc(totalChunks, sizeof(bool));
    if (chunkSent == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate memory for chunk tracking");
        return false;
    }

    // Send all chunks initially
    for (uint16_t i = 0; i < totalChunks; i++) {
        size_t offset = i * maxChunkSize;
        size_t chunkSize =
            (offset + maxChunkSize <= size) ? maxChunkSize : (size - offset);

        // Prepare chunk structure
        upStreamData chunkData;
        chunkData.packet_type = EspNowPacketType::BUFFER_CONTENT;
        chunkData.chunkNumber = i;
        chunkData.chunkSize = chunkSize;

        // Combine header and data
        uint8_t tempBuffer[sizeof(upStreamData) + maxChunkSize];
        memcpy(tempBuffer, &chunkData, sizeof(upStreamData));
        memcpy(tempBuffer + sizeof(upStreamData), buffer + offset, chunkSize);

        dataSize_ = sizeof(upStreamData) + chunkSize;
        memcpy(dataBuffer_, tempBuffer, dataSize_);

        // Limit logging to avoid overwhelming the console
        if (i % 10 == 0 || i == totalChunks - 1) {
            ESP_LOGI(TAG, "Sending chunk %u/%hu, size: %zu bytes", i + 1,
                     totalChunks, chunkSize);
        }

        ret = sendEspNowData(targetMac);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send chunk %u: %s", i,
                     esp_err_to_name(ret));
            // Continue with next chunk - we'll resend failed chunks later
        } else {
            chunkSent[i] = true;
        }

        // Flow control: add brief delay after each chunk batch to avoid
        // overwhelming receiver
        batchCount++;
        if (batchCount >= chunkBatchSize) {
            vTaskDelay(pdMS_TO_TICKS(5));  // 5ms delay after batch
            batchCount = 0;
        }
    }

    // Wait for final acknowledgment or NACK with missing chunks
    const uint32_t MAX_TOTAL_TIMEOUT_MS =
        3000;  // Maximum 3 seconds for entire transfer
    const uint32_t ACK_CHECK_INTERVAL_MS = 50;  // Check for ACK every 50ms
    startTime = esp_log_timestamp();
    bool transferComplete = false;
    uint8_t retryCount = 0;
    const uint8_t MAX_RETRIES = 3;  // Maximum number of retry cycles

    while ((esp_log_timestamp() - startTime) < MAX_TOTAL_TIMEOUT_MS &&
           retryCount < MAX_RETRIES) {
        bool receivedResponse = false;
        uint32_t checkStartTime = esp_log_timestamp();

        // Wait for a response for a limited time
        while ((esp_log_timestamp() - checkStartTime) < ACK_CHECK_INTERVAL_MS &&
               !receivedResponse) {
            if (hasNewMessage()) {
                uint8_t responseBuffer[251];  // Large enough for NACK with
                                              // missing chunks list
                size_t responseLength;

                if (getReceivedMessage(responseBuffer, &responseLength) ==
                    ESP_OK) {
                    receivedResponse = true;

                    if (responseLength == 1) {
                        if (responseBuffer[0] ==
                            static_cast<uint8_t>(
                                EspNowPacketType::TRANSFER_ACK)) {
                            ESP_LOGI(TAG, "Transfer completed successfully");
                            transferComplete = true;
                            break;
                        } else if (responseBuffer[0] ==
                                   static_cast<uint8_t>(
                                       EspNowPacketType::TRANSFER_NACK)) {
                            ESP_LOGW(
                                TAG,
                                "Transfer rejected by receiver (simple NACK)");
                            free(chunkSent);
                            return false;
                        }
                    } else if (responseLength > 1 &&
                               responseBuffer[0] ==
                                   static_cast<uint8_t>(
                                       EspNowPacketType::BUFFER_NACK)) {
                        // Parse missing chunks from NACK response
                        std::string missingChunksStr(
                            reinterpret_cast<const char*>(responseBuffer + 1),
                            responseLength - 1);
                        ESP_LOGW(TAG, "Received NACK with missing chunks: %s",
                                 missingChunksStr.c_str());

                        // Parse missing chunks format "MISSING:1,4,7,12"
                        if (missingChunksStr.substr(0, 8) == "MISSING:") {
                            std::string chunkList = missingChunksStr.substr(8);
                            size_t pos = 0;
                            std::string token;
                            size_t resendCount = 0;

                            // Reset all chunk status to track what we're
                            // resending
                            memset(chunkSent, 0, totalChunks * sizeof(bool));

                            ESP_LOGI(TAG, "Resending missing chunks");

                            // Parse the CSV list and resend each chunk
                            while ((pos = chunkList.find(',')) !=
                                       std::string::npos ||
                                   !chunkList.empty()) {
                                token = (pos != std::string::npos)
                                            ? chunkList.substr(0, pos)
                                            : chunkList;
                                if (pos != std::string::npos) {
                                    chunkList.erase(0, pos + 1);
                                } else {
                                    chunkList.clear();
                                }

                                if (!token.empty()) {
                                    char* endptr;
                                    uint16_t chunkNum =
                                        strtoul(token.c_str(), &endptr, 10);

                                    if (endptr == token.c_str() ||
                                        *endptr != '\0') {
                                        ESP_LOGE(
                                            TAG,
                                            "Failed to parse chunk number: %s",
                                            token.c_str());
                                    } else if (chunkNum < totalChunks) {
                                        // Resend this chunk
                                        size_t offset = chunkNum * maxChunkSize;
                                        size_t chunkSize =
                                            (offset + maxChunkSize <= size)
                                                ? maxChunkSize
                                                : (size - offset);

                                        upStreamData chunkData;
                                        chunkData.packet_type =
                                            EspNowPacketType::BUFFER_CONTENT;
                                        chunkData.chunkNumber = chunkNum;
                                        chunkData.chunkSize = chunkSize;

                                        // Combine header and data
                                        uint8_t
                                            tempBuffer[sizeof(upStreamData) +
                                                       maxChunkSize];
                                        memcpy(tempBuffer, &chunkData,
                                               sizeof(upStreamData));
                                        memcpy(
                                            tempBuffer + sizeof(upStreamData),
                                            buffer + offset, chunkSize);

                                        dataSize_ =
                                            sizeof(upStreamData) + chunkSize;
                                        memcpy(dataBuffer_, tempBuffer,
                                               dataSize_);

                                        ESP_LOGI(TAG,
                                                 "Resending chunk %u, size: "
                                                 "%zu bytes",
                                                 chunkNum, chunkSize);

                                        ret = sendEspNowData(targetMac);
                                        if (ret == ESP_OK) {
                                            chunkSent[chunkNum] = true;
                                            resendCount++;

                                            // Add a small delay between chunks
                                            vTaskDelay(pdMS_TO_TICKS(10));
                                        } else {
                                            ESP_LOGE(
                                                TAG,
                                                "Failed to resend chunk %u: %s",
                                                chunkNum, esp_err_to_name(ret));
                                        }
                                    } else {
                                        ESP_LOGE(TAG,
                                                 "Invalid chunk number %u "
                                                 "(total chunks: %hu)",
                                                 chunkNum, totalChunks);
                                    }
                                }
                            }

                            ESP_LOGI(TAG, "Resent %zu missing chunks",
                                     resendCount);

                            // Check if we've resent everything
                            bool allSent = true;
                            for (uint16_t i = 0; i < totalChunks; i++) {
                                if (!chunkSent[i]) {
                                    allSent = false;
                                    break;
                                }
                            }

                            if (allSent) {
                                ESP_LOGI(TAG, "All missing chunks resent");
                            } else {
                                ESP_LOGW(TAG,
                                         "Some chunks still failed to resend");
                            }

                            // Reset start time to give more time for ACK after
                            // resending
                            startTime = esp_log_timestamp();
                        } else {
                            ESP_LOGE(TAG, "Invalid missing chunks format: %s",
                                     missingChunksStr.c_str());
                        }
                    }
                }
            }
            vTaskDelay(
                pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
        }

        // If we received a complete transfer ACK, we're done
        if (transferComplete) {
            break;
        }

        // If we didn't get any response in this interval, check another time
        if (!receivedResponse) {
            retryCount++;
            ESP_LOGW(TAG, "No response received, retry %u/%u", retryCount,
                     MAX_RETRIES);
        }
    }

    free(chunkSent);

    if (!transferComplete) {
        ESP_LOGE(TAG, "Failed to complete transfer after %u retries",
                 retryCount);
    }

    return transferComplete;
}

bool EspNow::sendBuffer(const std::string& data, const uint8_t* mac) {
    // Delegate to the char* version to avoid code duplication
    return sendBuffer(data.c_str(), data.size(), mac);
}

bool EspNow::receiveBuffer(uint8_t* buffer, size_t maxSize, size_t* actualSize,
                           uint8_t* packetInfo) {
    static const char* TAG = "EspNow::receiveBuffer";

    if (buffer == nullptr || actualSize == nullptr) {
        ESP_LOGE(TAG, "Invalid arguments (null pointers)");
        return false;
    }

    // Check if we already have a pending buffer request that user is responding
    // to
    upStreamDataInfo* request = nullptr;

    if (isBuffer_ && newMessage_) {
        // Check if the pending message is actually a BUFFER_REQUEST

        // Reset the safe transfer flag at the start
        resetBufferState();
        if (currentDataSize_ >= sizeof(upStreamDataInfo)) {
            // Extract request information from the message
            request = reinterpret_cast<upStreamDataInfo*>(data_);

            // Set safe transfer flag based on packet type
            if (data_[0] ==
                static_cast<uint8_t>(EspNowPacketType::SAFE_BUFFER_REQUEST)) {
                bufferTransfer_.isSafeTransfer = true;
            }

            ESP_LOGI(
                TAG,
                "Using pending buffer request: %u chunks, %zu bytes (%s mode)",
                request->total_chunks, request->data_size,
                bufferTransfer_.isSafeTransfer ? "SAFE" : "STANDARD");

            // If the buffer is too small, return error
            if (maxSize < request->data_size) {
                ESP_LOGE(TAG,
                         "Provided buffer (%zu bytes) is too small for "
                         "transfer (%zu bytes)",
                         maxSize, request->data_size);
                return false;
            }

            // Clear the message flag since we've consumed it
            newMessage_ = false;
        }
    }

    // Use provided packet info if request info isn't available
    if (request == nullptr && packetInfo != nullptr) {
        request = reinterpret_cast<upStreamDataInfo*>(packetInfo);
    }

    // Reset the buffer completion flag at the start of a new transfer
    bufferComplete_ = false;
    bufferStatus_ = BufferTransferStatus::IN_PROGRESS;

    // Initialize buffer transfer state for raw buffer
    if (bufferMutex_ == nullptr) {
        bufferMutex_ = xSemaphoreCreateMutex();
        if (bufferMutex_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create buffer mutex");
            return false;
        }
    }

    // Initialize data queue if not already created
    if (dataQueue_ == nullptr) {
        dataQueue_ =
            xQueueCreate(QUEUE_SIZE, sizeof(uint8_t*) + sizeof(size_t));
        if (dataQueue_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create data queue");
            return false;
        }
    } else {
        // Clear any existing items in the queue
        struct {
            uint8_t* data;
            size_t len;
        } queueItem;

        while (xQueueReceive(dataQueue_, &queueItem, 0) == pdTRUE) {
            free(queueItem.data);  // Free the dynamically allocated memory
        }
    }

    // Initialize the buffer transfer state
    xSemaphoreTake(bufferMutex_, portMAX_DELAY);

    bufferTransfer_.active = true;
    bufferTransfer_.buffer = buffer;
    bufferTransfer_.bufferSize = maxSize;
    bufferTransfer_.stringBuffer = nullptr;
    bufferTransfer_.receivedChunks = 0;
    bufferTransfer_.bytesReceived = 0;

    // Set total chunks and data size if request information is available
    if (request != nullptr) {
        bufferTransfer_.totalChunks = request->total_chunks;
        bufferTransfer_.dataSize = request->data_size;

        // Set the actualSize pointer if available
        if (actualSize != nullptr) {
            *actualSize = request->data_size;
        }
    } else {
        // Reset the values when no request info is provided
        bufferTransfer_.totalChunks = 0;
        bufferTransfer_.dataSize = 0;
    }

    xSemaphoreGive(bufferMutex_);

    ESP_LOGI(TAG, "Initialized buffer reception for %zu bytes max", maxSize);

    // Start the buffer reception task
    BufferTaskParams* taskParams =
        (BufferTaskParams*)malloc(sizeof(BufferTaskParams));
    if (taskParams != nullptr) {
        taskParams->instance = this;
        // Copy the last peer MAC as the sender MAC
        getLastPeerMac(taskParams->senderMac);

        // Create task to process the buffer reception
        BaseType_t taskCreated =
            xTaskCreatePinnedToCore(bufferReceptionTask, "buffer_reception",
                                    4096,  // Stack size
                                    taskParams,
                                    1,     // Priority
                                    NULL,  // Task handle
                                    0      // Core ID (0 for core 0)
            );

        if (taskCreated != pdPASS) {
            ESP_LOGE(TAG, "Failed to create buffer reception task");
            free(taskParams);

            xSemaphoreTake(bufferMutex_, portMAX_DELAY);
            bufferTransfer_.active = false;
            xSemaphoreGive(bufferMutex_);

            return false;
        }
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for task parameters");

        xSemaphoreTake(bufferMutex_, portMAX_DELAY);
        bufferTransfer_.active = false;
        xSemaphoreGive(bufferMutex_);

        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(33));  // Small delay to allow task to start

    // Send acknowledgment to start receiving buffer chunks
    uint8_t senderMac[ESP_NOW_ETH_ALEN];
    if (getLastPeerMac(senderMac) == ESP_OK) {
        sendAck(senderMac, EspNowPacketType::TRANSFER_ACK);
    }

    ESP_LOGI(TAG, "Buffer reception task started, waiting for completion");
    return true;
}

bool EspNow::receiveBuffer(std::string& data, uint8_t* packetInfo) {
    static const char* TAG = "EspNow::receiveBuffer";

    // Check if we already have a pending buffer request
    upStreamDataInfo* request = nullptr;
    size_t requestedSize = 0;
    resetBufferState();

    if (isBuffer_ && newMessage_) {
        // Check if the pending message is a BUFFER_REQUEST
        if (currentDataSize_ >= sizeof(upStreamDataInfo)) {
            // Extract request information
            request = reinterpret_cast<upStreamDataInfo*>(data_);
            requestedSize = request->data_size;

            // Set safe transfer flag based on packet type
            if (data_[0] ==
                static_cast<uint8_t>(EspNowPacketType::SAFE_BUFFER_REQUEST)) {
                bufferTransfer_.isSafeTransfer = true;
            }

            ESP_LOGI(
                TAG,
                "Using pending buffer request for string: %u chunks, %zu bytes "
                "(%s mode)",
                request->total_chunks, requestedSize,
                bufferTransfer_.isSafeTransfer ? "SAFE" : "STANDARD");
        }
    } else if (packetInfo != nullptr) {
        // Use provided packet info if available
        request = reinterpret_cast<upStreamDataInfo*>(packetInfo);
        requestedSize = request->data_size;

        // Check if this is a safe transfer
        bufferTransfer_.isSafeTransfer =
            (request->packet_type == EspNowPacketType::SAFE_BUFFER_REQUEST);

        ESP_LOGI(TAG,
                 "Using provided packet info: %u chunks, %zu bytes (%s mode)",
                 request->total_chunks, requestedSize,
                 bufferTransfer_.isSafeTransfer ? "SAFE" : "STANDARD");
    }

    // If no request info is available, return error
    if (request == nullptr) {
        ESP_LOGE(TAG, "No buffer request information available");
        return false;
    }

    // Pre-allocate string buffer with the right capacity if we know the size
    data.clear();
    if (requestedSize > 0) {
        data.reserve(requestedSize);
    }

    // Reset flags
    bufferComplete_ = false;
    bufferStatus_ = BufferTransferStatus::IN_PROGRESS;

    // Initialize buffer transfer state for string
    if (bufferMutex_ == nullptr) {
        bufferMutex_ = xSemaphoreCreateMutex();
        if (bufferMutex_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create buffer mutex");
            return false;
        }
    }

    // Initialize data queue if not already created
    if (dataQueue_ == nullptr) {
        dataQueue_ =
            xQueueCreate(QUEUE_SIZE, sizeof(uint8_t*) + sizeof(size_t));
        if (dataQueue_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create data queue");
            return false;
        }
    } else {
        // Clear any existing items in the queue
        struct {
            uint8_t* data;
            size_t len;
        } queueItem;

        while (xQueueReceive(dataQueue_, &queueItem, 0) == pdTRUE) {
            free(queueItem.data);
        }
    }

    // Set up buffer transfer state
    xSemaphoreTake(bufferMutex_, portMAX_DELAY);

    bufferTransfer_.active = true;
    bufferTransfer_.buffer = nullptr;
    bufferTransfer_.stringBuffer = &data;
    bufferTransfer_.receivedChunks = 0;
    bufferTransfer_.bytesReceived = 0;

    // Set total chunks and data size if information is available
    if (request != nullptr) {
        bufferTransfer_.totalChunks = request->total_chunks;
        bufferTransfer_.dataSize = request->data_size;
        ESP_LOGI(TAG, "Using provided packet info: %hu chunks, %zu bytes",
                 bufferTransfer_.totalChunks, bufferTransfer_.dataSize);

        // Reset newMessage_ flag since we've processed this request
        if (newMessage_ && data_[0] == static_cast<uint8_t>(
                                           EspNowPacketType::BUFFER_REQUEST)) {
            newMessage_ = false;
        }
    } else {
        // Reset the values when no info is provided
        bufferTransfer_.totalChunks = 0;
        bufferTransfer_.dataSize = 0;
    }

    xSemaphoreGive(bufferMutex_);

    ESP_LOGI(TAG, "Initialized buffer reception for string");

    // Start the buffer reception task
    BufferTaskParams* taskParams =
        (BufferTaskParams*)malloc(sizeof(BufferTaskParams));
    if (taskParams == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate memory for task parameters");

        xSemaphoreTake(bufferMutex_, portMAX_DELAY);
        bufferTransfer_.active = false;
        xSemaphoreGive(bufferMutex_);

        return false;
    }

    taskParams->instance = this;
    // Copy the last peer MAC as the sender MAC
    getLastPeerMac(taskParams->senderMac);

    // Create task to process the buffer reception
    BaseType_t taskCreated =
        xTaskCreate(bufferReceptionTask, "buffer_reception",
                    4096,  // Stack size
                    taskParams,
                    1,    // Priority
                    NULL  // Task handle
        );

    if (taskCreated != pdPASS) {
        ESP_LOGE(TAG, "Failed to create buffer reception task");
        free(taskParams);

        xSemaphoreTake(bufferMutex_, portMAX_DELAY);
        bufferTransfer_.active = false;
        xSemaphoreGive(bufferMutex_);

        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(33));  // Small delay to allow task to start

    // Send acknowledgment to start receiving buffer chunks
    uint8_t senderMac[ESP_NOW_ETH_ALEN];
    if (getLastPeerMac(senderMac) == ESP_OK) {
        sendAck(senderMac, EspNowPacketType::TRANSFER_ACK);
    }

    ESP_LOGI(TAG, "Buffer reception task started, waiting for completion");
    return true;
}

void EspNow::processDataTransferPacket(const uint8_t* data, size_t len) {
    static const char* TAG = "EspNow::processDataTransferPacket";

    if (bufferMutex_ == nullptr) {
        bufferMutex_ = xSemaphoreCreateMutex();
    }

    if (len == 0 || data == nullptr) {
        ESP_LOGE(TAG, "Invalid data packet");
        return;
    }

    // Check if this is a data chunk
    if (len > sizeof(upStreamData)) {
        // If this is called from the callback, queue the data for processing
        if (dataQueue_ != nullptr && xTaskGetTickCount() != 0) {
            // Create a copy of the data for the queue
            uint8_t* dataCopy = (uint8_t*)malloc(len);
            if (dataCopy != nullptr) {
                memcpy(dataCopy, data, len);

                // Create queue item
                struct {
                    uint8_t* data;
                    size_t len;
                } queueItem;

                queueItem.data = dataCopy;
                queueItem.len = len;

                // Try to send to queue
                if (xQueueSend(dataQueue_, &queueItem, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Queue full, processing directly");
                    free(dataCopy);
                    // Process directly
                    processChunkData(data, len);
                }
            } else {
                ESP_LOGE(TAG, "Failed to allocate memory for data copy");
                // Process directly
                processChunkData(data, len);
            }
        } else {
            // Process directly if not in a task context or queue not available
            processChunkData(data, len);
        }
    }
}

esp_err_t EspNow::sendAck(const uint8_t* mac, EspNowPacketType type) {
    static const char* TAG = "EspNow::sendAck";

    uint8_t ackPacket = static_cast<uint8_t>(type);
    dataSize_ = 1;
    memcpy(dataBuffer_, &ackPacket, dataSize_);

    // Make sure the peer is properly registered before sending
    esp_now_peer_info_t peerInfo;
    esp_err_t ret = esp_now_get_peer(mac, &peerInfo);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Peer not found, adding before sending acknowledgment");
        addPeer(const_cast<uint8_t*>(mac));
    }

    ESP_LOGI(
        TAG, "Sending acknowledgment: %d to MAC: %02X:%02X:%02X:%02X:%02X:%02X",
        static_cast<int>(type), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return sendEspNowData(mac);
}

void EspNow::processChunkData(const uint8_t* data, size_t len) {
    static const char* TAG = "EspNow::processChunkData";
    static uint32_t lastChunkTime = 0;
    uint32_t currentTime = esp_log_timestamp();

    if (len <= sizeof(upStreamData)) {
        ESP_LOGE(TAG, "Chunk data too small");
        return;
    }

    // Extract header from the data
    upStreamData chunkHeader;
    memcpy(&chunkHeader, data, sizeof(upStreamData));

    // Calculate the actual data portion size
    size_t chunkDataSize = len - sizeof(upStreamData);

    // Validate chunk size
    if (chunkDataSize != chunkHeader.chunkSize) {
        ESP_LOGW(TAG, "Chunk size mismatch: expected %u, got %zu",
                 chunkHeader.chunkSize, chunkDataSize);
        // We'll still process the data with the actual received size
    }

    xSemaphoreTake(bufferMutex_, portMAX_DELAY);

    // Check if we're actively receiving a buffer
    if (!bufferTransfer_.active) {
        xSemaphoreGive(bufferMutex_);
        ESP_LOGW(TAG, "Received chunk but no active transfer");
        return;
    }

    if (bufferTransfer_.isSafeTransfer) {
        ESP_LOGD(TAG, "Processing chunk for SAFE buffer transfer mode");
    }

    // Initialize chunk tracking if not already done
    if (bufferTransfer_.receivedChunkMap.empty() &&
        bufferTransfer_.totalChunks > 0) {
        bufferTransfer_.receivedChunkMap.resize(bufferTransfer_.totalChunks,
                                                false);
        // Store sender MAC for responses
        memcpy(bufferTransfer_.senderMac, lastPeerMac_, ESP_NOW_ETH_ALEN);
    }

    // Get the data pointer after the header
    const uint8_t* chunkData = data + sizeof(upStreamData);

    // Check chunk number validity
    if (chunkHeader.chunkNumber >= bufferTransfer_.totalChunks) {
        ESP_LOGW(TAG, "Invalid chunk number %u (total chunks: %hu)",
                 chunkHeader.chunkNumber, bufferTransfer_.totalChunks);
        xSemaphoreGive(bufferMutex_);
        return;
    }

    // Mark chunk as received
    if (!bufferTransfer_.receivedChunkMap[chunkHeader.chunkNumber]) {
        bufferTransfer_.receivedChunkMap[chunkHeader.chunkNumber] = true;
        bufferTransfer_.receivedChunks++;
        lastChunkTime = currentTime;  // Only update lastChunkTime when we
                                      // receive a NEW chunk
    } else {
        // This chunk was already received, just log it
        ESP_LOGD(TAG, "Duplicate chunk %u received", chunkHeader.chunkNumber);

        // For safe transfers, we still need to send an ACK even for duplicates
        if (bufferTransfer_.isSafeTransfer) {
            xSemaphoreGive(bufferMutex_);
            sendAck(lastPeerMac_, EspNowPacketType::TRANSFER_ACK);
            return;
        }

        xSemaphoreGive(bufferMutex_);
        return;
    }

    // Store the data in the appropriate buffer
    if (bufferTransfer_.buffer != nullptr) {
        // For raw buffer, calculate the offset based on chunk number
        size_t offset =
            chunkHeader.chunkNumber * 200;  // Assuming 200-byte chunks

        // Check bounds first
        if (offset + chunkDataSize <= bufferTransfer_.bufferSize) {
            memcpy(bufferTransfer_.buffer + offset, chunkData, chunkDataSize);
            bufferTransfer_.bytesReceived += chunkDataSize;
            /*
            ESP_LOGI(TAG, "Chunk stored in raw buffer, total: %zu/%zu bytes,
            chunks: %u/%u", bufferTransfer_.bytesReceived,
            bufferTransfer_.dataSize, bufferTransfer_.receivedChunks,
            bufferTransfer_.totalChunks);*/
        } else {
            ESP_LOGE(TAG, "Buffer overflow! Discarding chunk.");
        }
    } else if (bufferTransfer_.stringBuffer != nullptr) {
        // For string buffer, we need to ensure proper ordering
        // We reserve the total expected size first and then insert at proper
        // positions
        if (bufferTransfer_.stringBuffer->size() < bufferTransfer_.dataSize) {
            bufferTransfer_.stringBuffer->resize(bufferTransfer_.dataSize,
                                                 '\0');
        }

        // Calculate offset based on chunk number
        size_t offset =
            chunkHeader.chunkNumber * 200;  // Assuming 200-byte chunks

        // Insert data at the correct position
        if (offset + chunkDataSize <= bufferTransfer_.stringBuffer->size()) {
            for (size_t i = 0; i < chunkDataSize; i++) {
                (*bufferTransfer_.stringBuffer)[offset + i] = chunkData[i];
            }
            bufferTransfer_.bytesReceived += chunkDataSize;
            /*
            ESP_LOGI(TAG, "Chunk stored in string buffer, total: %zu/%zu bytes,
            chunks: %u/%u", bufferTransfer_.bytesReceived,
            bufferTransfer_.dataSize, bufferTransfer_.receivedChunks,
            bufferTransfer_.totalChunks);*/
        } else {
            ESP_LOGE(TAG, "String buffer overflow! Discarding chunk.");
        }
    } else {
        ESP_LOGE(TAG, "No valid buffer to store the chunk!");
    }

    // If this is a safe transfer, send an ACK for this chunk immediately
    if (bufferTransfer_.isSafeTransfer) {
        bool checksumValid = false;

        // Release mutex before sending ACK
        xSemaphoreGive(bufferMutex_);

        if (len < sizeof(safeUpStreamData)) {
            ESP_LOGW(TAG, "Received safe chunk with invalid size %zu < %zu",
                     len, sizeof(safeUpStreamData));
            return;
        }

        const safeUpStreamData* safeHeader =
            reinterpret_cast<const safeUpStreamData*>(data);

        // Extract data after the header
        const uint8_t* chunkData = data + sizeof(safeUpStreamData);
        size_t chunkDataSize = len - sizeof(safeUpStreamData);

        // Verify the checksum
        uint32_t calculatedChecksum =
            calculateChecksum(chunkData, chunkDataSize);
        checksumValid = (calculatedChecksum == safeHeader->checkSum);

        // Add delay before sending ACK to ensure receiver is ready
        vTaskDelay(pdMS_TO_TICKS(5));

        if (!checksumValid) {
            ESP_LOGW(TAG,
                     "Checksum mismatch for chunk %hu, expected: %lu, got: %lu",
                     chunkHeader.chunkNumber, safeHeader->checkSum,
                     calculatedChecksum);
            // Send NACK for this chunk
            sendAck(lastPeerMac_, EspNowPacketType::TRANSFER_NACK);
            return;
        }

        ESP_LOGI(TAG, "Sending ACK for chunk %u in SAFE mode",
                 chunkHeader.chunkNumber);

        // Try multiple times to ensure ACK is sent
        bool ackSent = false;
        for (int retry = 0; retry < maxRetries_ && !ackSent; retry++) {
            esp_err_t ret =
                sendAck(lastPeerMac_, EspNowPacketType::TRANSFER_ACK);
            ackSent = (ret == ESP_OK);
            if (!ackSent) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        // Check if all chunks have been received for this safe transfer
        if (bufferTransfer_.receivedChunks == bufferTransfer_.totalChunks) {
            ESP_LOGI(TAG, "All chunks received in SAFE mode, total: %zu bytes",
                     bufferTransfer_.bytesReceived);

            // Add delay before sending final ACK
            vTaskDelay(pdMS_TO_TICKS(20));

            // Send final ACK for entire buffer
            sendAck(lastPeerMac_, EspNowPacketType::TRANSFER_ACK);

            // Notify about successful completion
            notifyBufferComplete(true);
        }

        return;
    }

    lastChunkTime = currentTime;

    // Check if all chunks have been received or if we have enough for a partial
    // NACK
    bool transferComplete =
        (bufferTransfer_.receivedChunks == bufferTransfer_.totalChunks) &&
        (bufferTransfer_.totalChunks > 0);

    // If 80% of chunks have been received and 500ms passed since last chunk,
    // send NACK for missing chunks
    bool timeToSendNack =
        (bufferTransfer_.receivedChunks >= bufferTransfer_.totalChunks * 0.8) &&
        (bufferTransfer_.receivedChunks < bufferTransfer_.totalChunks) &&
        ((currentTime - lastChunkTime) > 500);  // 500ms since last chunk

    xSemaphoreGive(bufferMutex_);

    // Send final acknowledgment if transfer is complete
    if (transferComplete) {
        ESP_LOGI(TAG, "All chunks received. Total bytes: %zu/%zu",
                 bufferTransfer_.bytesReceived, bufferTransfer_.dataSize);

        // Check if we got all expected data
        if (bufferTransfer_.bytesReceived == bufferTransfer_.dataSize) {
            ESP_LOGI(TAG, "Transfer successful, sending ACK");
            sendAck(lastPeerMac_, EspNowPacketType::TRANSFER_ACK);

            // Notify about successful completion
            notifyBufferComplete(true);
        } else {
            ESP_LOGW(
                TAG,
                "Transfer incomplete! Expected %zu bytes but got %zu bytes",
                bufferTransfer_.dataSize, bufferTransfer_.bytesReceived);

            // Generate and send NACK with list of missing chunks
            std::string missingChunks = getMissingChunksString();
            if (!missingChunks.empty()) {
                sendBufferNack(lastPeerMac_, missingChunks);
            } else {
                sendAck(lastPeerMac_, EspNowPacketType::TRANSFER_NACK);
            }

            // Notify about failed completion
            notifyBufferComplete(false);
        }
    } else if (timeToSendNack) {
        // Send intermediate NACK with missing chunks for faster recovery
        std::string missingChunks = getMissingChunksString();
        if (!missingChunks.empty()) {
            ESP_LOGW(TAG, "Sending intermediate NACK with missing chunks: %s",
                     missingChunks.c_str());
            sendBufferNack(lastPeerMac_, missingChunks);
        }
    }
}

void EspNow::bufferReceptionTask(void* parameters) {
    static const char* TAG = "EspNow::bufferReceptionTask";

    BufferTaskParams* params = (BufferTaskParams*)parameters;
    EspNow* instance = params->instance;
    uint8_t senderMac[ESP_NOW_ETH_ALEN];
    memcpy(senderMac, params->senderMac, ESP_NOW_ETH_ALEN);

    // Store task handle for notifications
    instance->bufferTaskHandle_ = xTaskGetCurrentTaskHandle();

    // Free the parameters struct as we've copied what we need
    free(parameters);

    ESP_LOGI(TAG, "Started buffer reception task");

    uint32_t lastDataTime =
        esp_log_timestamp();    // Track when we last received data
    uint32_t timeoutMs = 1000;  // 1000ms timeout
    bool timeoutOccurred = false;

    if (instance->bufferTransfer_.isSafeTransfer) {
        // Use longer timeouts for safe transfers
        const uint32_t safeTimeoutMs =
            5000;  // 5000ms timeout for safe transfers
        timeoutMs = safeTimeoutMs;

        ESP_LOGI(TAG, "Using extended timeouts for SAFE transfer mode");
    }

    // Process queue items until we're done or timeout
    while (!instance->bufferComplete_) {
        // Check for data in the queue with a shorter timeout
        struct {
            uint8_t* data;
            size_t len;
        } queueItem;

        if (xQueueReceive(instance->dataQueue_, &queueItem,
                          pdMS_TO_TICKS(50)) == pdTRUE) {
            lastDataTime = esp_log_timestamp();  // Reset timeout counter
            instance->processChunkData(queueItem.data, queueItem.len);
            free(queueItem.data);  // Free the allocated memory

            // Check if buffer is now complete after processing this chunk
            if (instance->bufferComplete_) {
                ESP_LOGI(TAG, "Buffer completed after processing chunk");
                break;
            }
        } else {
            // Check for timeout
            uint32_t currentTime = esp_log_timestamp();
            if ((currentTime - lastDataTime) > timeoutMs) {
                ESP_LOGW(TAG,
                         "Timeout waiting for buffer chunks (%lu ms elapsed)",
                         currentTime - lastDataTime);

                // Handle timeout
                xSemaphoreTake(instance->bufferMutex_, portMAX_DELAY);
                bool active = instance->bufferTransfer_.active;
                xSemaphoreGive(instance->bufferMutex_);

                if (active) {
                    instance->sendAck(senderMac,
                                      EspNowPacketType::TRANSFER_NACK);

                    // Mark transfer as inactive
                    xSemaphoreTake(instance->bufferMutex_, portMAX_DELAY);
                    instance->bufferTransfer_.active = false;
                    xSemaphoreGive(instance->bufferMutex_);

                    // Set timeout flag
                    timeoutOccurred = true;
                    break;
                }
            }

            // Check if the transfer is still active
            xSemaphoreTake(instance->bufferMutex_, portMAX_DELAY);
            bool isActive = instance->bufferTransfer_.active;
            xSemaphoreGive(instance->bufferMutex_);

            if (!isActive) {
                ESP_LOGW(TAG, "Buffer transfer no longer active, exiting task");
                break;
            }
        }

        // Short yield to let other tasks run
        taskYIELD();
    }

    // Cleanup
    if (timeoutOccurred) {
        ESP_LOGW(TAG, "Buffer transfer timeout, recieved %zu chunks",
                 instance->bufferTransfer_.receivedChunks);
        instance->notifyBufferComplete(false);
    } else if (!instance->bufferComplete_) {
        ESP_LOGW(TAG, "Buffer transfer cancelled, recieved %zu chunks",
                 instance->bufferTransfer_.receivedChunks);
        instance->notifyBufferComplete(false);
    }

    // Reset task handle
    instance->bufferTaskHandle_ = nullptr;

    ESP_LOGI(TAG, "Buffer reception task exiting");
    vTaskDelete(NULL);  // Delete the task
}

bool EspNow::waitForBufferCompletion(uint32_t timeoutMs) {
    static const char* TAG = "EspNow::waitForBufferCompletion";

    uint32_t startTime = esp_log_timestamp();

    ESP_LOGI(TAG, "Waiting for buffer completion (timeout: %lu ms)", timeoutMs);

    while ((esp_log_timestamp() - startTime) < timeoutMs) {
        if (bufferComplete_) {
            ESP_LOGI(TAG, "Buffer transfer completed successfully");
            return true;
        }
        if (bufferStatus_ == BufferTransferStatus::FAILED) {
            ESP_LOGW(TAG, "Buffer transfer failed");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
    }

    ESP_LOGW(TAG, "Timeout waiting for buffer completion");
    return false;
}

void EspNow::notifyBufferComplete(bool success) {
    static const char* TAG = "EspNow::notifyBufferComplete";

    ESP_LOGI(TAG, "Buffer transfer %s",
             success ? "completed successfully" : "failed");

    bufferComplete_ = success;

    if (success) {
        bufferStatus_ = BufferTransferStatus::COMPLETE;
    } else {
        bufferStatus_ = BufferTransferStatus::FAILED;
    }

    // Reset isBuffer_ flag ALWAYS, not just on success
    isBuffer_ = false;

    // Clear received chunk map for next transfer
    xSemaphoreTake(bufferMutex_, portMAX_DELAY);
    bufferTransfer_.receivedChunkMap.clear();
    xSemaphoreGive(bufferMutex_);

    // Notify task if registered
    if (bufferTaskHandle_ != nullptr) {
        xTaskNotify(bufferTaskHandle_, success ? 1 : 0, eSetValueWithOverwrite);
    }
}

BufferTransferStatus EspNow::getBufferTransferStatus() {
    return bufferStatus_;
}

std::string EspNow::getMissingChunksString() {
    static const char* TAG = "EspNow::getMissingChunksString";

    xSemaphoreTake(bufferMutex_, portMAX_DELAY);

    // Check if we have chunk tracking enabled
    if (bufferTransfer_.receivedChunkMap.empty() ||
        bufferTransfer_.totalChunks == 0) {
        xSemaphoreGive(bufferMutex_);
        ESP_LOGW(TAG, "No chunk tracking available");
        return "";
    }

    // Create a string with the format "MISSING:1,4,7,12"
    std::string missingChunks = "MISSING:";
    bool hasEntries = false;

    // Check which chunks are missing and add them to the list
    for (uint16_t i = 0; i < bufferTransfer_.totalChunks; i++) {
        if (!bufferTransfer_.receivedChunkMap[i]) {
            // Add comma separator if not the first entry
            if (hasEntries) {
                missingChunks += ",";
            }

            // Convert chunk number to string and append
            char chunkStr[10];
            snprintf(chunkStr, sizeof(chunkStr), "%u", i);
            missingChunks += chunkStr;

            hasEntries = true;
        }
    }

    xSemaphoreGive(bufferMutex_);

    // Return the missing chunks string, or empty string if all chunks received
    if (hasEntries) {
        ESP_LOGI(TAG, "Missing chunks: %s", missingChunks.c_str());
        return missingChunks;
    } else {
        ESP_LOGI(TAG, "No missing chunks");
        return "";
    }
}

esp_err_t EspNow::sendBufferNack(const uint8_t* mac,
                                 const std::string& missingChunks) {
    static const char* TAG = "EspNow::sendBufferNack";

    // Prepare the NACK packet with missing chunks information
    if (missingChunks.empty() || missingChunks.length() > 250) {
        ESP_LOGE(TAG, "Invalid missing chunks string (empty or too long)");
        return ESP_ERR_INVALID_ARG;
    }

    // First byte is packet type
    dataBuffer_[0] = static_cast<uint8_t>(EspNowPacketType::BUFFER_NACK);

    // Copy missing chunks string after the packet type
    memcpy(dataBuffer_ + 1, missingChunks.c_str(), missingChunks.length());

    // Set total data size
    dataSize_ = 1 + missingChunks.length();

    ESP_LOGI(TAG,
             "Sending BUFFER_NACK with missing chunks: %s (total %zu bytes)",
             missingChunks.c_str(), dataSize_);

    // Send the NACK
    return sendEspNowData(mac);
}

/**
 * @brief Aborts an in-progress buffer transfer
 * @return true if transfer was aborted, false if no transfer was active
 */
bool EspNow::abortBufferTransfer() {
    static const char* TAG = "EspNow::abortBufferTransfer";

    xSemaphoreTake(bufferMutex_, portMAX_DELAY);

    if (!bufferTransfer_.active) {
        xSemaphoreGive(bufferMutex_);
        ESP_LOGW(TAG, "No active buffer transfer to abort");
        return false;
    }

    // Mark transfer as inactive
    bufferTransfer_.active = false;

    // If we have a sender MAC, send NACK
    if (bufferTransfer_.receivedChunkMap.size() > 0) {
        uint8_t senderMac[ESP_NOW_ETH_ALEN];
        memcpy(senderMac, bufferTransfer_.senderMac, ESP_NOW_ETH_ALEN);
        xSemaphoreGive(bufferMutex_);

        // Send abort NACK
        sendAck(senderMac, EspNowPacketType::TRANSFER_NACK);
    } else {
        xSemaphoreGive(bufferMutex_);
    }

    // Set buffer status to aborted
    bufferStatus_ = BufferTransferStatus::ABORTED;

    // Notify task if needed
    notifyBufferComplete(false);

    ESP_LOGW(TAG, "Buffer transfer aborted by application");
    return true;
}

void EspNow::resetBufferState() {
    static const char* TAG = "EspNow::resetBufferState";

    ESP_LOGI(TAG, "Resetting buffer transfer state");

    // First check if mutex exists, if not create it
    if (bufferMutex_ == nullptr) {
        bufferMutex_ = xSemaphoreCreateMutex();
        if (bufferMutex_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create buffer mutex in resetBufferState");
            return;  // Can't proceed without mutex
        }
    }

    // Now it's safe to take the mutex
    xSemaphoreTake(bufferMutex_, portMAX_DELAY);

    // Reset all buffer transfer state
    bufferTransfer_.active = false;
    bufferTransfer_.buffer = nullptr;
    bufferTransfer_.stringBuffer = nullptr;
    bufferTransfer_.receivedChunks = 0;
    bufferTransfer_.bytesReceived = 0;
    bufferTransfer_.totalChunks = 0;
    bufferTransfer_.dataSize = 0;
    bufferTransfer_.receivedChunkMap.clear();
    bufferTransfer_.isSafeTransfer = false;

    xSemaphoreGive(bufferMutex_);

    // Reset flags
    bufferComplete_ = false;
    isBuffer_ = false;
    bufferStatus_ = BufferTransferStatus::NONE;

    // Also reset queue if it exists
    if (dataQueue_ != nullptr) {
        // Clear any existing items in the queue
        struct {
            uint8_t* data;
            size_t len;
        } queueItem;

        while (xQueueReceive(dataQueue_, &queueItem, 0) == pdTRUE) {
            free(queueItem.data);  // Free the dynamically allocated memory
        }
    }
}

// Add this helper function to calculate a simple checksum
uint32_t EspNow::calculateChecksum(const uint8_t* data, size_t size) {
    uint32_t checksum = 0;

    // Simple Adler-32-like algorithm (lightweight and fast)
    const uint32_t MOD_ADLER = 65521;
    uint32_t a = 1, b = 0;

    for (size_t i = 0; i < size; i++) {
        a = (a + data[i]) % MOD_ADLER;
        b = (b + a) % MOD_ADLER;
    }

    checksum = (b << 16) | a;
    return checksum;
}

esp_err_t EspNow::getPeerMacs(std::vector<uint8_t*>& macs) {
    // Clear any existing entries
    macs.clear();

    // First get the MAC count from NVS
    uint8_t macCount = 0;
    esp_err_t err = flashStorage_.get("mac_count", macCount);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No MAC count found in NVS. No peers available.");
        return ESP_OK;  // No error, just empty list
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading MAC count from NVS: %s",
                 esp_err_to_name(err));
        return err;
    }

    if (macCount == 0) {
        // No MACs stored, just return empty vector
        return ESP_OK;
    }

    // Reserve space in the vector
    macs.reserve(macCount);

    // Retrieve each MAC address from NVS
    for (uint8_t i = 0; i < macCount; ++i) {
        char key[10];
        snprintf(key, sizeof(key), "mac_%d", i);

        // Allocate memory for this MAC address
        uint8_t* mac = reinterpret_cast<uint8_t*>(malloc(ESP_NOW_ETH_ALEN));
        if (mac == nullptr) {
            ESP_LOGW(TAG, "Failed to allocate memory for MAC address");
            continue;
        }

        size_t required_size = ESP_NOW_ETH_ALEN;
        err = flashStorage_.get(key, mac, required_size);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read MAC at index %d: %s", i,
                     esp_err_to_name(err));
            free(mac);  // Free the allocated memory
            continue;
        }

        // Add this MAC to the vector
        macs.push_back(mac);

        ESP_LOGI(TAG, "Retrieved MAC %d: %02X:%02X:%02X:%02X:%02X:%02X", i,
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    ESP_LOGI(TAG, "Successfully retrieved %zu MAC addresses from NVS",
             macs.size());
    return ESP_OK;
}

void EspNow::freePeerMacs(std::vector<uint8_t*>& macs) {
    for (uint8_t* mac : macs) {
        free(mac);
    }
    macs.clear();
}
