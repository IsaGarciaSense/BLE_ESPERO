/*******************************************************************************
 * @file espnow_sense.hpp
 * @brief Contains utility functions for connectivity protocols.
 *
 * This header file provides class and method definitions for the communication
 * protocol templates used with the microcontroller. These utilities are part
 * of the Sense-AI project and are designed to simplify common tasks related
 * to connectivity.
 *
 * @version 0.5.0
 * @date 2025-04-13
 * @author daniel@sense-ai.co
 ******************************************************************************/

#pragma once

#include <esp_log.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include <string>
#include <vector>

#include "connectivity.hpp"
#include "esp_phy_init.h"  // For ESP_PHY_CAL_FULL and related definitions
#include "esp_system.h"    // Required for esp_read_mac
#include "esp_wifi.h"      // Include for esp_read_mac
#include "flash_sense.hpp"
#include "freertos/semphr.h"

/** @brief Type of ESP-NOW packet being sent/received. */
enum class EspNowPacketType : uint8_t {
    REPORT =
        0x00,  ///< Packet containing a report (e.g., temperature, humidity)
    BUFFER_REQUEST = 0x01,  ///< Packet initiating a transfer (filename, size)
    SAFE_BUFFER_REQUEST = 0x02,  ///<
    BUFFER_CONTENT = 0x03,  ///< Packet containing data chunks (up to 251 bytes)
    BUFFER_ACK = 0x04,      ///< Optional: Acknowledge successful transfer (Not
                            ///< implemented yet)
    BUFFER_NACK =
        0x05,  ///< Optional: Acknowledge failed transfer (Not implemented yet)
    TRANSFER_ACK = 0x06,  ///< Optional: Acknowledge successful transfer (Not
                          ///< implemented yet)
    TRANSFER_NACK =
        0x07  ///< Optional: Acknowledge failed transfer (Not implemented yet)
};

struct upStreamDataInfo {
    EspNowPacketType packet_type;  ///< Type of the received packet
    uint16_t total_chunks;         ///< Expected total chunks (from sender)
    size_t data_size;              ///< Actual size of data in data_ptr buffer
};

struct upStreamData {
    EspNowPacketType packet_type;  ///< Type of the received packet
    uint16_t chunkNumber;          ///< Number of the current chunk being sent
    uint16_t chunkSize;            ///< Size of the current chunk
    uint8_t* data_ptr;             ///< Pointer to the data buffer
};

struct safeUpStreamData {
    EspNowPacketType packet_type;  ///< Type of the received packet
    uint16_t chunkNumber;          ///< Number of the current chunk being sent
    uint16_t chunkSize;            ///< Size of the current chunk
    uint32_t checkSum;             ///< Size of the current chunk
    uint8_t* data_ptr;             ///< Pointer to the data buffer
};

struct nAckResponse {
    EspNowPacketType packet_type;  ///< Type of the received packet
    std::string missingChunks;     ///< Number of the current chunk being sent
};

enum class BufferTransferStatus {
    NONE,         // No transfer in progress
    IN_PROGRESS,  // Transfer is ongoing
    COMPLETE,     // Transfer completed successfully
    FAILED,       // Transfer failed (timeout, errors, etc.)
    ABORTED       // Transfer was aborted
};

/**
 * @brief Class to manage ESP-NOW communication as a derived type of
 *Connectivity.
 *
 * This class, `EspNow`, inherits from the base class `Connectivity` and
 * provides a specialized interface for establishing ESP-NOW communication
 * between devices. It supports functions such as initializing and managing
 * connections, sending and receiving data, handling peer devices, and
 * managing device MAC addresses.
 **/

class EspNow : public Connectivity {
public:
    /**
     * @brief Constructor with configurable parameters.
     * @param maxRetries Maximum number of retries for sending data.
     * @param channel ESP-NOW communication channel (default: 3).
     */
    EspNow(uint8_t maxRetries = 6, uint8_t channel = 3,
           bool setAsServer = false);

    /**
     * @brief Destroys the EspNow object.
     */
    ~EspNow();

    /**
     * @brief Functions derived from the Connectivity superclass
     *
     * Implementations of basic ESP-NOW functions for establishing connections,
     * managing data exchange, and terminating connections.
     **/
    esp_err_t initialize() override;

    esp_err_t connect() override;

    esp_err_t sendData() override;

    esp_err_t sendEspNowData(const uint8_t* mac);

    /**
     * @brief Checks if a buffer request has been received.
     * @return True if a buffer request has been received since the last call to
     * this function.
     */
    bool hasBufferRequest() {
        bool val = isBuffer_;
        isBuffer_ = false;
        return val;
    }

    bool isBufferComplete() {
        return bufferComplete_;
    }

    /**
     * @return Buffer status enum type.
     */
    BufferTransferStatus getBufferTransferStatus();

    /**
     * @brief Receives data and writes it into the provided std::string.
     * @param receivedData Reference to an std::string where the received data
     * will be written.
     * @return ESP_OK if data is successfully received, ESP_ERR_NOT_FOUND if no
     * new data is available.
     */
    esp_err_t receiveData() override;

    // Returns the received data as a string AFTER calling receiveData()
    std::string getReceivedData() {
        return receivedData_;
    };

    // Overloaded function to move the Data_ buffer to the receivedData_ string
    // and pass it as reference in one single step
    esp_err_t receiveData(std::string& receivedData);

    // Returns true if a new message has arrived since last buffer read
    bool hasNewMessage() {
        return newMessage_;
    };

    // Returns the data size of the last message arrived
    size_t getReceivedDataSize() {
        return currentDataSize_;
    };

    // Returns the ammount of retries made to send the last message
    uint8_t getRetries() {
        return retries_;
    };

    // Returns the last message received in the buffer
    esp_err_t getReceivedMessage(uint8_t* _message, size_t* _receivedLength);

    // End espnow connection and close nvs namespace
    esp_err_t disconnect() override;

    // Add to espnow_sense.hpp in the public section of the EspNow class
    /**
     * @brief Configure power saving parameters for ESP-NOW
     *
     * @param wakeInterval The wake interval in milliseconds
     * @param wakeWindow The wake window in milliseconds
     * @return esp_err_t ESP_OK on success, or an error code
     */
    esp_err_t configPowerSaving(uint16_t wakeInterval, uint16_t wakeWindow);

    /**
     * @brief Retrieves information from the reception callback.
     *
     * getClientMac(uint8_t *_mac): Copies the client’s MAC address from the
     * reception callback into `_mac`.
     * getReceivedData(uint8_t* _message, size_t* _receivedLength): Copies the
     * received message and its length from the callback into `_message`
     * and `_receivedLength`.
     *
     * @return ESP_OK on success or error code on failure.
     **/
    esp_err_t getLastPeerMac(uint8_t* _mac);

    /**
     * @brief Sends the current timestamp to a specified client.
     * @param _clientMillis Pointer to the MAC address of the target client.
     * @return ESP_OK on success or error code on failure.
     **/
    esp_err_t sendMillis(uint8_t* _clientMillis);

    /**
     * @brief Sends a confirmation message indicating readiness in uplink mode.
     * @param _clientUplink Pointer to the MAC address of the target client.
     * @return ESP_OK on success or error code on failure.
     **/
    // esp_err_t checkUplinkMode(uint8_t* _clientUplink);

    /**
     * @brief Manages MAC address storage in non-volatile memory (NVS).
     *
     * `saveMacToNvs(uint8_t* mac)`: Saves a single MAC address to NVS.
     * `savePeerToNvs(uint8_t* mac)`: Stores a paired MAC address in an array
     * within NVS, allowing the server to keep track of all connected peers.
     * @param mac Pointer to the MAC address to be stored.
     * @return ESP_OK on success or error code on failure.
     **/
    bool isMacStoredInNvs(const char* key);

    esp_err_t storeMacInNvs(uint8_t index, const uint8_t* mac);

    esp_err_t saveMacToNvs(const uint8_t* mac);

    esp_err_t savePeerToNvs(const uint8_t* mac);

    void resetStoredMacs();

    /**
     * @brief Reads a MAC address stored in non-volatile memory (NVS).
     *
     * Retrieves a single MAC address from NVS that was previously saved using
     * `saveMacToNvs`. The retrieved MAC address is copied into the provided
     * `_mac` variable.
     * @param _mac Pointer to a variable where the retrieved MAC address will
     * be stored.
     * @return ESP_OK on success or error code on failure.
     **/

    esp_err_t readMacFromNvs(uint8_t* _mac);

    /**
     * @brief Retrieves the MAC address of the ESP-NOW server.
     *
     * The `setServerMac` function allows you to set the MAC address of the
     * ESP-NOW server. If no MAC address is provided, the function reads the
     * server MAC address from NVS. The server MAC address is used to establish
     * a connection with the server and send data.
     * @param mac Pointer to the MAC address of the server.
     * @return ESP_OK on success or error code on failure.
     **/
    esp_err_t setServer(const uint8_t* mac);

    /**
     * @brief Loads and validates paired MAC addresses from non-volatile memory.
     *
     * Loads the array of MAC addresses previously stored using `savePeerToNvs`.
     * This function ensures that no duplicate MAC addresses are stored and
     * verifies that all addresses are paired correctly.
     * @return ESP_OK on success or error code on failure.
     */
    esp_err_t loadMacsFromNvs();

    /**
     * @brief Get a list of all peer MAC addresses currently registered
     *
     * @param[out] macs Vector to be filled with pointers to peer MAC addresses
     * @return esp_err_t ESP_OK on success, or an error code
     */
    esp_err_t getPeerMacs(std::vector<uint8_t*>& macs);

    /**
     * @brief Free the MAC addresses allocated by getPeerMacs
     *
     * @param macs Vector of MAC addresses to free
     */
    void freePeerMacs(std::vector<uint8_t*>& macs);

    /**
     * @brief Adds a new peer for ESP-NOW communication.
     * @param incomingMac Pointer to the MAC address of the new peer.
     * @return ESP_OK on success or error code on failure.
     */
    esp_err_t addPeer(uint8_t* peerMac);

    bool isLikelyString(const uint8_t* data, size_t length) const;

    /**
     * @brief Tests the connection with a specific MAC address.
     *
     * This function sends a test packet containing a timestamp and retry count
     * to verify if a connection with the specified MAC address is working.
     *
     * @param mac      Pointer to the 6-byte MAC address
     * @param timeStamp Timestamp to include in the test packet
     * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise
     */
    esp_err_t testConnection(const uint8_t* mac, uint64_t timeStamp,
                             uint32_t responseTimeoutMs = 1000);

    /**
     * @brief
     *
     *  The setMessage function is a template that allows you to store any type
     * of data in an internal data buffer.
     **/
    template <typename T>
    void setMessage(const T& report) {
        static_assert(sizeof(T) <= sizeof(dataBuffer_),
                      "Data exceeds buffer size!");
        dataSize_ = sizeof(report);
        memcpy(dataBuffer_, &report, dataSize_);
    }

    /**
     * @brief Sends data using the template-based `setMessage` function.
     * @tparam T The type of data to send.
     * @param data The data to send.
     * @return ESP_OK on success or error code on failure.
     */
    template <typename T>
    esp_err_t sendData(const T& data) {
        setMessage(data);   // Use the template to store the data in the buffer
        return sendData();  // Call the existing sendData() to transmit the data
    }

    /**
     * @brief Sends data using the template-based `setMessage` function to a
     * specific MAC address.
     * @tparam T The type of data to send.
     * @param mac The MAC address of the target device.
     * @param data The data to send.
     * @return ESP_OK on success or error code on failure.
     */
    template <typename T>
    esp_err_t sendData(const uint8_t* mac, const T& data) {
        setMessage(data);  // Use the template to store the data in the buffer
        return sendEspNowData(
            mac);  // Call the existing sendData() to transmit the data
    }

    /**
     * @brief Sends a `std::string` as data.
     * @param data The string to send.
     * @return ESP_OK on success or error code on failure.
     */
    esp_err_t sendData(const std::string& data) {
        if (data.size() > sizeof(dataBuffer_)) {
            ESP_LOGE("EspNow", "Data exceeds buffer size!");
            return ESP_ERR_INVALID_SIZE;
        }

        // Copy the string content into the buffer directly
        dataSize_ = data.size();
        memcpy(dataBuffer_, data.data(), dataSize_);

        // Call the existing sendData() to transmit the data
        return sendData();
    }

    /**
     * @brief Sends a `std::string` as data to a specific MAC address.
     * @param mac The MAC address of the target device.
     * @param data The string to send.
     * @return ESP_OK on success or error code on failure.
     */
    esp_err_t sendData(const uint8_t* mac, const std::string& data) {
        if (data.size() > sizeof(dataBuffer_)) {
            ESP_LOGE("EspNow", "Data exceeds buffer size!");
            return ESP_ERR_INVALID_SIZE;
        }

        // Copy the string content into the buffer directly
        dataSize_ = data.size();
        memcpy(dataBuffer_, data.data(), dataSize_);

        // Call the existing sendEspNowData() to transmit the data
        return sendEspNowData(mac);
    }

    /**
     * @brief Sends a broadcast message using a template-based `sendData`
     * function.
     * @tparam T The type of data to send.
     * @param data The data to send.
     * @return ESP_OK on success or error code on failure.
     */
    template <typename T>
    esp_err_t sendBroadcast(const T& data) {
        uint8_t broadcastMac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                  0xFF, 0xFF, 0xFF};
        return sendData(broadcastMac,
                        data);  // Use the existing sendData(mac, data) method
    }

    /**
     * @brief Sends a broadcast message with a `std::string` as data.
     * @param data The string to send.
     * @return ESP_OK on success or error code on failure.
     */
    esp_err_t sendBroadcast(const std::string& data) {
        uint8_t broadcastMac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                  0xFF, 0xFF, 0xFF};
        return sendData(broadcastMac,
                        data);  // Use the existing sendData(mac, data) method
    }

    /**
     * @brief Sends a large buffer by splitting it into chunks. Fast mode
     * doesn't wait for chunk ACKs.
     * @param buffer The data buffer to send.
     * @param size The size of the data buffer.
     * @param mac The MAC address of the target device. If nullptr, uses
     * serverMac_.
     * @return true on successful transfer, false on failure.
     */
    bool sendFastBuffer(const char* buffer, size_t size,
                        const uint8_t* mac = nullptr);

    /**
     * @brief Sends a large string by splitting it into chunks. Fast mode
     * doesn't wait for chunk ACKs.
     * @param data The string data to send.
     * @param mac The MAC address of the target device. If nullptr, uses
     * serverMac_.
     * @return true on successful transfer, false on failure.
     */
    bool sendFastBuffer(const std::string& data, const uint8_t* mac = nullptr);

    /**
     * @brief Sends a large buffer by splitting it into chunks with reliability
     * guarantees. Waits for ACK and resends missing chunks if needed.
     * @param buffer The data buffer to send.
     * @param size The size of the data buffer.
     * @param mac The MAC address of the target device. If nullptr, uses
     * serverMac_.
     * @return true on successful transfer, false on failure.
     */
    bool sendBuffer(const char* buffer, size_t size,
                    const uint8_t* mac = nullptr);

    /**
     * @brief Sends a large string by splitting it into chunks with reliability
     * guarantees. Waits for ACK and resends missing chunks if needed.
     * @param data The string data to send.
     * @param mac The MAC address of the target device. If nullptr, uses
     * serverMac_.
     * @return true on successful transfer, false on failure.
     */
    bool sendBuffer(const std::string& data, const uint8_t* mac = nullptr);

    /**
     * @brief Sends a large buffer with maximum reliability (one chunk at a
     * time). Waits for ACK after each chunk before sending the next one.
     * @param buffer The data buffer to send.
     * @param size The size of the data buffer.
     * @param mac The MAC address of the target device. If nullptr, uses
     * serverMac_.
     * @return true on successful transfer, false on failure.
     */
    bool sendSafeBuffer(const char* buffer, size_t size,
                        const uint8_t* mac = nullptr);

    /**
     * @brief Sends a large string with maximum reliability (one chunk at a
     * time). Waits for ACK after each chunk before sending the next one.
     * @param data The string data to send.
     * @param mac The MAC address of the target device. If nullptr, uses
     * serverMac_.
     * @return true on successful transfer, false on failure.
     */
    bool sendSafeBuffer(const std::string& data, const uint8_t* mac = nullptr);

    /**
     * @brief Receives a large buffer that was split into chunks.
     * @param buffer Pointer to a pre-allocated buffer to store the received
     * data.
     * @param maxSize Maximum size of the buffer.
     * @param actualSize Pointer to a variable to store the actual size of
     * received data.
     * @return true on successful reception, false on failure.
     */
    bool receiveBuffer(uint8_t* buffer, size_t maxSize, size_t* actualSize,
                       uint8_t* packetInfo);

    /**
     * @brief Receives a large buffer into a std::string.
     * @param data Reference to a std::string to store the received data.
     * @return true on successful reception, false on failure.
     */
    bool receiveBuffer(std::string& data, uint8_t* packetInfo);

    /**
     * @brief Send acknowledgment packet
     */
    esp_err_t sendAck(const uint8_t* mac, EspNowPacketType type);

    /**
     * @brief Waits for buffer completion with timeout.
     * @param timeoutMs Maximum time to wait in milliseconds.
     * @return True if buffer completed successfully, false on timeout.
     */
    bool waitForBufferCompletion(uint32_t timeoutMs = 5000);

    /**
     * @brief Task that processes buffer reception
     */
    static void bufferReceptionTask(void* parameters);

    /**
     * @brief Structure for task parameters
     */
    typedef struct {
        EspNow* instance;
        uint8_t senderMac[ESP_NOW_ETH_ALEN];
    } BufferTaskParams;

    /**
     * @brief Generates a string listing missing chunks in format
     * "MISSING:1,4,7,12"
     * @return String containing the list of missing chunks
     */
    std::string getMissingChunksString();

    /**
     * @brief Sends a NACK response with information about missing chunks
     * @param mac The MAC address of the target device
     * @param missingChunks String containing the list of missing chunks
     * @return ESP_OK on success or error code on failure
     */
    esp_err_t sendBufferNack(const uint8_t* mac,
                             const std::string& missingChunks);

private:
#define MAX_PEERS 17

    /**
     * @brief Object for handling NVS operations using the flash_storage
     * library.
     */
    FlashStorage flashStorage_;

    /**
     * @brief Maximum number of retries for sending data.
     */
    uint8_t maxRetries_;

    /**
     * @brief ESP-NOW communication channel.
     */
    uint8_t channel_;

    /**
     * @brief ESP-NOW wake interval for powersaving mode. 100ms suggested.
     */
    uint16_t wakeInterval_ = 0;  // Wake interval in seconds

    /**
     * @brief ESP-NOW wake window for powersaving mode. 60ms suggested.
     */
    uint16_t wakeWindow_ = 0;  // Wake window in seconds

    /*
     * @brief Flag to set the device as a server.
     */
    bool isServer_;

    /**
     * @brief Semaphore for sending data.
     */
    static SemaphoreHandle_t sendSemaphore;

    /*
     * @brief Number of retries for sending data.
     */
    uint8_t retries_;

    /**
     * @brief Flag to track the status of the message sent.
     */
    bool messageSent_;

    /**
     * @brief Stores the MAC address of the ESP-NOW server.
     **/
    uint8_t serverMac_[ESP_NOW_ETH_ALEN];

    /**
     * @brief Stores the MAC address of the "connected" client.
     **/
    uint8_t lastPeerMac_[ESP_NOW_ETH_ALEN];

    /**
     * @brief Buffer to store outgoing or incoming data (up to 251 bytes).
     **/
    uint8_t data_[251] = {0};

    std::string receivedData_;

    uint8_t dataBuffer_[251] = {0};

    /**
     * @brief Maximum size of the data packet in bytes.
     **/
    uint8_t maxDataPacketSize_ = 251;

    /**
     * @brief Array to store MAC addresses of up to 17 paired devices.
     **/
    uint8_t macAddresses_[MAX_PEERS][6];

    /**
     * @brief Keeps track of the number of MAC addresses stored in
     *'macAddresses_'.
     **/
    uint8_t macCount_ = 0;

    /**
     * @brief Index for writing data to the 'data_' buffer.
     */
    uint8_t writeIndex_ = 0;

    uint16_t timeout_ = 250;  // Timeout for sending data in milliseconds

    /**
     * @brief Stores the size of the data to be sent in bytes.
     *
     * `dataSize_`: Represents the size, in bytes, of the data currently stored
     *in `data_`. This variable is set in `setMessage()` when data is copied to
     *the `data_` buffer, and is then used in `sendData()` to specify the exact
     *amount of data to be transmitted.
     **/
    size_t dataSize_;

    /**
     * @brief Stores the size of the most recently received data.
     *
     * `currentDataSize_`: Holds the length of the data received in
     *`on_data_recv()` and is updated each time new data arrives. It allows
     *`getReceivedData()` to verify the current data size before copying it to
     *the output buffer.
     **/
    size_t currentDataSize_;

    /**
     * @brief Static pointer to the single instance of the `EspNow` class.
     *
     * This static pointer allows `EspNow` to ensure that only one instance of
     *the class exists throughout the program. It enables global access to the
     *instance, which is useful for handling shared resources and callbacks.
     **/
    static EspNow* instance;

    /**
     * @brief Flag to track the arrival of new messages.
     **/
    bool newMessage_;

    /**
     * @brief Flag to track buffer requests.
     **/
    bool isBuffer_ = false;

    bool bufferComplete_ = false;

    /**
     * @brief Flag to track buffer completion.
     **/
    BufferTransferStatus bufferStatus_ = BufferTransferStatus::NONE;

    /**
     * @brief Buffer transfer state
     */
    struct BufferTransferState {
        bool active = false;          // Whether a buffer transfer is active
        uint32_t transmissionId = 0;  // ID of the current transmission
        uint16_t totalChunks = 0;     // Total number of chunks expected
        uint16_t receivedChunks = 0;  // Number of chunks received so far
        size_t dataSize = 0;          // Total data size expected
        size_t bytesReceived = 0;     // Bytes received so far
        uint8_t* buffer = nullptr;    // Buffer to store the received data
        size_t bufferSize = 0;        // Size of the buffer
        std::string* stringBuffer =
            nullptr;  // String buffer (alternative to raw buffer)
        bool isSafeTransfer = false;          // Flag for safe transfer mode
        std::vector<bool> receivedChunkMap;   // Bitmap of received chunks
        uint8_t senderMac[ESP_NOW_ETH_ALEN];  // Store sender MAC for responses
    } bufferTransfer_;

    /**
     * @brief Queue for data transfer messages
     */
    static const int QUEUE_SIZE = 10;
    QueueHandle_t dataQueue_ = nullptr;
    SemaphoreHandle_t bufferMutex_ = nullptr;

    /**
     * @brief Static callback functions for ESP-NOW events.
     */
    static void on_data_static(const esp_now_recv_info_t* recv_info,
                               const uint8_t* data, int len);
    static void on_data_sent_static(const uint8_t* macAddr,
                                    esp_now_send_status_t status);

    /**
     * @brief Non-static member functions to process events.
     */
    void on_data_recv(const esp_now_recv_info_t* recv_info, const uint8_t* data,
                      int len);
    void on_data_sent(const uint8_t* macAddr, esp_now_send_status_t status);

    /**
     * @brief Returns the current Unix timestamp as a `long` integer
     **/
    long getUnixTime(void);

    /**
     * @brief Checks for duplicate MAC addresses in the stored array.
     * @return true if the MAC address is a duplicate; otherwise, returns
     *`false`.
     **/
    bool isMacDuplicate(const uint8_t* _newMac);

    /**
     * @brief Process a data transfer packet
     */
    void processDataTransferPacket(const uint8_t* data, size_t len);

    /**
     * @brief Task handle for buffer reception
     */
    TaskHandle_t bufferTaskHandle_ = nullptr;

    /**
     * @brief Process chunk data received from the sender
     */
    void processChunkData(const uint8_t* data, size_t len);

    /**
     * @brief Notifies about buffer completion status
     * @param success Whether the buffer transfer was successful
     */
    void notifyBufferComplete(bool success);

    /**
     * @brief Aborts the buffer transfer and cleans up resources
     * @return true if the transfer was aborted successfully, false otherwise
     */
    bool abortBufferTransfer();

    /**
     * @brief Resets the buffer transfer state and cleans up resources
     */
    void resetBufferState();

    /**
     * @brief Calculates the checksum for a given data buffer
     * @param data Pointer to the data buffer
     * @param size Size of the data buffer
     * @return Checksum value
     */
    uint32_t calculateChecksum(const uint8_t* data, size_t size);
};
