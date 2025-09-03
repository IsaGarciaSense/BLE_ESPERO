/*******************************************************************************
 * @file connectivity.hpp
 * @brief Contains utility functions for connectivity protocols.
 *
 * This header file provides class and method definitions for the communication
 * protocol templates used with the microcontroller. These utilities are part
 * of the Sense-AI project and are designed to simplify common tasks related
 * to connectivity.
 *
 * @date 2024-10-01
 * @author valeria@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include <esp_log.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "esp_system.h"  // Required for esp_read_mac
#include "esp_wifi.h"    // Include for esp_read_mac

/**
 * @brief Abstract base class for connectivity protocols, defining a common
 * interface.
 *
 * Connectivity: Provides a set of pure virtual methods that any derived
 * connectivity protocol must implement. This interface includes essential
 * functions for initialization, connection, data transmission, reception, and
 * disconnection. Derived classes must implement each method to manage specific
 * connectivity protocols.
 **/

class Connectivity {
public:
    virtual ~Connectivity() {
    }
    virtual esp_err_t initialize() = 0;
    virtual esp_err_t connect() = 0;
    virtual esp_err_t sendData() = 0;
    virtual esp_err_t receiveData() = 0;
    virtual esp_err_t disconnect() = 0;
};
