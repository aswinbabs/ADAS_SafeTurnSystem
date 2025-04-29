#include "CANBusHandler.hpp"
#include <inttypes.h>

static const char* TAG = "CANBusHandler";

CANBusHandler::CANBusHandler(gpio_num_t tx_pin, gpio_num_t rx_pin)
    : txPin(tx_pin), rxPin(rx_pin), isInitialized(false),
      prev_left_indicator(false), prev_right_indicator(false), prev_headlight(false), prev_foglight(false), prev_leftWarning(false), prev_rightWarning(false), prev_angle(0) {
}

CANBusHandler::~CANBusHandler() {
    if (isInitialized) {
        // Stop the TWAI driver
        twai_stop();
        // Uninstall the TWAI driver
        twai_driver_uninstall();
        ESP_LOGI(TAG, "CAN bus handler destroyed");
    }
}

bool CANBusHandler::begin() {
    // Configure TWAI (CAN) driver
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);
    
    // Set timing for 500 Kbit/s baud rate (common for automotive)
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    
    // Set filter to accept all messages (we'll filter by ID in software)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Install TWAI driver
    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(result));
        return false;
    }
    
    // Start TWAI driver
    result = twai_start();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(result));
        twai_driver_uninstall();
        return false;
    }
    
    isInitialized = true;
    ESP_LOGI(TAG, "CAN bus initialized successfully");
    return true;
}

bool CANBusHandler::sendCANMessage(uint32_t id, const uint8_t* data, uint8_t length) {
    if (!isInitialized) {
        ESP_LOGE(TAG, "CAN bus not initialized");
        return false;
    }
    
    if (length > 8) {
        ESP_LOGE(TAG, "CAN data length exceeds maximum (8 bytes)");
        return false;
    }
    
    // Prepare message
    twai_message_t message;
    message.identifier = id;
    message.data_length_code = length;
    message.flags = TWAI_MSG_FLAG_NONE; // Standard frame format
    
    // Copy data
    for (int i = 0; i < length; i++) {
        message.data[i] = data[i];
    }
    
    // Queue message for transmission
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(100));
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to queue message for transmission: %s", esp_err_to_name(result));
        return false;
    }
    ESP_LOGI(TAG, "CAN message sent with ID 0x%03" PRIx32 ", length %d", id, length);
    
    return true;
}

void CANBusHandler::updateLeftIndicator(bool state) {
    // Only send if state has changed
    if (state != prev_left_indicator) {
        prev_left_indicator = state;
        
        // Prepare data: single byte with 0 for OFF, 1 for ON
        uint8_t data = state ? 1 : 0;
        
        // Send CAN message
        if (sendCANMessage(CAN_ID_LEFT_INDICATOR, &data, 1)) {
            ESP_LOGI(TAG, "Left indicator state changed to %s", state ? "ON" : "OFF");
        }
    }
}

void CANBusHandler::updateRightIndicator(bool state) {
    // Only send if state has changed
    if (state != prev_right_indicator) {
        prev_right_indicator = state;
        
        // Prepare data: single byte with 0 for OFF, 1 for ON
        uint8_t data = state ? 1 : 0;
        
        // Send CAN message
        if (sendCANMessage(CAN_ID_RIGHT_INDICATOR, &data, 1)) {
            ESP_LOGI(TAG, "Right indicator state changed to %s", state ? "ON" : "OFF");
        }
    }
}

void CANBusHandler::updateHeadlight(bool state) {
    // Only send if state has changed
    if (state != prev_headlight) {
        prev_headlight = state;
        
        // Prepare data: single byte with 0 for OFF, 1 for ON
        uint8_t data = state ? 1 : 0;
        
        // Send CAN message
        if (sendCANMessage(CAN_ID_HEADLIGHT, &data, 1)) {
            ESP_LOGI(TAG, "Headlight state changed to %s", state ? "ON" : "OFF");
        }
    }
}

void CANBusHandler::updateFoglight(bool state) {
    // Only send if state has changed
    if (state != prev_foglight) {
        prev_foglight = state;
        
        // Prepare data: single byte with 0 for OFF, 1 for ON
        uint8_t data = state ? 1 : 0;
        
        // Send CAN message
        if (sendCANMessage(CAN_ID_FOGLIGHT, &data, 1)) {
            ESP_LOGI(TAG, "Foglight state changed to %s", state ? "ON" : "OFF");
        }
    }
}

void CANBusHandler::updateLeftWarning(bool state) {
    // Only send if state has changed
    if (state != prev_leftWarning) {
        prev_leftWarning = state;
        
        // Prepare data: single byte with 0 for OFF, 1 for ON
        uint8_t data = state ? 1 : 0;
        
        // Send CAN message
        if (sendCANMessage(CAN_ID_LEFT_INDICATOR_WARNING, &data, 1)) {
            ESP_LOGI(TAG, "Left indicator warning state changed to %s", state ? "ON" : "OFF");
        }
    }
}

void CANBusHandler::updateRightWarning(bool state) {
    // Only send if state has changed
    if (state != prev_rightWarning) {
        prev_rightWarning = state;
        
        // Prepare data: single byte with 0 for OFF, 1 for ON
        uint8_t data = state ? 1 : 0;
        
        // Send CAN message
        if (sendCANMessage(CAN_ID_RIGHT_INDICATOR_WARNING, &data, 1)) {
            ESP_LOGI(TAG, "Right indicator warning state changed to %s", state ? "ON" : "OFF");
        }
    }
}

void CANBusHandler::updateSteeringAngle(int angle) {

    if (angle < -180) angle = -180;
    if (angle > 180) angle = 180;
    //Only send if value +10 / -10 
    if(abs(angle - prev_angle) >= 10) {
        uint8_t data[1];
        data[0] = map(angle, -180, 180, 0, 255);

        if(sendCANMessage(CAN_ID_STEERING_ANGLE, data, 1)) {
            ESP_LOGI(TAG, "Steering angle changed to %d from %d", angle, prev_angle);
            prev_angle = angle;
        }
             
    }
}

long CANBusHandler::map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}