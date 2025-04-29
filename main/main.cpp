//main.cpp

#include "TurnIndicatorSystem.hpp"
#include "CANBusHandler.hpp"
#include "PinDefinitions.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "Main";

LightIndicatorSystem indicatorSystem;
CANBusHandler CANBUS(CAN_TX_PIN, CAN_RX_PIN);

extern "C" void app_main() {
    ESP_LOGI(TAG, "Starting ADAS Safe Turn System");

    bool CANInitilized = CANBUS.begin();
    if(!CANInitilized) {
        ESP_LOGE(TAG, "Failed to initialize CAN bus");
    }
    else {
        ESP_LOGI(TAG, "CAN bus initialized successfully");
    }

    indicatorSystem.begin();
    if(CANInitilized) {
        indicatorSystem.setCAN(&CANBUS);
    }

    while (true) {
        indicatorSystem.update();
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}
