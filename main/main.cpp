//main.cpp

#include "TurnIndicatorSystem.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

LightIndicatorSystem indicatorSystem;

extern "C" void app_main() {
    indicatorSystem.begin();

    while (true) {
        indicatorSystem.update();
        
        // The logging is now handled inside the update() method
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay to avoid rapid polling
    }
}
