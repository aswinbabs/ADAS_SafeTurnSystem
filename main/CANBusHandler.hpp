#ifndef CAN_BUS_HANDLER_HPP
#define CAN_BUS_HANDLER_HPP

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"

class CANBusHandler {
public:
    // Define CAN message IDs
    enum CANMessageID {
        CAN_ID_LEFT_INDICATOR = 0x100,
        CAN_ID_RIGHT_INDICATOR = 0x101,
        CAN_ID_HEADLIGHT = 0x102,
        CAN_ID_FOGLIGHT = 0x103,
        CAN_ID_LEFT_INDICATOR_WARNING = 0x104,
        CAN_ID_RIGHT_INDICATOR_WARNING = 0x105,
        CAN_ID_STEERING_ANGLE = 0x106
    };

    CANBusHandler(gpio_num_t tx_pin, gpio_num_t rx_pin);
    ~CANBusHandler();
    
    bool begin();  // Initialize CAN bus
    
    // Methods to update states
    void updateLeftIndicator(bool state);
    void updateRightIndicator(bool state);
    void updateHeadlight(bool state);
    void updateFoglight(bool state);
    void updateLeftWarning(bool state);
    void updateRightWarning(bool state);
    void updateSteeringAngle(int angle);

    
private:
    gpio_num_t txPin;
    gpio_num_t rxPin;
    bool isInitialized;
    
    // Previous states
    bool prev_left_indicator;
    bool prev_right_indicator;
    bool prev_headlight;
    bool prev_foglight;
    bool prev_leftWarning;
    bool prev_rightWarning;
    int prev_angle;

    
    // Helper method to send a CAN message
    bool sendCANMessage(uint32_t id, const uint8_t* data, uint8_t length);
    long map(long x, long in_min, long in_max, long out_min, long out_max);
};

#endif // CAN_BUS_HANDLER_HPP