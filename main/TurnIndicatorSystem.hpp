//TurnIndicatorSystem.hpp

#ifndef LIGHT_INDICATOR_SYSTEM_HPP
#define LIGHT_INDICATOR_SYSTEM_HPP

#include <esp_timer.h>
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "PinDefinitions.hpp"
#include <string>
#include <cmath>
#include <driver/ledc.h>
#include <iostream>

// MPU6050 Constants
#define MPU6050_ADDR          0x68    // MPU6050 I2C address
#define MPU6050_WHO_AM_I      0x75    // Identity register
#define MPU6050_PWR_MGMT_1    0x6B    // Power management register
#define MPU6050_GYRO_CONFIG   0x1B    // Gyroscope configuration register
#define MPU6050_ACCEL_CONFIG  0x1C    // Accelerometer configuration register
#define MPU6050_GYRO_XOUT_H   0x43    // Gyroscope data register
#define MPU6050_ACCEL_XOUT_H  0x3B    // Accelerometer data register

// I2C Constants
#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_FREQ_HZ    100000  // Try 100kHz instead of 400kHz
#define I2C_TIMEOUT_MS        1000

// LEDC constants for buzzer control
#define BUZZER_CHANNEL LEDC_CHANNEL_0
#define BUZZER_TIMER LEDC_TIMER_0
#define BUZZER_RESOLUTION LEDC_TIMER_10_BIT  // 10-bit resolution (0-1023)

// Define sound frequencies and durations
#define TICK_FREQ           3200    // Tick frequency in Hz
#define TICK_DURATION       15      // Tick duration in ms
#define TOCK_FREQ           2400    // Tock frequency in Hz
#define TOCK_DURATION       20      // Tock duration in ms
#define WARNING_FREQ        2000    // Warning frequency in Hz
#define WARNING_BEEP_DUR    200     // Warning beep duration in ms
#define WARNING_BEEP_GAP    200     // Gap between warning beeps
#define WARNING_NUM_BEEPS   3       // Number of beeps in warning sequence
#define LEDC_DUTY           77      // ~30% duty cycle for normal sounds
#define WARNING_DUTY        128     // 50% duty cycle for warning (louder)

class LightIndicatorSystem {
public:
    LightIndicatorSystem();
    void begin();
    void update();
    int readSteeringAngle();         // Read angle from potentiometer
    int readMPUAngle();              // Read angle from MPU6050
    std::string getSteeringDirection(int angle);  // Get direction as a string
    void compareAndLogReadings();    // Compare both sensors and log result
    void blinkIndicator();
    void warningTone();

private:
    void handleIndicator(gpio_num_t switchPin, gpio_num_t indicatorPin, bool &state);
    bool initI2C();                  // Initialize I2C communication
    bool initMPU6050();              // Initialize MPU6050 sensor
    bool writeMPURegister(uint8_t reg, uint8_t data);   // Write to MPU register
    bool readMPURegisters(uint8_t reg, uint8_t* data, size_t len); // Read MPU registers
    bool initLEDC();

    bool leftIndicatorState;
    bool rightIndicatorState;
    bool headlightState;
    bool foglightState;
    int steeringAnglePot;           // Angle from potentiometer
    int steeringAngleMPU;           // Angle from MPU6050
    bool mpuInitialized;            // Flag to check if MPU is initialized
    bool buzzerInitialized;

    unsigned long lastIndicatorToggle = 0;
    unsigned long lastWarningToggle = 0;
    bool indicatorState = false;
    bool warningActive = false;
    unsigned long warningStartTime = 0;
    TaskHandle_t warningToneTaskHandle = NULL;
    //ledc library for buzzer toning
    enum buzzerTone {
        TICK, 
        TOCK,
        WARNING, 
        OFF };
    buzzerTone currentTone = OFF;
    unsigned long toneStartTime = 0;
    void setTone(buzzerTone tone);

};

#endif // LIGHT_INDICATOR_SYSTEM_HPP

