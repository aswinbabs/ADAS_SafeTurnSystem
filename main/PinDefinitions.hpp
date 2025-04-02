//pinDefinition.hpp


#pragma once

#ifndef PIN_DEFINITIONS_HPP
#define PIN_DEFINITIONS_HPP

#include "driver/gpio.h"

// Indicator Lights
static const gpio_num_t LEFT_INDICATOR_PIN = GPIO_NUM_21;
static const gpio_num_t RIGHT_INDICATOR_PIN = GPIO_NUM_22;
static const gpio_num_t LEFT_SWITCH_PIN = GPIO_NUM_18;
static const gpio_num_t RIGHT_SWITCH_PIN = GPIO_NUM_19;

// Headlight and Fog Light
static const gpio_num_t HEADLIGHT_SW_PIN = GPIO_NUM_16;
static const gpio_num_t FOGGLIGHT_SW_PIN = GPIO_NUM_15;
static const gpio_num_t HEADLIGHT_PIN = GPIO_NUM_17;
static const gpio_num_t FOGGLIGHT_PIN = GPIO_NUM_5;

// Buzzer
static const gpio_num_t BUZZER_PIN = GPIO_NUM_23;

// Steering Sensor (Potentiometer)
static const gpio_num_t STEERING_SENSOR_PIN = GPIO_NUM_34; // ADC1 channel 6

// I2C Pins for MPU6050
static const gpio_num_t I2C_SDA_PIN = GPIO_NUM_25;
static const gpio_num_t I2C_SCL_PIN = GPIO_NUM_26;

#endif // PIN_DEFINITIONS_HPP



// #ifndef PIN_DEFINITIONS_HPP
// #define PIN_DEFINITIONS_HPP

// #include "driver/gpio.h"

// // Indicator Lights
// static const gpio_num_t LEFT_INDICATOR_PIN = GPIO_NUM_21;
// static const gpio_num_t RIGHT_INDICATOR_PIN = GPIO_NUM_22;
// static const gpio_num_t LEFT_SWITCH_PIN = GPIO_NUM_18;
// static const gpio_num_t RIGHT_SWITCH_PIN = GPIO_NUM_19;

// // Headlight and Fog Light
// static const gpio_num_t HEADLIGHT_SW_PIN = GPIO_NUM_16;
// static const gpio_num_t FOGGLIGHT_SW_PIN = GPIO_NUM_15;
// static const gpio_num_t HEADLIGHT_PIN = GPIO_NUM_17;
// static const gpio_num_t FOGGLIGHT_PIN = GPIO_NUM_5;

// // Buzzer
// static const gpio_num_t BUZZER_PIN = GPIO_NUM_23;

// // Steering Sensor (Potentiometer)
// static const gpio_num_t STEERING_SENSOR_PIN = GPIO_NUM_34; // ADC1 channel 6

// #endif // PIN_DEFINITIONS_HPP




// #ifndef PIN_DEFINITIONS_HPP
// #define PIN_DEFINITIONS_HPP

// #include "driver/gpio.h"

// // Define GPIO pins
// #define LEFT_INDICATOR_PIN   GPIO_NUM_2   
// #define RIGHT_INDICATOR_PIN  GPIO_NUM_4
// #define LEFT_SWITCH_PIN      GPIO_NUM_5
// #define RIGHT_SWITCH_PIN     GPIO_NUM_18
// #define BUZZER_PIN           GPIO_NUM_19


// // Headlight/Fog light pins
// static const gpio_num_t HEADLIGHT_SW_PIN = GPIO_NUM_16;    // Headlight switch
// static const gpio_num_t FOGGLIGHT_SW_PIN = GPIO_NUM_15;    // Fog light switch
// static const gpio_num_t HEADLIGHT_PIN = GPIO_NUM_17;       // Headlight output
// static const gpio_num_t FOGGLIGHT_PIN = GPIO_NUM_5;        // Fog light output

// // I2C pins for sensors
// static const gpio_num_t I2C_SDA_PIN = GPIO_NUM_21;
// static const gpio_num_t I2C_SCL_PIN = GPIO_NUM_22;

// #endif // PIN_DEFINITIONS_HPP








// #include "driver/gpio.h"

// // Pin definitions for the entire system
// namespace PinDefs {
//     // Indicator pins
//     static const gpio_num_t LEFT_INDICATOR_PIN = GPIO_NUM_18;  // Left indicator light
//     static const gpio_num_t RIGHT_INDICATOR_PIN = GPIO_NUM_19; // Right indicator light
//     static const gpio_num_t LEFT_SWITCH_PIN = GPIO_NUM_4;      // Left indicator switch
//     static const gpio_num_t RIGHT_SWITCH_PIN = GPIO_NUM_2;     // Right indicator switch
    
//     // Headlight/Fog light pins
//     static const gpio_num_t HEADLIGHT_SW_PIN = GPIO_NUM_16;    // Headlight switch
//     static const gpio_num_t FOGGLIGHT_SW_PIN = GPIO_NUM_15;    // Fog light switch
//     static const gpio_num_t HEADLIGHT_PIN = GPIO_NUM_17;       // Headlight output
//     static const gpio_num_t FOGGLIGHT_PIN = GPIO_NUM_5;        // Fog light output
    
//     // I2C pins for sensors
//     //static const gpio_num_t I2C_SDA_PIN = GPIO_NUM_21;
//     //static const gpio_num_t I2C_SCL_PIN = GPIO_NUM_22;
    
//     // Buzzer pin (from original code)
//     static const gpio_num_t BUZZER_PIN = GPIO_NUM_26;
// }