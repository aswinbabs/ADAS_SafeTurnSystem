//pinDefinition.hpp


#pragma once

#ifndef PIN_DEFINITIONS_HPP
#define PIN_DEFINITIONS_HPP

#include "driver/gpio.h"

// Indicator Lights
static const gpio_num_t LEFT_INDICATOR_PIN = GPIO_NUM_21;   
static const gpio_num_t RIGHT_INDICATOR_PIN = GPIO_NUM_22; 
static const gpio_num_t LEFT_SWITCH_PIN = GPIO_NUM_19;
static const gpio_num_t RIGHT_SWITCH_PIN = GPIO_NUM_18;

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
