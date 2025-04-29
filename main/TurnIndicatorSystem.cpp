//TurnIndicatorSystem.cpp
#include "TurnIndicatorSystem.hpp"



LightIndicatorSystem::LightIndicatorSystem() 
    : warningFlag(false), noLeftIndicator(false), noRightIndicator(false), leftIndicatorState(false), rightIndicatorState(false), 
      headlightState(false), foglightState(false), 
      steeringAnglePot(0), steeringAngleMPU(0), mpuInitialized(false), CANBusManager(nullptr), CANBusAvailable(false) {}

void LightIndicatorSystem::begin() {
    // Set GPIO directions
    gpio_set_direction(LEFT_INDICATOR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RIGHT_INDICATOR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(HEADLIGHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(FOGGLIGHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    
    gpio_set_direction(LEFT_SWITCH_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(RIGHT_SWITCH_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(HEADLIGHT_SW_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(FOGGLIGHT_SW_PIN, GPIO_MODE_INPUT);

    gpio_set_pull_mode(LEFT_SWITCH_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(RIGHT_SWITCH_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(HEADLIGHT_SW_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(FOGGLIGHT_SW_PIN, GPIO_PULLUP_ONLY);

    // Configure ADC for steering sensor
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);

    // Initialize I2C and MPU6050
    mpuInitialized = initI2C() && initMPU6050();
    if (!mpuInitialized) {
        ESP_LOGE("MPU6050", "Failed to initialize MPU6050");
    } else {
        ESP_LOGI("MPU6050", "MPU6050 initialized successfully");
    }
    buzzerInitialized = initLEDC();
    if(!buzzerInitialized){
        ESP_LOGE("LEDC", "Failed to initialze LEDC");
    }
    else{
        ESP_LOGI("LEDC", "LEDC initialized successfully");
    }

    // Create a one-shot timer. It will call toneOffCallback once after the delay.
    toneTimerHandle = xTimerCreate("ToneTimer", pdMS_TO_TICKS(100), pdFALSE, this, toneOffCallbackStatic);
    if (toneTimerHandle == NULL) {
        ESP_LOGE("TIMER", "Failed to create tone timer");
    }

}

void LightIndicatorSystem::setCAN(CANBusHandler* canBUS) {
    CANBusManager = canBUS;
    CANBusAvailable = (canBUS != nullptr);
    ESP_LOGI("LightIndicatorSystem", "CAN BUS manager %s", CANBusAvailable ? "connected" : "not available");
}

bool LightIndicatorSystem::initI2C() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "I2C parameter configuration failed: %s", esp_err_to_name(err));
        return false;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "I2C driver installation failed: %s", esp_err_to_name(err));
        return false;
    }
    
    return true;
}

bool LightIndicatorSystem::initLEDC() {
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = BUZZER_TIMER;
    ledc_timer.duty_resolution = LEDC_TIMER_8_BIT; // 8-bit resolution
    ledc_timer.freq_hz = 2000;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&ledc_timer);
    if(err != ESP_OK) {
        ESP_LOGE("LEDC", "LEDC timer configuration failed: %s", esp_err_to_name(err));
        return false;
    }

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num = BUZZER_PIN;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = BUZZER_CHANNEL;
    ledc_channel.timer_sel = BUZZER_TIMER;
    ledc_channel.duty = 0; // Initially off
    ledc_channel.hpoint = 0;
    
    err = ledc_channel_config(&ledc_channel);
    if(err != ESP_OK) {
        ESP_LOGE("LEDC", "LEDC channel configuration failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

bool LightIndicatorSystem::initMPU6050() {
    // Wake up the MPU6050
    if (!writeMPURegister(MPU6050_PWR_MGMT_1, 0x00)) {
        ESP_LOGE("MPU6050", "Failed to wake up MPU6050");
        return false;
    }
    
    // Check device ID
    uint8_t id;
    if (!readMPURegisters(MPU6050_WHO_AM_I, &id, 1) || id != MPU6050_ADDR) {
        ESP_LOGE("MPU6050", "MPU6050 ID verification failed. Expected: 0x%02x, Got: 0x%02x", MPU6050_ADDR, id);
        return false;
    }
    
    // Configure gyroscope range (±250°/s)
    if (!writeMPURegister(MPU6050_GYRO_CONFIG, 0x00)) {
        ESP_LOGE("MPU6050", "Failed to configure gyroscope");
        return false;
    }
    
    // Configure accelerometer range (±2g)
    if (!writeMPURegister(MPU6050_ACCEL_CONFIG, 0x00)) {
        ESP_LOGE("MPU6050", "Failed to configure accelerometer");
        return false;
    }
    
    return true;
}

bool LightIndicatorSystem::writeMPURegister(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to write to register 0x%02x: %s", reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool LightIndicatorSystem::readMPURegisters(uint8_t reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to read from register 0x%02x: %s", reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

int LightIndicatorSystem::readSteeringAngle() {
    int rawValue = adc1_get_raw(ADC1_CHANNEL_6);
    
    // Convert ADC value (0-4095) to Angle (-180 to +180 degrees)
    steeringAnglePot = ((rawValue - 2048) * 180) / 2048;

    return steeringAnglePot;
}

int LightIndicatorSystem::readMPUAngle() {
    if (!mpuInitialized) {
        ESP_LOGW("MPU6050", "MPU6050 not initialized. Cannot read angle.");
        return 0;
    }
    
    // Read accelerometer data
    uint8_t data[6];
    if (!readMPURegisters(MPU6050_ACCEL_XOUT_H, data, 6)) {
        ESP_LOGE("MPU6050", "Failed to read accelerometer data");
        return 0;
    }
    else {
        ESP_LOGD("MPU6050", "Raw Accelerometer Data: %d %d %d", data[0], data[1], data[2]);
    }
    
    // Convert the data to 16-bit signed values
    int16_t accelX = (data[0] << 8) | data[1];
    int16_t accelY = (data[2] << 8) | data[3];
    int16_t accelZ = (data[4] << 8) | data[5];
    
    // Calculate roll angle from accelerometer (rotation around X-axis)
    // arctan(y/sqrt(x^2 + z^2)) * (180/PI) to convert radians to degrees
    double roll = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 57.3; // 57.3 = 180/PI
    
    // Convert roll to steering angle (-180 to +180)
    steeringAngleMPU = (int)roll;
    
    steeringAngleMPU = -steeringAngleMPU; 
    
    return steeringAngleMPU;
}

std::string LightIndicatorSystem::getSteeringDirection(int angle) {
    if (angle < -60) return "LEFT";
    if (angle > 60) return "RIGHT";
    return "STRAIGHT";
}

void LightIndicatorSystem::compareAndLogReadings() {
    std::string potDirection = getSteeringDirection(steeringAnglePot);
    std::string mpuDirection = getSteeringDirection(steeringAngleMPU);
    
    ESP_LOGD("SENSORS", "Pot Angle: %d, MPU Angle: %d", steeringAnglePot, steeringAngleMPU);
    
    // Only log direction if both sensors agree, otherwise log mismatch
    if (potDirection == mpuDirection) {
        ESP_LOGI("DIRECTION", "MATCH: %s (Pot: %d, MPU: %d)", 
                 potDirection.c_str(), steeringAnglePot, steeringAngleMPU);
    } else {
        ESP_LOGW("DIRECTION", "MISMATCH: POT=%s (%d), MPU=%s (%d)", 
                 potDirection.c_str(), steeringAnglePot, 
                 mpuDirection.c_str(), steeringAngleMPU);
    }
}

void LightIndicatorSystem::blinkIndicator() {
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check if it's time to toggle the indicator
    if(currentTime - lastIndicatorToggle >= TOGGLEPERIOD) {
        lastIndicatorToggle = currentTime;
        
        // Toggle between TICK and TOCK
        tickState = !tickState;
        if(!warningFlag)
        {
            if(tickState) {
                setTone(TICK);
            } else {
                setTone(TOCK);
            }
        }
        // Handle indicator lights 
        if (leftIndicatorState) {
            gpio_set_level(LEFT_INDICATOR_PIN, tickState ? 1 : 0);
            gpio_set_level(RIGHT_INDICATOR_PIN, 0);
        }
        
        if (rightIndicatorState) {
            gpio_set_level(RIGHT_INDICATOR_PIN, tickState ? 1 : 0);
            gpio_set_level(LEFT_INDICATOR_PIN, 0);
        }
        
        if(leftIndicatorState && rightIndicatorState) {
            gpio_set_level(LEFT_INDICATOR_PIN, tickState ? 1 : 0);
            gpio_set_level(RIGHT_INDICATOR_PIN, tickState ? 1 : 0);
        }
    }
}

void LightIndicatorSystem::warningTone() {
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if(!warningActive) {
        warningActive = true;
        warningStartTime = currentTime;
        setTone(WARNING);
    }
    else {
        if(currentTime - warningStartTime >= WARNING_BEEP_DUR) {    
            warningActive = false;
        }
    }
}

void LightIndicatorSystem::setTone(buzzerTone tone) {
    currentTone = tone;
    
    // Stop any existing timer before starting a new one
    if (toneTimerHandle != NULL) {
        xTimerStop(toneTimerHandle, 0);
    }
    
    uint32_t toneDuration = 0;
    uint32_t toneFrequency = 0;
    uint32_t dutyCycle = LEDC_DUTY; 

    switch(tone) {   
        case WARNING:
            toneFrequency = WARNING_FREQ;
            toneDuration = WARNING_BEEP_DUR; 
            break;
        case TICK:
            toneFrequency = TICK_FREQ;
            toneDuration = TICK_DURATION;
            break;   
        case TOCK:
            toneFrequency = TOCK_FREQ;
            toneDuration = TOCK_DURATION;
            break; 
        case OFF:
            // Immediately turn off
            ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
            return;
    }
    
    // Configure and start the tone
    ledc_set_freq(LEDC_LOW_SPEED_MODE, BUZZER_TIMER, toneFrequency);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, dutyCycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
    
    // Start the timer to turn the tone off after tone Duration 
    if (toneTimerHandle != NULL) {
        xTimerChangePeriod(toneTimerHandle, pdMS_TO_TICKS(toneDuration), 0);
        xTimerStart(toneTimerHandle, 0);
    }
}


void LightIndicatorSystem::update() {
    static bool lastLeftSwitchState = false;
    static bool lastRightSwitchState = false;
    static bool lastHeadlightSwitchState = false;
    static bool lastFoglightSwitchState = false;

    bool currentLeftSwitchState = (gpio_get_level(LEFT_SWITCH_PIN) == 0);  // Switch pressed is low
    bool currentRightSwitchState = (gpio_get_level(RIGHT_SWITCH_PIN) == 0);  // Switch pressed is low
    bool currentHeadlightSwitchState = (gpio_get_level(HEADLIGHT_SW_PIN) == 0);  // Switch pressed is low
    bool currentFoglightSwitchState = (gpio_get_level(FOGGLIGHT_SW_PIN) == 0);  // Switch pressed is low

    // Read steering angles from both sensors
    steeringAnglePot = readSteeringAngle();

    if(CANBusAvailable) {
        CANBusManager->updateSteeringAngle(steeringAnglePot);
        ESP_LOGI("CAN", "Sent steering angle: %d to CAN", steeringAnglePot);
    }

    if (mpuInitialized) {
        steeringAngleMPU = readMPUAngle();
        // Compare and log readings (only if MPU is initialized)
        compareAndLogReadings();
    }

    // Update indicator states based on switch press
    if (currentLeftSwitchState != lastLeftSwitchState) {
        leftIndicatorState = currentLeftSwitchState; // Set indicator to on if switch is pressed (low), off if switch is released (high)
        if (!leftIndicatorState) {
            gpio_set_level(LEFT_INDICATOR_PIN, 0);  // Turn off left indicator if switch released
        }
        if(CANBusAvailable) {
            CANBusManager->updateLeftIndicator(leftIndicatorState);
            ESP_LOGI("CAN", "CANBusAvailable and send leftIndicatorState: %d to CAN", leftIndicatorState);
        }
    }

    if (currentRightSwitchState != lastRightSwitchState) {
        rightIndicatorState = currentRightSwitchState; // Set indicator to on if switch is pressed (low), off if switch is released (high)
        if (!rightIndicatorState) {
            gpio_set_level(RIGHT_INDICATOR_PIN, 0);  // Turn off right indicator if switch released
        }

        if(CANBusAvailable) {
            CANBusManager->updateRightIndicator(rightIndicatorState);
            ESP_LOGI("CAN", "CANBusAvailable and send rightIndicatorState: %d to CAN", leftIndicatorState);
        }
    }

    if (currentHeadlightSwitchState != lastHeadlightSwitchState) {
        headlightState = currentHeadlightSwitchState; // Same logic for headlight

        if(CANBusAvailable) {
        CANBusManager->updateHeadlight(headlightState);
        ESP_LOGI("CAN", "CANBusAvailable and send HeadlightState: %d to CAN", headlightState);
        }
    }

    if (currentFoglightSwitchState != lastFoglightSwitchState) {
        foglightState = currentFoglightSwitchState; // Same logic for foglight

        if(CANBusAvailable) {
            CANBusManager->updateFoglight(foglightState);
            ESP_LOGI("CAN", "CANBusAvailable and send FoglightState: %d to CAN", foglightState);
            }

    }

    // Get steering direction from potentiometer (primary sensor)
    std::string potDirection = getSteeringDirection(steeringAnglePot);

    if (potDirection == "LEFT" && !leftIndicatorState) {
    warningTone();  // Alert driver if indicator is not on while turning
    warningFlag = true;
    noLeftIndicator = true;
    if(CANBusAvailable) {
        CANBusManager->updateLeftWarning(noLeftIndicator);
        ESP_LOGI("CAN", "CANBusAvailable and send noLeftIndicator State: %d to CAN", noLeftIndicator);
        }
    } 
    else if (potDirection == "RIGHT" && !rightIndicatorState) {
        warningTone();
        warningFlag = true;
        noRightIndicator = true;
        if(CANBusAvailable) {
            CANBusManager->updateRightWarning(noRightIndicator);
            ESP_LOGI("CAN", "CANBusAvailable and send noRightIndicator State: %d to CAN", noRightIndicator);
            }
    } 
    else {
        setTone(OFF);
        warningFlag = false;
        noLeftIndicator = false;
        noRightIndicator = false;
        if(CANBusAvailable) {
        CANBusManager->updateLeftWarning(noLeftIndicator);
        CANBusManager->updateRightWarning(noRightIndicator);
        ESP_LOGI("CAN", "CANBusAvailable and send noLeftIndicator State: %d & noRightIndicator State: %d to CAN", noLeftIndicator, noRightIndicator);
        }
    } 
    

    // Set GPIO outputs
    if (leftIndicatorState || rightIndicatorState) {
        blinkIndicator();  // Blink the indicator while the switch is pressed
    }
    gpio_set_level(HEADLIGHT_PIN, headlightState);
    gpio_set_level(FOGGLIGHT_PIN, foglightState);

    // Save the current switch states for next update cycle
    lastLeftSwitchState = currentLeftSwitchState;
    lastRightSwitchState = currentRightSwitchState;
    lastHeadlightSwitchState = currentHeadlightSwitchState;
    lastFoglightSwitchState = currentFoglightSwitchState;
}

// Tone off callback function
void LightIndicatorSystem::toneOffCallback() {
    // Turn off the buzzer (set duty cycle to 0)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
}

void LightIndicatorSystem::toneOffCallbackStatic(TimerHandle_t xTimer) {
    // Retrieve the instance pointer from the timer's ID
    LightIndicatorSystem* instance = static_cast<LightIndicatorSystem*>(pvTimerGetTimerID(xTimer));
    if(instance) {
        instance->toneOffCallback();
    }
}