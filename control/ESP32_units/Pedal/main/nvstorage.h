#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

struct system_params {
    bool dhcp = true;
    uint8_t ip[4] = {0};
    uint8_t mask[4] = {0};
    uint8_t gateway[4] = {0};
};

struct pedal_params {
    float lowPosition = 0.3;
    float highPostition = 1.5;
    float DACVoltage = 5;
};

struct wheel_follower_params {
    int leftPosition = 10;
    int centerPosition = 15;
    int rightPosition = 20;
    int angleRange = 90;
    float position_P = 0.0;
    float position_I = 0.0;
    float position_D = 0.0;
    float ramp = 100000;
    float limit = 70;
    float filter = 0.1;
};

extern system_params systemParams;

extern pedal_params pedalParams;

extern wheel_follower_params wheelFollowerParams;


void nvs_init();

void loadSystemParams();

void saveSystemParams();

void loadWheelCalibrationParams();

void saveWheelCalibrationParams();

void loadPedalParams();

void savePedalParams();