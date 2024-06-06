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

struct brake_motor_params {
    bool calibrate = true;
    bool inverseEncoder = false;
    float encoderAngle = 3.14;
    float voltageLimit = 6;
};

struct gearbox_motor_params {
    bool calibrate = true;
    bool inverseEncoder = false;
    float encoderAngle = 3.14;
    float voltageLimit = 6;
    float velLimitHard = 999;
    float velProp = 1;
    float velIntegral = 25;
    float velDiff = 0.003;
    float velRamp = 1000;
    float velLimit = 7;
    float velFilter = 0.002;
    float angleProp = 47;
    float angleLimit = 100;
};

extern system_params systemParams;

extern gearbox_motor_params motorYparams;

extern gearbox_motor_params motorXparams;


void nvs_init();

void loadSystemParams();

void saveSystemParams();

void loadYMotorParams();

void saveYMotorParams();

void loadXMotorParams();

void saveXMotorParams();