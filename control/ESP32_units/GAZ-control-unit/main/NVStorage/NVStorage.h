#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

class NVStorage {
public:
    static inline struct system_params_t {
        bool dhcp = true;
        uint8_t ip[4] = {0};
        uint8_t mask[4] = {0};
        uint8_t gateway[4] = {0};
    } systemParams {
        .dhcp = true,
        .ip = {0},
        .mask = {0},
        .gateway = {0}
    };

    static inline struct brake_motor_params_t {
        float disabledPos;
        float zeroPos;
        float pressedPos;
        float curProp;
        float curIntegral;
        float curDiff;
        float curRamp;
        float curLimit;
        float curFilter;
        float velProp;
        float velIntegral;
        float velDiff;
        float velRamp;
        float velLimit;
        float velFilter;
        float posProp;
        float posLimit;
        float curOffset;
    } brakeMotorParams {
        .disabledPos = 0.01,
        .zeroPos = 0.05,
        .pressedPos = 0.45,
        .curProp = 0.02,
        .curIntegral = 17.2,
        .curDiff = 0.000045,
        .curRamp = 100000,
        .curLimit = 1,
        .curFilter = 0.01,
        .velProp = 1,
        .velIntegral = 25,
        .velDiff = 0.003,
        .velRamp = 1000,
        .velLimit = 7,
        .velFilter = 0.01,
        .posProp = 47,
        .posLimit = 100,
        .curOffset = -1
    };

    static inline struct clutch_motor_params_t {
        float disabledPos;
        float zeroPos;
        float fullEngPos;
        float minEngPos;
        float pressedPos;
        float curProp;
        float curIntegral;
        float curDiff;
        float curRamp;
        float curLimit;
        float curFilter;
        float velProp;
        float velIntegral;
        float velDiff;
        float velRamp;
        float velLimit;
        float velFilter;
        float posProp;
        float posLimit;
        float curOffset;
    } clutchMotorParams {
        .disabledPos = 0.01,
        .zeroPos = 0.05,
        .fullEngPos = 0.2,
        .minEngPos = 0.35,
        .pressedPos = 0.45,
        .curProp = 0.02,
        .curIntegral = 17.2,
        .curDiff = 0.000045,
        .curRamp = 100000,
        .curLimit = 1,
        .curFilter = 0.01,
        .velProp = 1,
        .velIntegral = 25,
        .velDiff = 0.003,
        .velRamp = 1000,
        .velLimit = 7,
        .velFilter = 0.01,
        .posProp = 47,
        .posLimit = 100,
        .curOffset = -1
    };

    static void init();

    static void loadSystemParams();
    static void saveSystemParams();

    static void loadBrakeMotorParams();
    static void saveBrakeMotorParams();

    static void loadClutchMotorParams();
    static void saveClutchMotorParams();
};