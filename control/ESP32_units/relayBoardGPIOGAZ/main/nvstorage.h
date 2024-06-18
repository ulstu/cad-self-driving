#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

struct system_params {
    bool useVlan = 0;
    bool dhcp = true;
    bool ntp = true;
    int32_t vlanid = 0;
    uint8_t ip[4] = {0};
    uint8_t mask[4] = {0};
    uint8_t gateway[4] = {0};
    int8_t timeOffset = 4;
    char ntpAddr[50] = "ru.pool.ntp.org";
};

extern system_params systemParams;

void nvs_init();

void loadSystemParams();

void saveSystemParams();

void loadMesParams();

void saveMesParams();

void saveOutputParams();

void loadOutputsParams();