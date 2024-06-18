#include "NVStorage.h"

static const char *TAG = "NVStorage";


void NVStorage::init(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}

void NVStorage::loadSystemParams(){
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,"Reading system params from NVS ...");

        nvs_get_i8(nvs_handler, "dhcp", (int8_t*)&systemParams.dhcp);
        nvs_get_i8(nvs_handler, "ip1", (int8_t*)&systemParams.ip[0]);
        nvs_get_i8(nvs_handler, "ip2", (int8_t*)&systemParams.ip[1]);
        nvs_get_i8(nvs_handler, "ip3", (int8_t*)&systemParams.ip[2]);
        nvs_get_i8(nvs_handler, "ip4", (int8_t*)&systemParams.ip[3]);
        nvs_get_i8(nvs_handler, "ma1", (int8_t*)&systemParams.mask[0]);
        nvs_get_i8(nvs_handler, "ma2", (int8_t*)&systemParams.mask[1]);
        nvs_get_i8(nvs_handler, "ma3", (int8_t*)&systemParams.mask[2]);
        nvs_get_i8(nvs_handler, "ma4", (int8_t*)&systemParams.mask[3]);
        nvs_get_i8(nvs_handler, "gw1", (int8_t*)&systemParams.gateway[0]);
        nvs_get_i8(nvs_handler, "gw2", (int8_t*)&systemParams.gateway[1]);
        nvs_get_i8(nvs_handler, "gw3", (int8_t*)&systemParams.gateway[2]);
        nvs_get_i8(nvs_handler, "gw4", (int8_t*)&systemParams.gateway[3]);

        nvs_close(nvs_handler);
    }
}

void NVStorage::saveSystemParams(){
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Saving system params to NVS ...");
        if (systemParams.dhcp == false){
            nvs_set_i8(nvs_handler, "ip1", systemParams.ip[0]);
            nvs_set_i8(nvs_handler, "ip2", systemParams.ip[1]);
            nvs_set_i8(nvs_handler, "ip3", systemParams.ip[2]);
            nvs_set_i8(nvs_handler, "ip4", systemParams.ip[3]);
            nvs_set_i8(nvs_handler, "ma1", systemParams.mask[0]);
            nvs_set_i8(nvs_handler, "ma2", systemParams.mask[1]);
            nvs_set_i8(nvs_handler, "ma3", systemParams.mask[2]);
            nvs_set_i8(nvs_handler, "ma4", systemParams.mask[3]);
            nvs_set_i8(nvs_handler, "gw1", systemParams.gateway[0]);
            nvs_set_i8(nvs_handler, "gw2", systemParams.gateway[1]);
            nvs_set_i8(nvs_handler, "gw3", systemParams.gateway[2]);
            nvs_set_i8(nvs_handler, "gw4", systemParams.gateway[3]);
        }
        nvs_set_i8(nvs_handler, "dhcp", systemParams.dhcp);

        nvs_commit(nvs_handler);
        nvs_close(nvs_handler);
    }
}

void NVStorage::loadBrakeMotorParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,"Reading brake motor params from NVS ...");
        nvs_get_u32(nvs_handler, "br_disabledPos",      (uint32_t*)&brakeMotorParams.disabledPos);
        nvs_get_u32(nvs_handler, "br_zeroPos",          (uint32_t*)&brakeMotorParams.zeroPos);
        nvs_get_u32(nvs_handler, "br_pressedPos",       (uint32_t*)&brakeMotorParams.pressedPos);
        nvs_get_u32(nvs_handler, "br_curProp",          (uint32_t*)&brakeMotorParams.curProp);
        nvs_get_u32(nvs_handler, "br_curIntegral",      (uint32_t*)&brakeMotorParams.curIntegral);
        nvs_get_u32(nvs_handler, "br_curDiff",          (uint32_t*)&brakeMotorParams.curDiff);
        nvs_get_u32(nvs_handler, "br_curRamp",          (uint32_t*)&brakeMotorParams.curRamp);
        nvs_get_u32(nvs_handler, "br_curLimit",         (uint32_t*)&brakeMotorParams.curLimit);
        nvs_get_u32(nvs_handler, "br_curFilter",        (uint32_t*)&brakeMotorParams.curFilter);
        nvs_get_u32(nvs_handler, "br_velProp",          (uint32_t*)&brakeMotorParams.velProp);
        nvs_get_u32(nvs_handler, "br_velIntegral",      (uint32_t*)&brakeMotorParams.velIntegral);
        nvs_get_u32(nvs_handler, "br_velDiff",          (uint32_t*)&brakeMotorParams.velDiff);
        nvs_get_u32(nvs_handler, "br_velRamp",          (uint32_t*)&brakeMotorParams.velRamp);
        nvs_get_u32(nvs_handler, "br_velLimit",         (uint32_t*)&brakeMotorParams.velLimit);
        nvs_get_u32(nvs_handler, "br_velFilter",        (uint32_t*)&brakeMotorParams.velFilter);
        nvs_get_u32(nvs_handler, "br_posProp",          (uint32_t*)&brakeMotorParams.posProp);
        nvs_get_u32(nvs_handler, "br_posLimit",         (uint32_t*)&brakeMotorParams.posLimit);
        nvs_get_u32(nvs_handler, "br_curOffset",        (uint32_t*)&brakeMotorParams.curOffset);

        nvs_close(nvs_handler);
    }
}

void NVStorage::saveBrakeMotorParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Saving brake motor params to NVS ...");
        nvs_set_u32(nvs_handler, "br_disabledPos",      *(uint32_t*)&brakeMotorParams.disabledPos);
        nvs_set_u32(nvs_handler, "br_zeroPos",          *(uint32_t*)&brakeMotorParams.zeroPos);
        nvs_set_u32(nvs_handler, "br_pressedPos",       *(uint32_t*)&brakeMotorParams.pressedPos);
        nvs_set_u32(nvs_handler, "br_curProp",          *(uint32_t*)&brakeMotorParams.curProp);
        nvs_set_u32(nvs_handler, "br_curIntegral",      *(uint32_t*)&brakeMotorParams.curIntegral);
        nvs_set_u32(nvs_handler, "br_curDiff",          *(uint32_t*)&brakeMotorParams.curDiff);
        nvs_set_u32(nvs_handler, "br_curRamp",          *(uint32_t*)&brakeMotorParams.curRamp);
        nvs_set_u32(nvs_handler, "br_curLimit",         *(uint32_t*)&brakeMotorParams.curLimit);
        nvs_set_u32(nvs_handler, "br_curFilter",        *(uint32_t*)&brakeMotorParams.curFilter);
        nvs_set_u32(nvs_handler, "br_velProp",          *(uint32_t*)&brakeMotorParams.velProp);
        nvs_set_u32(nvs_handler, "br_velIntegral",      *(uint32_t*)&brakeMotorParams.velIntegral);
        nvs_set_u32(nvs_handler, "br_velDiff",          *(uint32_t*)&brakeMotorParams.velDiff);
        nvs_set_u32(nvs_handler, "br_velRamp",          *(uint32_t*)&brakeMotorParams.velRamp);
        nvs_set_u32(nvs_handler, "br_velLimit",         *(uint32_t*)&brakeMotorParams.velLimit);
        nvs_set_u32(nvs_handler, "br_velFilter",        *(uint32_t*)&brakeMotorParams.velFilter);
        nvs_set_u32(nvs_handler, "br_posProp",          *(uint32_t*)&brakeMotorParams.posProp);
        nvs_set_u32(nvs_handler, "br_posLimit",         *(uint32_t*)&brakeMotorParams.posLimit);
        nvs_set_u32(nvs_handler, "br_curOffset",        *(uint32_t*)&brakeMotorParams.curOffset);
        
        nvs_commit(nvs_handler);
        nvs_close(nvs_handler);
    }
}

void NVStorage::loadClutchMotorParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,"Reading clutch motor params from NVS ...");
        nvs_get_u32(nvs_handler, "cl_disabledPos",      (uint32_t*)&clutchMotorParams.disabledPos);
        nvs_get_u32(nvs_handler, "cl_zeroPos",          (uint32_t*)&clutchMotorParams.zeroPos);
        nvs_get_u32(nvs_handler, "cl_fullEngPos",       (uint32_t*)&clutchMotorParams.fullEngPos);
        nvs_get_u32(nvs_handler, "cl_minEngPos",        (uint32_t*)&clutchMotorParams.minEngPos);
        nvs_get_u32(nvs_handler, "cl_pressedPos",       (uint32_t*)&clutchMotorParams.pressedPos);
        nvs_get_u32(nvs_handler, "cl_curProp",          (uint32_t*)&clutchMotorParams.curProp);
        nvs_get_u32(nvs_handler, "cl_curIntegral",      (uint32_t*)&clutchMotorParams.curIntegral);
        nvs_get_u32(nvs_handler, "cl_curDiff",          (uint32_t*)&clutchMotorParams.curDiff);
        nvs_get_u32(nvs_handler, "cl_curRamp",          (uint32_t*)&clutchMotorParams.curRamp);
        nvs_get_u32(nvs_handler, "cl_curLimit",         (uint32_t*)&clutchMotorParams.curLimit);
        nvs_get_u32(nvs_handler, "cl_curFilter",        (uint32_t*)&clutchMotorParams.curFilter);
        nvs_get_u32(nvs_handler, "cl_velProp",          (uint32_t*)&clutchMotorParams.velProp);
        nvs_get_u32(nvs_handler, "cl_velIntegral",      (uint32_t*)&clutchMotorParams.velIntegral);
        nvs_get_u32(nvs_handler, "cl_velDiff",          (uint32_t*)&clutchMotorParams.velDiff);
        nvs_get_u32(nvs_handler, "cl_velRamp",          (uint32_t*)&clutchMotorParams.velRamp);
        nvs_get_u32(nvs_handler, "cl_velLimit",         (uint32_t*)&clutchMotorParams.velLimit);
        nvs_get_u32(nvs_handler, "cl_velFilter",        (uint32_t*)&clutchMotorParams.velFilter);
        nvs_get_u32(nvs_handler, "cl_posProp",          (uint32_t*)&clutchMotorParams.posProp);
        nvs_get_u32(nvs_handler, "cl_posLimit",         (uint32_t*)&clutchMotorParams.posLimit);
        nvs_get_u32(nvs_handler, "cl_curOffset",        (uint32_t*)&clutchMotorParams.curOffset);

        nvs_close(nvs_handler);
    }
}

void NVStorage::saveClutchMotorParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Saving clutch motor params to NVS ...");
        nvs_set_u32(nvs_handler, "cl_disabledPos",      *(uint32_t*)&clutchMotorParams.disabledPos);
        nvs_set_u32(nvs_handler, "cl_zeroPos",          *(uint32_t*)&clutchMotorParams.zeroPos);
        nvs_set_u32(nvs_handler, "cl_fullEngPos",       *(uint32_t*)&clutchMotorParams.fullEngPos);
        nvs_set_u32(nvs_handler, "cl_minEngPos",        *(uint32_t*)&clutchMotorParams.minEngPos);
        nvs_set_u32(nvs_handler, "cl_pressedPos",       *(uint32_t*)&clutchMotorParams.pressedPos);
        nvs_set_u32(nvs_handler, "cl_curProp",          *(uint32_t*)&clutchMotorParams.curProp);
        nvs_set_u32(nvs_handler, "cl_curIntegral",      *(uint32_t*)&clutchMotorParams.curIntegral);
        nvs_set_u32(nvs_handler, "cl_curDiff",          *(uint32_t*)&clutchMotorParams.curDiff);
        nvs_set_u32(nvs_handler, "cl_curRamp",          *(uint32_t*)&clutchMotorParams.curRamp);
        nvs_set_u32(nvs_handler, "cl_curLimit",         *(uint32_t*)&clutchMotorParams.curLimit);
        nvs_set_u32(nvs_handler, "cl_curFilter",        *(uint32_t*)&clutchMotorParams.curFilter);
        nvs_set_u32(nvs_handler, "cl_velProp",          *(uint32_t*)&clutchMotorParams.velProp);
        nvs_set_u32(nvs_handler, "cl_velIntegral",      *(uint32_t*)&clutchMotorParams.velIntegral);
        nvs_set_u32(nvs_handler, "cl_velDiff",          *(uint32_t*)&clutchMotorParams.velDiff);
        nvs_set_u32(nvs_handler, "cl_velRamp",          *(uint32_t*)&clutchMotorParams.velRamp);
        nvs_set_u32(nvs_handler, "cl_velLimit",         *(uint32_t*)&clutchMotorParams.velLimit);
        nvs_set_u32(nvs_handler, "cl_velFilter",        *(uint32_t*)&clutchMotorParams.velFilter);
        nvs_set_u32(nvs_handler, "cl_posProp",          *(uint32_t*)&clutchMotorParams.posProp);
        nvs_set_u32(nvs_handler, "cl_posLimit",         *(uint32_t*)&clutchMotorParams.posLimit);
        nvs_set_u32(nvs_handler, "cl_curOffset",        *(uint32_t*)&clutchMotorParams.curOffset);
        
        nvs_commit(nvs_handler);
        nvs_close(nvs_handler);
    }
}