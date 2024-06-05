#include "nvstorage.h"

static const char *TAG = "NVS";

system_params systemParams;

gearbox_motor_params motorYparams;

gearbox_motor_params motorXparams;


void nvs_init(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}

void loadSystemParams(){
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

void saveSystemParams(){
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

void loadYMotorParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,"Reading brake motor params from NVS ...");

        nvs_get_i8(nvs_handler, "BrCalibrate", (int8_t*)&motorYparams.calibrate);
        nvs_get_i8(nvs_handler, "BrInverseEnc", (int8_t*)&motorYparams.inverseEncoder);
        nvs_get_u32(nvs_handler, "BrEncAngle", (uint32_t*)&motorYparams.encoderAngle);
        nvs_get_u32(nvs_handler, "BrVoltLimit", (uint32_t*)&motorYparams.voltageLimit);

        nvs_close(nvs_handler);
    }
}

void saveYMotorParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Saving brake motor params to NVS ...");
        nvs_set_i8(nvs_handler, "BrCalibrate", motorYparams.calibrate);
        nvs_set_i8(nvs_handler, "BrInverseEnc", motorYparams.inverseEncoder);
        nvs_set_u32(nvs_handler, "BrEncAngle", *(uint32_t*)(&motorYparams.encoderAngle));
        nvs_set_u32(nvs_handler, "BrVoltLimit", *(uint32_t*)(&motorYparams.voltageLimit));
        
        nvs_commit(nvs_handler);
        nvs_close(nvs_handler);
    }
}

void loadXMotorParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,"Reading gearbox motor params from NVS ...");

        nvs_get_i8(nvs_handler, "GbCalibrate", (int8_t*)&motorXparams.calibrate);
        nvs_get_i8(nvs_handler, "GbInverseEnc", (int8_t*)&motorXparams.inverseEncoder);
        nvs_get_u32(nvs_handler, "GbEncAngle", (uint32_t*)&motorXparams.encoderAngle);
        nvs_get_u32(nvs_handler, "GbVoltLimit", (uint32_t*)&motorXparams.voltageLimit);
        nvs_get_u32(nvs_handler, "GbVelLimitHard", (uint32_t*)&motorXparams.velLimitHard);
        nvs_get_u32(nvs_handler, "GbVelProp", (uint32_t*)&motorXparams.velProp);
        nvs_get_u32(nvs_handler, "GbVelIntegral", (uint32_t*)&motorXparams.velIntegral);
        nvs_get_u32(nvs_handler, "GbVelDiff", (uint32_t*)&motorXparams.velDiff);
        nvs_get_u32(nvs_handler, "GbVelRamp", (uint32_t*)&motorXparams.velRamp);
        nvs_get_u32(nvs_handler, "GbVelLimit", (uint32_t*)&motorXparams.velLimit);
        nvs_get_u32(nvs_handler, "GbVelFilter", (uint32_t*)&motorXparams.velFilter);
        nvs_get_u32(nvs_handler, "GbAngleProp", (uint32_t*)&motorXparams.angleProp);
        nvs_get_u32(nvs_handler, "GbAngleLimit", (uint32_t*)&motorXparams.angleLimit);

        nvs_close(nvs_handler);
    }
}

void saveXMotorParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Saving gearbox motor params to NVS ...");
        nvs_set_i8(nvs_handler, "GbCalibrate", motorXparams.calibrate);
        nvs_set_i8(nvs_handler, "GbInverseEnc", motorXparams.inverseEncoder);
        nvs_set_u32(nvs_handler, "GbEncAngle", *(uint32_t*)(&motorXparams.encoderAngle));
        nvs_set_u32(nvs_handler, "GbVoltLimit", *(uint32_t*)(&motorXparams.voltageLimit));
        nvs_set_u32(nvs_handler, "GbVelLimitHard", *(uint32_t*)(&motorXparams.velLimitHard));
        nvs_set_u32(nvs_handler, "GbVelProp", *(uint32_t*)(&motorXparams.velProp));
        nvs_set_u32(nvs_handler, "GbVelIntegral", *(uint32_t*)(&motorXparams.velIntegral));
        nvs_set_u32(nvs_handler, "GbVelDiff", *(uint32_t*)(&motorXparams.velDiff));
        nvs_set_u32(nvs_handler, "GbVelRamp", *(uint32_t*)(&motorXparams.velRamp));
        nvs_set_u32(nvs_handler, "GbVelLimit", *(uint32_t*)(&motorXparams.velLimit));
        nvs_set_u32(nvs_handler, "GbVelFilter", *(uint32_t*)(&motorXparams.velFilter));
        nvs_set_u32(nvs_handler, "GbAngleProp", *(uint32_t*)(&motorXparams.angleProp));
        nvs_set_u32(nvs_handler, "GbAngleLimit", *(uint32_t*)(&motorXparams.angleLimit));
        
        nvs_commit(nvs_handler);
        nvs_close(nvs_handler);
    }
}