#include "nvstorage.h"

static const char *TAG = "NVS";

system_params systemParams;

pedal_params pedalParams;

wheel_follower_params wheelFollowerParams;


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

void loadPedalParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,"Reading pedal params from NVS ...");

        nvs_get_u32(nvs_handler, "lowPosition", (uint32_t*)&pedalParams.lowPosition);
        nvs_get_u32(nvs_handler, "highPostition", (uint32_t*)&pedalParams.highPostition);
        nvs_get_u32(nvs_handler, "DACVoltage", (uint32_t*)&pedalParams.DACVoltage);

        nvs_close(nvs_handler);
    }
}

void savePedalParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Saving pedal params to NVS ...");

        nvs_set_u32(nvs_handler, "lowPosition", *(uint32_t*)(&pedalParams.lowPosition));
        nvs_set_u32(nvs_handler, "highPostition", *(uint32_t*)(&pedalParams.highPostition));
        nvs_set_u32(nvs_handler, "DACVoltage", *(uint32_t*)(&pedalParams.DACVoltage));
        
        nvs_commit(nvs_handler);
        nvs_close(nvs_handler);
    }
}

void loadWheelCalibrationParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,"Reading gearbox motor params from NVS ...");

        nvs_get_u32(nvs_handler, "wheelCalLeft", (uint32_t*)&wheelFollowerParams.leftPosition);
        nvs_get_u32(nvs_handler, "wheelCalCenter", (uint32_t*)&wheelFollowerParams.centerPosition);
        nvs_get_u32(nvs_handler, "wheelCalRight", (uint32_t*)&wheelFollowerParams.rightPosition);
        nvs_get_u32(nvs_handler, "wheelCalRange", (uint32_t*)&wheelFollowerParams.angleRange);
        nvs_get_u32(nvs_handler, "wheelPosP", (uint32_t*)(&wheelFollowerParams.position_P));
        nvs_get_u32(nvs_handler, "wheelPosI", (uint32_t*)(&wheelFollowerParams.position_I));
        nvs_get_u32(nvs_handler, "wheelPosD", (uint32_t*)(&wheelFollowerParams.position_D));
        nvs_get_u32(nvs_handler, "wheelPosRamp", (uint32_t*)(&wheelFollowerParams.ramp));
        nvs_get_u32(nvs_handler, "wheelPosLimit", (uint32_t*)(&wheelFollowerParams.limit));
        nvs_get_u32(nvs_handler, "wheelPosFilter", (uint32_t*)(&wheelFollowerParams.filter));

        nvs_close(nvs_handler);
    }
}

void saveWheelCalibrationParams() {
    esp_err_t err;
    nvs_handle_t nvs_handler;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handler);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Saving wheel follower params to NVS ...");

        nvs_set_u32(nvs_handler, "wheelCalLeft", *(uint32_t*)(&wheelFollowerParams.leftPosition));
        nvs_set_u32(nvs_handler, "wheelCalCenter", *(uint32_t*)(&wheelFollowerParams.centerPosition));
        nvs_set_u32(nvs_handler, "wheelCalRight", *(uint32_t*)(&wheelFollowerParams.rightPosition));
        nvs_set_u32(nvs_handler, "wheelCalRange", *(uint32_t*)(&wheelFollowerParams.angleRange));
        nvs_set_u32(nvs_handler, "wheelPosP", *(uint32_t*)(&wheelFollowerParams.position_P));
        nvs_set_u32(nvs_handler, "wheelPosI", *(uint32_t*)(&wheelFollowerParams.position_I));
        nvs_set_u32(nvs_handler, "wheelPosD", *(uint32_t*)(&wheelFollowerParams.position_D));
        nvs_set_u32(nvs_handler, "wheelPosRamp", *(uint32_t*)(&wheelFollowerParams.ramp));
        nvs_set_u32(nvs_handler, "wheelPosLimit", *(uint32_t*)(&wheelFollowerParams.limit));
        nvs_set_u32(nvs_handler, "wheelPosFilter", *(uint32_t*)(&wheelFollowerParams.filter));

        nvs_commit(nvs_handler);
        nvs_close(nvs_handler);
    }
}
