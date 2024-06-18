#pragma once
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"

class ADC {
public:
    static void init();
    static volatile inline DRAM_ATTR uint32_t clutchPos = 0;
    static volatile inline DRAM_ATTR uint32_t clutchCS = 0;
    static volatile inline DRAM_ATTR uint32_t brakePos = 0;
    static volatile inline DRAM_ATTR uint32_t brakeCS = 0;
    static inline adc_cali_handle_t adc_calibration_handle = NULL;
private:
    static bool IRAM_ATTR NOINLINE_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
};