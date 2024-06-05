#include "ADC.h"
#include <esp_log.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_adc/adc_cali_scheme.h"
#include <hardware.h>

#define TAG "ADC"

#define ADC_CHANNELS        4

void ADC::init() {
    adc_continuous_handle_t handle;
    adc_continuous_handle_cfg_t adc_config;
    adc_config.max_store_buf_size = 1024;
    adc_config.conv_frame_size = ADC_CHANNELS * SOC_ADC_DIGI_DATA_BYTES_PER_CONV;

    adc_continuous_new_handle(&adc_config, &handle);

    adc_digi_pattern_config_t adc_pattern[ADC_CHANNELS];

    adc_pattern[0].atten = ADC_ATTEN_DB_11;
    adc_pattern[0].channel = ADC_CHANNEL_CLUTCH_POS;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = 12;

    adc_pattern[1].atten = ADC_ATTEN_DB_11;
    adc_pattern[1].channel = ADC_CHANNEL_CLUTCH_CS;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = 12;

    adc_pattern[2].atten = ADC_ATTEN_DB_11;
    adc_pattern[2].channel = ADC_CHANNEL_BRAKE_POS;
    adc_pattern[2].unit = ADC_UNIT_1;
    adc_pattern[2].bit_width = 12;

    adc_pattern[3].atten = ADC_ATTEN_DB_11;
    adc_pattern[3].channel = ADC_CHANNEL_BRAKE_CS;
    adc_pattern[3].unit = ADC_UNIT_1;
    adc_pattern[3].bit_width = 12;

    adc_continuous_config_t dig_cfg = {};

    dig_cfg.sample_freq_hz = ADC_FREQ_HZ,
    dig_cfg.conv_mode = ADC_CONV_MODE;
    dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    dig_cfg.adc_pattern = adc_pattern;
    dig_cfg.pattern_num = ADC_CHANNELS;

    adc_continuous_config(handle, &dig_cfg);

    adc_continuous_evt_cbs_t cbs = {};
    cbs.on_conv_done = s_conv_done_cb;
    adc_continuous_register_event_callbacks(handle, &cbs, nullptr);
    adc_continuous_start(handle);
    ESP_LOGI(TAG, "ADC started with %dHz and %d samples per callback", ADC_FREQ_HZ, ADC_CHANNELS);

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_calibration_handle));
    ESP_LOGI(TAG, "ADC calibrated (Curve Fitting)");
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &adc_calibration_handle));
    ESP_LOGI(TAG, "ADC calibrated (Line Fitting)");
#endif

}

bool ADC::s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    for (uint32_t sample = 0; sample < edata->size; sample += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *adc_result = (adc_digi_output_data_t*) & (edata->conv_frame_buffer[sample]);
        switch (adc_result->type2.channel) {
        case ADC_CHANNEL_CLUTCH_POS:
            clutchPos = adc_result->type2.data;
            break;
        case ADC_CHANNEL_CLUTCH_CS:
            clutchCS = adc_result->type2.data;
            break;
        case ADC_CHANNEL_BRAKE_POS:
            brakePos = adc_result->type2.data;
            break;
        case ADC_CHANNEL_BRAKE_CS:
            brakeCS = adc_result->type2.data;
            break;
        default:
            break;
        }
    }
    return false;
}