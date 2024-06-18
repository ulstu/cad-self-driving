#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

class WheelSensor {
public:
    static void init();
    static int get_position();
private:
    static adc_oneshot_unit_handle_t adc1_handle;
    static bool do_calibration1_chan0;
    static adc_cali_handle_t adc1_cali_chan0_handle;
    static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
};