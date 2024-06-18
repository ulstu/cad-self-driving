#include "BTS7960.h"
#include <esp_log.h>

static const char *TAG = "BTS7960";

BTS7960::BTS7960(gpio_num_t en_num, gpio_num_t l_pwm_num, gpio_num_t r_pwm_num, ledc_channel_t l_pwm_ch, ledc_channel_t r_pwm_ch) {
    en_pin_num = en_num;
    l_pwm_pin_num = l_pwm_num;
    r_pwm_pin_num = r_pwm_num;
    l_pwm_channel = l_pwm_ch;
    r_pwm_channel = r_pwm_ch;
}

void BTS7960::init(ledc_timer_t pwm_timer) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << en_pin_num;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    disable();

    ledc_channel_config_t ledc_l_pwm_channel = {
        .gpio_num = l_pwm_pin_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = l_pwm_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = pwm_timer,
        .duty = 0,
        .hpoint = 0,
    }; ESP_ERROR_CHECK(ledc_channel_config(&ledc_l_pwm_channel));

    ledc_channel_config_t ledc_r_pwm_channel = {
        .gpio_num = r_pwm_pin_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = r_pwm_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = pwm_timer,
        .duty = 0,
        .hpoint = 0,
    }; ESP_ERROR_CHECK(ledc_channel_config(&ledc_r_pwm_channel));
    
    set_motor_speed(0.0);

    ESP_LOGI(TAG, "BTS7960 has been successfully initialized!");
    
}

void BTS7960::enable() {
    if (enabled)
        return;
    ESP_ERROR_CHECK(gpio_set_level(en_pin_num, 1));
    enabled = true;
}

void BTS7960::disable() {
    if (!enabled)
        return;
    ESP_ERROR_CHECK(gpio_set_level(en_pin_num, 0));
    enabled = false;
}

bool BTS7960::getState() {
    return enabled;
}

void BTS7960::set_motor_speed(float speed) {
    // speed in [-1;1]

    if (speed < 0) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, l_pwm_channel, 8191 * abs(speed)));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, r_pwm_channel, 0));
    }
    else {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, l_pwm_channel, 0));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, r_pwm_channel, 8191 * abs(speed)));
    }
    
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, l_pwm_channel));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, r_pwm_channel));
}
