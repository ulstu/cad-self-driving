#pragma once
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <inttypes.h>

class BTS7960 {
public:
    BTS7960(gpio_num_t en_num, gpio_num_t l_pwm_num, gpio_num_t r_pwm_num, ledc_channel_t l_pwm_ch, ledc_channel_t r_pwm_ch);
    void init(ledc_timer_t pwm_timer);
    void enable();
    void disable();
    void set_motor_speed(float speed);
    bool getState();
private:
    bool enabled = false;
    gpio_num_t en_pin_num, l_pwm_pin_num, r_pwm_pin_num;
    ledc_channel_t l_pwm_channel, r_pwm_channel;
};
