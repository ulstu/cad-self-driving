#pragma once

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
extern "C" {
    #include "u8g2_esp32_hal.h"
}
#include "U8g2lib.h"

class LCD {
public:
    static void init(gpio_num_t clk_num, gpio_num_t mosi_num, gpio_num_t cs_num, gpio_num_t reset_num, gpio_num_t dc_num, bool cs_pos = false);
    static void send_message_to_queue(char* message, bool to_back = false);
private:
    static inline U8G2 display;
    static inline char display_buffer[256];

    static enum Pages {
        UNITS_CONNECTION = 0,
        CONTROL_MODE = 1,
        MESSAGES = 2,
    } display_pages;

    static inline QueueHandle_t messages_queue;
    static void draw_task(void *args);
};
