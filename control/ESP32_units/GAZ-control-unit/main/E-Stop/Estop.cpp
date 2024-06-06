#include "Estop.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

static const char *TAG = "E-Stop";

#define ESTOP_TIME_THRESHOLD_US     50000

void Estop::init(gpio_num_t buttonPin) {
    Estop::buttonPin = buttonPin;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << buttonPin;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if (gpio_get_level(buttonPin) == 0) {
        timer = esp_timer_get_time();
        isNormal = true;
        ESP_LOGI(TAG, "E-Stop button in normal state");
    } else {
        ESP_LOGW(TAG, "E-Stop button triggered");
    }

    xTaskCreate(Estop::task, "Estop_task", 4096, NULL, 6, NULL);
}

bool Estop::normal() {
    return isNormal;
}

void Estop::task(void* args) {
    while (true) {
        if (gpio_get_level(buttonPin) == 0) {
            timer = esp_timer_get_time();
            if (!isNormal) {
                isNormal = true;
                ESP_LOGI(TAG, "E-Stop button returned to normal state");
            }
        }
        else if (isNormal && esp_timer_get_time() > timer + 50000) {
            isNormal = false;
            ESP_LOGW(TAG, "E-Stop button triggered");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}