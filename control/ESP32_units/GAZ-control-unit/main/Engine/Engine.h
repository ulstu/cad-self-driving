#pragma once
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class Engine {
public:
    enum EngineState {
        disabled = 0,
        preparing,
        starting,
        engaged
    };
    static void init(gpio_num_t ignitionPin, gpio_num_t starterPin);
    static void set_state(bool enabled);
    static void start();
    static void stop();
    static EngineState getStatus();
    
private:
    static EngineState engine_state;
    static gpio_num_t ignitionPin;
    static gpio_num_t starterPin;

    static SemaphoreHandle_t engine_mutex;

    static int64_t engine_timer;

    static void engineTask(void* args);
};