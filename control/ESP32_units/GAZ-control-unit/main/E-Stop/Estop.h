#pragma once
#include "inttypes.h"
#include "driver/gpio.h"

class Estop {
public:
    static void init(gpio_num_t buttonPin);
    static bool normal();
private:
    static inline bool isNormal = false;
    static inline int64_t timer = -10000000;
    static void task(void* args);
    static inline gpio_num_t buttonPin;
};