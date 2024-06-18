#include "Throttle.h"

static const char *TAG = "THROTTLE";

void Throttle::set_target_value(float value) {
    if (value < 0.0 || value > 1.0)
        return;
    target_value = value * 100.0;
}

int32_t Throttle::get_target_value() { return target_value; }
