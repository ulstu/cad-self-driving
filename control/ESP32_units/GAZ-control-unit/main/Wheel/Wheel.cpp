#include "Wheel.h"

static const char *TAG = "WHEEL";

void Wheel::set_target_angle(float wheel_ratio) { target_angle = wheel_degrees * wheel_ratio; }
int32_t Wheel::get_target_angle() { return target_angle; }
