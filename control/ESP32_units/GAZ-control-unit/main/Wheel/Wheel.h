#pragma once

#include <cstdint>

class Wheel {
public:
    static void set_target_angle(float wheel_ratio);
    static int32_t get_target_angle();
private:
    static inline int32_t target_angle = 0;
    static inline float wheel_degrees = 400.0;
};
