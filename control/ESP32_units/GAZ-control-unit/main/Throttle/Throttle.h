#pragma once

#include <cstdint>

class Throttle {
public:
    static void set_target_value(float value);
    static int32_t get_target_value();
private:
    static inline int32_t target_value = 0;
};
