#pragma once

class Gearbox {
public:
    static void set_current_gear(int target_gear);
    static int get_current_gear();
private:
    static inline int current_gear = 0;
};
