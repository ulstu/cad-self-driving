#pragma once

class PCCommunication {
public:
    static float get_target_speed();
    static float get_target_wheel_angle();
    static void set_parameters(double speed, double angle);
private:
    static inline float target_speed = 0, target_wheel_angle = 0;
};
