#include "lowpass_filter.h"
#include "pid.h"

class WheelFollower {
public:
    static int resistor_position;
    static int target_position;
    static int absolute_angle;
    static int sensevity;
    static void start();
    static void set_target(int32_t angle);
    static void set_params(float position_P, float position_I, float position_D, float ramp, float limit, float filter);
private:
    static void loop(void* args);
    static inline LowPassFilter filter = LowPassFilter(0.1);
    static inline PIDController pid = PIDController(5, 30, 0, 100000, 70);
};