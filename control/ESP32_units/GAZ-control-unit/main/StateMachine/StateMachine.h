#pragma once

class StateMachine {
public:
    enum Mode {
        EStop = 0,
        Manual = 1,
        Auto = 2,
    };

    static void init();
    static Mode get_current_control_mode();
private:
    static inline Mode current_control_mode;
};
