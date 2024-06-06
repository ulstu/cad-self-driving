#pragma once

class RPCCommunication {
public:
    static void set_accelerator_torque(int torque);
    static void set_accelerator_relay(int state);
    static void set_left_turn_lights(bool state);
    static void set_right_turn_lights(bool state);
    static void set_alarm(bool state);
    static void set_signal(bool state);
    static void rpc_communication_task(void *args);
private:
    static inline int accelerator = 0;
    static inline int accelerator_relay = 0;
    static inline bool left_turn_lights = false;
    static inline bool right_turn_lights = false;
    static inline bool alarm = false;
    static inline bool signal = false;
};
