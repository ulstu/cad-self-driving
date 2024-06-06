#pragma once

enum TURN_SIGNAL_STATE {
    SIGNAL_OFF,
    SIGNAL_LEFT,
    SIGNAL_RIGHT,
    SIGNAL_BOTH
};

class Relays {
private:
    TURN_SIGNAL_STATE current_turn;
    bool signal_state;
    int signal_interval;
public:
    Relays(TURN_SIGNAL_STATE state, int interval);
    void signal_update_task();
    void keys_update_task();
    void set_signal_off();
    void set_signal_left();
    void set_signal_right();
    void set_signal_both();
};