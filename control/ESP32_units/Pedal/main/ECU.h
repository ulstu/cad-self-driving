class ECU {
public:
    static void init();
    static void set_throttle(int32_t throttle);
    static void set_relay(bool state);
};