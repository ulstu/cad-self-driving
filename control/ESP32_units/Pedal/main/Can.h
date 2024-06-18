#include <esp_system.h>

class CAN {
public:
    inline static float tacho;
    inline static int speed;
    static void init();
private:
    static void TWAI_thread(void* args);
    static void receive_message();
    static void send_message(int pid);
};