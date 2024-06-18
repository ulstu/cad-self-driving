#pragma once

#include "lwip/sockets.h"

class RPCClient {
public:
    static void init();
    static void send_request(char *method, int value, sockaddr_in dest_addr);
    static void send_request(char *method, int value_1, int value_2, sockaddr_in dest_addr);
    static void send_request(char *method, int value_1, float value_2, sockaddr_in dest_addr);
    static void send_request(char *method, bool value_1, bool value_2, bool value_3, bool value_4, sockaddr_in dest_addr);
private:
    static inline int sock;
    static inline SemaphoreHandle_t mutex;
};
