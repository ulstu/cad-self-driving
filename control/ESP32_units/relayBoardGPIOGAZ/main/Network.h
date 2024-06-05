#pragma once
#include <esp_event.h>
#include "esp_netif.h"
#include <esp_system.h>
#include "driver/gpio.h"
#include "../common/dt_device_tree.h"

class Ethernet {
public:
    static esp_netif_t *g_eth_netif;
    static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void init(ddt_t ddt);
};