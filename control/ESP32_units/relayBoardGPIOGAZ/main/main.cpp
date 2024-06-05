#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_netif.h"
#include <esp_event.h>
#include "VPIO.h"
#include "Network.h"
#include "webserver.h"
#include "Can.h"
#include "Relays.h"

#define TAG "main"

//Relays relays = Relays(SIGNAL_OFF, 500);

void start_signal_updater(void* args) {
    relays.signal_update_task();
}

void start_keys_updater(void* args) {
    relays.keys_update_task();
}

extern "C" void app_main(void) {
    VPIO::init(DDT_DEF_VALUE());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //Ethernet::init(DDT_DEF_VALUE());

    int32_t i;

    VPIO::relay_w(4, DT_HIGH);
    VPIO::relay_w(3, DT_HIGH);
    VPIO::iox_sync();
    vTaskDelay(HW_MS(500));

    VPIO::relay_w(4, DT_LOW);
    VPIO::relay_w(3, DT_LOW);
    VPIO::iox_sync();
    vTaskDelay(HW_MS(100));

    xTaskCreate(start_keys_updater, "Keys_updater", 4096, NULL, 4, NULL);

    start_web_server();
    xTaskCreate(start_signal_updater, "Signal_updater", 4096, NULL, 4, NULL);
    CanInit();
    ESP_LOGW(TAG, "Version 1");
    
}
