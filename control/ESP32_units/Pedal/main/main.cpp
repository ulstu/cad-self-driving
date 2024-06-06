#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "udp_server.h"
#include "webserver.h"
#include "Ethernet.h"
#include "ECU.h"
#include "ESA.h"
#include "WheelSensor.h"
#include "WheelFollower.h"
#include "udp_server.h"
#include "Sound.h"
#include "Can.h"

#define PORT 90

#define TAG "Main"

void safetyTask(void* args) {
    while(true) {
        if (esp_timer_get_time() - udpLastCommandTime > 500000) {
            ECU::set_throttle(0);
            ECU::set_relay(false);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    Ethernet::init(21, 35, 36, 37, 38, SPI3_HOST);
    Ethernet::start();

    ECU::init();
    ESA::init();
    WheelSensor::init();
    WheelFollower::start();
    Sound::init();
    CAN::init();

    start_web_server();
    start_udp_server();

    xTaskCreate(safetyTask, "safety_task", 4096, NULL, 4, NULL);

    Sound::play(&soundPullup);
}
