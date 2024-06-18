#include <stdio.h>
#include "esp_log.h"
#include "SimpleFOC.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "Ethernet.h"
#include "driver/spi_master.h"
#include "webserver.h"
#include "variables.h"
#include "nvstorage.h"
#include "motors.h"
#include "Gearbox.h"
#include "Can.h"

static const char* TAG = "main";


void motorTask(void* args) {
    while (true) {
        motorX.loopFOC();
        motorX.move();
        motorY.loopFOC();
        motorY.move();
    }
}


extern "C" void app_main(void) {
    xTaskCreatePinnedToCore(init_can, "Init can", 8192, NULL, 5, NULL, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    initMotorsSpi();
    nvs_init();
    loadSystemParams();
    vTaskDelay(20 / portTICK_PERIOD_MS);
    initMotorY();
    initMotorX();

    // motorX.disable();
    // motorY.disable();

    //motorY.sensor_offset = -2;

    xTaskCreatePinnedToCore(motorTask, "Motor task", 8192, NULL, 99, NULL, 1);
    xTaskCreate(gearbox_task, "Gearbox task", 8192, NULL, 99, NULL);


    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    Ethernet::init(40, 41, 2, 42, 1, SPI3_HOST);
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    Ethernet::start();

    start_web_server();
    
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    start_can();
    
    // set_gear(1);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // set_gear(1);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // set_gear(1);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // set_gear(1);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // set_gear(6);
    // for (int i = 0; i < 5; i++) {
    //     for (int j = 0; j <= 5; j++) {
    //         if (j == 5) j = 6;
    //         set_gear(j);
    //         vTaskDelay(4000 / portTICK_PERIOD_MS);
    //     }
    // }
}