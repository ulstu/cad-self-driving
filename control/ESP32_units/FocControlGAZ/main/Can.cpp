#include "Can.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "Gearbox.h"

/* --------------------- Definitions and static variables ------------------ */

#define TX_GPIO_NUM             GPIO_NUM_9
#define RX_GPIO_NUM             GPIO_NUM_11
#define TAG             "TWAI Master"

//PIDs
#define PID_GEAR_SWITCH        0x10

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
//Filter all other IDs except MSG_ID
static const twai_filter_config_t f_config = {.acceptance_code = (PID_GEAR_SWITCH << 21),
                                             .acceptance_mask = ~(0x7FF << 21),
                                             .single_filter = true};;
//Set to NO_ACK mode due to self testing with single module
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

void TWAI_thread(void* args) {
    ESP_LOGI(TAG, "Start can thread");
    twai_message_t rx_message;
    rx_message.identifier = 10;
    rx_message.data[0] = 99;
    rx_message.data_length_code = 1;

    while (true) {
        //Receive message and print message data
        ESP_ERROR_CHECK(twai_receive(&rx_message, portMAX_DELAY));
        switch (rx_message.identifier)
        {
        case PID_GEAR_SWITCH:
            ESP_LOGI(TAG, "0x%x Gear received - number = %d", (unsigned int)rx_message.identifier, rx_message.data[0]);
            set_gear(rx_message.data[0]);
            break;
        default:
            break;
        }

    }
}

void start_can() {
    xTaskCreatePinnedToCore(TWAI_thread, "CAN thread", 4096, NULL, 4, NULL, 1);
}

void init_can(void* args) {
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");
    vTaskDelete(NULL);
}
