#include <stdio.h>
#include "Can.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

/* --------------------- Definitions and static variables ------------------ */

//Configurations
#define NO_OF_MSGS              100
#define TX_GPIO_NUM             GPIO_NUM_41
#define RX_GPIO_NUM             GPIO_NUM_42
#define TAG                     "CAN"

//PIDs
#define PID_TACHO               0x0C
#define PID_SPEED               0x0D
#define PID_CAN_ID              0x7DF

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
// static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

void CAN::receive_message() {
    twai_message_t rx_message;
    esp_err_t result = twai_receive(&rx_message, 10 / portTICK_PERIOD_MS);
    if (result == ESP_OK) {
        switch (rx_message.data[2]) {
        case PID_SPEED:
            CAN::speed = rx_message.data[3];
            break;
        case PID_TACHO:
            CAN::tacho = ((256. * rx_message.data[3]) + rx_message.data[4]) / 4.;
            break;
        default:
            break;
        }
    }
}

void CAN::send_message(int pid) {
    twai_message_t tx_message;
    tx_message.identifier = PID_CAN_ID;
    tx_message.data_length_code = 1;
    tx_message.data[0] = 0x02;
    tx_message.data[1] = 0x01;
    tx_message.data[2] = pid;
}

void CAN::TWAI_thread(void* args) {
    while (true) {
        receive_message();
        vTaskDelay(20 / portTICK_PERIOD_MS);

        send_message(PID_SPEED);
        vTaskDelay(20 / portTICK_PERIOD_MS);

        receive_message();
        vTaskDelay(20 / portTICK_PERIOD_MS);
        
        send_message(PID_TACHO);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void CAN::init() {
    twai_general_config_t g_config = {};
    g_config.mode = TWAI_MODE_NORMAL;
    g_config.tx_io = TX_GPIO_NUM;
    g_config.rx_io = RX_GPIO_NUM;
    g_config.clkout_io = TWAI_IO_UNUSED;
    g_config.bus_off_io = TWAI_IO_UNUSED;
    g_config.tx_queue_len = 5;
    g_config.rx_queue_len = 5;
    g_config.alerts_enabled = TWAI_ALERT_NONE;
    g_config.clkout_divider = 0;
    g_config.intr_flags = ESP_INTR_FLAG_LEVEL2;

    twai_driver_install(&g_config, &t_config, &f_config);
    ESP_LOGI(TAG, "CAN driver installed");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN driver started");

    CAN::speed = -1;
    CAN::tacho = -1;

    xTaskCreatePinnedToCore(TWAI_thread, "CAN_rx", 4096, NULL, 4, NULL, 1);
}