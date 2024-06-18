#include <stdio.h>
#include "Can.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "VPIO.h"

/* --------------------- Definitions and static variables ------------------ */

//Example Configurations
#define NO_OF_MSGS              100
#define TX_GPIO_NUM             GPIO_NUM_5
#define RX_GPIO_NUM             GPIO_NUM_4
#define EXAMPLE_TAG             "TWAI Master"

//PIDs
#define PID_RELAY_SWITCH        0x0a1   //11 bit standard format ID
#define PID_TURN_SIGNAL         0x0a2
#define PID_BEEP         0x21

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
//Filter all other IDs except MSG_ID
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
//Set to NO_ACK mode due to self testing with single module
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

void TWAI_receive_thread(void* args) {
    twai_message_t rx_message;
    bool beeping = false;
    bool leftLight = false;
    bool rightLight = false;
    while (true) {
        //Receive message and print message data
        esp_err_t result = twai_receive(&rx_message, 10 / portTICK_PERIOD_MS);
        if (result != ESP_OK) {
            vTaskDelay(1);
            continue;
        }
        //ESP_LOGI(EXAMPLE_TAG, "Msg received - number = %d, state = %d", rx_message.data[0], rx_message.data[1]);
        
        switch (rx_message.identifier)
        {
        case PID_RELAY_SWITCH:
            VPIO::relay_w(rx_message.data[0], rx_message.data[1] == 0 ? DT_LOW : DT_HIGH);
            VPIO::iox_sync();
            break;
        case PID_BEEP:
            beeping = rx_message.data[0] & 1;
            leftLight = rx_message.data[0] & 2;
            rightLight = rx_message.data[0] & 4;
            VPIO::relay_w(5, beeping);
            VPIO::relay_w(7, beeping);            
            VPIO::iox_sync();
            if (leftLight && rightLight) {
                relays.set_signal_both();
            } else if (leftLight) {
                relays.set_signal_left();
            } else if (rightLight) {
                relays.set_signal_right();
            } else {
                relays.set_signal_off();
            }
            break;
        default:
            break;
        }
        vTaskDelay(1);

    }
}

void TWAI_transmit_thread(void* args) {
    twai_message_t tx_message;
    
    while (true) {
        tx_message.identifier = 0x20;
        tx_message.data_length_code = 1;
        tx_message.data[0] = inputs[0];
        twai_transmit(&tx_message, 10 / portTICK_PERIOD_MS);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


void CanInit() {
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");

    //Start TWAI Driver for this iteration
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    xTaskCreatePinnedToCore(TWAI_receive_thread, "CAN_rx", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(TWAI_transmit_thread, "CAN_tx", 4096, NULL, 4, NULL, 1);
}