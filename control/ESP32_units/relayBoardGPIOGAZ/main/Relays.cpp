#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "VPIO.h"
#include "Relays.h"

#define TAG "relays"

Relays::Relays(TURN_SIGNAL_STATE state, int interval) {
    current_turn = state;
    signal_interval = interval;
}

void Relays::signal_update_task() {
    while (true) {
        switch (current_turn)
        {
        case SIGNAL_OFF:
            VPIO::relay_w(3, DT_LOW);
            VPIO::relay_w(4, DT_LOW);
            break;
        case SIGNAL_LEFT:
            signal_state = !signal_state;
            VPIO::relay_w(3, signal_state ? DT_LOW : DT_HIGH);
            VPIO::relay_w(4, DT_LOW);
            break;
        case SIGNAL_RIGHT:
            signal_state = !signal_state;
            VPIO::relay_w(3, DT_LOW);
            VPIO::relay_w(4, signal_state ? DT_LOW : DT_HIGH);
            break;
        case SIGNAL_BOTH:
            signal_state = !signal_state;
            VPIO::relay_w(3, signal_state ? DT_LOW : DT_HIGH);
            VPIO::relay_w(4, signal_state ? DT_LOW : DT_HIGH);
            break;
        default:
            break;
        }
        VPIO::iox_sync();
        vTaskDelay(HW_MS(signal_interval));
    }
}

void Relays::keys_update_task() {
    while (1) {
        for (int i = 0; i < 3; i++) {
            VPIO::relay_w(i, (inputs[0] >> i) & 1);
        }
        VPIO::iox_sync();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void Relays::set_signal_off() {
    current_turn = SIGNAL_OFF;
}

void Relays::set_signal_left() {
    current_turn = SIGNAL_LEFT;
}

void Relays::set_signal_right() {
    current_turn = SIGNAL_RIGHT;
}

void Relays::set_signal_both() {
    current_turn = SIGNAL_BOTH;
}