#include "Gearbox.h"
#include "motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "Gearbox"

int target_x = GM_MIDDLE;
int target_y = GM_MIDDLE;
int gearbox_state = GM_INIT;
int target_gear = 0;

void set_gear(int target) {
    ESP_LOGI(TAG, "target gear: %d", target);
    if (gearbox_state != GM_READY && target != 0) {
        ESP_LOGW(TAG, "Abort! Gearbox not ready");
        return;
    }
    if (target_gear == target) {
        // ESP_LOGW(TAG, "Abort! Already set");
        return;
    }
    target_gear = target;
    if (target == 0) {
        target_y = GM_MIDDLE;
        // target_x = GM_MIDDLE;
        // target_x = motorX.getPosition();
    }
    else if (target == 1) {
        target_x = GM_LOW;
        target_y = GM_LOW;
    }
    else if (target == 2) {
        target_x = GM_LOW;
        target_y = GM_HIGH;
    }
    else if (target == 3) {
        target_x = GM_MIDDLE;
        target_y = GM_LOW;
    }
    else if (target == 4) {
        target_x = GM_MIDDLE;
        target_y = GM_HIGH;
    }
    else if (target == 5) {
        target_x = GM_HIGH;
        target_y = GM_LOW;
    }
    else if (target == 6) {
        target_x = GM_HIGH;
        target_y = GM_HIGH;
    }

    gearbox_state = GM_MOVE;
}

void gearbox_task(void* args) {
    while (true) {
        if (gearbox_state == GM_INIT) {
            motorY.target = motorY.middle;
            motorY.enable();
            while (motorY.getPosition() != GM_MIDDLE) vTaskDelay(10 / portTICK_PERIOD_MS);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            motorY.disable();
            gearbox_state = GM_READY;
        }
        else if (gearbox_state == GM_MOVE) {
            if (motorY.getPosition() == target_y && motorX.getPosition() == target_x) {
                gearbox_state = GM_READY;
            }
            else {
                int64_t stageTimer = esp_timer_get_time();
                motorY.target = motorY.middle;
                motorY.enable();
                while (motorY.getPosition() != GM_MIDDLE) vTaskDelay(10 / portTICK_PERIOD_MS);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                uint8_t attemptsCount = 4;
BOTH_AXIS_SWITCH:
                if (attemptsCount == 0) {
                    gearbox_state = GM_READY;
                    motorX.disable();
                    motorY.disable();
                    continue;
                }
                //ESP_LOGW(TAG, "attempts %d", attemptsCount);
                motorY.disable();
                if (target_gear != 0) {
                    motorX.target = motorX.getFloatPosition(target_x);
                    motorX.enable();
                    stageTimer = esp_timer_get_time();
                    while (motorX.getPosition() != target_x && esp_timer_get_time() - stageTimer < 1500000) {
                        motorX.target = motorX.getFloatPosition(target_x);
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    }
                    if (esp_timer_get_time() - stageTimer >= 2000000) {
                        motorX.target = motorX.middle;
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        motorX.disable();
                        attemptsCount--;
                        goto BOTH_AXIS_SWITCH;
                    }
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                }
                stageTimer = esp_timer_get_time();
                motorY.target = motorY.getFloatPosition(target_y);
                motorY.enable();
                while (motorY.getPosition() != target_y && esp_timer_get_time() - stageTimer < 2000000) {
                    motorY.target = motorY.getFloatPosition(target_y);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
                if (esp_timer_get_time() - stageTimer >= 2000000) {
                    motorY.target = motorY.middle;
                    motorX.disable();
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    attemptsCount--;
                    goto BOTH_AXIS_SWITCH;
                } 
                gearbox_state = GM_READY;
                vTaskDelay(500 / portTICK_PERIOD_MS);
                motorX.disable();
                motorY.disable();
            }
        }
        
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}
