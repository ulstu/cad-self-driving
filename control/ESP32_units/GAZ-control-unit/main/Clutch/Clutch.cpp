#include "Clutch.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "Motors.h"

static const char* TAG = "Clutch";

void Clutch::setState(Clutch::ClutchState state) {
    if (Clutch::state == state)
        return;
    
    Clutch::state = state;

    if (state == Pause) {
        pauseValue = clutch_motor.getPosition();
    }
}

void Clutch::init() {
    setState(Pause);
    xTaskCreate(Clutch::task, "Clutch task", 6144, NULL, 1, NULL);
}

void Clutch::task(void* args) {
    while (true) {
        switch (state) {
        case Pause:
            clutch_motor.setTarget(pauseValue);
            clutch_motor.setSpeedLimit(0);
            break;
        case Press:
            clutch_motor.setTarget(pressedPos);
            clutch_motor.setSpeedLimit(0);
            break;
        case Release:
            clutch_motor.setTarget(zeroPos);
            if (clutch_motor.getPosition() > fullEngPos && clutch_motor.getPosition() < minEngPos) {
                clutch_motor.setSpeedLimit(0.05);
            } else {
                clutch_motor.setSpeedLimit(0);
            }
            break;
        
        default:
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void Clutch::setParams(float disabledPos, float zeroPos, float fullEngPos, float minEngPos, float pressedPos) {
    Clutch::disabledPos = disabledPos;
    Clutch::zeroPos = zeroPos;
    Clutch::fullEngPos = fullEngPos;
    Clutch::minEngPos = minEngPos;
    Clutch::pressedPos = pressedPos;
}