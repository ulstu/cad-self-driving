#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "math.h"
#include "WheelSensor.h"
#include "ESA.h"
#include "nvstorage.h"


#include "WheelFollower.h"

#define TAG "WheelFollower"

int WheelFollower::sensevity;

int WheelFollower::resistor_position;
int WheelFollower::absolute_angle = 0;
int WheelFollower::target_position;

int resistor_range;
int center_position;

void WheelFollower::loop(void* args) {    
    while (true) {
        resistor_position = filter(WheelSensor::get_position());
        absolute_angle = ((float)(resistor_position - wheelFollowerParams.leftPosition - center_position) / resistor_range) * wheelFollowerParams.angleRange;

        int wheel_diviation = absolute_angle - target_position;
        // if (wheel_diviation >= 0) {
        //     wheel_diviation = sqrt(wheel_diviation) * sensevity;
        // }
        // else {
        //     wheel_diviation = -sqrt(-wheel_diviation) * sensevity;
        // }
        // if (wheel_diviation > 100) wheel_diviation = 100;
        // if (wheel_diviation < -100) wheel_diviation = -100;
        // ESP_LOGI(TAG, "diviation: %d", wheel_diviation);
        // ESA::set_steering(wheel_diviation);
        ESA::set_steering(pid(wheel_diviation));
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void WheelFollower::start() {
    loadWheelCalibrationParams();
    resistor_range = wheelFollowerParams.rightPosition - wheelFollowerParams.leftPosition;
    center_position = wheelFollowerParams.centerPosition - wheelFollowerParams.leftPosition;
    resistor_position = WheelSensor::get_position();
    target_position = ((float)(resistor_position - wheelFollowerParams.leftPosition - center_position) / resistor_range) * wheelFollowerParams.angleRange;;
    sensevity = 10;
    WheelFollower::set_params(wheelFollowerParams.position_P, wheelFollowerParams.position_I, wheelFollowerParams.position_D, 
        wheelFollowerParams.ramp, wheelFollowerParams.limit, wheelFollowerParams.filter);   
    xTaskCreate(loop, "safety_task", 4096, NULL, 4, NULL);
}

void WheelFollower::set_target(int32_t angle) { target_position = angle; }

void WheelFollower::set_params(float position_P, float position_I, float position_D, float ramp, float limit, float filter) {
    WheelFollower::filter.Tf = filter;

    pid.P = position_P;
    pid.I = position_I;
    pid.D = position_D;
    pid.output_ramp = ramp;
    pid.limit = limit;
    
    pid.reset();
}
