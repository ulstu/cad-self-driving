#include "BrushedMotor.h"
#include <esp_log.h>
#include <esp_timer.h>

#define SPEED_LOOP_PERIOD_US        9500

static const char *TAG = "BrushedMotor";

float min(float a, float b) {
    return a < b ? a : b;
}

BrushedMotor::BrushedMotor(BTS7960* driver, volatile uint32_t* current_sense, volatile uint32_t* position_fb) {
    this->driver = driver;
    this->current_sense = current_sense;
    this->position_fb = position_fb;
}

void BrushedMotor::init(adc_cali_handle_t adc_cali_handle, int32_t zero_current_offset) {
    this->adc_cali_handle = adc_cali_handle;
    if (zero_current_offset == -1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, *current_sense, &this->zero_current_offset));
    } else {
        this->zero_current_offset = zero_current_offset;
    }
    ESP_LOGI(TAG, "Motor zero current offset = %d", this->zero_current_offset);
}

float BrushedMotor::readCurrent() {
    int adcVoltage;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, *current_sense, &adcVoltage));
    return (float)(adcVoltage - zero_current_offset) / 93 * -1;
}

float BrushedMotor::readPosition() {
    return (float)(*position_fb) / 4095;
}

void BrushedMotor::updateVelocityAndPosition() {
    int64_t now = esp_timer_get_time();
    if (now < velocityTimer + SPEED_LOOP_PERIOD_US)
        return;
    
    velocity = (position - lastPosition) / (now - velocityTimer) * 1000000;
    lastPosition = position;
    velocityTimer = now;

    targetVelocity = PIDController::constrain((target - position) * positionP, -min(positionVelLimit, speedLimit), min(positionVelLimit, speedLimit));
}

void BrushedMotor::controlLoop() {
    position = position_filter(readPosition());
    updateVelocityAndPosition();
    if (controlMode == Position) {
        targetCurrent = velocity_pid(targetVelocity - velocity) * -1;
    } else if (controlMode == Current) {
        targetCurrent = target;
    }
    
    current = current_filter(readCurrent());
    driver->set_motor_speed(current_pid(targetCurrent - current));
}

void BrushedMotor::setTarget(float target) {
    this->target = target;
}

float BrushedMotor::getCurrent() {
    return current;
}

float BrushedMotor::getTarget() {
    return target;
}

float BrushedMotor::getPosition() {
    return position;
}

float BrushedMotor::getVelocity() {
    return velocity;
}

void BrushedMotor::setCurrentPID(float P, float I, float D, float ramp, float limit, float filter) {
    current_pid.P = P;
    current_pid.I = I;
    current_pid.D = D;
    current_pid.output_ramp = ramp;
    current_pid.limit = limit;
    current_filter.Tf = filter;
}

void BrushedMotor::setVelocityPID(float P, float I, float D, float ramp, float limit, float filter) {
    velocity_pid.P = P;
    velocity_pid.I = I;
    velocity_pid.D = D;
    velocity_pid.output_ramp = ramp;
    velocity_pid.limit = limit;
    position_filter.Tf = filter;
}

void BrushedMotor::setPositionP(float P, float limit) {
    positionP = P;
    positionVelLimit = limit;
}

void BrushedMotor::setControlMode(ControlMode controlMode) {
    if (controlMode == this->controlMode)
        return;

    if (controlMode == Disabled) {
        driver->disable();
    }
    else if (this->controlMode == Disabled) {
        driver->enable();
    }

    if (controlMode == Position) {
        velocity_pid.reset();
        current_pid.reset();
    }
    else if (controlMode == Current) {
        current_pid.reset();
    }

    this->controlMode = controlMode;
}

void BrushedMotor::pidReset() {
    velocity_pid.reset();
    current_pid.reset();
}

void BrushedMotor::setSpeedLimit(float speedLimit) {
    if (speedLimit <= 0) {
        this->speedLimit = 999;
    } else {
        this->speedLimit = speedLimit;
    }
}