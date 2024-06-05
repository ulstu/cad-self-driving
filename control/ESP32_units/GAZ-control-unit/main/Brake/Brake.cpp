#include "Brake.h"
#include "pid.h"
#include "esp_log.h"
#include "Motors.h"

static const char* TAG = "Brake";

void Brake::setBrakeTorque(float brakeTorque) {
    if (brakeTorque == lastBrakeTorque)
        return;

    lastBrakeTorque = brakeTorque;
    
    if (brakeTorque <= 0) {
        brake_motor.setControlMode(BrushedMotor::ControlMode::Position);
        brake_motor.setTarget(zeroPosition);
    } else {
        brake_motor.setControlMode(BrushedMotor::ControlMode::Current);
        brakeTorque = PIDController::constrain(brakeTorque, 0, 1);
        brake_motor.setTarget(brakeTorque * maxCurrent * -1.0);
    }
}

void Brake::setParams(float maxCurrent, float zeroPosition) {
    Brake::maxCurrent = maxCurrent;
    Brake::zeroPosition = zeroPosition;
}