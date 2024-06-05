#pragma once
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "BTS7960.h"
#include "lowpass_filter.h"
#include "pid.h"

class BrushedMotor {
public:
    enum ControlMode {
        Disabled = 0,
        Current,
        Position
    } controlMode = Disabled;

    BrushedMotor(BTS7960* driver, volatile uint32_t* current_sense, volatile uint32_t* position_fb);
    
    void init(adc_cali_handle_t adc_cali_handle, int32_t zero_current_offset = -1);
    void controlLoop();
    void setTarget(float target);
    float getCurrent();
    float getTarget();
    float getPosition();
    float getVelocity();
    void setCurrentPID(float P, float I, float D, float ramp, float limit, float filter);
    void setVelocityPID(float P, float I, float D, float ramp, float limit, float filter);
    void setPositionP(float P, float limit);
    void setControlMode(ControlMode controlMode);
    void pidReset();
    void setSpeedLimit(float speedLimit = 0);

private:
    BTS7960* driver;
    volatile uint32_t* current_sense;
    volatile uint32_t* position_fb;
    adc_cali_handle_t adc_cali_handle;

    LowPassFilter current_filter = LowPassFilter(0.01);
    PIDController current_pid = PIDController(0.02, 15, 0.00005, 10000, 1);

    LowPassFilter position_filter = LowPassFilter(0.01);
    PIDController velocity_pid = PIDController(0.05, 10, 0, 100, 4);

    float positionP = 1;
    float positionVelLimit = 1;

    int zero_current_offset;

    float target = 0;

    float current = 0;
    float targetCurrent = 0;

    float targetVelocity = 0;
    int64_t velocityTimer = 0;
    float velocity = 0;

    float position = 0;
    float lastPosition = 0;

    float speedLimit = 999;

    float readCurrent();
    float readPosition();

    void updateVelocityAndPosition();
};