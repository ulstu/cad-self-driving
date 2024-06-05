#include "Motors.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <hardware.h>
#include <NVStorage.h>
#include "ADC.h"

static const char *TAG = "Motors";

BTS7960 clutch_motor_driver(PIN_CLUTCH_EN, PIN_CLUTCH_PWM_L, PIN_CLUTCH_PWM_R, CLUTCH_PWM_CHANNEL_L, CLUTCH_PWM_CHANNEL_R);
BrushedMotor clutch_motor(&clutch_motor_driver, &ADC::clutchCS, &ADC::clutchPos);

BTS7960 brake_motor_driver(PIN_BRAKE_EN, PIN_BRAKE_PWM_L, PIN_BRAKE_PWM_R, BRAKE_PWM_CHANNEL_L, BRAKE_PWM_CHANNEL_R);
BrushedMotor brake_motor(&brake_motor_driver, &ADC::brakeCS, &ADC::brakePos);

static void motors_timer_callback(void *arg)
{
    int64_t timer = esp_timer_get_time();
    clutch_motor.controlLoop();
    brake_motor.controlLoop();
    timer = esp_timer_get_time() - timer;
    // ESP_LOGI(TAG, "Control loop took %lld us", timer);
}

void motors_init()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_LOGI(TAG, "LEDC timer initialized");

    clutch_motor_driver.init(LEDC_TIMER);
    clutch_motor.init(ADC::adc_calibration_handle, (int32_t)NVStorage::clutchMotorParams.curOffset);

    brake_motor_driver.init(LEDC_TIMER);
    brake_motor.init(ADC::adc_calibration_handle, (int32_t)NVStorage::brakeMotorParams.curOffset);

    const esp_timer_create_args_t motors_timer_args = {
        .callback = &motors_timer_callback,
        .name = "motors_timer"};

    esp_timer_handle_t motors_timer;
    ESP_ERROR_CHECK(esp_timer_create(&motors_timer_args, &motors_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(motors_timer, 2000));

    ESP_LOGI(TAG, "Motors initialized");
}

void set_clutch_motor_params()
{
    clutch_motor.setCurrentPID(NVStorage::clutchMotorParams.curProp, NVStorage::clutchMotorParams.curIntegral, NVStorage::clutchMotorParams.curDiff,
                               NVStorage::clutchMotorParams.curRamp, NVStorage::clutchMotorParams.curLimit, NVStorage::clutchMotorParams.curFilter);

    clutch_motor.setVelocityPID(NVStorage::clutchMotorParams.velProp, NVStorage::clutchMotorParams.velIntegral, NVStorage::clutchMotorParams.velDiff,
                                NVStorage::clutchMotorParams.velRamp, NVStorage::clutchMotorParams.velLimit, NVStorage::clutchMotorParams.velFilter);

    clutch_motor.setPositionP(NVStorage::clutchMotorParams.posProp, NVStorage::clutchMotorParams.posLimit);
}

void set_brake_motor_params()
{
    brake_motor.setCurrentPID(NVStorage::brakeMotorParams.curProp, NVStorage::brakeMotorParams.curIntegral, NVStorage::brakeMotorParams.curDiff,
                              NVStorage::brakeMotorParams.curRamp, NVStorage::brakeMotorParams.curLimit, NVStorage::brakeMotorParams.curFilter);

    brake_motor.setVelocityPID(NVStorage::brakeMotorParams.velProp, NVStorage::brakeMotorParams.velIntegral, NVStorage::brakeMotorParams.velDiff,
                               NVStorage::brakeMotorParams.velRamp, NVStorage::brakeMotorParams.velLimit, NVStorage::brakeMotorParams.velFilter);

    brake_motor.setPositionP(NVStorage::brakeMotorParams.posProp, NVStorage::brakeMotorParams.posLimit);
}