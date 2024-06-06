#include "ADC.h"
#include "Brake.h"
#include "Clutch.h"
#include "driver/gpio.h"
#include "Engine.h"
#include "Estop.h"
#include <esp_check.h>
#include "esp_event.h"
#include <esp_log.h>
#include "esp_netif.h"
#include <esp_timer.h>
#include "Ethernet.h"
#include "FlySkyIBus.h"
#include <freertos/FreeRTOS.h>
#include "Gearbox.h"
#include "hardware.h"
#include "jsonrpc-lean/server.h"
#include "LCD.h"
#include "lowpass_filter.h"
#include "Motors.h"
#include "NVStorage.h"
#include "RPCCommunication.h"
#include "StateMachine.h"
#include "Throttle.h"
#include "Webserver.h"
#include "Wheel.h"
#include "PCCommunication.h"
#include "udp_server.h"

static const char *TAG = "MAIN";

LowPassFilter wheel_filter = LowPassFilter(0.001);

void remote_control_task(void *args) {
    while (true) {
        if (FlySkyIBus::available()) {
            float setpoint = ((float)FlySkyIBus::readChannel(3) - 1500) / 500 * 3;

            // clutch_motor.setTarget(setpoint);

            float clutchPosition = ((float)FlySkyIBus::readChannel(2) - 1000) / 1000 * 0.155 + 0.02;

            // ESP_LOGI(TAG, "%f", clutchPosition);

            // clutch_motor.setTarget(clutchPosition);

            float brakeTorque = ((float)FlySkyIBus::readChannel(1) - 1500) / 500 * -1.0;

            Brake::setBrakeTorque(brakeTorque);

            if (brakeTorque < -0.75) {
                Clutch::setState(Clutch::ClutchState::Release);
            } else if (brakeTorque < -0.33) {
                Clutch::setState(Clutch::ClutchState::Pause);
            } else {
                Clutch::setState(Clutch::ClutchState::Press);
            }

            if (FlySkyIBus::readChannel(4) < 1300) {
                Gearbox::set_current_gear(0);
            }
            else if (FlySkyIBus::readChannel(4) < 1600) {
                Gearbox::set_current_gear(1);
            }
            else {
                Gearbox::set_current_gear(6);
            }

            if (FlySkyIBus::readChannel(8) < 1500) {
                Wheel::set_target_angle((float)(FlySkyIBus::readChannel(0) - 1500) / 500);
            }
            else {
                Wheel::set_target_angle(wheel_filter(PCCommunication::get_target_wheel_angle()));
            }

            Throttle::set_target_value((float)(FlySkyIBus::readChannel(2) - 1000) / 1000);

            /*

            uint16_t channel_value;

            

            string channel_values = "";

            for (int channel_number = 0; channel_number < 10; ++channel_number) {
                channel_value = FlySkyIBus::readChannel(channel_number);
                channel_values += "Channel " + to_string(channel_number + 1) + ": " + to_string(channel_value) + ' ';
            }

            ESP_LOGI(TAG, "%s", channel_values.c_str());

            */
        }
        else {
            ESP_LOGW(TAG, "RC connection lost!");
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


float kp = 0.02;
float ki = 17.2;
float kd = 0.000045;


extern "C" void app_main(void) {

    Estop::init(PIN_ESTOP_BUTTON);
    FlySkyIBus::init();

    NVStorage::init();
    NVStorage::loadSystemParams();
    NVStorage::loadClutchMotorParams();
    NVStorage::loadBrakeMotorParams();

    set_clutch_motor_params();
    set_brake_motor_params();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    Ethernet::init(PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_SCK, PIN_ETH_CS, PIN_ETH_INT, SPI_BUS_ETH);

    Webserver::init();

    xTaskCreate(RPCCommunication::rpc_communication_task, "RPC TX", 8192, NULL, 5, NULL);

    Engine::init(PIN_ENGINE_IGNITION, PIN_ENGINE_STARTER);

    ADC::init();

    motors_init();

    StateMachine::init();

    LCD::init(GPIO_NUM_7, GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_6, GPIO_NUM_NC, true);

    clutch_motor.setControlMode(BrushedMotor::ControlMode::Position);

    Brake::setParams(NVStorage::brakeMotorParams.velLimit, NVStorage::brakeMotorParams.zeroPos);

    Clutch::setParams(NVStorage::clutchMotorParams.disabledPos, NVStorage::clutchMotorParams.zeroPos, NVStorage::clutchMotorParams.fullEngPos, NVStorage::clutchMotorParams.minEngPos, NVStorage::clutchMotorParams.pressedPos);
    Clutch::init();

    gpio_set_direction(PIN_WHEEL_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_WHEEL_EN, 1);

    start_udp_server();

    xTaskCreate(remote_control_task, "RC TASK", 6144, NULL, 1, NULL);

    while (true) {
        // ESP_LOGI(TAG, "%f", clutch_motor.getVelocity());
        // ESP_LOGI(TAG, "%ld", ADC::clutchCS);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
