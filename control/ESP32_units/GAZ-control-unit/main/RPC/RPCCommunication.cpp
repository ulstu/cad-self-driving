#include "esp_log.h"
#include "Gearbox.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "RPCClient.h"
#include "RPCCommunication.h"
#include "Throttle.h"
#include "Wheel.h"

static const char *TAG = "RPCCOMMUNICATION";

void RPCCommunication::set_accelerator_torque(int torque) { accelerator = torque; }
void RPCCommunication::set_accelerator_relay(int state) { accelerator_relay = state; }
void RPCCommunication::set_left_turn_lights(bool state) { left_turn_lights = state; }
void RPCCommunication::set_right_turn_lights(bool state) { right_turn_lights = state; }
void RPCCommunication::set_alarm(bool state) { alarm = state; }
void RPCCommunication::set_signal(bool state) { signal = state; }

void RPCCommunication::rpc_communication_task(void *args) {
    RPCClient::init();

    struct sockaddr_in dest_addr_gearbox;
    dest_addr_gearbox.sin_addr.s_addr = inet_addr("192.168.1.57");
    dest_addr_gearbox.sin_family = AF_INET;
    dest_addr_gearbox.sin_port = htons(90);

    struct sockaddr_in dest_addr_wheel;
    dest_addr_wheel.sin_addr.s_addr = inet_addr("192.168.1.182");
    dest_addr_wheel.sin_family = AF_INET;
    dest_addr_wheel.sin_port = htons(90);

    /*

    struct sockaddr_in dest_addr_accelerator;
    dest_addr_accelerator.sin_addr.s_addr = inet_addr("192.168.X.X");
    dest_addr_accelerator.sin_family = AF_INET;
    dest_addr_accelerator.sin_port = htons(90);

    struct sockaddr_in dest_addr_relay;
    dest_addr_relay.sin_addr.s_addr = inet_addr("192.168.X.X");
    dest_addr_relay.sin_family = AF_INET;
    dest_addr_relay.sin_port = htons(90);

    */

    while (true) {
        RPCClient::send_request("set_gear", Gearbox::get_current_gear(), dest_addr_gearbox);
        vTaskDelay(10 / portTICK_PERIOD_MS);

        RPCClient::send_request("angle_set", Wheel::get_target_angle(), dest_addr_wheel);
        vTaskDelay(10 / portTICK_PERIOD_MS);

        RPCClient::send_request("throttle_set", Throttle::get_target_value(), 1, dest_addr_wheel);
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // RPCClient::send_request("set_values", left_turn_lights, right_turn_lights, alarm, signal dest_addr_relay);
        // vTaskDelay(10 / portTICK_PERIOD_MS);

        // RPCClient::send_request("throttle_set", accelerator, accelerator_relay, dest_addr_accelerator);
        // vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
