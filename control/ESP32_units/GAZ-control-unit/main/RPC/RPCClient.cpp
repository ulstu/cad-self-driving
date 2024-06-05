#include "RPCClient.h"
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

static const char *TAG = "RPCCLIENT";

void RPCClient::init() {
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "Socket created");
    }

    mutex = xSemaphoreCreateMutex();
}

void RPCClient::send_request(char *method, int value, sockaddr_in dest_addr) {
    char payload[256];

    sprintf(
        payload, 
        "{"
            "\"jsonrpc\":\"2.0\","
            "\"method\":\"%s\","
            "\"params\":[%d]"
        "}", 
        method, value
    );

    xSemaphoreTake(mutex, 500 / portTICK_PERIOD_MS);
    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    xSemaphoreGive(mutex);

    // if (err < 0)
    //     ESP_LOGE(TAG, "Error during sending RPCClient request");
}

void RPCClient::send_request(char *method, int value_1, int value_2, sockaddr_in dest_addr) {
    char payload[256];

    sprintf(
        payload, 
        "{"
            "\"jsonrpc\":\"2.0\","
            "\"method\":\"%s\","
            "\"params\":[%d, %d]"
        "}", 
        method, value_1, value_2
    );

    xSemaphoreTake(mutex, 500 / portTICK_PERIOD_MS);
    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    xSemaphoreGive(mutex);

    // if (err < 0)
    //     ESP_LOGE(TAG, "Error during sending RPCClient request");
}

void RPCClient::send_request(char *method, int value_1, float value_2, sockaddr_in dest_addr) {
    char payload[256];

    sprintf(
        payload, 
        "{"
            "\"jsonrpc\":\"2.0\","
            "\"method\":\"%s\","
            "\"params\":[%d,%f]"
        "}", 
        method, value_1, value_2
    );

    xSemaphoreTake(mutex, 500 / portTICK_PERIOD_MS);
    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    xSemaphoreGive(mutex);

    // if (err < 0)
    //     ESP_LOGE(TAG, "Error during sending RPCClient request");
}

void RPCClient::send_request(char *method, bool value_1, bool value_2, bool value_3, bool value_4, sockaddr_in dest_addr) {
    char payload[256];

    sprintf(
        payload, 
        "{"
            "\"jsonrpc\":\"2.0\","
            "\"method\":\"%s\","
            "\"params\":[%d, %d, %d, %d]"
        "}", 
        method, value_1, value_2, value_3, value_4
    );

    xSemaphoreTake(mutex, 500 / portTICK_PERIOD_MS);
    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    xSemaphoreGive(mutex);

    // if (err < 0)
    //     ESP_LOGE(TAG, "Error during sending RPCClient request");
}
