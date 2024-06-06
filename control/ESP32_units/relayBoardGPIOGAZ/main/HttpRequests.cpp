#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "192.168.1.57"
#define WEB_SERVER_RELAYS "192.168.1.56"
#define WEB_PORT "80"

#define TAG "Http requests"

int targetGear = 0;
float brakeTorque = 0;
bool lTurn = false;
bool rTurn = false;
bool alarmS = false;
bool signalS = false;
bool ignition = false;
bool starter = false;

void http_get_task(void *pvParameters) {
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[512];

    while(1) {
        std::string reqStr = "GET /api?gear=" + std::to_string(targetGear) + "&brake=" + std::to_string(brakeTorque);
        reqStr += " HTTP/1.0\r\nHost: 192.168.1.57:80\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n";
        //ESP_LOGE(TAG, "%s", reqStr.c_str());
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        freeaddrinfo(res);

        if (write(s, reqStr.c_str(), reqStr.size()) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 1;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        /* Read HTTP response */
        // do {
        //     bzero(recv_buf, sizeof(recv_buf));
        //     r = read(s, recv_buf, sizeof(recv_buf)-1);
        //     for(int i = 0; i < r; i++) {
        //         putchar(recv_buf[i]);
        //     }
        // } while(r > 0);

        //ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        ESP_LOGI(TAG, "Box success, closing");
        close(s);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void relay_task(void *args) {
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[512];

    while(1) {
        std::string reqStr = "GET /api?l=" + std::to_string(lTurn ? 1 : 0)
        + "&r=" + std::to_string(rTurn ? 1 : 0)
        + "&a=" + std::to_string(alarmS ? 1 : 0)
        + "&s=" + std::to_string(signalS ? 1 : 0)
        + "&i=" + std::to_string(ignition ? 1 : 0)
        + "&start=" + std::to_string(starter ? 1 : 0);
        reqStr += " HTTP/1.0\r\nHost: 192.168.1.56:80\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n";
        //ESP_LOGE(TAG, "%s", reqStr.c_str());
        int err = getaddrinfo(WEB_SERVER_RELAYS, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        freeaddrinfo(res);

        if (write(s, reqStr.c_str(), reqStr.size()) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 1;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        /* Read HTTP response */
        // do {
        //     bzero(recv_buf, sizeof(recv_buf));
        //     r = read(s, recv_buf, sizeof(recv_buf)-1);
        //     for(int i = 0; i < r; i++) {
        //         putchar(recv_buf[i]);
        //     }
        // } while(r > 0);

        //ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        // ESP_LOGI(TAG, "Relays success, closing");
        close(s);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
