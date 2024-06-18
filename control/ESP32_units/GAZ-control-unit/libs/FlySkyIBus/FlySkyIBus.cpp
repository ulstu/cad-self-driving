#include <esp_log.h>
#include <esp_timer.h>
#include "FlySkyIBus.h"

static const char *TAG = "FLYSKYIBUS";
static const int RXD_BUF_SIZE = 1024;

uart_port_t FlySkyIBus::uart_pin_num;
gpio_num_t FlySkyIBus::rxd_pin_num;
char FlySkyIBus::data[2048];

uint8_t FlySkyIBus::state;
int64_t FlySkyIBus::last;
uint8_t FlySkyIBus::buffer[PROTOCOL_LENGTH];
uint8_t FlySkyIBus::ptr;
uint8_t FlySkyIBus::len;
uint16_t FlySkyIBus::channel[PROTOCOL_CHANNELS];
uint16_t FlySkyIBus::chksum;
uint8_t FlySkyIBus::lchksum;

void FlySkyIBus::init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_pin_num = UART_NUM_1;
    rxd_pin_num = GPIO_NUM_35;

    ESP_ERROR_CHECK(uart_driver_install(uart_pin_num, RXD_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_pin_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_pin_num, UART_PIN_NO_CHANGE, rxd_pin_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    state = DISCARD;
    last = -10000000;
    ptr = 0;
    len = 0;
    chksum = 0;
    lchksum = 0;

    xTaskCreate(FlySkyIBus::loop, "IBus_task", 6144, NULL, 1, NULL);

    ESP_LOGI(TAG, "FlySkyIBus has been successfully initialized!");
}

void FlySkyIBus::loop(void* args) {
    while (true) {
        int64_t now = esp_timer_get_time();

        if (now - last >= PROTOCOL_TIMEGAP)
            state = GET_PREAM;

        const int data_bytes_length = uart_read_bytes(uart_pin_num, &data, RXD_BUF_SIZE, 0 / portTICK_PERIOD_MS);

        if (data_bytes_length > 0) {
            data[data_bytes_length] = 0;
            // ESP_LOGI(TAG, "%d of the following bytes have been read: %s", data_bytes_length, data);

            for (int i = 0; i < data_bytes_length; ++i) {
                switch (state) {
                    case GET_PREAM:
                        if (data[i] == PROTOCOL_CMD) {
                            ptr = 0;
                            len = 28;
                            chksum = 0;
                            state = GET_DATA;
                            
                        }
                        else {
                            state = DISCARD;
                        }
                        break;

                    case GET_DATA:
                        buffer[ptr++] = data[i];

                        if (ptr % 2) {
                        	chksum += (uint16_t)data[i];
                        }
                        else {
                        	chksum += (uint16_t)(data[i]) << 8;
                        }

                        if (ptr == len)
                            state = GET_CHKSUML;
                        break;
                    
                    case GET_CHKSUML:
                        lchksum = data[i];
                        state = GET_CHKSUMH;
                        break;

                    case GET_CHKSUMH:
                        if (chksum == ((uint16_t)data[i] << 8) + lchksum) {
                            for (uint8_t i = 0; i < PROTOCOL_CHANNELS * 2 + 1; i += 2)
                                channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
                            last = now;
                            state = GET_PREAM;
                        } else {
                            ESP_LOGE(TAG, "FlySkyIBus checksum error!");
                            state = DISCARD;
                        }
                        break;

                    case DISCARD:
                        default:
                            break;
                }
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

uint16_t FlySkyIBus::readChannel(uint8_t channelNr) {
    if (channelNr < PROTOCOL_CHANNELS)
        return channel[channelNr];
    return 0;
}

bool FlySkyIBus::available() {
    if (esp_timer_get_time() - last < 1000000)
        return true;
    return false;
}

