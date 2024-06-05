#include <esp_log.h>
#include "Estop.h"
#include "FlySkyIBus.h"
#include "LCD.h"
#include "NVStorage.h"
#include "StateMachine.h"

#define ST7920

static const char *TAG = "LCD";

void LCD::init(gpio_num_t clk_pin_num, gpio_num_t mosi_pin_num, gpio_num_t cs_pin_num, gpio_num_t reset_pin_num, gpio_num_t dc_pin_num, bool cs_positive) {
#ifdef ST7920
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;

    u8g2_esp32_hal.bus.spi.clk = clk_pin_num;
    u8g2_esp32_hal.bus.spi.mosi = mosi_pin_num;
    u8g2_esp32_hal.bus.spi.cs = cs_pin_num;
    u8g2_esp32_hal.reset = reset_pin_num;
    u8g2_esp32_hal.positiveCS = cs_positive;

    u8g2_esp32_hal_init(u8g2_esp32_hal);
#else
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;

    u8g2_esp32_hal.bus.spi.clk = clk_pin_num;
    u8g2_esp32_hal.bus.spi.mosi = mosi_pin_num;
    u8g2_esp32_hal.bus.spi.cs = cs_pin_num;
    u8g2_esp32_hal.reset = reset_pin_num;
    u8g2_esp32_hal.positiveCS = cs_positive;
    u8g2_esp32_hal.dc = dc_pin_num;

    u8g2_esp32_hal_init(u8g2_esp32_hal);
#endif

    display = U8G2();
    display.getU8g2();

#ifdef ST7920
    u8g2_Setup_st7920_s_128x64_f(display.getU8g2(), U8G2_R0, u8g2_esp32_spi_byte_cb, u8g2_esp32_gpio_and_delay_cb);
#else
    u8g2_Setup_st7565_64128n_f(display.getU8g2(), U8G2_R0, u8g2_esp32_spi_byte_cb, u8g2_esp32_gpio_and_delay_cb);
#endif

    display.initDisplay();
    display.setPowerSave(0);
    display.setContrast(45);

    display.enableUTF8Print();
    display.setFont(u8g2_font_6x12_t_cyrillic);

    messages_queue = xQueueCreate(10, sizeof(char*));
    
    xTaskCreate(LCD::draw_task, "LCD DRAW TASK", 4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "LCD has been successfully initialized!");
}

void LCD::send_message_to_queue(char* message, bool to_back) {
    if (!to_back) {
        xQueueSendToFront(messages_queue, &message, 0);
        ESP_LOGI(TAG, "The message has been successfully sent in the front of the queue!");
    }
    else {
        xQueueSendToBack(messages_queue, &message, 0);
        ESP_LOGI(TAG, "The message has been successfully sent in the back of the queue!");
    }
}

void LCD::draw_task(void *args) {
    Pages current_display_page = UNITS_CONNECTION;
    display.sendBuffer();

    while (true) {
        display.clearBuffer();

        switch (current_display_page) {
            case UNITS_CONNECTION:
                sprintf(display_buffer, "IP: %d.%d.%d.%d", NVStorage::systemParams.ip[0], NVStorage::systemParams.ip[1], 
                    NVStorage::systemParams.ip[2], NVStorage::systemParams.ip[3]);

                display.drawButtonUTF8(64, 10, U8G2_BTN_HCENTER | U8G2_BTN_INV, 124, 3, 3, display_buffer);

                true ? sprintf(display_buffer, "1. Блок МКПП:     OK") : sprintf(display_buffer, "1. Блок МКПП:   FAIL");
                display.drawUTF8(4, 25, display_buffer);

                true ? sprintf(display_buffer, "2. Блок педалей:  OK") : sprintf(display_buffer, "2. Блок педалей:FAIL");
                display.drawUTF8(4, 37, display_buffer);

                true ? sprintf(display_buffer, "3. Блок пульта:   OK") : sprintf(display_buffer, "3. Блок пульта: FAIL");
                display.drawUTF8(4, 49, display_buffer);

                true ? sprintf(display_buffer, "4. Блок реле:     OK") : sprintf(display_buffer, "4. Блок реле:   FAIL");
                display.drawUTF8(4, 61, display_buffer);

                current_display_page = CONTROL_MODE;
                break;

            case CONTROL_MODE:
                display.drawButtonUTF8(64, 10, U8G2_BTN_HCENTER | U8G2_BTN_INV, 124, 3, 3, "Режим управления");

                switch (StateMachine::get_current_control_mode()) {
                    case 0:
                        sprintf(display_buffer, "АВАРИЙНАЯ ОСТАНОВКА!");
                        display.drawUTF8(5, 29, display_buffer);
                        break;

                    case 1:
                        sprintf(display_buffer, "РУЧНОЕ УПРАВЛЕНИЕ");
                        display.drawUTF8(14, 29, display_buffer);
                        break;

                    case 2:
                        sprintf(display_buffer, "АВТОПИЛОТ");
                        display.drawUTF8(38, 29, display_buffer);
                        break;
                }

                FlySkyIBus::available() 
                    ? sprintf(display_buffer, "1. Связь с ПДУ:   OK") 
                    : sprintf(display_buffer, "1. Связь с ПДУ: FAIL");
                display.drawUTF8(4, 43, display_buffer);

                Estop::normal() 
                    ? sprintf(display_buffer, "2. E-Stop:        OK") 
                    : sprintf(display_buffer, "2. E-Stop:      FAIL");
                display.drawUTF8(4, 58, display_buffer);

                current_display_page = MESSAGES;
                break;

            case MESSAGES:
                char* message;

                display.drawButtonUTF8(64, 10, U8G2_BTN_HCENTER | U8G2_BTN_INV, 124, 3, 3, "Главный блок БПТС");
                display.drawUTF8(4, 26, "Текст сообщения...");

                /*

                if (xQueuePeek(messages_queue, &message, portMAX_DELAY) != pdFALSE) {
                    xQueueReceive(messages_queue, &message, portMAX_DELAY);
                    display.drawUTF8(4, 26, message);
                    free(message);
                }

                */

                current_display_page = UNITS_CONNECTION;
                break;
        }

        display.nextPage();
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
