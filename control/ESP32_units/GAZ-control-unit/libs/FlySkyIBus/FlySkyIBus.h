#include <driver/gpio.h>
#include <driver/uart.h>
#include <inttypes.h>

class FlySkyIBus
{
public:
    static void init();
    static void loop(void* args);
    static uint16_t readChannel(uint8_t channelNr);
    static bool available();

private:
    enum State {
        GET_PREAM,
        GET_DATA,
        GET_CHKSUML,
        GET_CHKSUMH,
        DISCARD,
    };

    static const uint8_t PROTOCOL_LENGTH = 0x20;
    static const uint8_t PROTOCOL_OVERHEAD = 3;
    static const int64_t PROTOCOL_TIMEGAP = 5 * 1000;
    static const uint8_t PROTOCOL_CHANNELS = 10;
    static const uint8_t PROTOCOL_CMD = 0x55;

    static uart_port_t uart_pin_num;
    static gpio_num_t rxd_pin_num;
    static char data[2048];

    static uint8_t state;
    static int64_t last;
    static uint8_t buffer[PROTOCOL_LENGTH];
    static uint8_t ptr;
    static uint8_t len;
    static uint16_t channel[PROTOCOL_CHANNELS];
    static uint16_t chksum;
    static uint8_t lchksum;
};
