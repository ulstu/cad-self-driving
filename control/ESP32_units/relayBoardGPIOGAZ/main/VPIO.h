#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


#include "../../common/dt_device_tree.h"

extern uint8_t inputs[4];


#define VGPIO_ENTER_CRITICAL()  portENTER_CRITICAL(&g_io.gpio_spinlock)
#define VGPIO_EXIT_CRITICAL()   portEXIT_CRITICAL(&g_io.gpio_spinlock)

#define VGPIO_REFRESH(igpio, ogpio, i_bytes, o_bytes) \
do { \
    VGPIO_ENTER_CRITICAL(); \
    vpio_refresh((igpio), (ogpio), (i_bytes), (o_bytes)); \
    HC595165_PL(DT_LOW);\
    VGPIO_EXIT_CRITICAL(); \
} while(0)


#define GPIO_CNT_ALIGN4  BYTE_ALIGN(sizeof(u32_t), GPIO_CNT)
#define GPIO_BIT2BYTE4   BIT_ALIGN_BYTE(u32_t, GPIO_CNT)

#define GPIO_I_BIT2BYTE4   BIT_ALIGN_BYTE(u32_t, VGPIO_I_MAX)

#define VPIO_I_BYTE(_i_cnt_)  (((_i_cnt_)+7)/8)
#define VPIO_O_BYTE(_o_cnt_)  VPIO_I_BYTE(_o_cnt_)
#define VPIO_IB8    VPIO_I_BYTE(8)
#define VPIO_OB8    VPIO_O_BYTE(8)
#define VPIO_IB16   VPIO_I_BYTE(16)
#define VPIO_OB16   VPIO_O_BYTE(16)
#define VPIO_IB24   VPIO_I_BYTE(24)
#define VPIO_OB24   VPIO_O_BYTE(24)
#define VPIO_IB32   VPIO_I_BYTE(32)
#define VPIO_OB32   VPIO_O_BYTE(32)

#define VPIO_I_BYTE_START   BIT_ALIGN_BYTE(u8_t, GPIO_I_START)
#define VPIO_O_BYTE_START   BIT_ALIGN_BYTE(u8_t, GPIO_O_START)

#define VPIO_I_BIT2BYTE    BIT_ALIGN_BYTE(u8_t, VGPIO_I_MAX)
#define VPIO_O_BIT2BYTE    BIT_ALIGN_BYTE(u8_t, VGPIO_O_MAX)

#define GPIO_I_BIT2BYTE    BIT_ALIGN_BYTE(u8_t, DT_I_CNT)
#define GPIO_O_BIT2BYTE    BIT_ALIGN_BYTE(u8_t, DT_R_CNT)

#define INPUT_DEBOUNCE_MS   100
#define FACTORY_TIMEOUT     INPUT_DEBOUNCE_MS


class VPIO {
public:
    enum {
        GPIO_START,

        GPIO_I_START = GPIO_START,
        GPIO_I_END = GPIO_I_START+(VGPIO_I_MAX-1),

        GPIO_O_START,
        GPIO_O_END = GPIO_O_START+(VGPIO_I_MAX-1),

        GPIO_I_FACTORY,

        GPIO_END,
        GPIO_CNT = GPIO_END,
    };
    typedef struct _dt_io_t {
        u8_t xpio_map[GPIO_CNT_ALIGN4];

        u8_t t_xpio_bits[GPIO_BIT2BYTE4];
        u8_t xpio_bits[GPIO_BIT2BYTE4];

        u32_t input_debounce_ms[VGPIO_I_MAX];
        u8 at_debounce[GPIO_I_BIT2BYTE4];

        QueueHandle_t gpio_queue;
        portMUX_TYPE gpio_spinlock;

        b8_t started;
        u8_t factory_btn;
        u8_t factory_led;
        u8_t res[1];
        u32 factory_press_ms;
    } dt_io_t;

    static dt_io_t g_io;

    enum {
        QUEUE_GPIO_INPUT_START = 0,
        QUEUE_GPIO_INPUT_END = QUEUE_GPIO_INPUT_START + (GPIO_MAX-1),
        QUEUE_VPIO_INPUT_START,
        QUEUE_VPIO_INPUT_END = QUEUE_VPIO_INPUT_START + (VGPIO_I_MAX-1),
        QUEUE_VPIO_OUTPUT_START,
        QUEUE_VPIO_OUTPUT_END = QUEUE_VPIO_OUTPUT_START + (VGPIO_O_MAX-1),

        QUEUE_GPIO_SYNC,
        QUEUE_GPIO_START,
        QUEUE_GPIO_EXIT,
    };

    static ddt_t g_dt;

    static void setConfig(ddt_t ddt);
    static void factory_led_init(void);
    static void factory_led_action(b8_t on);
    static u32_t time_dec(u32_t now, u32_t before);
    static void vpio_tx_byte(u8_t byte);
    static u8_t vpio_rx_byte(void);
    static u8_t vpio_tx_rx_byte(u8_t w_byte);
    static void vpio_refresh(u8_t *igpio, u8_t *ogpio, u8_t i_bytes, u8_t o_bytes);
    static void vpio_init(dt_io_t *iox);
    static b8 xpio_r(u8 gpio);
    static void xpio_w(u8 gpio, b8 on);
    static void xpio_sync(dt_io_t *iox);
    static void gpio_capture_factory(dt_io_t *iox, u32_t io_num);
    static void gpio_check_factory(dt_io_t *iox);
    static void gpio_i_change(dt_io_t *iox, u32_t io_num);
    static void vpio_i_change(dt_io_t *iox, u32_t io_num);
    static u32_t vpio_check_input(dt_io_t *iox);
    static void xpio_input_debounce(dt_io_t *iox);
    static void IRAM_ATTR gpio_isr_handler(void* arg);
    static void gpio_task(void* arg);
    static b8_t relay_r(u8_t i);
    static void relay_w(u8_t i, b8_t on);
    static void iox_sync(void);
    static void iox_start(void);
    static void iox_exit(void);
    static void init(ddt_t ddt);
};