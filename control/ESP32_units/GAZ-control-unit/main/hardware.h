#pragma once

#define PIN_ESTOP_BUTTON        GPIO_NUM_42

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_FREQUENCY_HZ       8000

#define ADC_FREQ_HZ             4000
#define ADC_UNIT                ADC_UNIT_1
#define ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1
#define ADC_CHANNEL_CLUTCH_POS  ADC_CHANNEL_7
#define ADC_CHANNEL_CLUTCH_CS   ADC_CHANNEL_4
#define ADC_CHANNEL_BRAKE_POS   ADC_CHANNEL_8
#define ADC_CHANNEL_BRAKE_CS    ADC_CHANNEL_3

#define PIN_CLUTCH_EN           GPIO_NUM_41
#define PIN_CLUTCH_PWM_L        GPIO_NUM_40
#define PIN_CLUTCH_PWM_R        GPIO_NUM_39
#define CLUTCH_PWM_CHANNEL_L    LEDC_CHANNEL_2
#define CLUTCH_PWM_CHANNEL_R    LEDC_CHANNEL_3

#define PIN_BRAKE_EN            GPIO_NUM_38
#define PIN_BRAKE_PWM_L         GPIO_NUM_37
#define PIN_BRAKE_PWM_R         GPIO_NUM_36
#define BRAKE_PWM_CHANNEL_L     LEDC_CHANNEL_0
#define BRAKE_PWM_CHANNEL_R     LEDC_CHANNEL_1

#define PIN_ENGINE_IGNITION     GPIO_NUM_47
#define PIN_ENGINE_STARTER      GPIO_NUM_46

#define PIN_ETH_MISO            GPIO_NUM_10
#define PIN_ETH_MOSI            GPIO_NUM_14
#define PIN_ETH_CS              GPIO_NUM_12
#define PIN_ETH_SCK             GPIO_NUM_13
#define PIN_ETH_INT             GPIO_NUM_11
#define SPI_BUS_ETH             SPI3_HOST

#define PIN_WHEEL_EN            GPIO_NUM_21
