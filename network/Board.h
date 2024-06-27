
#ifndef __BOARD_H
#define __BOARD_H

#define Board_CC1310_LAUNCHXL

//#define COMBINE_CONV //Нужно для миниRF модуля. Для исключения передачи пакетов из RF в UART для миниRF модуля
#define RF_MAX //нужен для работы "больших" модулей без "маленьких"

#ifdef __cplusplus
extern "C" {
#endif

#include "CC1310_LAUNCHXL.h"

#define MY_NUMBER 0x00000001
#define START_BYTE '$'
#define STOP_BYTE '#'

#define Board_initGeneral()     CC1310_LAUNCHXL_initGeneral()

#define Board_GPIO_LED_TX       CC1310_LAUNCHXL_GPIO_LED_RED
#define Board_GPIO_LED_RX       CC1310_LAUNCHXL_GPIO_LED_GREEN

#define Board_UART0             CC1310_LAUNCHXL_UART0

#define Board_WATCHDOG0         CC1310_LAUNCHXL_WATCHDOG0
#define Board_NVSINTERNAL       CC1310_LAUNCHXL_NVSCC26XX0

#define RFEASYLINKRX_ASYNC
#define RFEASYLINKRX_ADDR_FILTER

#define DELTA_SEND_TO_UART 1000

#define TICK_RX_UART 2

#define Event_Id_CLOCK_1ms      Event_Id_00
#define Event_Id_UARTrx         Event_Id_01
#define Event_Id_UARTtxen       Event_Id_02
#define Event_Id_RFrx           Event_Id_04
#define Event_Id_RFtx           Event_Id_05

#define CLOCK_KVANT_LIMIT 500
#define MAIL_BOX_PACKET_SIZE 128
#define PACKET_SIZE_MAX MAIL_BOX_PACKET_SIZE
#define SIZE_PACKET_BUFFER_UART 256

#define WANTED_RX_BYTES 1

#define MAIL_BOX_PACET_IN_BUFFER_UARTtx 2
#define MAIL_BOX_PACET_IN_BUFFER_RFtx 2
#define MAIL_BOX_PACET_IN_BUFFER_RFrx 2
#define MAIL_BOX_BUFFER_SIZE (MAIL_BOX_PACKET_SIZE*MAIL_BOX_PACET_IN_BUFFER+28)
#define MAIL_BOX_BUFFER_SIZE_UARTtx ((MAIL_BOX_PACKET_SIZE+28)*MAIL_BOX_PACET_IN_BUFFER_UARTtx)
#define MAIL_BOX_BUFFER_SIZE_RFtx ((MAIL_BOX_PACKET_SIZE+28)*MAIL_BOX_PACET_IN_BUFFER_RFtx)
#define MAIL_BOX_BUFFER_SIZE_RFrx ((MAIL_BOX_PACKET_SIZE+28)*MAIL_BOX_PACET_IN_BUFFER_RFrx)

typedef struct MsgObj {
    uint16_t  len;            /* Длина пакета */
    uint8_t   buf[MAIL_BOX_PACKET_SIZE];       /* пакет */
} MsgObj, *Msg;

typedef struct receiveUART
{
 uint16_t tick;
 uint16_t index;
 uint8_t buf[WANTED_RX_BYTES];
 uint8_t buf_all[SIZE_PACKET_BUFFER_UART];
} receiveUART;

#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             (MAIL_BOX_PACKET_SIZE-2*0) /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */

//Протокол
#define MASKA_STATUS 0x03
#define TYPE_PACKET_MY 1
#define TYPE_PACKET_COMBINE 2
#define TYPE_PACKET_SENSOR_CONF 3
#define TYPE_PACKET_NULL 128


#define NUMBER_OF_DEVICES 250 //Количество устройств
#define TTL_NU 30
#define REPEAT_NU 2
#define SIGMA_SEND_NU  0xF //временной интервал в квантах для передачи пакета между двумя ретрансляторами
#define MULTI_K 10     //отличие длительности пакетов маршрутизации от архивных пакетов (14 байт и 128 байт)
#define KVANT_TO_SECOND 1 //при скорости 50 кб/с длительность одного кванта для пакетов маршрутизации 3.2 мс
#define PAUZA_REPIT_FROM_ROUTER_DOWN 4


#define BROADCAST_ADDRESS 0xFFFFFFFF

#define SIGMA_WAIT_ANSWER_SURVEY_DEVICES_IN_LEVEL_PLUS 4 //Время ожидания ответа на команду опроса устройства
#define SIGMA_WAIT_2s 14000 
#define SIGMA_PROCESSING_ARCHIVE 30

#define LIMIT_TRUST 3
#define LIMIT_HANDOVER_TRUST 3//лимит доверия, к роутеру при handover
#define LIMIT_HANDOVER_TRUST_DOP 0
#define LIMIT_HANDOVER_CYCLE 0xFFF //4095 предельное значение длительности цикла опроса
#define SIZE_MY_NUMBER 4
#define MAX_SIZE_HANDOVER_LIST 10 //число потенциальных роутеров под handover
#define MAX_PACKET_SEND_PROCESSING_ARCHIVE 17//максимальное число пакетов, отправляемых их буфера за один сеанс связи

typedef struct PTmp {
    uint8_t len;        /* Длина пакета */
    uint8_t buf[PACKET_SIZE_MAX];
} PTmp;

typedef struct __attribute__((__packed__)) PctBuf {
    uint8_t  len;                   // Длина пакета
    uint8_t  buf[PACKET_SIZE_MAX];  // пакет
    uint16_t time_tmp;              //текущее время
    uint16_t time_delta_for_send;   //временной порог простоя перед отправкой пакета
    uint8_t  repeat;                //число повторов
} PctBuf, *pPctBuf;
#define SIZE_PCTBUF 32// 32//64 - минимум 16 для 250 устройств
#define INDX_PCT_BUF_0 (SIZE_PCTBUF-1)

typedef struct __attribute__((__packed__)) KA {
    uint8_t Start;
    uint32_t Number_Module;
    uint8_t Role;
    uint8_t TTL;
    uint8_t Repeat;
    uint8_t Sigma_send; //временной интервал для передачи пакета в квантах
} KA, *pKA;//Subscriber

typedef struct __attribute__((__packed__)) HPacket {
    uint8_t Start;
    uint8_t Type_Comand;
    uint32_t Number_Destination;
    uint32_t Number_Source;
    uint8_t Level;
    uint8_t N_Paketa;
    uint8_t TTL;
    uint8_t Status;
} HPacket, *pHPacket;


typedef struct Stat {
    uint8_t Stage;
    uint16_t time_Stage;
    uint16_t time_Stage_limit;
    uint16_t counter_Stage_Processing_Archive;
    uint32_t Router_Down;//нижний роутер
    uint8_t Level_now;
    uint8_t Router_Down_Counter_Trust;
    uint8_t Number_Queue;
    uint32_t List_devices_in_level_1[NUMBER_OF_DEVICES]; //список устройств в Level+1
    uint8_t Status_devices_in_level_1[NUMBER_OF_DEVICES]; //эффективность опроса устройств в Level+1
} Stat;

typedef struct Hover
{
    uint32_t Router_Down;
    uint8_t Counter;
    uint8_t Level;
}Hover;


typedef enum TypeComand {
    TYPE_COMAND_SURVEY_DEVICES_IN_LEVEL_PLUS=0,
    TYPE_COMAND_I_AM_NEW_DEVICES_IN_LEVEL_PLUS,
    TYPE_COMAND_ARCHIVE

} TipComand;

typedef enum StageProtocol {
    STAGE_IDLE=0,
    STAGE_SURVEY_DEVICES_IN_LEVEL_PLUS,
    STAGE_RECEIVING_MESSAGES_FROM_NEW_DEVISES_LEVEL_PLUS,//время приема пакетов от новых устройств Level+1
    STAGE_RECEIVING_ANSWER_SURVEY_DEVICE_IN_LEVEL_PLUS,//время ожидания ответа от текущего верхнего устройства
    STAGE_PROCESSING_ARCHIVE
} StageProtocol;

typedef enum KArejim {
    KA_NU=0,
    KA_WRITE
} KArejim;

typedef enum RoleDevice {
    ROLE_DEVICES=0,
    ROLE_SERVER
} RoleDevice;

#define FLAG_KA_MIN 0xEE
#define FLAG_KA 0xEEEE
#define FLAG_READ_KA 0xE0EE

#define TX_POWER_MIN 0x0EC1;//2EC6;
#define TX_POWER_MAX 0xB83F;

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
