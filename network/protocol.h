#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "Board.h"

extern KA Subscriber;
uint8_t indxPctBuf;
uint8_t indx_last_packet;
PctBuf PacketBuf[SIZE_PCTBUF]; //пакеты на передачу и ретрансл€цию (буфер)
uint8_t NumberPacket;
uint16_t time_now;
PTmp PacketReceiv;
Stat Statistics;
Hover Handover[MAX_SIZE_HANDOVER_LIST];
uint8_t CounterRepeat;
uint16_t CounterTrust;
MsgObj msg_uart;
CInterval CycleIntervalArchiv[NUMBER_OF_DEVICES];
uint16_t WDT_tmp;
int8_t last_rssi_G;
PTmp DataUp;
PTmp DataDown;

void Preset_protocol(void);
void Package_processing(uint8_t* Received_packet, uint8_t len, int8_t last_rssi);
void Procedure_sending_packages(uint8_t** pBuf_sending_packages, uint8_t* pLen_sending_packages);

#endif /* PROTOCOL_H_ */
