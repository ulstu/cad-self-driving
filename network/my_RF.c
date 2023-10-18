/* XDCtools Header files */
#include <xdc/std.h>
#include <stdlib.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Semaphore.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"

/* EasyLink API Header files */

#include <smartrf_settings/smartrf_settings.h>
#include <ti/devices/cc13x0/driverlib/rf_common_cmd.h>
#include <ti/drivers/rf/RF.h>
#include <ti/devices/DeviceFamily.h>
#include "RFQueue.h"
#include <protocol.h>

#define PAYLOAD_LENGTH         132
#pragma DATA_ALIGN(rxDataEntryBuffer, 4)
uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES, PAYLOAD_LENGTH, NUM_APPENDED_BYTES)];

RF_Handle rfHandle;
RF_Params rfParams;
RF_Object rfObject;

RF_Stat ret;
RF_CmdHandle rfRxCmdHandle;

extern Event_Handle evtHandleRF;
extern Event_Handle evtHandleUART;
extern Mailbox_Handle mbxHandleRFtx;
extern Mailbox_Handle mbxHandleRFrx;
extern Mailbox_Handle mbxHandleUARTtx;

rfc_propRxOutput_t rxStatistics;

rfc_dataEntryGeneral_t* currentDataEntry;

int8_t rssi;
MsgObj msg_RF;

///////////////////////////////////////////////////////////
void rxrfcallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
 pHPacket pInputPacket;
 if(e & RF_EventRxEntryDone)
 {
  GPIO_toggle(Board_GPIO_LED_RX);
  do
  {
  /* Get current unhandled data entry */
   currentDataEntry = RFQueue_getDataEntry();
   pInputPacket=(pHPacket)(&currentDataEntry->data+1);
   if((pInputPacket->Start==START_BYTE) && ((*(uint8_t*)(&currentDataEntry->data + (*(uint8_t*)(&currentDataEntry->data))))==STOP_BYTE) && (pInputPacket->TTL!=0) && ((*(uint8_t*)(&currentDataEntry->data))<=PACKET_SIZE_MAX))
   {//ѕришел пакет
    Package_processing((uint8_t*)(&currentDataEntry->data + 1), *(uint8_t*)(&currentDataEntry->data), rxStatistics.lastRssi); //на обработку
   }
  } while(RFQueue_nextEntry()==DATA_ENTRY_FINISHED);
 }
}

void txrfcallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
 if(e & RF_EventCmdDone)
 {
  GPIO_toggle(Board_GPIO_LED_TX);
  Event_post(evtHandleRF, Event_Id_RFtx);
 }
}

/////////////////////////////////////////////////////////////
Void RFFxn(UArg arg0, UArg arg1)
{//RF_Stat rfStat;
 MsgObj msg;//, msg_uart;
 UInt posted;
 uint8_t abortGraceful;
 uint16_t clock_tick;
 uint8_t len_sending_packages;
 uint8_t* pBuf_sending_packages;
 dataQueue_t dataQueue;
 RF_Params rfParams;

 Preset_protocol();

 rfParams.nInactivityTimeout = BIOS_WAIT_FOREVER;
 RF_Params_init(&rfParams);

 if( RFQueue_defineQueue(&dataQueue, rxDataEntryBuffer, sizeof(rxDataEntryBuffer), NUM_DATA_ENTRIES, MAX_LENGTH + NUM_APPENDED_BYTES))
 {
  /* Failed to allocate space for all data entries */
  while(1);
 }

/* Modify CMD_PROP_RX command for application needs */
 RF_cmdPropRx.pQueue = &dataQueue;           /* Set the Data Entity queue for received data */
 RF_cmdPropRx.pOutput = (uint8_t*)&rxStatistics;
 RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;  /* Discard ignored packets from Rx queue */
 RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;   /* Discard packets with CRC error from Rx queue */
 RF_cmdPropRx.maxPktLen = 0xFF;//MAX_LENGTH;        /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
 RF_cmdPropRx.pktConf.bRepeatOk = 1;         //Ostaemcya v Rx
 RF_cmdPropRx.pktConf.bRepeatNok = 1;
 RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;//TRIG_ABSTIME;
 RF_cmdPropTx.startTrigger.pastTrig = 1;

 {
  /* Request access to the radio */
  rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
  /* Set the frequency */
  RF_CmdHandle rfCmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
  /* Enter RX mode and stay forever in RX */
  rfRxCmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &rxrfcallback, RF_EventRxEntryDone);
 }

 clock_tick=0;
 while (1)
 {
  posted = Event_pend(evtHandleRF,
                      Event_Id_NONE,
                      (Event_Id_RFtx+Event_Id_RFrx),
                      (1*100));
   /* Process all the events that have occurred */

  if(!posted)
  {
   clock_tick++;
   if(clock_tick>CLOCK_KVANT_LIMIT) //прошло 500 мс
   {
    clock_tick=0;
    //провер€ем буффер вывода в RF
    len_sending_packages=0;
    Procedure_sending_packages(&pBuf_sending_packages, &len_sending_packages);
    if((len_sending_packages) && (len_sending_packages<=PACKET_SIZE_MAX))
    {
     //есть, что выводить в радио
     abortGraceful = 0;
     RF_cancelCmd(rfHandle, rfRxCmdHandle, abortGraceful);

     // Send packet
     msg_RF.len=len_sending_packages;
     memcpy(msg_RF.buf, pBuf_sending_packages, msg_RF.len);
     RF_cmdPropTx.pktLen=msg_RF.len;
     RF_cmdPropTx.pPkt=msg_RF.buf;
     RF_CmdHandle result = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, &txrfcallback, RF_EventCmdDone);
     Mailbox_post(mbxHandleUARTtx, &msg_RF, BIOS_NO_WAIT);//дл€ отладки
     continue;
    }

   }

  }// !//ѕрошла 1 мс

  //включение приемника FR
  if(posted & (Event_Id_RFtx)) //может надо только после передачи, дл€ восстановлени€ приема?
  {
   // Enter RX mode and stay forever in RX
   rfRxCmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &rxrfcallback, RF_EventRxEntryDone);
  }

 }//while(1)
}
