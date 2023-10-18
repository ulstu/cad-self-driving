/* XDCtools Header files */
#include <stdlib.h>
#include <xdc/std.h>
#include <string.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSCC26XX.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"

extern UART_Handle Handleuart;
extern Event_Handle evtHandleUART;
extern Event_Handle evtHandleRF;
extern Mailbox_Handle mbxHandleUARTtx;
extern Mailbox_Handle mbxHandleRFtx;

uint8_t response_comb[5]={VAL_HEAD_RESPONSE1,VAL_HEAD_RESPONSE2,VAL_RESPONSE_OK,0,0};

uint8_t selectBufUART;
receiveUART receiveUARTbuf[2];
receiveUART* p_receiveUART;
receiveUART* p_accumulationUART;

KA Subscriber;
pKA pSubscriber;

pHPacket_WC pBuf;//указываем на буфер передачи

void Set_KA(KA *pSubscriber, uint8_t size_Subscriber, enum KArejim rejim)
{
 char* pKAtmp;
 char buffer[4];

 NVS_Handle nvsHandle;
 NVS_Attrs regionAttrs;
 NVS_Params nvsParams;

 NVSCC26XX_init();
 NVS_Params_init(&nvsParams);
 nvsHandle = NVSCC26XX_open(Board_NVSINTERNAL, &nvsParams);

 NVSCC26XX_getAttrs(nvsHandle, &regionAttrs);

 pKAtmp=(char*)pSubscriber;

 if(rejim==KA_NU)
 {//начальные условия
  NVSCC26XX_read(nvsHandle, 0, (void *) buffer, 4); //Чтение из NVS в buffer
  if((buffer[0]!=FLAG_KA_MIN) || (buffer[1]!=FLAG_KA_MIN)) //во flash нет карточки абонента
  {
   NVSCC26XX_write(nvsHandle, 0, (void *) pKAtmp, size_Subscriber, NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
  }
  else
  {
   NVSCC26XX_read(nvsHandle, 0, (void *) pKAtmp, size_Subscriber); //Чтение из NVS в pKAtmp
  }
 }

 if(rejim==KA_WRITE)
 {
  NVSCC26XX_write(nvsHandle, 0, (void *) pKAtmp, size_Subscriber, NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
  NVSCC26XX_read(nvsHandle, 0, (void *) pKAtmp, size_Subscriber); //Чтение из NVS в pKAtmp
 }

 NVSCC26XX_close(nvsHandle);
}

// Callback function
void uartReadCallback(UART_Handle Handleuart, void *input, size_t size)
{
    //GPIO_toggle(Board_GPIO_LED0);
 Event_post(evtHandleUART, Event_Id_UARTrx);
}

// Callback function
void uartWriteCallback(UART_Handle Handleuart, void *output, size_t size)
{
 Event_post(evtHandleUART, Event_Id_UARTtxen);
 //Оказываемся здесь после передачи всего пакета
}


void InitUART(void)
{
 UART_Params uartParams;

// Call driver init functions
 UART_init();

// Create a UART with data processing off.
 UART_Params_init(&uartParams);
 uartParams.writeDataMode = UART_DATA_BINARY;
 uartParams.readDataMode = UART_DATA_BINARY;
 uartParams.readReturnMode = UART_RETURN_FULL;
 uartParams.readMode = UART_MODE_CALLBACK;
 uartParams.writeMode = UART_MODE_CALLBACK;
 uartParams.readEcho = UART_ECHO_OFF;
 uartParams.baudRate = 115200;
 uartParams.readCallback = uartReadCallback;
 uartParams.writeCallback = uartWriteCallback;

 Handleuart = UART_open(Board_UART0, &uartParams);

 if (Handleuart == NULL) {
       System_abort("Error opening the UART");
 }
}


Void uartFxn(UArg arg0, UArg arg1)
{
 MsgObj msg, msg_uart;
 UInt posted;
 uint8_t clock_wait_uart_tx;


 InitUART();

 selectBufUART=0;
 p_accumulationUART=&receiveUARTbuf[selectBufUART];
 p_accumulationUART->tick=0;
 p_accumulationUART->index=0;
 p_receiveUART=&receiveUARTbuf[(selectBufUART^1)];
 UART_read(Handleuart, receiveUARTbuf[0].buf, WANTED_RX_BYTES);
 Event_post(evtHandleRF, Event_Id_UARTtxen);

 while (1)
 {
  posted=Event_pend(evtHandleUART,
                    Event_Id_NONE,  // andMask
                    Event_Id_UARTrx+Event_Id_UARTtxen,    // orMask
                    (1*100));//1 ms

  if(posted & Event_Id_UARTrx)
  {//Байт из UART
   p_accumulationUART->tick=0;
   p_accumulationUART->buf_all[p_accumulationUART->index] = receiveUARTbuf[0].buf[0];
   if(p_accumulationUART->index<(SIZE_PACKET_BUFFER_UART-1)) p_accumulationUART->index++;
   UART_read(Handleuart, receiveUARTbuf[0].buf, WANTED_RX_BYTES);
  }

  if(posted & Event_Id_UARTtxen)
  {//передача в UART разрешена
   clock_wait_uart_tx=5;//пауза после последнего пакета
  }

  if(!posted)
  {//Прошла 1 мс
   if(p_accumulationUART->index)
   {
    p_accumulationUART->tick++;
    if(p_accumulationUART->tick>TICK_RX_UART)
    {//Принят пакет из UART
     p_receiveUART=p_accumulationUART;
     selectBufUART^=1;//меняем места буферы UART
     p_accumulationUART=&receiveUARTbuf[selectBufUART];
     p_accumulationUART->tick=0;
     p_accumulationUART->index=0;

     //To RF мои пакеты
     if((p_receiveUART->index<=PACKET_SIZE_MAX))//w && (p_receiveUART->buf_all[p_receiveUART->index-1]==STOP_BYTE))
     {
      msg.len=p_receiveUART->index;
      memcpy(msg.buf, p_receiveUART->buf_all, msg.len);
      Mailbox_post(mbxHandleRFtx, &msg, BIOS_NO_WAIT);
     }
    }
   }

   //В uart из RF
   if(clock_wait_uart_tx) clock_wait_uart_tx--;
   else
   {
    if(Mailbox_pend(mbxHandleUARTtx, &msg_uart, BIOS_NO_WAIT))
    {
     if(msg_uart.len<=MAIL_BOX_PACKET_SIZE)
     {
      UART_write(Handleuart, msg_uart.buf, msg_uart.len);
     }
    }
   }

  }//!Прошла 1 мс

 }
}

