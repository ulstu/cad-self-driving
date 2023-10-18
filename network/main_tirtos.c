#include <stdlib.h>
#include <stddef.h>
#include <xdc/cfg/global.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Types.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>


/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Power.h>

/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */
#include <smartrf_settings/smartrf_settings.h>
#include <ti/drivers/rf/RF.h>
#include <ti/devices/DeviceFamily.h>
#include "RFQueue.h"

#include <ti/drivers/pin/PINCC26XX.h>

#include "my_RF.h"
#include "my_UART.h"


#define TASKSTACKSIZE     1024
#define UART_TASK_PRIORITY   1
#define RF_TASK_PRIORITY   1

UART_Handle Handleuart;

Task_Struct taskStructUART;
Char taskStackUART[TASKSTACKSIZE];

Task_Struct taskStructRF;
Char taskStackRF[TASKSTACKSIZE];

Mailbox_Struct mbxStructUARTtx;
Mailbox_Handle mbxHandleUARTtx;
Mailbox_Struct mbxStructRFrx;
Mailbox_Handle mbxHandleRFrx;
Mailbox_Struct mbxStructRFtx;
Mailbox_Handle mbxHandleRFtx;

Event_Struct evtStructUART;
Event_Handle evtHandleUART;
Event_Struct evtStructRF;
Event_Handle evtHandleRF;

uint8_t MBbufUARTtx[MAIL_BOX_BUFFER_SIZE_UARTtx];
uint8_t MBbufRFtx[MAIL_BOX_BUFFER_SIZE_RFtx];
uint8_t MBbufRFrx[MAIL_BOX_BUFFER_SIZE_RFrx];

///////////////////////////////////////////////////////////
int main(void)
{
 //   Clock_Params clkParams; // Construct BIOS Objects
    Task_Params taskParams;
    Mailbox_Params mbxParams;

    /* Call driver init functions */
    Board_initGeneral();

    GPIO_init();
    /* Configure the LED and button pins */
    GPIO_setConfig(Board_GPIO_LED_RX, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_GPIO_LED_TX, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &taskStackUART;
    taskParams.priority = UART_TASK_PRIORITY;
    taskParams.instance->name = "workUART";
    Task_construct(&taskStructUART, (Task_FuncPtr)uartFxn, &taskParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &taskStackRF;
    taskParams.priority = RF_TASK_PRIORITY;
    taskParams.instance->name = "workRF";
    Task_construct(&taskStructRF, (Task_FuncPtr)RFFxn, &taskParams, NULL);

    Event_construct(&evtStructRF, NULL); // Obtain event instance handle
    evtHandleRF = Event_handle(&evtStructRF);
    Event_construct(&evtStructUART, NULL); // Obtain event instance handle
    evtHandleUART = Event_handle(&evtStructUART);

    /* Construct a Mailbox Instance */ //U0tx
    Mailbox_Params_init(&mbxParams);
    mbxParams.buf=(void*)MBbufUARTtx;
    mbxParams.bufSize=sizeof(MBbufUARTtx);
    Mailbox_construct(&mbxStructUARTtx,sizeof(MsgObj), MAIL_BOX_PACET_IN_BUFFER_UARTtx, &mbxParams, NULL);
    mbxHandleUARTtx = Mailbox_handle(&mbxStructUARTtx);

    /* Construct a Mailbox Instance */ //RFtx
    Mailbox_Params_init(&mbxParams);
    mbxParams.buf=(void*)MBbufRFtx;
    mbxParams.bufSize=sizeof(MBbufRFtx);
    Mailbox_construct(&mbxStructRFtx,sizeof(MsgObj), MAIL_BOX_PACET_IN_BUFFER_RFtx, &mbxParams, NULL);
    mbxHandleRFtx = Mailbox_handle(&mbxStructRFtx);

    /* Construct a Mailbox Instance */ //RFrx
    Mailbox_Params_init(&mbxParams);
    mbxParams.buf=(void*)MBbufRFrx;
    mbxParams.bufSize=sizeof(MBbufRFrx);
    Mailbox_construct(&mbxStructRFrx,sizeof(MsgObj), MAIL_BOX_PACET_IN_BUFFER_RFrx, &mbxParams, NULL);
    mbxHandleRFrx = Mailbox_handle(&mbxStructRFrx);

    //Power_setConstraint(4);//Power_setConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
    /* Start BIOS */
    BIOS_start();

    return (0);
}

