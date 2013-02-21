/*********************************************************************************************************
* Name: main.c
* Desc: A test suite for the ImageProc 2.2 system. These tests should not be
* considered rigorous, exhaustive tests of the hardware, but rather
* "smoke tests" - ie. turn on the functionality and make sure the 
* hardware/software doesn't start "smoking."
*
* The architecture is based on a function pointer queue scheduling model. The
* meat of the testing logic resides in test.c. If the radio has received a 
* command packet during the previous timer interval for Timer2, the appropriate
* function pointer is added to a queue in the interrupt service routine for 
* Timer2 (interrupts.c). The main loop simply pops the function pointer off
* the top of the queue and executes it. 
*
* Date: 2011-04-13
* Author: AMH, Ryan Julian
*********************************************************************************************************/
#include "p33Fxxxx.h"
#include "init.h"
#include "init_default.h"
#include "timer.h"
#include "utils.h"
#include "queue.h"
#include "radio.h"
#include "tih.h"
#include "ams-enc.h"
#include "radio_settings.h"
#include "tests.h" // TODO (fgb) : define/includes need to live elsewhere
#include "dfmem.h"
#include "interrupts.h"
#include "mpu6000.h"
#include "sclock.h"
#include "spi_controller.h"
#include "interrupts.h"
#include "pid-ip2.5.h"
#include "cmd.h"
#include "uart_driver.h"

#include <stdlib.h>

Payload rx_payload;
MacPacket rx_packet;
Test* test;

int main() {
    
    fun_queue = queueInit(FUN_Q_LEN);
    test_function tf;
    
    /* Initialization */
   SetupClock();
   SwitchClocks();
   SetupPorts();
   sclockSetup();
   
   //SetupInterrupts();
   
   
   //dfmemSetup(0);
   //mpuSetup(1);
   //tiHSetup();
   LED_3 = 1;

   // Need delay for encoders to be ready
   delay_ms(100);
   amsEncoderSetup();
   cmdSetup();
   pidSetup();
   //uartInit();

   // Radio setup
   radioInit(RADIO_RXPQ_MAX_SIZE, RADIO_TXPQ_MAX_SIZE, 0);
   radioSetChannel(RADIO_MY_CHAN);
   radioSetSrcAddr(RADIO_SRC_ADDR);
   radioSetSrcPanID(RADIO_PAN_ID);
   setupTimer6(RADIO_FCY); // Radio and buffer loop timer
    SetupTimer2();
    
   EnableIntT2;
   LED_3 = 0;
   LED_1 = 1;
   long count = 1000;
   while(1){
       //radioProcess();
       if(--count == 0) {
           count = 4000;
           amsEncoderStartAsyncRead();
           //amsEncoderBlockingRead(0);
           //amsEncoderBlockingRead(1);
       }
       
       while(!queueIsEmpty(fun_queue))
       {
           test = queuePop(fun_queue);
           rx_payload = macGetPayload(test->packet);
           tf = test->tf;
           (*tf)(payGetType(rx_payload), payGetStatus(rx_payload), payGetDataLength(rx_payload), payGetData(rx_payload));
           radioReturnPacket(test->packet);
           free(test);
       }
   }
   return 0;
}
