#include "cmd.h"
#include "cmd_const.h"
#include "dfmem.h"
#include "utils.h"
#include "ports.h"
#include "sclock.h"
#include "led.h"
#include "blink.h"
#include "payload.h"
#include "mac_packet.h"
#include "dfmem.h"
#include "radio.h"
#include "dfmem.h"
#include "tests.h"
#include "queue.h"
#include "version.h"
#include "radio_settings.h"
#include "timer.h"
#include "tih.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void (*cmd_func[MAX_CMD_FUNC])(unsigned char, unsigned char, unsigned char, unsigned char*);
void cmdError(void);

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static void cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

//Motor and PID functions
static void cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);



/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
void cmdSetup(void) {

    unsigned int i;

    // initialize the array of func pointers with Nop()
    for(i = 0; i < MAX_CMD_FUNC; ++i) {
        cmd_func[i] = &cmdNop;
    }
    cmd_func[CMD_TEST_RADIO] = &test_radio;
    cmd_func[CMD_SET_THRUST_OPENLOOP] = &cmdSetThrustOpenLoop;

}

void cmdPushFunc(MacPacket rx_packet)
{   Payload rx_payload;
    unsigned char command;  
	 rx_payload = macGetPayload(rx_packet);
	 
	 Test* test = (Test*) malloc(sizeof(Test));
     if(!test) return;
	 
	 test->packet = rx_packet;
     
     command = payGetType(rx_payload);

	   if( command < MAX_CMD_FUNC)
	  {     test->tf=cmd_func[command];
		   queuePush(fun_queue, test); 
	  }   
	  else 
	 {  cmdError();   // halt on error - could also just ignore....
	 }

}

// Motor / PID Commands

void cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
 {	int thrust1 = frame[0] + (frame[1] << 8);
	int thrust2 = frame[2] + (frame[3] << 8);
	unsigned int run_time_ms = frame[4] + (frame[5] << 8);

	//DisableIntT1;	// since PID interrupt overwrites PWM values

  	tiHSetDC(1, thrust1);
	tiHSetDC(2, thrust2); 
	delay_ms(run_time_ms);
	tiHSetDC(1,0);
	tiHSetDC(2,0);

	//EnableIntT1;
 } 

void cmdError()
{ int i;
 	EmergencyStop();
	for(i= 0; i < 10; i++)
	 {	LED_1 ^= 1;
			delay_ms(200);
			LED_2 ^= 1;
			delay_ms(200);
			LED_3 ^= 1;
			delay_ms(200);
          }
}

static void cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
    Nop();
}