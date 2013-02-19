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
#include "pid-ip2.5.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void (*cmd_func[MAX_CMD_FUNC])(unsigned char, unsigned char, unsigned char, unsigned char*);
void cmdError(void);

extern pidPos pidObjs[NUM_PIDS];

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static void cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetAMSPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

//Motor and PID functions
static void cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdPIDStartMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

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
    cmd_func[CMD_SET_PID_GAINS] = &cmdSetPIDGains;
    cmd_func[CMD_PID_START_MOTORS] = &cmdPIDStartMotors;
    cmd_func[CMD_SET_VEL_PROFILE] = &cmdSetVelProfile;
    cmd_func[CMD_WHO_AM_I] = &cmdWhoAmI;
    cmd_func[CMD_GET_AMS_POS] = &cmdGetAMSPos;

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

// send robot info when queried
void cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) 
{   unsigned char i, string_length; unsigned char *version_string;
// maximum string length to avoid packet size limit
	version_string = (unsigned char *)"DHALDANE_VRoACH;PID-HARD;STEER-HARD: Tue Feb 12 14:04:47 2013"
;
	i = 0;
	while((i < 127) && version_string[i] != '\0')
	{ i++;}
	string_length=i;     
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_WHO_AM_I, status, string_length, version_string);  
      return; //success
}

void cmdGetAMSPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) 
{ 	long motor_count[2]; 
	motor_count[0] = pidObjs[0].p_state;
	motor_count[1] = pidObjs[1].p_state;

	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_GET_AMS_POS,\
		 status, sizeof(motor_count), (unsigned char *)motor_count);  
}

// ==== Motor PID Commands ======================================================================================
// ================================================================================================================ 

void cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
 {	int thrust1 = frame[0] + (frame[1] << 8);
	int thrust2 = frame[2] + (frame[3] << 8);
	unsigned int run_time_ms = frame[4] + (frame[5] << 8);

	DisableIntT1;	// since PID interrupt overwrites PWM values

  	tiHSetDC(1, thrust1);
	tiHSetDC(2, thrust2); 
	delay_ms(run_time_ms);
	tiHSetDC(1,0);
	tiHSetDC(2,0);

	EnableIntT1;
 } 

 void cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int Kp, Ki, Kd, Kaw, ff;
	int idx = 0;

	Kp = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Ki = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kd = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kaw = frame[idx] + (frame[idx+1] << 8); idx+=2;
	ff = frame[idx] + (frame[idx+1] << 8); idx+=2;
	pidSetGains(0,Kp,Ki,Kd,Kaw, ff);
	Kp = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Ki = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kd = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kaw = frame[idx] + (frame[idx+1] << 8); idx+=2;
	ff = frame[idx] + (frame[idx+1] << 8); idx+=2;
	pidSetGains(1,Kp,Ki,Kd,Kaw, ff);

	//Send confirmation packet
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_SET_PID_GAINS, status, 20, frame);  
      return; //success
}

void cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int interval[NUM_VELS], delta[NUM_VELS], vel[NUM_VELS];
	int idx = 0, i = 0;

	for(i = 0; i < NUM_VELS; i ++){
		interval[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2;	
	 }
	for(i = 0; i < NUM_VELS; i ++){
		delta[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	
	 }
	for(i = 0; i < NUM_VELS; i ++){
		vel[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	
	 }

	setPIDVelProfile(0, interval, delta, vel);

	for(i = 0; i < NUM_VELS; i ++){
		interval[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2;	
	 }
	for(i = 0; i < NUM_VELS; i ++){
		delta[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	
	 }
	for(i = 0; i < NUM_VELS; i ++){
		vel[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	
	 }
	setPIDVelProfile(1, interval, delta, vel);

	//Send confirmation packet
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_SET_VEL_PROFILE, status, 48, frame);  
     return; //success
}

void cmdPIDStartMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{	int thrust1 = frame[0] + (frame[1] << 8);
	unsigned int run_time_ms1 = frame[2] + (frame[3] << 8);
	int thrust2 = frame[4] + (frame[5] << 8);
	unsigned int run_time_ms2 = frame[6] + (frame[7] << 8);
	//currentMove = manualMove;
	pidSetInput(0 ,thrust1, run_time_ms1);
	pidOn(0);
	pidSetInput(1 ,thrust2, run_time_ms2);
	pidOn(1);
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