/*
 * Name: UpdatePID.c
 * Desc: Control code to compute the new input to the plant
 * Date: 2009-04-03
 * Author: AMH
   modified to include hall effect sensor by RSF.
 * modified Dec. 2011 to include telemetry capture
 * modified Jan. 2012 to include median filter on back emf
 * modified Jan. 2013 to include AMS Hall encoder, and MPU 6000 gyro
 */
#include "pid-ip2.5.h"
#include "dfmem.h"
#include "timer.h"
#include "adc_pid.h"
#include "pwm.h"
#include "led.h"
#include "adc.h"
#include "p33Fxxxx.h"
#include "sclock.h"
#include "ams-enc.h"
#include "tih.h"
#include "mpu6000.h"
#include "uart_driver.h"
#include "ppool.h"
#include "cmd.h"

#include <stdlib.h> // for malloc
#include "init.h"  // for Timer1

#define MC_CHANNEL_PWM1     1
#define MC_CHANNEL_PWM2     2
#define MC_CHANNEL_PWM3     3
#define MC_CHANNEL_PWM4     4

//#define HALFTHROT 10000
#define HALFTHROT 2000
#define FULLTHROT 2*HALFTHROT
// MAXTHROT has to allow enough time at end of PWM for back emf measurement
// was 3976
#define MAXTHROT 3800

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

// PID control structure
pidPos pidObjs[NUM_PIDS];

// structure for reference velocity for leg
static pidVelLUT  pidVel[NUM_PIDS];

#define T1_MAX 0xffffff  // max before rollover of 1 ms counter
// may be glitch in longer missions at rollover
static volatile unsigned long t1_ticks;

// Telemetry objects
//for battery voltage:
static char calib_flag = 0;   // flag is set if doing calibration
static long offsetAccumulatorL, offsetAccumulatorR;
static unsigned int offsetAccumulatorCounter;

// State of synchronization LED
static unsigned char sync;

// UART streaming objects
static volatile unsigned char interrupt_count = 0;
extern volatile MacPacket uart_tx_packet;
extern volatile unsigned char uart_tx_flag;

// Internal update functions
static inline void pidUpdateState(int pid_num);
static inline void pidUpdateSetpoint(int pid_num);
static inline void pidUpdateControl(int pid_num);
static inline void pidUpdateTelem(telemStruct_t *telemBuffer);

// ----------   all the initializations  -------------------------
// -------------------------------------------
// called from main()
void pidSetup(void) {
    int i;
    for(i = 0; i < NUM_PIDS; i++){
        pidInitPos(i, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KAW, DEFAULT_FF);
        pidInitVelProfile(i);
    }
    

    unsigned int T1CON1value, T1PERvalue;
    T1CON1value = T1_ON & T1_SOURCE_INT & T1_PS_1_8 & T1_GATE_OFF &
              T1_SYNC_EXT_OFF & T1_INT_PRIOR_4;
    T1PERvalue = 0x03E8; //clock period = 0.0002s = ((T1PERvalue * prescaler)/FCY) (5000Hz)
    t1_ticks = 0;
    OpenTimer1(T1CON1value, T1PERvalue);

    //  initialize PID structures before starting Timer1
    pidSetInput(0,0);
    pidSetInput(1,0);

    EnableIntT1; // turn on pid interrupts

    //calibBatteryOffset(100); //???This is broken for 2.5
}

// called from pidSetup()
void pidInitPos(int pid_num, int Kp, int Ki, int Kd, int Kaw, int ff) {
    pidObjs[pid_num].p_input = 0;
    pidObjs[pid_num].v_input = 0;
    pidObjs[pid_num].p = 0;
    pidObjs[pid_num].i = 0;
    pidObjs[pid_num].d = 0;
    pidObjs[pid_num].Kp = Kp;
    pidObjs[pid_num].Ki= Ki;
    pidObjs[pid_num].Kd = Kd;
    pidObjs[pid_num].Kaw = Kaw;
    pidObjs[pid_num].feedforward = 0;
    pidObjs[pid_num].output = 0;
    pidObjs[pid_num].onoff = 0;
    pidObjs[pid_num].p_error = 0;
    pidObjs[pid_num].v_error = 0;
    pidObjs[pid_num].i_error = 0;
}

// set expire time for first segment in pidSetInput - use start time from MoveClosedLoop
// set points and velocities for one revolution of leg
// called from pidSetup()
void pidInitVelProfile(int pid_num) {
    int i;
    pidVel[pid_num].index = 0;  // point to first velocity
    pidVel[pid_num].interpolate = 0;
    pidVel[pid_num].leg_stride = 0;  // set initial leg count

    // interpolate values between setpoints, <<4 for resolution
    for(i = 0; i < NUM_VELS; i++) {
        pidVel[pid_num].interval[i] = 128;  // 128 ms intervals
        pidVel[pid_num].delta[i] =  0x1000; // 1/16 rev
        pidVel[pid_num].vel[i] = (pidVel[pid_num].delta[i] << 8) / pidVel[pid_num].interval[i];
    }
    pidObjs[pid_num].p_input = 0; // initialize first set point
    pidObjs[pid_num].v_input = (int)(((long) pidVel[pid_num].vel[0] * K_EMF) >>8);	//initialize first velocity, scaled
}

// calibrate A/D offset, using PWM synchronized A/D reads inside
// timer 1 interrupt loop
// BATTERY CHANGED FOR IP2.5 ***** need to fix
void pidCalibBatteryOffset(int spindown_ms) {
    long temp;  // could be + or -
    unsigned int battery_voltage;
    // save current PWM config
    int tempPDC1 = PDC1;
    int tempPDC2 = PDC2;
    PDC1 = 0; PDC2 = 0;  /* SFR for PWM? */

    // save current PID status, and turn off PID control
    short tempPidObjsOnOff[NUM_PIDS];
    tempPidObjsOnOff[0] = pidObjs[0].onoff;
    tempPidObjsOnOff[1] = pidObjs[1].onoff;
    pidObjs[0].onoff = 0; pidObjs[1].onoff = 0;

    delay_ms(spindown_ms); //motor spin-down
    offsetAccumulatorL = 0;
    offsetAccumulatorR = 0;
    offsetAccumulatorCounter = 0; // updated inside servo loop
    calib_flag = 1;  // enable calibration
    while(offsetAccumulatorCounter < 100); // wait for 100 samples
    calib_flag = 0;  // turn off calibration
    battery_voltage = adcGetVbatt();
    //Left
    temp = offsetAccumulatorL;
    temp = temp/(long)offsetAccumulatorCounter;
    pidObjs[0].inputOffset = (int) temp;

    //Right
    temp = offsetAccumulatorR;
    temp = temp/(long)offsetAccumulatorCounter;
    pidObjs[1].inputOffset = (int) temp;

    // restore PID values
    PDC1 = tempPDC1;
    PDC2 = tempPDC2;
    pidObjs[0].onoff = tempPidObjsOnOff[0];
    pidObjs[1].onoff = tempPidObjsOnOff[1];
}

void pidOn(int pid_num) {
    pidObjs[pid_num].onoff = 1;
    t1_ticks = 0;
}

// zero position setpoint for both motors (avoids big offset errors)
void pidZeroPos(int pid_num) {
    // disable interrupts to reset state variables
    DisableIntT1; // turn off pid interrupts
    amsEncoderResetPos(); //  reinitialize rev count and relative zero encoder position for both motors
    pidObjs[pid_num].p_state = 0;

    // reset position setpoint as well
    pidObjs[pid_num].p_input = 0;
    pidObjs[pid_num].v_input = 0;
    pidVel[pid_num].leg_stride = 0; // strides also reset
    EnableIntT1; // turn on pid interrupts
}

// from cmd.c  PID set gains
void pidSetGains(int pid_num, int Kp, int Ki, int Kd, int Kaw, int ff) {
    pidObjs[pid_num].Kp  = Kp;
    pidObjs[pid_num].Ki  = Ki;
    pidObjs[pid_num].Kd  = Kd;
    pidObjs[pid_num].Kaw = Kaw;
    pidObjs[pid_num].feedforward = ff;
}

// called from set thrust closed loop, etc. Thrust
void pidSetInput(int pid_num, int input_val) {
    // use velocity setpoint + throttle for compatibility between Hall and Pullin code
    // otherwise, miss first velocity set point
    pidObjs[pid_num].v_input = input_val + (int)(( (long)pidVel[pid_num].vel[0] * K_EMF) >> 8);	//initialize first velocity ;
    pidObjs[pid_num].start_time = t1_ticks;

    //zero out running PID values
    pidObjs[pid_num].i_error = 0;
    pidObjs[pid_num].p = 0;
    pidObjs[pid_num].i = 0;
    pidObjs[pid_num].d = 0;

    // set initial time for next move set point
    /*   need to set index =0 initial values */
    /* position setpoints start at 0 (index=0), then interpolate until setpoint 1 (index =1), etc */
    pidVel[pid_num].expire = (long) pidVel[pid_num].interval[0];   // end of first interval
    pidVel[pid_num].interpolate = 0;
    //  this should be set only after first .expire time to avoid initial transients
    pidVel[pid_num].index = 0; // reset setpoint index
}

// set values from packet - leave previous motor_count, p_input, etc.
// called from cmd.c
void pidSetVelProfile(int pid_num, int *interval, int *delta, int *vel) {
    int i;
    for (i = 0; i < NUM_VELS; i++) {
        pidVel[pid_num].interval[i]= interval[i];
        pidVel[pid_num].delta[i]= delta[i];
        pidVel[pid_num].vel[i]= vel[i];
    }
}

void pidSetSync(unsigned char new_sync) {
    sync = new_sync;
    if(sync) {
        SetDCMCPWM(3, 10000, 0);
    } else {
        tiHSetDC(3,0);
    }
}

/*********************** Stop Motor and Interrupts *********************************************/
void pidEmergencyStop(void) {
    pidSetInput(0, 0);
    pidSetInput(1, 0);
    DisableIntT1; // turn off pid interrupts
    SetDCMCPWM(MC_CHANNEL_PWM1, 0, 0);    // set PWM to zero
    SetDCMCPWM(MC_CHANNEL_PWM2, 0, 0);
}

/*********************** Motor Control Interrupt *********************************************/

/* update setpoint  only leg which has run_time + start_time > t1_ticks */
/* turn off when all PIDs have finished */

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    int j;
    LED_3 = 1;
    interrupt_count++;

    if(interrupt_count == 4) {
        mpuBeginUpdate();
        amsEncoderStartAsyncRead();
    } else if(interrupt_count == 5) {
        interrupt_count = 0;

        if (t1_ticks++ == T1_MAX) {
            t1_ticks = 0;
        }
        
        LED_3 = 0;
        for (j = 0; j< NUM_PIDS; j++) {
            LED_3 = 1;
            // only update tracking setpoint if time has not yet expired
            pidUpdateState(j); // always update state, even if motor is coasting

            if (pidObjs[j].onoff) {
                pidUpdateSetpoint(j);
                pidUpdateControl(j);
            } else {
                tiHSetDC(j+1,0);
            }
            LED_3 = 0;
        }

        LED_3 = 1;
        if(pidObjs[0].onoff && !uart_tx_flag) {
            uart_tx_packet = ppoolRequestFullPacket(sizeof(telemStruct_t));
            if(uart_tx_packet != NULL) {
                //time|Left pstate|Right pstate|Commanded Left pstate| Commanded Right pstate|DCR|DCL|RBEMF|LBEMF|Gyrox|Gyroy|Gyroz|Ax|Ay|Az
                //bytes: 4,4,4,4,4,2,2,2,2,2,2,2,2,2,2
                paySetType(uart_tx_packet->payload, CMD_PID_TELEMETRY);
                paySetStatus(uart_tx_packet->payload, 0);
                pidUpdateTelem((telemStruct_t*)payGetData(uart_tx_packet->payload));
                uart_tx_flag = 1;
            }
        }
    }
    LED_3 = 0;
    _T1IF = 0;
}

// update desired velocity and position tracking setpoints for each leg
static inline void pidUpdateSetpoint(int pid_num) {
    int index;
    long temp_v;
    index = pidVel[pid_num].index;
    // update desired position between setpoints, scaled by 256
    pidVel[pid_num].interpolate += pidVel[pid_num].vel[index];

    /**** maybe need to handle round off in position set point ***/
    if (t1_ticks >= pidVel[pid_num].expire) { // time to reach previous setpoint has passed
        pidVel[pid_num].interpolate = 0;
        pidVel[pid_num].index++;
        if (pidVel[pid_num].index >= NUM_VELS) {
            pidVel[pid_num].index = 0;
            pidVel[pid_num].leg_stride++;  // one full leg revolution
        }  // loop on index

        index = pidVel[pid_num].index;
        pidObjs[pid_num].p_input += pidVel[pid_num].delta[index];	//update to next set point
        pidVel[pid_num].expire += pidVel[pid_num].interval[index];  // expire time for next interval
        temp_v = ((long)pidVel[pid_num].vel[index] * K_EMF)>>8;  // scale velocity to A/D units
        pidObjs[pid_num].v_input = (int)(temp_v);	  //update to next velocity
    }
}

/* update state variables including motor position and velocity */
static inline void pidUpdateState(int pid_num) {
    long p_state;
    calib_flag = 0;  //BEMF disable
    // get diff amp offset with motor off at startup time
    if(calib_flag) {
        offsetAccumulatorL += adcGetMotorA();
        offsetAccumulatorR += adcGetMotorB();
        offsetAccumulatorCounter++;
    }
 
    long oldpos, velocity;
    oldpos = pidObjs[pid_num].p_state;

    // only works to +-32K revs- might reset after certain number of steps? Should wrap around properly
    p_state = (long)(encPos[pid_num].pos << 2);		// pos 14 bits 0x0 -> 0x3fff
    p_state = p_state + (encPos[pid_num].oticks << 16);
    p_state = p_state - (long)(encPos[pid_num].offset <<2); 	// subtract offset to get zero position
    if (pid_num==0) {
            pidObjs[pid_num].p_state = -p_state; //fix fo encoder alignment
    } else {
            pidObjs[pid_num].p_state = p_state;
    }

    // use first difference on position for velocity estimate
    velocity = pidObjs[pid_num].p_state - oldpos;  // 2**16 * revs per ms
    velocity = velocity >> 6; // divide by 2**16, mult by 2**10 to get approx revs/sec
    if (velocity > 0x7fff) {
        velocity = 0x7fff; // saturate to int
    }
    if (velocity < -0x7fff) {
        velocity = -0x7fff;
    }
    pidObjs[pid_num].v_state = (int) velocity;
}

// Calulate and apply control based on error state
static inline void pidUpdateControl(int pid_num) {
    // p_input has scaled velocity interpolation to make smoother
    // p_state is [16].[16]
    pidObjs[pid_num].p_error = pidObjs[pid_num].p_input + pidVel[pid_num].interpolate  - pidObjs[pid_num].p_state;
    pidObjs[pid_num].v_error = pidObjs[pid_num].v_input - pidObjs[pid_num].v_state;  // v_input should be revs/sec

    //Update values
    pidObjs[pid_num].p = ((long)pidObjs[pid_num].Kp * pidObjs[pid_num].p_error) >> 12 ;  // scale so doesn't over flow
    pidObjs[pid_num].i = ((long)pidObjs[pid_num].Ki * pidObjs[pid_num].i_error) >> 12;
    pidObjs[pid_num].d = (long)pidObjs[pid_num].Kd * (long) pidObjs[pid_num].v_error;

    // better check scale factors
    pidObjs[pid_num].preSat = pidObjs[pid_num].feedforward + pidObjs[pid_num].p +
    ((pidObjs[pid_num].i ) >> 4) +  // divide by 16
    (pidObjs[pid_num].d >> 4); // divide by 16
    pidObjs[pid_num].output = pidObjs[pid_num].preSat;

    /* i_error say up to 1 rev error 0x10000, X 256 ms would be 0x1 00 00 00
    scale p_error by 16, so get 12 bit angle value*/
    pidObjs[pid_num].i_error = (long)pidObjs[pid_num]. i_error + ((long)pidObjs[pid_num].p_error >> 4); // integrate error

    // saturate output - assume only worry about >0 for now
    // apply anti-windup to integrator
    if (pidObjs[pid_num].preSat > MAXTHROT) {
        pidObjs[pid_num].output = MAXTHROT;
        pidObjs[pid_num].i_error = (long) pidObjs[pid_num].i_error +
        (long)(pidObjs[pid_num].Kaw) * ((long)(MAXTHROT) - (long)(pidObjs[pid_num].preSat))
        / ((long)GAIN_SCALER);
    }

    if (pidObjs[pid_num].preSat < -MAXTHROT) {
        pidObjs[pid_num].output = -MAXTHROT;
        pidObjs[pid_num].i_error = (long) pidObjs[pid_num].i_error +
        (long)(pidObjs[pid_num].Kaw) * ((long)(MAXTHROT) - (long)(pidObjs[pid_num].preSat))
        / ((long)GAIN_SCALER);
    }

    tiHSetDC(pid_num+1, pidObjs[pid_num].output);
}

// store current PID info into structure. Used by telemSaveSample and CmdGetPIDTelemetry
static inline void pidUpdateTelem(telemStruct_t *telemBuffer) {
    telemBuffer->timeStamp = (long)sclockGetTime();
    telemBuffer->sync = sync;

    telemBuffer->posL = pidObjs[0].p_state;
    telemBuffer->posR = pidObjs[1].p_state;
    telemBuffer->composL = pidObjs[0].p_input;
    telemBuffer->composR = pidObjs[1].p_input;
    telemBuffer->dcL = pidObjs[0].output;	// left
    telemBuffer->dcR = pidObjs[1].output;	// right
    
    mpuGetGyro(telemBuffer->gyro);
    mpuGetXl(telemBuffer->accel);
    
    telemBuffer->bemfL = (int) adcGetMotorA();
    telemBuffer->bemfR = (int) adcGetMotorB();
    telemBuffer->Vbatt = (int) adcGetVbatt();
}
