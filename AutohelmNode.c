/*
 * File:   Autohelm.c
 * Author: Taylor Furtado
 *
 * GNU GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. {http://fsf.org/}
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.
 *
 */

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pps.h>
#include "AutohelmNode.h"
#include "MotorControl.h"
#include "ADC.h"
#include "Hall Encoder.h"
#include <adc.h>
#include "Uart1.h"
#include "timers.h"
#include <qei32.h>
#include "Ecan1.h"
#include "Node.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define AMBER_TRIS TRISAbits.TRISA4
#define AMBER_LAT  LATAbits.LATA4
#define SWITCH_2_TRIS   TRISBbits.TRISB12
#define SWITCH_2        PORTBbits.RB12
#define SWITCH_1_TRIS   TRISAbits.TRISA0
#define SWITCH_1        PORTAbits.RA0

#define HIGH_BOUND         32767.0 //4294967295.0
#define LOW_BOUND          -32768.0 //-2147483648.0
#define FULL_BOUND         65535.0

#define Kp              150.0
#define Kd              1.0
#define Ki              0.5
#define tCurrent        0
#define tMinus1         1
#define tMinus2         2

#define UPDATE_TIMER    0
#define UPDATE_TIME     500
#define PID_TIMER       1
#define PID_TIME        10

/*
 * Pic shadow register pragmas.  These set main oscillator sources, and
 * other low-level hardware stuff like PGD/PGC (debug/programming) pin positions.
 */
#ifdef _FSS       /* for chip with memory protection options */
_FSS(RSS_NO_RAM & SSS_NO_FLASH & SWRP_WRPROTECT_OFF)
#endif
_FOSCSEL(FNOSC_FRC);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
_FWDT(FWDTEN_OFF);
_FICD(JTAGEN_OFF & ICS_PGD3);

/*******************************************************************************
 * PRIVATE Structs/Unions/Enums                                                            *
 ******************************************************************************/

union errors{
    unsigned main            :1;
    unsigned overBound       :1;
    unsigned underBound      :1;
};
union errors errorType;

//States for Calibrate State Machine
typedef enum {
    minimum, maximum, middle, calibrated
} CalibrateStates_t;
//States for High Level State Machine
typedef enum {
    calibrating, inAuto, inManual
} AutohelmStates_t;

static CalibrateStates_t CalibrateState = minimum;
static AutohelmStates_t AutohelmState;

/*******************************************************************************
 * PRIVATE Variables                                                            *
 ******************************************************************************/

static signed int minRev, midRev, maxRev;
/*PID Variables*/
static int32_t targetTick, currentTick;
static int32_t PID, error, proportion, derivative, integral;
static int32_t inPID[3] = {0,0,0};
CanMessage message;

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

/**
 * ClockInit()
 * Function: Initializes Clock
 * @param: None
 * @return: None
 * @remark: Initializes primairy oscillator for PIC33E to 39.61MIPS.
 *          -Sets PLLFBD = 43 PLLPOST = 0(N1=2) and PLLPRE = 0(N2=2)
 * @author Taylor Furtado
 * @date: September 9, 2013
*/
void ClockInit(void);

/**
 * ()
 * Function:
 * @param:
 * @return:
 * @remark:
 * @author Taylor Furtado
 * @date:
*/
//void Timer2Init(uint16_t prescalar);

/**
 * Calibrate()
 * Function: Calibrates the Autohelm Actuator using a FSM
 * @param: None
 * @return: None
 * @remark: Uses a FSM to find the minimum, maximum and center values of the rudder.
 *          Must be called continuously until FSM reaches 'Calibrated' state.
 *          Caliculates min-, max- and midpoints in REVs and stores them as static variables
 * @author Taylor Furtado
 * @date: September 9, 2013
*/
uint8_t Calibrate(void);

/**
 * PrintUpdate()
 * Function: Uses timer 0 to print to serial.
 * @param: None
 * @return: None
 * @remark: Used for Debugging. Prints multi-line messages to serial every half second.
 *          Update time is #defined as UPDATE_TIME
 * @author Taylor Furtado
 * @date: September 9, 2013
*/
void PrintUpdate(void);

/**
 * PID_Control()
 * Function: Uses PID to bring actuator to a target point
 * @param: int32_t target: set point, set by user or controller
 *         int32_t current: current position of the actuator, defined by system (QEI)
 *         ***Both target and current must be in same units (REV, Tick, DEG, etc.)***
 * @return: PWM value, mapped between -1023 and +1023
 * @remark: Uses PID Feedback control to bring current value to target value by setting
 *          an approprite value PWM.
 * @author Taylor Furtado
 * @date: September 9, 2013
*/
int32_t PID_Control(int32_t target, int32_t current);

/*******************************************************************************
 * Autohelm Main()                                                          *
 ******************************************************************************/

int main(void)
{

    AutohelmInit();
    NodeTransmitStatus();
    InitTimer(UPDATE_TIMER,UPDATE_TIME);
    InitTimer(PID_TIMER, PID_TIME);


    while (!errorType.main) {
        if(IsTimerExpired(UPDATE_TIMER)) {
            PrintUpdate();
            InitTimer(UPDATE_TIMER, UPDATE_TIME);
        }
        switch (AutohelmState) {
            case calibrating:
                ClutchControl(CLUTCH_ENGAGED);
                if(Calibrate() == calibrated){
                    AutohelmState = inAuto;
                }
                break; //end calibrating
            case inAuto:
                ClutchControl(CLUTCH_ENGAGED);
                /*Map current value to the same range as the input (16-bits)*/
                currentTick = ((GetRevCount()- minRev) * (HIGH_BOUND - LOW_BOUND)) / (maxRev - minRev) + (LOW_BOUND);

                /*Limit target values to their appropriate bounds (signed 16-bit int)*/
                targetTick = ((GetMappedADC() * (FULL_BOUND)) / 1023) + (LOW_BOUND);
                /*Verify valid target values*/
                if (targetTick > HIGH_BOUND) {
                    errorType.overBound = 1;
                } else if (targetTick < LOW_BOUND) {
                    errorType.underBound = 1;
                } else {
                    SetTargetPWM((signed int)PID_Control(targetTick, currentTick));
                    errorType.overBound = 0;
                    errorType.underBound = 0;
                }

                break; //end inAuto
            case inManual:
                ClutchControl(CLUTCH_DISENGAGED);
                SetTargetPWM(STOP);
                break; //end inManual
            default:
                printf ("[AutohelmState]in default\n");
                errorType.main = 1;
                break;// end default
        }
    }

    printf("[MAIN]error: exit loop\n");
    while(errorType.main){
        SetTargetPWM(STOP);
    }
    return 0;
}

/*******************************************************************************
 * PUBLIC FUNCTION DEFFINITIONS                                                          *
 ******************************************************************************/

void AutohelmInit()
{
    AMBER_TRIS = 0; //Amber LED PortA Pin 4
    SWITCH_1_TRIS = 1;
    SWITCH_2_TRIS = 1;
    ANSELAbits.ANSA0 = 0;
    
    ClockInit();
    MotorControlInit();
//    Timer2Init(200 * MILLISECOND); // (n * 80), Use for printing
    ADCInit();
//    Uart1Init(21);
    QEI_Init();
    TIMERS_Init();
    PPSUnLock;
        RPINR26bits.C1RXR = 36;
        RPOR2bits.RP39R = 0b001110;
    PPSLock;
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB4 = 1;
    Ecan1Init();

    ResetRevCount();
    AutohelmState = calibrating;
    
}

/*******************************************************************************
 * PRIVATE FUNCTION DEFFINITIONS                                                          *
 ******************************************************************************/

void ClockInit() {
    //Clock init  M=43, N1,2 = 2 == 39.61MIPS
    PLLFBD = 43;
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 0; // N2 = 2
    OSCTUN = 0;
    RCONbits.SWDTEN = 0;

    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to Primary (3?)

    __builtin_write_OSCCONL(0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur

    while (OSCCONbits.LOCK != 1) {
    };
    //End of clock init.
}

uint8_t Calibrate() {

    switch (CalibrateState) {
        case minimum:
            SetTargetPWM(FULL_NEGATIVE);
            if (GetRevCount() > 0){
                printf("[Calibrate]error: Encoder direction inverted\n");
                errorType.main = 1;
            }
            if (!SWITCH_1) {
//                ResetRevCount();
                CalibrateState = maximum;
                minRev = GetRevCount();
            }
            break; //end minimum
        case maximum:
            SetTargetPWM(FULL_POSITIVE);
            if (!SWITCH_2) {
                CalibrateState = middle;
                maxRev = GetRevCount();
                midRev = (maxRev + minRev) >> 1;
            }
            break; //end maximum
        case middle:
            if (GetRevCount() > midRev)
                SetTargetPWM(FULL_NEGATIVE);
            else if (GetRevCount() < midRev)
                SetTargetPWM(FULL_POSITIVE);
            else {
                SetTargetPWM(STOP);
                CalibrateState = calibrated;
            }
            break; //end middle
        case calibrated:
            SetTargetPWM(STOP);
            break; //end calibrated
        default:
            printf("[CalibrateState] in default\n");
    }//end Calibrate switch
    return CalibrateState;
}

int32_t PID_Control(int32_t target, int32_t current)
{
    inPID[tCurrent] = current;
    /*Calculate PID*/
    error = target - inPID[tCurrent];
    proportion = (inPID[tCurrent] - inPID[tMinus1]);
    integral = (PID_TIME * error);
    derivative = ((inPID[tCurrent] - (2*inPID[tMinus1]) + inPID[tMinus2]) / (float)(PID_TIME));
    
    PID = PID - Kp*proportion + Ki*integral - Kd*derivative;

    if(PID > FULL_POSITIVE)
        PID = FULL_POSITIVE;
    if(PID < FULL_NEGATIVE)
        PID = FULL_NEGATIVE;
    
    inPID[tMinus2] = inPID[tMinus1];
    inPID[tMinus1] = inPID[tCurrent];
    
    return PID;
}

void PrintUpdate() {
//    printf("PRE\n");
//    NodeTransmitStatus();
//    printf("POST\n");
//    printf("ERR:%d\n", errorType);
//    printf("AUT: %d\n", AutohelmState);
//    printf("CAL: %d\n", CalibrateState);
//    printf("ADC: %d\n", GetMappedADC());
//    printf("TAR: %ld\n", targetTick);
//    printf("CUR: %ld\n", currentTick);
//    printf("PID: %ld\n", PID_Control(targetTick, (int32_t)GetRevCount()));
//    printf("CUR: %ld\n", currentIn);
//    printf("ERR: %ld\n", error);
//    printf("PRO: %ld\n", proportion);
//    printf("DER: %ld\n", derivative);
//    printf("INT: %ld\n", integral);
//    printf("PWM: %d\n", GetCurrentPWM());
//    printf("REV: %ld\n", (int32_t)GetRevCount());
//    printf("TPD: %3.2f\n", tickPerDegree);
//    printf("DPR: %3.2f\n", degreePerRev);
//    printf("TPR: %3.2f\n", tickPerRev);
//    printf("TAR: %ld\n", targetTick);
//    printf("MAX: %d\n", maxRev);
//    printf("MIN: %d\n", minRev);
//    printf("MID: %d\n", midRev);
//    printf("MIT: %ld\n", minTick);
//    printf("MAT: %ld\n", maxTick);
//    printf("SW1: %d\n", SWITCH_1);
//    printf("SW2: %d\n", SWITCH_2);
    printf("\n\n");
}
