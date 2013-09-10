/*
 * File:   MotorControl.c
 * Author: Taylor Furtado
 * Design: Autohelm Modification Project
 *
 * Created on June 28, 2013
 *
 * GNU GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. {http://fsf.org/}
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.
 */

#include "MotorControl.h"
#include "ADC.h"
#include <hspwm.h>


/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define millisecond     80

#define directionPIN_TRIS TRISBbits.TRISB0  //Direction Pin (B0)
#define directionPIN_LAT  LATBbits.LATB0    //Direction Pin (B0)
#define CLUTCH_TRIS       TRISBbits.TRISB2  //Clutch Pin    (B2)
#define CLUTCH_LAT        LATBbits.LATB2    //Clutch Pin    (B2)

/*******************************************************************************
 * PRIVATE Variables                                                            *
 ******************************************************************************/

static signed int targetPWM, currentPWM;
static uint8_t direction, clutchControl;

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

void Timer1Init(uint16_t prescalar);

/**
 * UpdatePWM()
 * Function: Increments or decrements the current duty cycle to approach the target
 *           duty cycle set with SetTargetPWM().
 * @param None
 * @return None
 * @remark Gradually changes the duty cycle written to the pin to approach the
 *         target PWM. This prevents the H-Bridge from faulting by preventing
 *         the direction or speed from changing instantaneously. This function
 *         runs using the 1ms timer-driven interrupt initialized in MotorControlInit().
 *         CHANGE_RATE defines the change in duty per interrupt
 * @author Taylor Furtado
 * @date June 28, 2013
*/
void UpdatePWM(void);

/*******************************************************************************
 * PUBLIC FUNCTION DEFFINITIONS                                                          *
 ******************************************************************************/

void MotorControlInit(void)
{
    /*Timer Initialization*/
    Timer1Init(1 * millisecond);

    /*Pin Setup*/
    CLUTCH_TRIS = 0;
    directionPIN_TRIS = 0;

    /*High Speed PWM Initialization
     Independent PWM Mode Operation
     Pin RB14 -> PWM1H*/

    PTPER = 1000;                           //Set PWM Period
    PHASE1 = 0;                             //No Phase Shift
    IOCON1bits.PMOD = 3;                    //True Independent Mode
    IOCON1bits.PENH = 1;                    //PWM1H pin controlled by PWM Generator
    IOCON1bits.PENL = 0;                    //PWM1L pin controlled by GPIO
    IOCON2bits.PENH = IOCON2bits.PENL = 0;  //PWM2 pins controlled by GPIO
    IOCON3bits.PENH = IOCON3bits.PENL = 0;  //PWM3 pins controlled by GPIO
    PWMCON1 = 0x0000;                       //NO Time base, Edge-Alignment, or Independent Duty Cycles
    FCLCON1 = 0x0003;                       //Fault input disabled
    PTCON2 = 0x0000;                        //1:1 Prescaler
    PTCON = 0x8000;                         //Enable PWM Module

    SetHSPWMDutyCycle1(0);                  //Init Duty Cycle to 0
    SetHSPWMDeadTime1(0,0);                 //No dead time

}

void SetTargetPWM(signed int target)
{
    targetPWM = target;
    //Limit Duty MAX = +1023, MIN= -1023
    if (targetPWM > FULL_POSITIVE)
        targetPWM = FULL_POSITIVE;
    if (targetPWM < FULL_NEGATIVE)
        targetPWM = FULL_NEGATIVE;

        
}

void ClutchControl (uint8_t control)
{
    clutchControl = control;
    if (control == CLUTCH_ENGAGED) {
        CLUTCH_LAT = CLUTCH_ENGAGED;
    } else if (control == CLUTCH_DISENGAGED) {
        CLUTCH_LAT = CLUTCH_DISENGAGED;
    } else {
        ;
    }
}

signed int GetCurrentPWM(void)
{
    return currentPWM;
}

signed int GetTargetPWM(void)
{
    return targetPWM;
}

/*******************************************************************************
 * PRIVATE FUNCTION DEFFINITIONS                                                          *
 ******************************************************************************/

void Timer1Init(uint16_t prescalar)
{
    //Timer 1 Setup
        T1CON = 0;              //Reset Timer 1
        IFS0bits.T1IF = 0;      // Reset Timer1 interrupt flag
	IPC0bits.T1IP1 = 1;     // Timer1 Interrupt priority level=1
 	IEC0bits.T1IE = 1;      // Enable Timer1 interrupt
	PR1 = prescalar;        // Timer1 period register = 32768 (1sec)
        T1CON = 0x8030;
}

void UpdatePWM (void)
{
    //temporary variables, prevent actual variables from changing
    signed int tempTarget = targetPWM;
    signed int tempCurrent;

    //Toggle Direction
    if (currentPWM < 0){
        tempCurrent = currentPWM * (-1);
        direction = CLOCKWISE;
    } else if (currentPWM >= 0) {
        direction = COUNTERCLOCKWISE;
        tempCurrent = currentPWM;
    }
    //write direction to B0
    directionPIN_LAT = direction;

    //Gradually increase/decrease speed
    if (currentPWM < tempTarget){
        currentPWM += CHANGE_RATE;
        SetHSPWMDutyCycle1(tempCurrent);
    } else if(currentPWM > tempTarget) {
        currentPWM -= CHANGE_RATE;
        SetHSPWMDutyCycle1(tempCurrent);
    } 
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt( void )
{

    UpdatePWM();
    IFS0bits.T1IF = 0;          //Reset interrupt flag
}
