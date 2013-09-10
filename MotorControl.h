/*
 * File:   MotorControl.h
 * Author: Taylor Furtado
 * Design: Autohelm Modification Project
 *
 * Created on June 28, 2013

GNU GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. {http://fsf.org/}
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.
 */

#ifndef MOTORCONTROL_H
#define	MOTORCONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/*******************************************************************************
 * PUBLIC #DEFINES                                                            *
 ******************************************************************************/

#define FULL_POSITIVE            1023        //11 bits
#define FULL_NEGATIVE           -1023
#define STOP                      0

#define CLOCKWISE                 1
#define COUNTERCLOCKWISE          0
    
#define CLUTCH_ENGAGED            1
#define CLUTCH_DISENGAGED         0

#define CHANGE_RATE       10

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/
/**
 * MotorControlInit()
 * Function: Initializes the PWM module, Timer1 Module and sets te pins
 * to be used in MotorControl.c
 * @param: None
 * @return: None
 * @remark: Initializes PWM-H1 (Port B14) in Free-Running mode.
 *          Initializes Timer1 to generate an interrurpt every 1 millisecond.
 *          Initializes Direction pin (B0) and Clutch pin(B2) as outputs
 * @author Taylor Furtado
 * @date June 26,2013
*/
void MotorControlInit(void);

/**
 * SetTargetPWM()
 * @Function: Sets a desired PWM duty cycle for the motor to reach. The actual
 * PWM will lag behind the desired duty cycle by a time determined by the
 * CHANGE_RATE variable [ lag time = (target_duty - actual_duty)/CHANGE_RATE ]
 * The units of CHANGE_RATE are (change in duty) per millisecond.
 * @param: target: The target duty cycle to be reached by the motor. Defined by
 * FULL_CLOCKWISE (+2048), FULL_COUNTERCLOCKWISE (-2048) and STOP (0).
 * @return: NONE
 * @remark
 * @author Taylor Furtado
 * @date June 26, 2013
*/
void SetTargetPWM(signed int target);

/**
 * ClutchControl()
 * Function: Energizes or de-energizes the soleniod controlling the clutch using
 * a constant signal as an enable on the H-Bridge. The direction of this H-bridge
 * chanel can be tied to either ground or 3.3v.
 * @param control: Sets the soleniod as either energized (CLUTCH_ENGAGED) or
 * de-energized (CLUTCH_DISENGAGED).
 * @return NONE
 * @remark: Any other input (other than CLUTCH_ENGAGED or CLUTCH_DISENGAGED)
 * will not effect the status of the solenoid.
 * @author Taylor Furtado
 * @date July 1, 2013
*/
void ClutchControl(uint8_t control);

/**
 * GetCurrentPWM()
 * Function: Returns the current PWM .
 * @param: None
 * @return: Signed int containing the CurrentPWM (-2048 - +2048)
 * @remark: Returns the CurrentPWM
 * @author Taylor Furtado
 * @date July 17,2013
*/
signed int GetCurrentPWM(void);

/**
 * GetTargetPWM()
 * Function: Returns the target PWM .
 * @param: None
 * @return: Signed int containing the last TargetPWM (-2048 - +2048)
 * @remark: Returns the TargetPWM
 * @author Taylor Furtado
 * @date July 17,2013
*/
signed int GetTargetPWM(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTORCONTROL_H */

