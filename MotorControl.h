/*
 * File:   MotorControl.h
 * Author: Taylor Furtado
 * Design: Autohelm Modification Project
 *
 * Created on June 28, 2013

Copyright (c) [2013] [Taylor E. Furtado]

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
#define CLUTCH_UNKNOWN            -1

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

void SetCurrentPWM(signed int PWM);

void ClutchControl(uint8_t control);

/**
 * IsClutchEngaged()
 * Function: Determines if the physical clutch is engaged or disengaged.
 * @param: None
 * @return: CLUTCH_ENGAGED, if the physical clutch is engaged
 *          CLUTCH_DISENGAGED, if the physical clutch is NOT engaged
 *          CLUTCH_UNKNOWN, the status of the clutch cannot be determined
 * @remark: Returns status of the physical clutch. Based on the movement of the
 * Autohelm potentiometer when the motor is running. If the motor is not running
 * when IsClutchEngaged, the function will return CLUTCH_UNKNOWN.
 * @author Taylor Furtado
 * @date July 17,2013
*/
uint8_t IsClutchEngaged(void);

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

