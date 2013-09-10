/* 
 * File:   test.h
 * Author: tfurtado
 *
 * Created on June 27, 2013, 12:41 PM
 *
 * GNU GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. {http://fsf.org/}
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.
 */

#ifndef TEST_H
#define	TEST_H

#ifdef	__cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Public #DEFINES                                                            *
 ******************************************************************************/

#define MAX_ANGLE       120.0
#define SYSTEM_CLOCK 80000000L
#define PB_CLOCK (SYSTEM_CLOCK >> 1)

#define ERROR -1
#define SUCCESS 1

/*******************************************************************************
 * Public Variables                                                            *
 ******************************************************************************/


/*******************************************************************************
 * Public Functions                                                            *
 ******************************************************************************/

/**
 * AutohelmInit()
 * Function: Initializes Autohelm actuator for use on PIC33E
 * @param: None
 * @return: None
 * @remark: -Initializes clock(39.61MIPS), Motor Controller, ADC, UART(baud: 115200),
 *          QEI, and Timers
 *          -Sets TRIS of switches, LEDs and sets switches as digital ports
 *          -Resets Revolution Counter (QEI) and sets AutohelmState to Calibrating
 * @author Taylor Furtado
 * @date: September 9, 2013
*/
void AutohelmInit(void);
    

#ifdef	__cplusplus
}
#endif

#endif	/* TEST_H */

