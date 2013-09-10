/* 
 * File:   test.h
 * Author: tfurtado
 *
 * Created on June 27, 2013, 12:41 PM
 */

#ifndef TEST_H
#define	TEST_H

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum {
    minimum, maximum, middle, calibrated
} CalibrateStates_t;
typedef enum {
    calibrating, inAuto, inManual
} AutohelmStates_t;

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

