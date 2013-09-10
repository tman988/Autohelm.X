/* 
 * File:   Hall Encoder.h
 * Author: Taylor Furtado
 * Design: Autohelm Modification Project
 *
 * Created on July 15, 2013
 *

 */



#ifndef HALL_ENCODER_H
#define	HALL_ENCODER_H

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
#define POSCNT_H_REG        POS1CNTH    //Position Counter High Word Register
#define POSCNT_L_REG        POS1CNTL    //Position Counter Low Word Register
#define VELCNT_REG          VEL1CNT     //Velocity Counter Register
#define POSCNT_GTE_COMP     QEI1STATbits.PCHEQIRQ //Position Counter Greater Than or Equal Compare Status bit
#define POSCNT_LTE_COMP     QEI1STATbits.PCLEQIRQ //Position Counter Less Than or Equal Compare Status bit


#define MAX_COUNT               5
#define POSITIVE_DIRECTION      1
#define NEGATIVE_DIRECTION      0

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/
/**
 * QEI_Init()
 * Function: Initializes the QEI Module on the dsPIC33 and maps the appropriate
 * pins to run QEI.
 * @param: None
 * @return: None
 * @remark: Pins:
 *                  B8  -> QEI1A
 *                  B9  -> QEI1B
 *                  B10 -> UPDN (direction)
 * @author Taylor Furtado
 * @date July 19,2013
*/
void QEI_Init(void);

/**
 * GetRevCount()
 * Function: Calculates and returns the number of positive turns of the hall
 *  sensor minus the number of negaive turns since the last reset or init.
 * @param: None
 * @return: net number of turns since last reset or init (positive - negative)
 * @remark: Returns rev count of the hall sensor, not the main shaft of the
 *  Autohelm.
 * @author Taylor Furtado
 * @date July 22,2013
 */
signed int GetRevCount(void);

/**
 * ResetRevCount()
 * Function: Resets the signed RevCount variable to zero.
 * @param: None
 * @return: None
 * @remark:
 * @author Taylor Furtado
 * @date July 22,2013
 */
void ResetRevCount(void);

/**
 * SetRevCount()
 * Function: Sets the signed RevCount variable to a desired count.
 * @param: Count: The signed value of revCount
 * @return: None
 * @remark:
 * @author Taylor Furtado
 * @date July 22,2013
 */
void SetRevCount(signed int count);

/**
 * GetDirection()
 * Function: Returns the current direction of the hall sensor
 * @param: None
 * @return: Positive or Negative based on the direction of rotation
 * @remark: Returns the direction of the hall sensor not the direction of the
 *  main shaft of the Autohelm
 * @author Taylor Furtado
 * @date July 22,2013
 */
uint8_t GetDirection(void);

#ifdef	__cplusplus
}
#endif

#endif	/* HALL_ENCODER_H */

