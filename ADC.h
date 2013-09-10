/* 
 * File:   ADC.h
 * Author: Taylor Furtado
 * Design: Autohelm Modification Project
 *
 * Created on July 15, 2013
 */

#ifndef ADC_H
#define	ADC_H

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

// Calculated limits of the Autohelm Potentiometer
#define POT_LOW_BOUND   320 //Spring tensioned extreme
#define POT_HIGH_BOUND  695 //Spring relaxed extreme

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/
/**
 * ADCInit()
 * Function: Initializes the ADC Module on the dsPIC33.
 * @param: None
 * @return: None
 * @remark: Sets pin B1 as analog input. Sets input as a 10-bit unsigned integer
 * @author Taylor Furtado
 * @date July 15,2013
*/
void ADCInit(void);

/**
 * GetMappedADC()
 * Function: Returns a mapped output of the ADC module. Based on the defined high
 * and low limits of the Autohelm potentiometer.
 * @param: None
 * @return: Mapped 10-bit output of the ADC.
 * @remark: returns an unsigned integer between 0 and 1023.
 * @author Taylor Furtado
 * @date July 15,2013
*/
uint16_t GetMappedADC(void);

/**
 * GetRawADC()
 * Function: Returns the raw output of the ADC module.
 * @param: None
 * @return: Raw 10-bit output of the ADC
 * @remark: Returns the 10-bit value of the ADC Buffer in an unsigned 16-bit integer.
 * @author Taylor Furtado
 * @date July 15,2013
*/
uint16_t GetRawADC(void);

#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */

