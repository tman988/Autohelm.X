
/*
 * File:   ADC.c
 * Author: Taylor Furtado
 * Design: Autohelm Modification Project
 *
 * Created on July 15, 2013
 */

#include "ADC.h"
#include <adc.h>

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define ADC_BUFFER ADC1BUF0


/*******************************************************************************
 * PRIVATE VARIABLES                                                            *
 ******************************************************************************/

static uint16_t adcBuffer;
static uint16_t low = 512;
static uint16_t high = 513;

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

uint16_t MapADC (void);

/*******************************************************************************
 * PUBLIC FUNCTION DEFINES                                                     *
 ******************************************************************************/

void ADCInit(void)
{
    AD1CON1 = 0;                //Clear CON1
    AD1CON1bits.SSRC = 0b111;   //Internal counter (auto-convert)
    AD1CON1bits.ASAM = 1;       //Sampling begins immediately after last conversion
    AD1CON2 = 0;                //Clear CON2
    AD1CON3 = 0;                //Clear CON3
    AD1CON3bits.SAMC = 0x0F;    //Auto-sampling time: TAD = 15
    AD1CON4 = 0;                //Clear CON4
    ANSELA = ANSELB = 0x0000;   //Clear ANSEL registers, set to digital I/O
    ANSELBbits.ANSB1 = 1;       //Set AN3/RB1 to analog
    AD1CHS0 = 0;                //Clear Channel 0 select
    AD1CHS0bits.CH0SA = 0x03;   //Set input to AN3 (RB1)
    AD1CSSLbits.CSS3 = 1;       //Scan AN3 (RB1)

    ConfigIntADC1(ADC_INT_ENABLE & ADC_INT_PRI_3);  //Configure ADC1 interrupt (priority 3)

    AD1CON1bits.ADON = 1;       //Enable ADC1
}

uint16_t GetMappedADC(void)
{
    return MapADC();
}

uint16_t GetRawADC(void)
{
    return adcBuffer;
}

/*******************************************************************************
 * PRIVATE FUNCTION DEFFINITIONS                                                          *
 ******************************************************************************/

uint16_t MapADC(void)
{
    long unsigned int ADC;
    
    //map Pot range (~335 - ~715) to ADC range (0 - 1023)
    //ADC = ((adcBuffer - POT_LOW_BOUND) * 1023.0) / (POT_HIGH_BOUND - POT_LOW_BOUND);

    if (adcBuffer > high)
        high = adcBuffer;
    if (adcBuffer < low)
        low = adcBuffer;

    if (low < POT_LOW_BOUND)
        low = POT_LOW_BOUND + 10;
    if (high > POT_HIGH_BOUND)
        high = POT_HIGH_BOUND - 10;

    ADC = ((adcBuffer - low) * (1023.0)) / (high - low);

    //Set Minimum (0) and Maximum (1023)
    if (ADC >= 1023)
        ADC = 1023;
    if (ADC < 0)
        ADC = 0;

    return (uint16_t)ADC;
}

void __attribute__((interrupt, no_auto_psv))_AD1Interrupt(void)
{
    adcBuffer = ADC1BUF0;;
   // Sampled value in adcBuffer

   IFS0bits.AD1IF = 0;
}
