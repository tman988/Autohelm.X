
/*
 * File:   Hall Encoder.c
 * Author: Taylor Furtado
 * Design: Autohelm Modification Project
 *
 * Created on July 19, 2013
 */

#include "Hall Encoder.h"
#include <pps.h>
#include <qei32.h>

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/


#define POSCNT_OVERFLOW     QEI1STATbits.POSOVIRQ //Position Counter Overflow Status bit
#define POSCNT_INIT         QEI1STATbits.PCIIRQ   //Position Counter (Homing) Initialization Process Complete Status bit
#define VEL_OVERFLOW        QEI1STATbits.VELOVIRQ //Velocity Counter Overflow Status bit
#define HOME_EVENT          QEI1STATbits.HOMIRQ   //Status Flag for Home Event Status bit
#define INDEX_EVENT         QEI1STATbits.IDXIRQ   //Status Flag for Index Event Status bit


#define POSCNT_HOLD_REG     POS1HLD     //Position Counter Hold Register
#define INDEX_H_REG         INDX1CNTH   //Index Counter High Word Register
#define INDEX_L_REG         INDX1CNTL   //Index Counter Low Word Register
#define INDEX_HOLD          INDX1HLD    //Index Counter Hold Register
#define INIT_H_REG          QEI1ICH     //Initialization/Capture High Word Register
#define INIT_L_REG          QEI1ICL     //Initialization/Capture Low Word Register
#define LTE_COMP_H_REG      QEI1LECH    //Less Than or Equal Compare High Word Register
#define LTE_COMP_L_REG      QEI1LECL    //Less Than or Equal Compare Low Word Register
#define GTE_COMP_H_REG      QEI1GECH    //Greater Than or Equal Compare High Word Register
#define GTE_COMP_L_REG      QEI1GECL    //Greater Than or Equal Compare Low Word Register
#define INT_TIME_H_REG      INT1TMRH    //Interval Timer High Word Register
#define INT_TIME_L_REG      INT1TMRL    //Interval Timer Low Word Register
#define INT_TIME_H_HOLD     INT1HLDH    //Interval Timer Hold High Word Register
#define INT_TIME_L_HOLD     INT1HLDL    //Interval Timer Hold Low Word Register

/*******************************************************************************
 * PRIVATE VARIABLES                                                            *
 ******************************************************************************/

static signed int revCount;

/*******************************************************************************
 * PRIVATE FUNCTION Prototypes
 ******************************************************************************/



/*******************************************************************************
 * PUBLIC FUNCTION DEFINES                                                     *
 ******************************************************************************/

void QEI_Init(void)
{
    //Pin Mapping
    PPSUnLock;
    RPINR14bits.QEA1R = 40;       //QEI A -> RB8
    RPINR14bits.QEB1R = 41;       //QEI B -> RB9
    PPSLock;

    /*QEI1 CON register*/
    QEI1CONbits.QEIEN   = 1;         //Disable
    QEI1CONbits.QEISIDL = 0;         //Continue in Idle Mode
    QEI1CONbits.PIMOD   = 0b110;     //Reset POSCNT when POSCNT = QEI1GEC register
    QEI1CONbits.IMV     = 0;         //index does not effect POSCNT
    QEI1CONbits.INTDIV  = 0b000;     //1:1 Prescaler
    QEI1CONbits.CNTPOL  = 0;         //POSCNT and Index are positive
    QEI1CONbits.GATEN   = 0;         //Disbale External Gate
    QEI1CONbits.CCM     = 0b00;      //Quadrature Mode

    /*QEI I/O CON register*/
    QEI1IOCbits.QCAPEN  = 0;        //HOME input does not trigger POSCNT
    QEI1IOCbits.FLTREN  = 1;        //Disable Digital Filter
    QEI1IOCbits.QFDIV   = 0b000;    //1:1 Clock Divide
    QEI1IOCbits.OUTFNC  = 0b11;     //QEI Module Output Function Mode Select bits
    QEI1IOCbits.SWPAB   = 0;        //A and B are not Swapped
    QEI1IOCbits.HOMPOL  = 0;        //HOME polarity is not inverted
    QEI1IOCbits.IDXPOL  = 0;        //INDEX is not inverted
    QEI1IOCbits.QEAPOL  = 0;        //QEA is not inverted
    QEI1IOCbits.QEBPOL  = 0;        //QEB is not inverted

    /*QEI Status Register*/
    QEI1STATbits.PCHEQIEN = 1;      //PSCNT greater than or equal Interrupt
    QEI1STATbits.PCLEQIEN = 1;      //PSCNT less than or equal interrupt
    QEI1STATbits.POSOVIEN = 0;      //Overflow Interrupt
    QEI1STATbits.PCIIEN   = 0;      //POSCNT Homing Interupt
    QEI1STATbits.VELOVIEN = 0;      //Velocity overflow Interrupt
    QEI1STATbits.HOMIEN   = 0;      //Home input Interrupt
    QEI1STATbits.IDXIEN   = 0;      //Index input Interrupt

    //Greater than or equal compare to POSCNT (COMP = 2n, where n == # of changes of one channel)
    GTE_COMP_H_REG = 0;             //Upper 16 bits
    GTE_COMP_L_REG = 1;            //Lower 16 bits

    LTE_COMP_H_REG = 0;             //Low Bound == 0
    LTE_COMP_L_REG = 0;

    EnableIntQEI1;
    SetPriorityIntQEI1(1);

    QEI1CONbits.QEIEN = 1;
}

signed int GetRevCount(void)
{
    return revCount;
}

void ResetRevCount(void)
{
    SetRevCount(0);
}

void SetRevCount(signed int count)
{
    revCount = count;
}

uint8_t GetDirection(void)
{
//    return DIRECTION;
}

/*******************************************************************************
 * PRIVATE FUNCTION DEFFINITIONS                                                          *
 ******************************************************************************/

void __attribute__((interrupt, auto_psv)) _QEI1Interrupt(void)
{
    if(POSCNT_GTE_COMP == 1){
        revCount++;
        POSCNT_GTE_COMP = 0;
    }
    if(POSCNT_LTE_COMP == 1) {
        revCount--;
        POSCNT_LTE_COMP = 0;
    }
//    printf("RevCount:%d\n", GetRevCount());
    IFS3bits.QEI1IF = 0;
}
