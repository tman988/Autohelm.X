#include "Ecan1.h"
#include "CircularBuffer.h"
#include "Node.h"

#include <string.h>
#include <stdbool.h>
#include <ecan.h>
#include <dma.h>
#include <pps.h>

/**
 * @file   Ecan1.c
 * @author Bryant Mairs
 * @author Pavlo Manovi
 * @date   September 28th, 2012
 * @brief  Provides C functions for ECAN blocks
 */

// Specify the number of 8-byte CAN messages buffer supports.
// This can be overridden by user code.
#ifndef ECAN1_BUFFERSIZE
#define ECAN1_BUFFERSIZE 8 * 24
#endif

#define TX_BUFFER   0

// Declare space for our message buffer in DMA
// NOTE: This DMA space is aligned along 256-byte boundaries due to Lubin's blockset aligning it's
// DMA variables to this same boundaries. If you don't you'll end up with memory collisions.
unsigned int ecan1MsgBuf[32][8] __attribute__((aligned(32*16)));

// Initialize our circular buffers and data arrays for transreceiving CAN messages
static CircularBuffer ecan1RxCBuffer;
static uint8_t rxDataArray[ECAN1_BUFFERSIZE];
static CircularBuffer ecan1TxCBuffer;
static uint8_t txDataArray[ECAN1_BUFFERSIZE];

// Track whether or not we're currently transmitting
static bool currentlyTransmitting = 0;

// Also track how many messages are pending for reading.
static uint8_t receivedMessagesPending = 0;
void dma_init(const uint16_t *parameters);

void Ecan1Init(void)
{
    // Make sure the ECAN module is in configuration mode.
    // It should be this way after a hardware reset, but
    // we make sure anyways.
//    C1CTRL1bits.REQOP = 4;
//    while (C1CTRL1bits.OPMODE != 4);
//
//    // Initialize our circular buffers. If this fails, we crash and burn.
    if (!CB_Init(&ecan1TxCBuffer, txDataArray, ECAN1_BUFFERSIZE)) {
		FATAL_ERROR();
    }
    if (!CB_Init(&ecan1RxCBuffer, rxDataArray, ECAN1_BUFFERSIZE)) {
		FATAL_ERROR();
    }

    /* Set up the ECAN1 module to operate at 250 kbps. The ECAN module should be first placed
    in configuration mode. */
     C1CTRL1bits.REQOP = 4;
     while(C1CTRL1bits.OPMODE != 4);

     C1CTRL1bits.WIN = 0;

     /* Set up the CAN module for 250kbps speed with 10 Tq per bit. */
//     C1CFG1 = 0x47;                 //BRP = 8 SJW = 01
//     C1CFG2 = 0x292;              //SEG2PH = 2, SEG2PHTS = 1, SAM = 0, SEG1PH = 2, PRESEG = 2
    
     C1CFG1bits.BRP = 7;
     C1CFG1bits.SJW = 0;

     C1CFG2bits.PRSEG = 2;          //Propigation
     C1CFG2bits.SEG2PH =  2;        //Phase 2
     C1CFG2bits.SEG1PH = 2;         //Phase 1
     C1CFG2bits.SEG2PHTS = 1;       //Phase 2 programmable
     C1CFG2bits.SAM = 1;            //Sample once (0) or three times (1) at set point

     C1CTRL1bits.CANCKS = 0;
     C1FCTRLbits.DMABS = 0;         // Use 4 buffers in DMA RAM
     C1FCTRL = 0xC01F;              //32 Buffers, Start with buffer RB31 (#buff - 1)

     // Clear all interrupt bits
     C1RXFUL1 = C1RXFUL2 = C1RXOVF1 = C1RXOVF2 = 0x0000;

     // Enable interrupts for ECAN1
     IEC2bits.C1IE = 1; // Enable interrupts for ECAN1 peripheral
     C1INTEbits.TBIE = 1; // Enable TX buffer interrupt
     C1INTEbits.RBIE = 1; // Enable RX buffer interrupt

     // Configure Message Buffer 0 for Transmission and assign priority */
     C1TR01CONbits.TXEN0 = 0x1;
     C1TR01CONbits.TX0PRI = 0x3;

     /* At this point the ECAN1 module is ready to transmit a message. Place the ECAN module in
    Normal mode. */
     C1CTRL1bits.REQOP = 0;
     while(C1CTRL1bits.OPMODE != 0);
     
     /* Assign 32x8word Message Buffers for ECAN1 in device RAM. This example uses DMA0 for TX.
    Refer to 21.8.1 ?DMA Operation for Transmitting Data? for details on DMA channel
    configuration for ECAN transmit. */
     DMA0CONbits.SIZE = 0x0;
     DMA0CONbits.DIR = 0x1;
     DMA0CONbits.AMODE = 0x2;
     DMA0CONbits.MODE = 0x0;
     DMA0REQ = 70;
     DMA0CNT = 7;
     DMA0PAD = (volatile unsigned int)&C1TXD;
     DMA0STAL = (unsigned int) &ecan1MsgBuf;
     DMA0STAH = (unsigned int) &ecan1MsgBuf;
     DMA0STBH = 0x0;
     DMA0STBL = 0x0;
     IEC0bits.DMA0IE = 0; // Enable DMA Channel 1 interrupt
     DMA0CONbits.CHEN = 0x1;

     //RX
     DMA1CONbits.SIZE = 0x0;
     DMA1CONbits.DIR = 0x0;                 //Read to RAM from peripheral
     DMA1CONbits.AMODE = 0x2;               // Continuous mode, single buffer
     DMA1CONbits.MODE = 0x0;                // Peripheral Indirect Addressing
     DMA1REQ = 0x22;                        // Select ECAN1 RX as DMA Request source
     DMA1CNT = 7;                           //8 words per transfer
     DMA1PAD = (volatile unsigned int)&C1RXD;
     DMA1STAL = (unsigned int) &ecan1MsgBuf;
     DMA1STAH = (unsigned int) &ecan1MsgBuf;
     DMA0STBH = 0x0;
     DMA0STBL = 0x0;
     IEC0bits.DMA1IE = 0; // Enable DMA Channel 1 interrupt
     DMA1CONbits.CHEN = 0x1;

     // Setup message filters and masks.
    C1CTRL1bits.WIN = 1; // Allow configuration of masks and filters

    C1FMSKSEL1bits.F0MSK=0x0; //Filter 0 will use mask 0
    C1RXM0SIDbits.SID = 0b11111111111; //Set mask
#ifdef TESTTRANSMITTER
    C1RXF0SIDbits.SID = 1;//0x123c;//0x1; //set filter
#else
    C1RXF0SIDbits.SID = 0;//0x123c;//0x1; //set filter
#endif
    C1RXM0SIDbits.MIDE = 0x1; //only receive standard frames
    C1RXF0SIDbits.EXIDE= 0x0;
    C1BUFPNT1bits.F0BP = 0x1; //use message buffer 1 to receive data
    C1FEN1bits.FLTEN0=0x1; //enable channel

    C1CTRL1bits.WIN = 0;
 


//    ecan1MsgBuf[0][0] = 0x123C;
//    ecan1MsgBuf[0][1] = 0x0000;
//    ecan1MsgBuf[0][2] = 0x0008;
//    ecan1MsgBuf[0][3] = 0x1;
//    ecan1MsgBuf[0][4] = 0x2;
//    ecan1MsgBuf[0][5] = 0x3;
//    ecan1MsgBuf[0][6] = 0xabcd;
//    C1TR01CONbits.TXREQ0 = 0x1;
}

int Ecan1Receive(CanMessage *msg, uint8_t *messagesLeft)
{
    int foundOne = CB_ReadMany(&ecan1RxCBuffer, msg, sizeof(CanMessage));

    if (messagesLeft) {
        if (foundOne) {
            *messagesLeft = --receivedMessagesPending;
        } else {
            *messagesLeft = 0;
        }
    }

    return foundOne;
}

/**
 * This function transmits a CAN message on the ECAN1 CAN bus.
 * This function is for internal use only as it bypasses the circular buffer. This means that it
 * can squash existing transfers in progress.
 */
void _ecan1TransmitHelper(const CanMessage *message)
{
    uint16_t word0 = 0, word1 = 0, word2 = 0;
    uint16_t sid10_0 = 0, eid5_0 = 0, eid17_6 = 0;
    uint16_t *ecan_msg_buf_ptr = ecan1MsgBuf[message->buffer];

    // Variables for setting correct TXREQ bit
    uint16_t bit_to_set;
    uint16_t offset;
    uint16_t *bufferCtrlRegAddr;

    // Divide the identifier into bit-chunks for storage
    // into the registers.
    if (message->frame_type == CAN_FRAME_EXT) {
        eid5_0 = (message->id & 0x3F);
        eid17_6 = (message->id >> 6) & 0xFFF;
        sid10_0 = (message->id >> 18) & 0x7FF;
        word0 = 1;
        word1 = eid17_6;
    } else {
        sid10_0 = (message->id & 0x7FF);
    }
    word0 |= (sid10_0 << 2);
    word2 |= (eid5_0 << 10);

    // Set remote transmit bits
    if (message->message_type == CAN_MSG_RTR) {
        word0 |= 0x2;
        word2 |= 0x0200;
    }
    ecan_msg_buf_ptr[0] = word0;
    ecan_msg_buf_ptr[1] = word1;
    ecan_msg_buf_ptr[2] = ((word2 & 0xFFF0) + message->validBytes);
    ecan_msg_buf_ptr[3] = ((uint16_t) message->payload[1] << 8 | ((uint16_t) message->payload[0]));
    ecan_msg_buf_ptr[4] = ((uint16_t) message->payload[3] << 8 | ((uint16_t) message->payload[2]));
    ecan_msg_buf_ptr[5] = ((uint16_t) message->payload[5] << 8 | ((uint16_t) message->payload[4]));
    ecan_msg_buf_ptr[6] = ((uint16_t) message->payload[7] << 8 | ((uint16_t) message->payload[6]));
    // Set the correct transfer intialization bit (TXREQ) based on message buffer.
    offset = message->buffer >> 1;
    bufferCtrlRegAddr = (uint16_t *) (&C1TR01CON + offset);
    bit_to_set = 1 << (3 | ((message->buffer & 1) << 3));
    *bufferCtrlRegAddr |= bit_to_set;
//    C1TR01CONbits.TXREQ0 = 0x1;
    // Keep track of whether we're in a transmission train or not.
    currentlyTransmitting = 1;
}

/**
 * Transmits a CanMessage using the transmission circular buffer.
 */
void Ecan1Transmit(const CanMessage *msg)
{
    // Append the message to the queue.
    // Message are only removed upon successful transmission.
    // They will be overwritten by newer message overflowing
    // the circular buffer however.
    CB_WriteMany(&ecan1TxCBuffer, msg, sizeof(CanMessage), true);

    // If this is the only message in the queue, attempt to
    // transmit it.
    
    if (!currentlyTransmitting) {
        _ecan1TransmitHelper(msg);
    }
}

void Ecan1GetErrorStatus(uint8_t errors[2])
{
    // Set transmission errors in first array element.
    if (C1INTFbits.TXBO) {
        errors[0] = 3;
    } else if (C1INTFbits.TXBP) {
        errors[0] = 2;
    } else if (C1INTFbits.TXWAR) {
        errors[0] = 1;
    }

    // Set reception errors in second array element.
    if (C1INTFbits.RXBP) {
        errors[1] = 2;
    } else if (C1INTFbits.RXWAR) {
        errors[1] = 1;
    }
}

void dma_init(const uint16_t *parameters)
{
    // Determine the correct addresses for all needed registers
    uint16_t offset = (parameters[4]*6);
    uint16_t *chanCtrlRegAddr = (uint16_t *) (&DMA0CON + offset);
    uint16_t *irqSelRegAddr = (uint16_t *) (&DMA0REQ + offset);
    uint16_t *addrOffsetRegAddr = (uint16_t *) (&DMA0STAL + offset);
    uint16_t *secAddrOffsetRegAddr = (uint16_t *) (&DMA0STBL + offset);
    uint16_t *periAddrRegAddr = (uint16_t *) (&DMA0PAD + offset);
    uint16_t *transCountRegAddr = (uint16_t *) (&DMA0CNT + offset);

//    DMACS0 = 0; // Clear the status register

    *periAddrRegAddr = (uint16_t) parameters[1]; // Set the peripheral address that will be using DMA
    *transCountRegAddr = (uint16_t) parameters[2]; // Set data units to words or bytes
    *irqSelRegAddr = (uint16_t) (parameters[0] >> 8); // Set the IRQ priority for the DMA transfer
    *addrOffsetRegAddr = (uint16_t) parameters[3]; // Set primary DPSRAM start address bits
    *secAddrOffsetRegAddr = (uint16_t) parameters[5]; // Set secondary DPSRAM start address bits

    // Setup the configuration register & enable DMA
    *chanCtrlRegAddr = (uint16_t) (0x8000 | ((parameters[0] & 0x00F0) << 7) | ((parameters[0] & 0x000C) << 2));
}

/**
 * This is an interrupt handler for the ECAN1 peripheral.
 * It clears interrupt bits and pushes received message into
 * the circular buffer.
 */
void __attribute__((interrupt, no_auto_psv))_C1Interrupt(void)
{
    // Give us a CAN message struct to populate and use
    static CanMessage message;
    uint8_t ide = 0;
    uint8_t srr = 0;
    uint32_t id = 0;
    uint16_t *ecan_msg_buf_ptr;

    // If the interrupt was set because of a transmit, check to
    // see if more messages are in the circular buffer and start
    // transmitting them.
    if (C1INTFbits.TBIF) {

        // After a successfully sent message, there should be at least
        // one message in the queue, so pop it off.
        CB_ReadMany(&ecan1TxCBuffer, &message, sizeof(CanMessage));

        // Check for a buffer overflow. Then clear the entire buffer if there was.
        if (ecan1TxCBuffer.overflowCount) {
            CB_Init(&ecan1TxCBuffer, txDataArray, ECAN1_BUFFERSIZE);
        }

        // Now if there's still a message left in the buffer,
        // try to transmit it.
        if (ecan1TxCBuffer.dataSize >= sizeof(CanMessage)) {
            static CanMessage msg;
            CB_PeekMany(&ecan1TxCBuffer, &msg, sizeof(CanMessage));
            _ecan1TransmitHelper(&msg);
        } else {
            currentlyTransmitting = 0;
        }

        C1INTFbits.TBIF = 0;
    }

    // If the interrupt was fired because of a received message
    // package it all up and store in the circular buffer.
    if (C1INTFbits.RBIF) {

        // Obtain the buffer the message was stored into, checking that the value is valid to refer to a buffer
        if (C1VECbits.ICODE < 32) {
            message.buffer = C1VECbits.ICODE;
        }

        ecan_msg_buf_ptr = ecan1MsgBuf[message.buffer];

        // Clear the buffer full status bit so more messages can be received.
        if (C1RXFUL1 & (1 << message.buffer)) {
            C1RXFUL1 &= ~(1 << message.buffer);
        }

        //  Move the message from the DMA buffer to a data structure and then push it into our circular buffer.

        // Read the first word to see the message type
        ide = ecan_msg_buf_ptr[0] & 0x0001;
        srr = ecan_msg_buf_ptr[0] & 0x0002;

        /* Format the message properly according to whether it
         * uses an extended identifier or not.
         */
        if (ide == 0) {
            message.frame_type = CAN_FRAME_STD;

            message.id = (uint32_t) ((ecan_msg_buf_ptr[0] & 0x1FFC) >> 2);
        } else {
            message.frame_type = CAN_FRAME_EXT;

            id = ecan_msg_buf_ptr[0] & 0x1FFC;
            message.id = id << 16;
            id = ecan_msg_buf_ptr[1] & 0x0FFF;
            message.id |= id << 6;
            id = ecan_msg_buf_ptr[2] & 0xFC00;
            message.id |= id >> 10;
        }

        /* If message is a remote transmit request, mark it as such.
         * Otherwise it will be a regular transmission so fill its
         * payload with the relevant data.
         */
        if (srr == 1) {
            message.message_type = CAN_MSG_RTR;
        } else {
            message.message_type = CAN_MSG_DATA;

            message.validBytes = (uint8_t) (ecan_msg_buf_ptr[2] & 0x000F);
            message.payload[0] = (uint8_t) ecan_msg_buf_ptr[3];
            message.payload[1] = (uint8_t) ((ecan_msg_buf_ptr[3] & 0xFF00) >> 8);
            message.payload[2] = (uint8_t) ecan_msg_buf_ptr[4];
            message.payload[3] = (uint8_t) ((ecan_msg_buf_ptr[4] & 0xFF00) >> 8);
            message.payload[4] = (uint8_t) ecan_msg_buf_ptr[5];
            message.payload[5] = (uint8_t) ((ecan_msg_buf_ptr[5] & 0xFF00) >> 8);
            message.payload[6] = (uint8_t) ecan_msg_buf_ptr[6];
            message.payload[7] = (uint8_t) ((ecan_msg_buf_ptr[6] & 0xFF00) >> 8);
        }

        // Store the message in the buffer
        CB_WriteMany(&ecan1RxCBuffer, &message, sizeof(CanMessage), true);

        // Increase the number of messages stored in the buffer
        ++receivedMessagesPending;

        // Be sure to clear the interrupt flag.
        C1INTFbits.RBIF = 0;
    }

    // Clear the general ECAN1 interrupt flag.
    IFS2bits.C1IF = 0;

}
