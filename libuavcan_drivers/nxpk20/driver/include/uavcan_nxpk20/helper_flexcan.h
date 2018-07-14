/*
 * File:    helper_flexcan.h
 * Purpose: helper functions and defines for flexcan
 * Source: https://github.com/collin80/FlexCAN_Library/blob/master/FlexCAN.cpp
 */

#ifndef	HELPER_FLEXCAN_H
#define	HELPER_FLEXCAN_H

#include <Arduino.h>
#include <stdint.h>
#include "kinetis_flexcan.h"

#define flexcanBase FLEXCAN0_BASE

#define FLEXCANb_MCR(b)                   (*(vuint32_t*)(b))
#define FLEXCANb_CTRL1(b)                 (*(vuint32_t*)(b+4))
#define FLEXCANb_RXMGMASK(b)              (*(vuint32_t*)(b+0x10))
#define FLEXCANb_IMASK1(b)                (*(vuint32_t*)(b+0x28))
#define FLEXCANb_IFLAG1(b)                (*(vuint32_t*)(b+0x30))
#define FLEXCANb_RXFGMASK(b)              (*(vuint32_t*)(b+0x48))
#define FLEXCANb_MBn_CS(b, n)             (*(vuint32_t*)(b+0x80+n*0x10))
#define FLEXCANb_MBn_ID(b, n)             (*(vuint32_t*)(b+0x84+n*0x10))
#define FLEXCANb_MBn_WORD0(b, n)          (*(vuint32_t*)(b+0x88+n*0x10))
#define FLEXCANb_MBn_WORD1(b, n)          (*(vuint32_t*)(b+0x8C+n*0x10))
#define FLEXCANb_MB_MASK(b, n)            (*(vuint32_t*)(b+0x880+(n*4)))
#define FLEXCANb_IDFLT_TAB(b, n)          (*(vuint32_t*)(b+0xE0+(n*4)))



uint8_t bitTimingTable[21][3] = {
    // prop, seg1, seg2 (4 + prop + seg1 + seg2, seg2 must be at least 1)
    // No value can go over 7 here.
    {0,0,1}, //5
    {1,0,1}, //6
    {1,1,1}, //7
    {2,1,1}, //8
    {2,2,1}, //9
    {2,3,1}, //10
    {2,3,2}, //11
    {2,4,2}, //12
    {2,5,2}, //13
    {2,5,3}, //14
    {2,6,3}, //15
    {2,7,3}, //16
    {2,7,4}, //17
    {3,7,4}, //18
    {3,7,5}, //19
    {4,7,5}, //20
    {4,7,6}, //21
    {5,7,6}, //22
    {6,7,6}, //23
    {6,7,7}, //24
    {7,7,7}, //25
};


/*
 * \brief Loops forever if wrong device.
 * \param None.
 * \retval None.
 */
void wrongDevice()
{
  while(true);
}

/*
 * \brief Initializes CAN pin definitions.
 * \param None.
 * \retval None.
 */
void setPins(){

  // select pin setup
  #if defined(__MK20DX256__)
    CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
    CORE_PIN4_CONFIG = PORT_PCR_MUX(2);
  #elif
    wrongDevice();
  #endif

  // TODO integrate more options from here: https://github.com/collin80/FlexCAN_Library/blob/master/FlexCAN.cpp#L294
}



/*
 * \brief Intializes bus baud rate setting.
 * \param baud - desired baud rate
 * \retval None.
 *
  now using a system that tries to automatically generate a viable baud setting.
  Bear these things in mind:
  - The master clock is 16Mhz
  - You can freely divide it by anything from 1 to 256
  - There is always a start bit (+1)
  - The rest (prop, seg1, seg2) are specified 1 less than their actual value (aka +1)
  - This gives the low end bit timing as 5 (1 + 1 + 2 + 1) and the high end 25 (1 + 8 + 8 + 8)
  A worked example: 16Mhz clock, divisor = 19+1, bit values add up to 16 = 16Mhz / 20 / 16 = 50k baud
*/

void setBaudRate(uint32_t baud) {
    // have to find a divisor that ends up as close to the target baud as possible while keeping the end result between 5 and 25

    uint32_t divisor = 0;
    uint32_t bestDivisor = 0;
    uint32_t result = 16000000 / baud / (divisor + 1);
    int error = baud - (16000000 / (result * (divisor + 1)));
    int bestError = error;

    while (result > 5) {
        divisor++;
        result = 16000000 / baud / (divisor + 1);

        if (result <= 25) {
            error = baud - (16000000 / (result * (divisor + 1)));

            if (error < 0)
                error *= -1;

            // if this error is better than we've ever seen then use it - it's the best option

            if (error < bestError) {
                bestError = error;
                bestDivisor = divisor;
            }

            // If this is equal to a previously good option then
            // switch to it but only if the bit time result was in the middle of the range
            // this biases the output to use the middle of the range all things being equal
            // Otherwise it might try to use a higher divisor and smaller values for prop, seg1, seg2
            // and that's not necessarily the best idea.

            if ((error == bestError) && (result > 11) && (result < 19)) {
                bestError = error;
                bestDivisor = divisor;
            }
        }
    }

    divisor = bestDivisor;
    result = 16000000 / baud / (divisor + 1);

    if ((result < 5) || (result > 25) || (bestError > 300)) {
        Serial.println ("Abort in CAN begin. Couldn't find a suitable baud config!");
        return;
    }

    result -= 5; // the bitTimingTable is offset by 5 since there was no reason to store bit timings for invalid numbers
    uint8_t propSeg = bitTimingTable[result][0];
    uint8_t pSeg1   = bitTimingTable[result][1];
    uint8_t pSeg2   = bitTimingTable[result][2];

    FLEXCANb_CTRL1 (flexcanBase) = (FLEXCAN_CTRL_PROPSEG(propSeg) | FLEXCAN_CTRL_RJW(1) | FLEXCAN_CTRL_ERR_MSK |
                                    FLEXCAN_CTRL_PSEG1(pSeg1) | FLEXCAN_CTRL_PSEG2(pSeg2) | FLEXCAN_CTRL_PRESDIV(divisor));
}

/*
 * \brief Selects the clock source to 16MHz xtal
 * \param None.
 * \retval None.
 */
void setClockSource() {
   // select clock source 16MHz xtal
  OSC0_CR |= OSC_ERCLKEN;
  // select clock source
  SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0;
  FLEXCANb_CTRL1(FLEXCAN0_BASE) &= ~FLEXCAN_CTRL_CLK_SRC;
}

/*
 * \brief Enables the flexcan module
 * \param None.
 * \retval None.
 */
void enableCanModule() {
  FLEXCANb_MCR(FLEXCAN0_BASE) &= ~FLEXCAN_MCR_MDIS;
}

/*
 * \brief Disables self reception
 * \param None.
 * \retval None.
 */
void disableSelfReception() {
   // disable self-reception
  FLEXCANb_MCR(FLEXCAN0_BASE) |= FLEXCAN_MCR_SRX_DIS;
}

/*
 * \brief Tests if CAN is in low power mode
 * \param None.
 * \retval true, if CAN bus is in low power mode.
 */
bool isLowPowerMode(){
  return FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_LPM_ACK;
}


/*
 * \brief Waits until CAN is not in low power mode
 * \param None.
 * \retval None.
 */
void waitNotLowPowerMode() {
  // wait for freeze ack
  while(isLowPowerMode());
}

  
/*
 * \brief Tests is CAN bus frozen.
 * \param None.
 * \retval true, if CAN bus is frozen.
 */
bool isFrozen() {
  return FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_FRZ_ACK;
}

/*
 * \brief Waits until CAN bus is frozen
 * \param None.
 * \retval None.
 *
 */
void waitFrozen() {
  // wait for freeze ack
  while (!isFrozen());
}

/*
 * \brief Waits until CAN bus is not frozen.
 * \param None.
 * \retval None.
 *
 */
void waitNotFrozen() {
  // wait for freeze ack
  while(isFrozen());
}

/*
 * \brief Freezes CAN bus.
 * \param None.
 * \retval None.
 *
 */
void freeze() {
  FLEXCANb_MCR(flexcanBase) |= FLEXCAN_MCR_FRZ;
}


/*
 * \brief Halts CAN bus.
 * \param None.
 * \retval None.
 *
 */
void halt() {
    FLEXCANb_MCR(flexcanBase) |= (FLEXCAN_MCR_HALT);
    waitFrozen();
}

/*
 * \brief Exits from hat state.
 * \param None.
 * \retval None.
 *
 */
void exitHalt() {
    // exit freeze mode and wait until it is unfrozen.
    FLEXCANb_MCR(flexcanBase) &= ~(FLEXCAN_MCR_HALT);
    waitNotFrozen();
}

/*
 * \brief Makes CAN bus soft reset.
 * \param None.
 * \retval None.
 */
void softReset() {
  FLEXCANb_MCR (flexcanBase) ^=  FLEXCAN_MCR_SOFT_RST;
  while (FLEXCANb_MCR (flexcanBase) & FLEXCAN_MCR_SOFT_RST);
}

/*
 * \brief Bring the hardware into freeze which drops it off the CAN bus
 * \param none
 * \retval none
 */
void end(){
    // enter freeze mode
    halt();
    FLEXCANb_MCR(flexcanBase) |= (FLEXCAN_MCR_HALT);

    while(!(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_FRZ_ACK));
}


/*
 * \brief Waits until CAN bus is ready
 * \param None.
 * \retval None.
 */
void waitReady() {
  while(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_NOT_RDY);
}





#endif
