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

/*
 * \brief Tests is CAN bus frozen.
 * \param None.
 * \retval true, if CAN bus is frozen.
 */
bool isFrozen() {
  return (FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_FRZ_ACK);
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
void end(void)
{
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
