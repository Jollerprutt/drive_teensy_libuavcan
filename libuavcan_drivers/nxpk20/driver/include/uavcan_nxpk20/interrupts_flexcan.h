/*
 * File:    interrupts_flexcan.h
 * Purpose: flexcan interrupt functions
 * Source: https://github.com/collin80/FlexCAN_Library/blob/master/FlexCAN.cpp
 */

#ifndef	INTERRUPTS_FLEXCAN_H
#define	INTERRUPTS_FLEXCAN_H

#include <Arduino.h>
#include "kinetis_flexcan.h"

#define IRQ_PRIORITY 64 // 0 = highest, 255 = lowest

// TODO: more processors here: https://github.com/collin80/FlexCAN_Library/blob/master/FlexCAN.cpp#L105
#if defined(__MK20DX256__)
    #define IrqMessage IRQ_CAN_MESSAGE
#endif


/*
 * \brief Enable Per-Mailbox Filtering.
 * \param None.
 * \retval None.
 */
void enablePerMailboxFilter()
{
    FLEXCANb_MCR(FLEXCAN0_BASE) |= FLEXCAN_MCR_IRMQ;
}



#endif