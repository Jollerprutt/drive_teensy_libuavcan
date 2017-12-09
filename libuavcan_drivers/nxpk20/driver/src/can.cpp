/*
 * Teensy 3.2 header for UAVCAN
 * @author fwindolf - Florian Windolf  florianwindolf@gmail.com
 */

#include <Arduino.h>
#include <uavcan_nxpk20/can.hpp>
#include <uavcan_nxpk20/clock.hpp>
#include "kinetis_flexcan.h"

using namespace uavcan;

namespace uavcan_nxpk20
{

#define FLEXCANb_MCR(b)                   (*(vuint32_t*)(b))
#define FLEXCANb_CTRL1(b)                 (*(vuint32_t*)(b+4))
#define FLEXCANb_RXMGMASK(b)              (*(vuint32_t*)(b+0x10))
#define FLEXCANb_IFLAG1(b)                (*(vuint32_t*)(b+0x30))
#define FLEXCANb_RXFGMASK(b)              (*(vuint32_t*)(b+0x48))
#define FLEXCANb_MBn_CS(b, n)             (*(vuint32_t*)(b+0x80+n*0x10))
#define FLEXCANb_MBn_ID(b, n)             (*(vuint32_t*)(b+0x84+n*0x10))
#define FLEXCANb_MBn_WORD0(b, n)          (*(vuint32_t*)(b+0x88+n*0x10))
#define FLEXCANb_MBn_WORD1(b, n)          (*(vuint32_t*)(b+0x8C+n*0x10))
#define FLEXCANb_IDFLT_TAB(b, n)          (*(vuint32_t*)(b+0xE0+(n*4)))

// Buffers before first are occupied by FIFO
#define TX_BUFFER_FIRST                   8
#define TX_BUFFER_COUNT                   8
#define RX_BUFFER_FIRST                   0

// Init static variable
CanDriver CanDriver::self;

CanDriver::CanDriver()
 : errorCount(0)
{
  // setup pins
  CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
  CORE_PIN4_CONFIG = PORT_PCR_MUX(2);

  // select clock source
  SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0;
  FLEXCANb_CTRL1(FLEXCAN0_BASE) &= ~FLEXCAN_CTRL_CLK_SRC;

  //Serial.println("CanDriver enable CAN");
  // enable CAN
  FLEXCANb_MCR(FLEXCAN0_BASE) |= FLEXCAN_MCR_FRZ;
  FLEXCANb_MCR(FLEXCAN0_BASE) &= ~FLEXCAN_MCR_MDIS;
  // wait until enabled bit is set
  while(FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_MDIS) {;}

  // do soft reset
  FLEXCANb_MCR(FLEXCAN0_BASE) ^= FLEXCAN_MCR_SOFT_RST;
  // wait for soft reset came through and freeze acknwoledge
  while(FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_SOFT_RST) {;}
  // while(FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_FRZ_ACK) {;}


  //Serial.println("CanDriver disable self-reception");
  // disable self-reception
  FLEXCANb_MCR(FLEXCAN0_BASE) |= FLEXCAN_MCR_SRX_DIS;

  //Serial.println("CanDriver enable RX FIFO");
  // enable RX FIFO
  FLEXCANb_MCR(FLEXCAN0_BASE) |= FLEXCAN_MCR_FEN;
}


// TODO: provide implementation
uint32_t CanDriver::detectBitRate()
{
  //Serial.println("CanDriver detectBitRate");
  return 500000;
}

int CanDriver::init(uint32_t bitrate)
{
  //Serial.println("CanDriver init");
  switch(bitrate)
  {
    case 50000:
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3)
                                      | FLEXCAN_CTRL_PRESDIV(19));
      break;
    case 100000:
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3)
                                      | FLEXCAN_CTRL_PRESDIV(9));
      break;
    case 250000:
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3)
                                      | FLEXCAN_CTRL_PRESDIV(3));
      break;
    case 500000:
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3)
                                      | FLEXCAN_CTRL_PRESDIV(1));
      break;
    case 1000000:
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(0)
                                      | FLEXCAN_CTRL_PSEG1(1) | FLEXCAN_CTRL_PSEG2(1)
                                      | FLEXCAN_CTRL_PRESDIV(1));
      break;
    default: // 125000
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                    | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3)
                                    | FLEXCAN_CTRL_PRESDIV(7));
  }

  // set default filter mask
  FLEXCANb_RXMGMASK(FLEXCAN0_BASE) = 0;

  //Serial.println("CanDriver start CAN");
  // start the CAN
  FLEXCANb_MCR(FLEXCAN0_BASE) &= ~(FLEXCAN_MCR_HALT);
  // wait until freeze acknowledged and not ready bit set
  while(FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_FRZ_ACK) {;}
  //Serial.println("CanDriver FRZ_ACK");
  while(FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_NOT_RDY) {;}

  //Serial.println("CanDriver activate RX buf");
  // activate tx buffers
  for(int i = TX_BUFFER_FIRST; i < TX_BUFFER_FIRST + TX_BUFFER_COUNT; i++)
  {
    FLEXCANb_MBn_CS(FLEXCAN0_BASE, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }
  return 0;
}


int16_t CanDriver::send(const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags)
{
  //Serial.println("CanDriver send");
  // Frame was not transmitted until tx deadline
  if(!tx_deadline.isZero() && clock::getMonotonic() >= tx_deadline)
  {
    return -1;
  }

  // Search for available buffer
  int buffer = -1;
  for(int i = TX_BUFFER_FIRST; ;)
  {
    // Check if this buffer is available
    if((FLEXCANb_MBn_CS(FLEXCAN0_BASE, i) & FLEXCAN_MB_CS_CODE_MASK)
        == FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE))
    {
      buffer = i;
      break;
    }
    // If there is no deadline for transmission, check all available buffers
    if(tx_deadline.isZero())
    {
      // Already checked every available buffer?
      if(i > TX_BUFFER_FIRST + TX_BUFFER_COUNT)
      {
        return 0; // no more buffers to check
      }
      i++;
    }
    else
    {
      // Check if timed out
      if(clock::getMonotonic() >= tx_deadline)
      {
        return -1; // too late
      }
    }
    // pass on control to other tasks
    yield();
  }

  //Serial.println("CanDriver transmit frame");
  // Transmit the frame
  FLEXCANb_MBn_CS(FLEXCAN0_BASE, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);

  // Set header
  if(frame.isExtended())
  {
    FLEXCANb_MBn_ID(FLEXCAN0_BASE, buffer) = (frame.id & FLEXCAN_MB_ID_EXT_MASK);
  }
  else
  {
    FLEXCANb_MBn_ID(FLEXCAN0_BASE, buffer) = FLEXCAN_MB_ID_IDSTD(frame.id);
  }

  // Transfer data to the buffer
  FLEXCANb_MBn_WORD0(FLEXCAN0_BASE, buffer) = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | (frame.data[3]);
  FLEXCANb_MBn_WORD1(FLEXCAN0_BASE, buffer) = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | (frame.data[7]);

  // Start transmission
  if(frame.isExtended())
  {
    FLEXCANb_MBn_CS(FLEXCAN0_BASE, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                                         | FLEXCAN_MB_CS_LENGTH(frame.dlc) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  }
  else
  {
    FLEXCANb_MBn_CS(FLEXCAN0_BASE, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                                         | FLEXCAN_MB_CS_LENGTH(frame.dlc);
  }
  return 1;
}


int16_t CanDriver::receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic, UtcTime& out_ts_utc,
                        CanIOFlags& out_flags)
{
  //Serial.println("CanDriver receive");
  // Check if a new frame is available
  if((FLEXCANb_IFLAG1(FLEXCAN0_BASE) & FLEXCAN_IMASK1_BUF5M) == 0)
  {
    //Serial.println("CanDriver no new frame available");
    return 0;
  }

  // save timestamp
  out_ts_monotonic = clock::getMonotonic();
  out_ts_utc = UtcTime(); // TODO: change to clock::getUtc() when properly implemented

  //Serial.println("CanDriver processing new frame");
  // get identifier and dlc
  out_frame.dlc = FLEXCAN_get_length(FLEXCANb_MBn_CS(FLEXCAN0_BASE, RX_BUFFER_FIRST));
  out_frame.id = (FLEXCANb_MBn_CS(FLEXCAN0_BASE, RX_BUFFER_FIRST) & FLEXCAN_MB_ID_EXT_MASK) ? 1 : 0;

  // shift id to the right position if non-extended id
  if(FLEXCANb_MBn_CS(FLEXCAN0_BASE, RX_BUFFER_FIRST) & FLEXCAN_MB_CS_IDE)
  {
    out_frame.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
  }

  //Serial.println("CanDriver copy message");
  // copy data to message
  uint32_t data = FLEXCANb_MBn_WORD0(FLEXCAN0_BASE, RX_BUFFER_FIRST);
  out_frame.data[3] = data;
  data >>= 8;
  out_frame.data[2] = data;
  data >>= 8;
  out_frame.data[1] = data;
  data >>= 8;
  out_frame.data[0] = data;
  // shortcut if last bytes are empty anyways
  if(out_frame.dlc  > 4)
  {
    data = FLEXCANb_MBn_WORD1(FLEXCAN0_BASE, RX_BUFFER_FIRST);
    out_frame.data[7] = data;
    data >>= 8;
    out_frame.data[6] = data;
    data >>= 8;
    out_frame.data[5] = data;
    data >>= 8;
    out_frame.data[4] = data;
  }
  //Serial.println("CanDriver set read flag");
  // set read flags
  FLEXCANb_IFLAG1(FLEXCAN0_BASE) = FLEXCAN_IMASK1_BUF5M;
  return 1;
}

int16_t CanDriver::select(CanSelectMasks& inout_masks,
                       const CanFrame* (&)[MaxCanIfaces],
                       MonotonicTime blocking_deadline)
{
  //Serial.println("CanDriver select");
  // TODO: Provide implementation

  // for now: pretend there is nothing there
  inout_masks.read = 0;


  return 0;
}

int16_t CanDriver::configureFilters(const CanFilterConfig* filter_configs,
                         uint16_t num_configs)
{
  //Serial.println("CanDriver configureFilters");
 // TODO: Provide implementation
 return 0;
}

uint64_t CanDriver::getErrorCount() const
{
  //Serial.println("CanDriver getErrorCount");
  return errorCount;
}

uint16_t CanDriver::getNumFilters() const
{
  //Serial.println("CanDriver getNumFilters");
  // TODO: Provide implementation
  return 0;
}


ICanIface* CanDriver::getIface(uint8_t iface_index)
{
  //Serial.println("CanDriver getIface");
  return (ICanIface*) &self;
}

uint8_t CanDriver::getNumIfaces() const
{
  //Serial.println("CanDriver getNumIfaces");
  // TODO: Provide implementation
  return 1;
}

} // uavcan_nxpk20
