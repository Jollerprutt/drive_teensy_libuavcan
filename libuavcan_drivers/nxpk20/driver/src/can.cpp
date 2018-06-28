/*
 * Teensy 3.2 driver for UAVCAN
 * @author fwindolf - Florian Windolf  florianwindolf@gmail.com
 * @author TUM Phoenix Robotics
 */

#include <Arduino.h>
#include <uavcan_nxpk20/can.hpp>
#include <uavcan_nxpk20/clock.hpp>
#include "kinetis_flexcan.h"
#include "helper_flexcan.h"

using namespace uavcan;

namespace uavcan_nxpk20
{

// number of tx and rx buffer
static uint8_t rx_buffer_count;
static uint8_t tx_buffer_count;
static uint8_t rx_buffer_first;
static uint8_t tx_buffer_first;

// current tx_buffer to use
static uint8_t tx_buffer;
static MonotonicTime last_deadline = MonotonicTime::fromMSec(0);


// Init static variable
CanDriver CanDriver::self;

// rxb?
static const int rxb = 0;

void wrongDevice()
{
  while(true){ Serial.println("This processor is not supported!"); }
}


CanDriver::CanDriver()
{
  // select pin setup
  #if defined(__MK20DX256__)
    CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
    CORE_PIN4_CONFIG = PORT_PCR_MUX(2);
  #elif
    wrongDevice();
  #endif

  // select clock source 16MHz xtal
  OSC0_CR |= OSC_ERCLKEN;
  // select clock source
  SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0;
  FLEXCANb_CTRL1(FLEXCAN0_BASE) &= ~FLEXCAN_CTRL_CLK_SRC;

  // enable CAN
  freeze();
  FLEXCANb_MCR(FLEXCAN0_BASE) &= ~FLEXCAN_MCR_MDIS;

  // wait until
  while (FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_LPM_ACK){;}
  // wait until enabled bit is set
  while(FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_MDIS) {;}

  // do soft reset
  softReset();
  waitFrozen();

  // disable self-reception
  FLEXCANb_MCR(FLEXCAN0_BASE) |= FLEXCAN_MCR_SRX_DIS;

}


// TODO: provide implementation
uint32_t CanDriver::detectBitRate()
{
  return 1000000;
}

int CanDriver::init(const uint32_t bitrate, const uint8_t rx_buf, const uint8_t tx_buf)
{

  // set bitrate
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

  // default mask is allow everything
  CAN_filter_t mask;
  mask.ext =  0;
  mask.id = 0;
  mask.rtr = 0;
  // set rx and tx buffer
  #if defined(__MK20DX256__)
    if(rx_buf + tx_buf > 16) // there are 16 mailboxes/buffer in hardware
    {while(true){Serial.println("Too many rx and tx buffer.");}}
  #elif
    wrongDevice();
  #endif

  rx_buffer_count = rx_buf;
  tx_buffer_count = tx_buf;
  rx_buffer_first = 0;
  tx_buffer_first = rx_buf;
  tx_buffer = tx_buffer_first;

  //enable reception of all messages that fit the mask
  if (mask.ext) {
    FLEXCANb_RXFGMASK(FLEXCAN0_BASE) = ((mask.rtr?1:0) << 31) | ((mask.ext?1:0) << 30) | ((mask.id & FLEXCAN_MB_ID_EXT_MASK) << 1);
  } else {
    FLEXCANb_RXFGMASK(FLEXCAN0_BASE) = ((mask.rtr?1:0) << 31) | ((mask.ext?1:0) << 30) | (FLEXCAN_MB_ID_IDSTD(mask.id) << 1);
  }

  // start the CAN
  exitHalt();

  // wait until not ready bit set
  while(FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_NOT_RDY) {;}


  // activate tx buffers
  for(int i = tx_buffer_first; i < tx_buffer_first + tx_buffer_count; i++)
  {
    FLEXCANb_MBn_CS(FLEXCAN0_BASE, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }

  //lowest buffers transmit first
  FLEXCANb_CTRL1(FLEXCAN0_BASE) |= FLEXCAN_CTRL_LBUF;

  return 0;
}


int16_t CanDriver::send(const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags)
{
  // Check if timed out
  if(clock::getMonotonic() >= tx_deadline)
  {
    return -1; // too late
  }

  // check if we already got a message with this deadline
  if(last_deadline == tx_deadline)
  {

    // use next buffer position
    tx_buffer++;

    // do another check if this buffer is inactive
    if((FLEXCANb_MBn_CS(FLEXCAN0_BASE, tx_buffer) & FLEXCAN_MB_CS_CODE_MASK)
                      != FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE))
                      {
                          // something went wrong
                          Serial.println("something went wrong");
                          return -1;
                      }

    // we reached the limit of available buffers
    if(tx_buffer_count-1 == tx_buffer - tx_buffer_first)
    {
      // reset old deadline
      last_deadline = MonotonicTime::fromMSec(0);
    }

  // we got a message with a new deadline
  }else{

    // check if all buffers are clean from previous messages
    for(int i = tx_buffer_first; i < tx_buffer_first + tx_buffer_count; i++)
    {
      while((FLEXCANb_MBn_CS(FLEXCAN0_BASE, i) & FLEXCAN_MB_CS_CODE_MASK)
                        != FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE))
                        { } // wait here
    }

    // use first buffer
    tx_buffer = tx_buffer_first;

    // remember deadline (maybe we get more messages with this deadline)
    last_deadline = tx_deadline;
  }

  // Transmit the frame
  FLEXCANb_MBn_CS(FLEXCAN0_BASE, tx_buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);

  // Set header
  if(frame.isExtended())
  {
    FLEXCANb_MBn_ID(FLEXCAN0_BASE, tx_buffer) = (frame.id & FLEXCAN_MB_ID_EXT_MASK);
  }
  else
  {
    FLEXCANb_MBn_ID(FLEXCAN0_BASE, tx_buffer) = FLEXCAN_MB_ID_IDSTD(frame.id);
  }

  // Transfer data to the buffer
  FLEXCANb_MBn_WORD0(FLEXCAN0_BASE, tx_buffer) = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | (frame.data[3]);
  FLEXCANb_MBn_WORD1(FLEXCAN0_BASE, tx_buffer) = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | (frame.data[7]);

  // Start transmission
  if(frame.isExtended())
  {
    FLEXCANb_MBn_CS(FLEXCAN0_BASE, tx_buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                                              | FLEXCAN_MB_CS_LENGTH(frame.dlc) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  }
  else
  {
    FLEXCANb_MBn_CS(FLEXCAN0_BASE, tx_buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                                              | FLEXCAN_MB_CS_LENGTH(frame.dlc);
  }
  return 1;
}


int16_t CanDriver::receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic, UtcTime& out_ts_utc,
                           CanIOFlags& out_flags)
{

  uint16_t rx_buffer = 999;

  for(int i=rx_buffer_first; i<rx_buffer_first+rx_buffer_count; i++)
  {
    if(FLEXCANb_IFLAG1(FLEXCAN0_BASE) & (uint32_t) 1 << i)
    {
      rx_buffer = i;
      break;
    }
  }

  if(999 == rx_buffer)
  {
    Serial.println("Nothing found. Something went wrong!");
    return 0;
  }


  // save timestamp
  out_ts_monotonic = clock::getMonotonic();
  out_ts_utc = UtcTime(); // TODO: change to clock::getUtc() when properly implemented

  // get identifier and dlc
  out_frame.dlc = FLEXCAN_get_length(FLEXCANb_MBn_CS(FLEXCAN0_BASE, rx_buffer));
  out_frame.id = (FLEXCAN0_MBn_ID(rxb) & FLEXCAN_MB_ID_EXT_MASK);

  // is extended identifier?
  bool is_ext = (FLEXCAN0_MBn_CS(rxb) & FLEXCAN_MB_CS_IDE)? 1:0;
  if(!is_ext){
    out_frame.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
  }else{
    out_frame.id &= uavcan::CanFrame::MaskExtID;
    out_frame.id |= uavcan::CanFrame::FlagEFF;
  }

  // copy data to message
  uint32_t data = FLEXCANb_MBn_WORD0(FLEXCAN0_BASE, rx_buffer);
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
    data = FLEXCANb_MBn_WORD1(FLEXCAN0_BASE, rx_buffer);
    out_frame.data[7] = data;
    data >>= 8;
    out_frame.data[6] = data;
    data >>= 8;
    out_frame.data[5] = data;
    data >>= 8;
    out_frame.data[4] = data;
  }

  // set read flags
  FLEXCANb_IFLAG1(FLEXCAN0_BASE) |= (uint32_t) 1 << rx_buffer;
  return 1;
}

int16_t CanDriver::select(CanSelectMasks& inout_masks,
                       const CanFrame* (&)[MaxCanIfaces],
                       MonotonicTime blocking_deadline)
{
  uint32_t mask = (uint32_t) 1  <<  rx_buffer_count;
  mask = mask-1;
  inout_masks.read = (FLEXCANb_IFLAG1(FLEXCAN0_BASE) & mask)? 1:0;

  return 0;
}

int16_t CanDriver::configureFilters(const CanFilterConfig* filter_configs,
                                   uint16_t num_configs)
{
  if(num_configs > rx_buffer_count)
  {
    Serial.println("Something went wrong!");
    return 1;
  }

  // bool wasFrozen=isFrozen();
  //
  // if (!wasFrozen) {
  //     freeze();
  //     halt();
  // }

  uint16_t config = 0;
  for(int i=rx_buffer_first; i<rx_buffer_first + num_configs; i++)
  {
    FLEXCANb_MBn_ID(FLEXCAN0_BASE, i) = filter_configs[config].id;
    FLEXCANb_MB_MASK(FLEXCAN0_BASE, i) = filter_configs[config].mask;
    FLEXCANb_MBn_CS(FLEXCAN0_BASE, i) |= FLEXCAN_MB_CS_IDE;

    config++;
  }

  // if (!wasFrozen) exitHalt();

 return 0;
}

uint64_t CanDriver::getErrorCount() const
{
  uint8_t tx_err = 0;
  uint8_t rx_err = 0;

  FLEXCAN_ECR_TX_ERR_COUNTER(tx_err);
  FLEXCAN_ECR_RX_ERR_COUNTER(rx_err);

  return tx_err + rx_err;
}

uint16_t CanDriver::getNumFilters() const
{
  return rx_buffer_count;
}


ICanIface* CanDriver::getIface(uint8_t iface_index)
{
  return (ICanIface*) &self;
}

uint8_t CanDriver::getNumIfaces() const
{
  uint8_t devices;

  #if defined(__MK20DX256__)
    // the MK20DX256 has only CAN0
    devices = 1;
  #elif
    wrongDevice();
  #endif

  return devices;
}

} // uavcan_nxpk20
