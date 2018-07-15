/*
 * Teensy 3.2 header for UAVCAN
 * @author fwindolf - Florian Windolf  florianwindolf@gmail.com
 */

#include <Arduino.h>
#include <uavcan_nxpk20/can.hpp>
#include <uavcan_nxpk20/clock.hpp>

using namespace uavcan;

namespace uavcan_nxpk20
{

// Init static variable
CanDriver CanDriver::self;

CanDriver::CanDriver()
{
  
}

// detects the current bit rate
uint32_t CanDriver::detectBitRate()
{
  // TODO: implementation missing
  return 0;
}


// initialize can driver
int CanDriver::init(uint32_t bitrate)
{
  Can0.begin(bitrate);
  return 0;
}

// sends a CAN frame
int16_t CanDriver::send(const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags)
{

  // Frame was not transmitted until tx deadline
  if(!tx_deadline.isZero() && clock::getMonotonic() >= tx_deadline)
  {
    return -1;
  }

  CAN_message_t msg;
  msg.id = frame.id;
  msg.flags.extended = frame.isExtended();
  msg.flags.remote = frame.isRemoteTransmissionRequest();
  msg.len = frame.dlc;
  for(int i=0; i<frame.dlc; i++)
  {
    msg.buf[i] = frame.data[i];
  }
  return Can0.write(msg);
}

// receives a CAN frame
int16_t CanDriver::receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic, UtcTime& out_ts_utc,
                           CanIOFlags& out_flags)
{

  CAN_message_t msg;
  int result = Can0.read(msg);


  out_frame.id = msg.id;
  if(msg.flags.extended)
  {
    out_frame.id &= uavcan::CanFrame::MaskExtID;
    out_frame.id |= uavcan::CanFrame::FlagEFF;
  }
  

  out_frame.dlc = msg.len;
  for(int i=0; i<msg.len; i++)
  {
    out_frame.data[i] = msg.buf[i];
  }

  
  // save timestamp
  out_ts_monotonic = clock::getMonotonic();
  out_ts_utc = UtcTime();               // TODO: change to clock::getUtc() when properly implemented

  return result;
}

int16_t CanDriver::select(CanSelectMasks& inout_masks,
                       const CanFrame* (&)[MaxCanIfaces],
                       MonotonicTime blocking_deadline)
{
  inout_masks.read = (Can0.available() > 0) ? 1:0;
  return 0;
}

int16_t CanDriver::configureFilters(const CanFilterConfig* filter_configs,
                         uint16_t num_configs)
{
  Serial.println("CanDriver configureFilters");
 // TODO: Provide implementation
 return 0;
}

uint64_t CanDriver::getErrorCount() const
{
  return Can0.getStats().ringRxFramesLost;
}

uint16_t CanDriver::getNumFilters() const
{
  // TODO
  return 0;
}


ICanIface* CanDriver::getIface(uint8_t iface_index)
{
  return (ICanIface*) &self;
}

uint8_t CanDriver::getNumIfaces() const
{
  return 1;
}

} // uavcan_nxpk20
