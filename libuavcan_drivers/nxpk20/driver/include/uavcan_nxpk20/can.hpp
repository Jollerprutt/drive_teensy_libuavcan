/*
 * Teensy 3.2 driver for UAVCAN
 * @author fwindolf - Florian Windolf  florianwindolf@gmail.com
 * @author TUM Phoenix Robotics
 */

#ifndef UAVCAN_NXPK20_CAN_HPP_INCLUDED
#define UAVCAN_NXPK20_CAN_HPP_INCLUDED


#include <uavcan/driver/can.hpp>

namespace uavcan_nxpk20
{

  typedef struct CAN_filter_t {
  uint8_t rtr;
  uint8_t ext;
  uint32_t id;
} CAN_filter_t;

/*
 * Implement the CAN Interfaces in non-redundant way
 * Singleton class
 */
class CanDriver:
  public uavcan::ICanDriver,
  public uavcan::ICanIface,
  uavcan::Noncopyable
{
private:
  static CanDriver self;
  /**
   * Constructor of the can driver.
   */
  CanDriver();


public:
  /**
   * Returns the only reference.
   */
  static CanDriver& instance() { return self; }

  /**
   * Detect bus bit rate, blocks for some time
   * @return On success: detected bit rate, in bits per second.
   *         On failure: zero.
   */
  static uavcan::uint32_t detectBitRate();

  /**
   * Initialize the driver
   * @return On success: zero
   *         On failure (baudrate cannot be used): negative number
   */
  int init(const uint32_t bitrate, const uint8_t rx_buf=8, const uint8_t tx_buf=8);

  /**
   * Send a frame before tx_deadline
   */
  uavcan::int16_t send(const uavcan::CanFrame& frame,
                       uavcan::MonotonicTime tx_deadline,
                       uavcan::CanIOFlags flags) override;


  /**
   * Receive a frame from bus
   */
  uavcan::int16_t receive(uavcan::CanFrame& out_frame,
                          uavcan::MonotonicTime& out_ts_monotonic,
                          uavcan::UtcTime& out_ts_utc,
                          uavcan::CanIOFlags& out_flags) override;

  uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks,
                         const uavcan::CanFrame* (&)[uavcan::MaxCanIfaces],
                         uavcan::MonotonicTime blocking_deadline) override;

  uavcan::int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs,
                                   uavcan::uint16_t num_configs) override;

  uavcan::uint64_t getErrorCount() const override;

  uavcan::uint16_t getNumFilters() const override;

  uavcan::ICanIface* getIface(uavcan::uint8_t iface_index) override;

  /**
   * returns the number of CAN devices
   */
  uavcan::uint8_t getNumIfaces() const override;

};

} // uavcan_nxpk20

#endif // UAVCAN_NXPK20_CAN_HPP_INCLUDED
