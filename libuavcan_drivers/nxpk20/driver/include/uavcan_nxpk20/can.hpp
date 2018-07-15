/*
 * Teensy 3.2 header for UAVCAN
 * @author fwindolf - Florian Windolf  florianwindolf@gmail.com
 */

#ifndef UAVCAN_NXPK20_CAN_HPP_INCLUDED
#define UAVCAN_NXPK20_CAN_HPP_INCLUDED


#include <uavcan/driver/can.hpp>
#include "FlexCAN.h"

namespace uavcan_nxpk20
{

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
  int init(uavcan::uint32_t bitrate);

  /**
   * Non-blocking transmission.
   *
   * If the frame wasn't transmitted upon TX deadline, the driver should discard it.
   *
   * Note that it is LIKELY that the library will want to send the frames that were passed into the select()
   * method as the next ones to transmit, but it is NOT guaranteed. The library can replace those with new
   * frames between the calls.
   *
   * @return 1 = one frame transmitted, 0 = TX buffer full, negative for error.
   */
  uavcan::int16_t send(const uavcan::CanFrame& frame,
                       uavcan::MonotonicTime tx_deadline,
                       uavcan::CanIOFlags flags) override;


  /**
   * Non-blocking reception.
   *
   * Timestamps should be provided by the CAN driver, ideally by the hardware CAN controller.
   *
   * Monotonic timestamp is required and can be not precise since it is needed only for
   * protocol timing validation (transfer timeouts and inter-transfer intervals).
   *
   * UTC timestamp is optional, if available it will be used for precise time synchronization;
   * must be set to zero if not available.
   *
   * Refer to @ref ISystemClock to learn more about timestamps.
   *
   * @param [out] out_ts_monotonic Monotonic timestamp, mandatory.
   * @param [out] out_ts_utc       UTC timestamp, optional, zero if unknown.
   * @return 1 = one frame received, 0 = RX buffer empty, negative for error.
   */
  uavcan::int16_t receive(uavcan::CanFrame& out_frame,
                          uavcan::MonotonicTime& out_ts_monotonic,
                          uavcan::UtcTime& out_ts_utc,
                          uavcan::CanIOFlags& out_flags) override;


  
  /**
   * Block until the deadline, or one of the specified interfaces becomes available for read or write.
   *
   * Iface masks will be modified by the driver to indicate which exactly interfaces are available for IO.
   *
   * Bit position in the masks defines interface index.
   *
   * Note that it is allowed to return from this method even if no requested events actually happened, or if
   * there are events that were not requested by the library.
   *
   * The pending TX argument contains an array of pointers to CAN frames that the library wants to transmit
   * next, per interface. This is intended to allow the driver to properly prioritize transmissions; many
   * drivers will not need to use it. If a write flag for the given interface is set to one in the select mask
   * structure, then the corresponding pointer is guaranteed to be valid (not UAVCAN_NULLPTR).
   *
   * @param [in,out] inout_masks        Masks indicating which interfaces are needed/available for IO.
   * @param [in]     pending_tx         Array of frames, per interface, that are likely to be transmitted next.
   * @param [in]     blocking_deadline  Zero means non-blocking operation.
   * @return Positive number of ready interfaces or negative error code.
   */
  uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks,
                         const uavcan::CanFrame* (&)[uavcan::MaxCanIfaces],
                         uavcan::MonotonicTime blocking_deadline) override;



  uavcan::int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs,
                                   uavcan::uint16_t num_configs) override;

  uavcan::uint64_t getErrorCount() const override;

  uavcan::uint16_t getNumFilters() const override;

  uavcan::ICanIface* getIface(uavcan::uint8_t iface_index) override;

  uavcan::uint8_t getNumIfaces() const override;

};

} // uavcan_nxpk20

#endif // UAVCAN_NXPK20_CAN_HPP_INCLUDED
