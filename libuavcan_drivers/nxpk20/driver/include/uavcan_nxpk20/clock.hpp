#pragma once

#include <uavcan/driver/system_clock.hpp>

namespace uavcan_nxpk20
{
namespace clock
{

/*
 * Starts the clock, after first init calling the function will not do anything
 */
void init();

/*
 * Updates time and empties ellapsed time.
 */
// void sampleTime();

/**
 *  Returns the elapsed MonotonicTime since init() was called
 */
uavcan::MonotonicTime getMonotonic();

/**
 * Returns UTC time if set, else zero
 */
uavcan::UtcTime getUtc();

/**
 * Adjusts the UTC time, until then getMonotonic will return zero
 */
void adjustUtc(uavcan::UtcDuration adjustment);

/**
 * Returns the clock error to previous adjustUTC, positive is hardware is slower
 */
uavcan::UtcDuration getPrevUtcAdjustment();

}

/**
 * Adapter for uavcan::ISystemClock.
 */

class SystemClock :
  public uavcan::ISystemClock,
  public uavcan::Noncopyable
{
private:
  /*
   * Instance of SystemClock
   */
  static SystemClock self;

  SystemClock() { }

  uavcan::MonotonicTime getMonotonic()      const override { return clock::getMonotonic(); }
  uavcan::UtcTime getUtc()                  const override { return clock::getUtc(); }
  void adjustUtc(uavcan::UtcDuration adjustment)  override { clock::adjustUtc(adjustment); }

public:
  /**
   * Calls clock::init() as needed.
   */
  static SystemClock& instance();
};

} // uavcan_nxpk20
