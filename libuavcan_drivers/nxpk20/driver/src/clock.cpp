#include <Arduino.h>
#include <uavcan_nxpk20/clock.hpp>

namespace uavcan_nxpk20
{
namespace clock
{
namespace
{

bool initialized = false;
bool utc_set = false;

std::int64_t utc_correction = 0;
std::int64_t prev_adjustment = 0;

std::uint64_t time_mono = 0;
std::uint64_t time_utc = 0;

elapsedMicros usecElapsed;

}

#ifdef __GNUC__
__attribute__((noreturn))
#endif
static void fail()
{
  while(true) {}
}

void init()
{
  //Serial.println("SystemClock init");
  if(!initialized)
  {
    initialized = true;
    usecElapsed = 0;
    // usecElapsed = 3900000000;
    // Serial.println("SystemClock init 3900000000");
  }
}

static std::uint64_t sampleTime()
{
  if(usecElapsed >= 4000000000)
  {
  std::uint32_t usecTemp = usecElapsed;
  usecElapsed -= usecTemp;
  time_mono += usecTemp;
  // Serial.println(usecElapsed);
  // Serial.println("Emptied usecElapsed");
  // Serial.println(usecElapsed);
  // return time_mono + usecElapsed;
  }
  // else
  // {
  //   return time_mono + usecElapsed;
  // }
  return time_mono + usecElapsed;
}

// static void sampleTime()
// {
//   // if(usecElapsed >= 4000000000)
//   // {
//   std::uint32_t usecTemp = usecElapsed;
//   usecElapsed -= usecTemp;
//   time_mono += usecTemp;
//   // return time_mono += usecTemp;
//   // }
//   // else
//   // {
//   //   return time_mono + usecElapsed;
//   // }
// }

uavcan::MonotonicTime getMonotonic()
{
  // Serial.println("SystemClock getMonotonic");
  if(!initialized)
  {
    fail();
  }
  std::uint64_t usec = 0;
  {
    usec = sampleTime();
  }
  // sampleTime();

  // return uavcan::MonotonicTime::fromUSec(time_mono);
  return uavcan::MonotonicTime::fromUSec(usec);
}

uavcan::UtcTime getUtc()
{
  Serial.println("SystemClock getUtc - not implemented, returned 0");
  // Only return the time if time was adjusted
  std::uint64_t _usec = 0;
  // if(utc_set)
  // {
  //   _usec = time_utc;
  // }
  // return uavcan::UtcTime::fromUSec(_usec);
  return uavcan::UtcTime::fromUSec(_usec);
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
  //Serial.println("SystemClock adjustUtc");
  // const std::int64_t adj_delta = adjustment.toUSec() - prev_adjustment;
  prev_adjustment = adjustment.toUSec();

  // On first time
  if(!utc_set)
  {
    utc_set = true;
    utc_correction = 0;
    time_utc = time_mono;
  }

  // TODO: find better implementation for compensating clock speed, for example
  // add x ticks at every timer overflow and handle elapsed time with micros()
  // plus overflowing in the correct ISR
  time_utc += adjustment.toUSec();
}

} // clock

// Init static variable
SystemClock SystemClock::self;

SystemClock& SystemClock::instance()
{
  //Serial.println("SystemClock instance");
  clock::init();
  return self;
}

} // uavcan_nxpk20
