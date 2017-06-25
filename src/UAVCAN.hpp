/**
 * Inlcudes for teensy 3.2
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan_nxpk20/uavcan_nxpk20.hpp>


extern "C"{
  int _getpid(){ return -1;}
  int _kill(int pid, int sig){ return -1; }
  int _write(){return -1;}
}
