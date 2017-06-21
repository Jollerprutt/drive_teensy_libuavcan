UAVCAN stack in C++
===================

Portable reference implementation of the [UAVCAN protocol stack](http://uavcan.org) in C++ for embedded systems
and Linux.

UAVCAN is a lightweight protocol designed for reliable communication in aerospace and robotic applications via CAN bus.

## Documentation

* [UAVCAN website](http://uavcan.org)
* [UAVCAN discussion group](https://groups.google.com/forum/#!forum/uavcan)
* [Libuavcan overview](http://uavcan.org/Implementations/Libuavcan/)
* [List of platforms officially supported by libuavcan](http://uavcan.org/Implementations/Libuavcan/Platforms/)
* [Libuavcan tutorials](http://uavcan.org/Implementations/Libuavcan/Tutorials/)
* [Teensy CMake](https://github.com/xya/teensy-cmake)

## Library usage

### Dependencies

* Python 2.7 or 3.3 or newer
* [Arduino IDE](https://www.arduino.cc/en/main/software)
* [Teensyduino](https://www.pjrc.com/teensy/td_download.html)
* [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded)

Install Arduino and Teensyduino (for example in /opt).

### Cloning the repository

```bash
git clone https://github.com/tum-phoenix/drive_teensy_libuavcan
cd drive_teensy_libuavcan
git checkout <correct branch>
git submodule update --init
```

Change paths in `cmake/teensy-arm.toolchain.cmake`:
* `TOOLCHAIN_ROOT`
* `TEENSY_CORES_ROOT`
* `ARDUINO_LIB_ROOT`

You may need to change software version in `cmake/teensy-arm.toolchain.cmake`:
* `ARDUINO_VERSION`
* `TEENSYDUINO_VERSION`

Device settings should be correct for Teensy 3.2.


### Building and installing

Cross-compile with CMake:

```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/teensy-arm.toolchain.cmake -DUAVCAN_PLATFORM=teensy32
make
```
