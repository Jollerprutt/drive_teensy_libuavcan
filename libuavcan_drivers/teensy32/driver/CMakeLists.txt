set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(${CMAKE_SOURCE_DIR}/cmake/Teensy.cmake)


include_directories(
    ./include
    )

add_teensy_executable(uavcan_teensy32_driver
    ./src/can.cpp
    ./src/clock.cpp
    ./src/kinetis_flexcan.h
    )

add_dependencies(uavcan_teensy32_driver uavcan)
