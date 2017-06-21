include_directories(
    ./include
    )

add_library(uavcan_nxpk20_driver STATIC
    ./src/can.cpp
    ./src/clock.cpp
    ./src/kinetis_flexcan.h
    )

add_dependencies(uavcan_nxpk20_driver uavcan)

install(DIRECTORY include/uavcan_nxpk20 DESTINATION include)
install(TARGETS uavcan_nxpk20_driver DESTINATION lib)

# vim: set et ft=cmake fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :
