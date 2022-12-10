cmake_minimum_required(VERSION 3.23)
project(single_chan_pkt_fwd C)

set(CMAKE_C_STANDARD 11)

# Compile
add_executable(pkt_fwd main.c base64.c)
target_link_libraries(pkt_fwd ${LIBRARIES} -lwiringPi  -lwiringPiDev)
