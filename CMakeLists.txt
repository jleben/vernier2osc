cmake_minimum_required(VERSION 2.8)

project(vernier2osc)

set( src
main.cpp
)

include_directories( /usr/include/NGIO-1.0.0 )
include_directories( /usr/include/lo )

add_executable(vernier2osc ${src})

target_link_libraries(vernier2osc NGIO usb-1.0 lo)

