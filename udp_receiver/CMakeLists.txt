cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(udp_receiver)

add_executable(udp_receiver
    udp_receiver.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(udp_receiver
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(udp_receiver PRIVATE -Wall -Wextra)
else()
    add_compile_options(udp_receiver PRIVATE -W2)
endif()
