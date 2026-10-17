cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(udp_emitter)

add_executable(udp_emitter
    udp_emitter.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(udp_emitter
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(udp_emitter PRIVATE -Wall -Wextra)
else()
    add_compile_options(udp_emitter PRIVATE -W2)
endif()
