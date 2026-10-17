cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(takeoff_squareForever)

add_executable(takeoff_squareForever
    takeoff_squareForever.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(takeoff_squareForever
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(takeoff_squareForever PRIVATE -Wall -Wextra)
else()
    add_compile_options(takeoff_squareForever PRIVATE -W2)
endif()
