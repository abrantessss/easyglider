cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(getGPS)

add_executable(getGPS
    getGPS.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(getGPS
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(getGPS PRIVATE -Wall -Wextra)
else()
    add_compile_options(getGPS PRIVATE -W2)
endif()
