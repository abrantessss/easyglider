cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(getSysInfo)

add_executable(getSysInfo
    getSysInfo.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(getSysInfo
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(getSysInfo PRIVATE -Wall -Wextra)
else()
    add_compile_options(getSysInfo PRIVATE -W2)
endif()
