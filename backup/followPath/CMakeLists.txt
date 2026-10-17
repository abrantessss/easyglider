cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(followPath)

add_executable(followPath
    followPath.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(followPath
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(followPath PRIVATE -Wall -Wextra)
else()
    add_compile_options(followPath PRIVATE -W2)
endif()
