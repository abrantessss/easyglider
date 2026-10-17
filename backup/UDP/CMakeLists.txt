cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(UDP)

# Add the current directory to the include directories (where json.hpp is located)
include_directories(${CMAKE_SOURCE_DIR})

# Create the executable
add_executable(UDP udp.cpp)

# Find and link MAVSDK package
find_package(MAVSDK REQUIRED)

# Link required libraries to the UDP target
target_link_libraries(UDP MAVSDK::mavsdk)

# Add compiler flags for non-MSVC and MSVC platforms
if(NOT MSVC)
    add_compile_options(UDP PRIVATE -Wall -Wextra)
else()
    add_compile_options(UDP PRIVATE -W2)
endif()
