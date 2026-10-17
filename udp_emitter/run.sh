#!/bin/bash

# Run CMake to configure the project
cmake -Bbuild -H.

# Build the project with 4 parallel jobs
cmake --build build -j4

# Run the resulting executable with the specified UDP address
#build/udp_emitter udp://:14540
build/udp_emitter serial:///dev/ttyAMA0:115200
