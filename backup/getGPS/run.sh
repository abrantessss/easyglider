#!/bin/bash

# Run CMake to configure the project
cmake -Bbuild -H.

# Build the project with 4 parallel jobs
cmake --build build -j4

# Run the resulting executable with the specified UDP address
#build/getGPS udp://:14540
build/getGPS serial:///dev/ttyAMA0:115200

