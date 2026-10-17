#ifndef SIMULATE_RC_INPUT_H
#define SIMULATE_RC_INPUT_H

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <iostream>
#include <thread>

using namespace mavsdk;
using namespace std;

// Function to create MAVLink RC Channels message
mavlink_message_t create_rc_channels_message();

// Function to simulate RC input
void simulate_rc_input(std::optional<std::shared_ptr<mavsdk::System>> system);

// Global variables for RC channel values
extern int chan1_raw_global;
extern int chan2_raw_global;
extern int chan3_raw_global;
extern int chan4_raw_global;
extern int chan5_raw_global;
extern int chan6_raw_global;
extern int chan7_raw_global;
extern int chan8_raw_global;

#endif // SIMULATE_RC_INPUT_H
