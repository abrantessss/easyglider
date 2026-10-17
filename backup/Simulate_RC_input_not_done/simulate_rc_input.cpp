#include "simulate_rc_input.h"

int chan1_raw_global = 1500;
int chan2_raw_global = 1500;
int chan3_raw_global = 1500;
int chan4_raw_global = 1500;
int chan5_raw_global = 1500;
int chan6_raw_global = 1500;
int chan7_raw_global = 1500;
int chan8_raw_global = 1500;

mavlink_message_t create_rc_channels_message() {
    mavlink_message_t msg;
    mavlink_rc_channels_t rc_channels;

    rc_channels.time_boot_ms = 0; // Timestamp in milliseconds
    rc_channels.chancount = 8; // Number of channels
    rc_channels.chan1_raw = chan1_raw_global; // Example RC value for channel 1
    rc_channels.chan2_raw = chan2_raw_global; // Example RC value for channel 2
    rc_channels.chan3_raw = chan3_raw_global; // Example RC value for channel 3
    rc_channels.chan4_raw = chan4_raw_global; // Example RC value for channel 4
    rc_channels.chan5_raw = chan5_raw_global; // Example RC value for channel 5
    rc_channels.chan6_raw = chan6_raw_global; // Example RC value for channel 6
    rc_channels.chan7_raw = chan7_raw_global; // Example RC value for channel 7
    rc_channels.chan8_raw = chan8_raw_global; // Example RC value for channel 8
    rc_channels.rssi = 100; // Signal strength (0-100%)

    mavlink_msg_rc_channels_encode(1, 200, &msg, &rc_channels);
    return msg;
}

void simulate_rc_input(std::optional<std::shared_ptr<mavsdk::System>> system){

    // Instantiate plugins.
    auto mavlink_passthrough = MavlinkPassthrough(system.value());

    // Function to send RC_CHANNELS_OVERRIDE MAVLink message
    auto send_rc_channels_override = [&]() {
        std::function<mavlink_message_t(MavlinkAddress, uint8_t)> message_function = [](MavlinkAddress, uint8_t) {
            return create_rc_channels_message();
        };

        // Send the MAVLink message
        MavlinkPassthrough::Result result =  mavlink_passthrough.queue_message(message_function);
        if (result != MavlinkPassthrough::Result::Success) {
            std::cerr << "Failed to send message: " << result << std::endl;
        }
    };

    // Set up a callback to receive incoming MAVLink messages
    mavlink_passthrough.subscribe_message(MAVLINK_MSG_ID_RC_CHANNELS, [](const mavlink_message_t& message) {
        mavlink_rc_channels_t rc_channels;
        mavlink_msg_rc_channels_decode(&message, &rc_channels);

        std::cout << "Received RC_CHANNELS message:" << std::endl;
        std::cout << "Time boot ms: " << rc_channels.time_boot_ms << std::endl;
        std::cout << "Ch1 RAW: " << rc_channels.chan1_raw << std::endl;
        std::cout << "Ch2 RAW: " << rc_channels.chan2_raw << std::endl;
        std::cout << "Ch3 RAW: " << rc_channels.chan3_raw << std::endl;
        std::cout << "Ch4 RAW: " << rc_channels.chan4_raw << std::endl;
        std::cout << "Ch5 RAW: " << rc_channels.chan5_raw << std::endl;
        std::cout << "Ch6 RAW: " << rc_channels.chan6_raw << std::endl;
        std::cout << "Ch7 RAW: " << rc_channels.chan7_raw << std::endl;
        std::cout << "Ch8 RAW: " << rc_channels.chan8_raw << std::endl;
        std::cout << "RSSI: " << static_cast<int>(rc_channels.rssi) << std::endl;
    });

    char keyInput;  
    int* currentChannel = nullptr;
    while(true){
        std::cin>>keyInput;
        switch (keyInput){
        case '1':
            currentChannel = &chan1_raw_global;
            break;

        case '2':
            currentChannel = &chan2_raw_global;
            break;

        case '3':
            currentChannel = &chan3_raw_global;
            break;

        case '4':
            currentChannel = &chan4_raw_global;
            break;

        case '5':
            currentChannel = &chan5_raw_global;
            break;

        case '6':
            currentChannel = &chan6_raw_global;
            break;

        case '7':
            currentChannel = &chan7_raw_global;
            break;

        case '8':
            currentChannel = &chan8_raw_global;
            break;

        case 'w':
            *currentChannel-= 100;
            if(*currentChannel < 0)
                *currentChannel = 0;
            break;

        case 's':
            *currentChannel += 100;
            if(*currentChannel > 2000)
                *currentChannel = 2000;
            break;

        case 'q':
            return;
            
        default:
            std::cerr << "Invalid key press." << std::endl;
            break;
        }
        
        send_rc_channels_override();
        printf("chan1_raw_global: %d\n", chan1_raw_global);
        printf("chan2_raw_global: %d\n", chan2_raw_global);
        printf("chan3_raw_global: %d\n", chan3_raw_global);
        printf("chan4_raw_global: %d\n", chan4_raw_global);
        printf("chan5_raw_global: %d\n", chan5_raw_global);
        printf("chan6_raw_global: %d\n", chan6_raw_global);
        printf("chan7_raw_global: %d\n", chan7_raw_global);
        printf("chan8_raw_global: %d\n", chan8_raw_global);
    }

    return;
}
