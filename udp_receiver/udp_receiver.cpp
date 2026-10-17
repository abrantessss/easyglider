#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <array>
#include <cstring>
#include <stdexcept>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#define PORT 8080

// Struct to hold all telemetry data
struct TelemetryData {
    mavsdk::Telemetry::Position position;
    mavsdk::Telemetry::VelocityNed velocity;
    mavsdk::Telemetry::EulerAngle euler;
};

TelemetryData deserialize_from_binary(const std::vector<uint8_t>& buffer) {
    TelemetryData data;
    
    size_t offset = 0;

    // Extract position data
    std::memcpy(&data.position, buffer.data() + offset, sizeof(data.position));
    offset += sizeof(data.position);

    // Extract velocity data
    std::memcpy(&data.velocity, buffer.data() + offset, sizeof(data.velocity));
    offset += sizeof(data.velocity);

    // Extract euler data
    std::memcpy(&data.euler, buffer.data() + offset, sizeof(data.euler));

    return data;
}

int main() {
    const int BUFFER_SIZE = 4096;
    TelemetryData data;

    // Create a UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Bind the socket to the port
    sockaddr_in server_addr{};
    memset(&server_addr, 0, sizeof(server_addr)); 

    sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);
    memset(&client_addr, 0, addr_len); 

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Error: Unable to bind socket to port");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    std::cout << "Listening for UDP packets on port " << PORT << "..." << std::endl;

    try {
        while (true) {
            std::array<char, BUFFER_SIZE> buffer{};

            // Receive data from the socket
            ssize_t received_bytes = recvfrom(sockfd, buffer.data(), buffer.size(), MSG_WAITALL, (struct sockaddr*)&client_addr, &addr_len);
            if (received_bytes < 0) {
                std::cerr << "Error: Failed to receive data." << std::endl;
                continue;
            }
            else
                printf("number of input bytes: %ld\n", received_bytes);

            // Convert received data to std::vector<uint8_t>
            std::vector<uint8_t> received_data(buffer.begin(), buffer.begin() + received_bytes);

            // Deserialize the data
            data = deserialize_from_binary(received_data);

            std::cout << "Latitude: " << data.position.latitude_deg << " degrees\n";
            std::cout << "Longitude: " << data.position.longitude_deg << " degrees\n";
            std::cout << "Absolute Altitude: " << data.position.absolute_altitude_m << " meters\n";
            std::cout << "Relative Altitude: " << data.position.relative_altitude_m << " meters\n";

            std::cout << "Velocity NED (North): " << data.velocity.north_m_s << " m/s\n";
            std::cout << "Velocity NED (East): " << data.velocity.east_m_s << " m/s\n";
            std::cout << "Velocity NED (Down): " << data.velocity.down_m_s << " m/s\n";

            std::cout << "Yaw: " << data.euler.yaw_deg*57.2957795131 << " deg\n";

            printf("\n\n");
        
        }
    } catch (...) {
        std::cerr << "Server shutting down." << std::endl;
    }

    close(sockfd);
    return 0;
}
