#include <chrono>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/info/info.h>
#include <thread>
#include <sstream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

#define DEST_IP "192.168.50.196"
#define PORT 8080

struct SystemInfoData {
    std::string vendor_name;
    std::string product_name;
    std::string flight_sw_version;
    std::string hardware_uid;
};

std::string format_info(const SystemInfoData& info_data) {
    std::ostringstream oss;
    oss << "Vendor: " << info_data.vendor_name
        << ", Product: " << info_data.product_name
        << ", Flight SW: " << info_data.flight_sw_version
        << ", HW UID: " << info_data.hardware_uid;
    return oss.str();
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <connection_url>\n";
        return 1;
    }
    
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system.\n";
        return 1;
    }

    // Create plugin AFTER system is verified
    auto info = std::make_unique<Info>(system.value());

    auto version = info->get_version().second;
    auto product = info->get_product().second;
    auto id = info->get_identification().second;

    SystemInfoData info_data{
        product.vendor_name,
        product.product_name,
        std::to_string(version.flight_sw_major) + "." +
        std::to_string(version.flight_sw_minor) + "." +
        std::to_string(version.flight_sw_patch),
        id.hardware_uid
    };

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket creation failed");
        return 1;
    }

    sockaddr_in servaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = inet_addr(DEST_IP);

    std::string msg = format_info(info_data);

    while (true) {
        sendto(sock, msg.c_str(), msg.length(), 0,
               (const struct sockaddr*)&servaddr, sizeof(servaddr));
        std::cout << "Sent: " << msg << std::endl;
        sleep_for(seconds(1));
    }

    return 0;
}
