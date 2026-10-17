#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/info/info.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <memory.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <errno.h>
#include <stdlib.h>
#include <iostream>


using namespace mavsdk;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define DEST_IP "192.168.50.196"
#define PORT 8080

struct Waypoint {
    Offboard::PositionNedYaw position; // Position data
    double radius;                     // Radius for arrival condition

    // Constructor for Waypoint
    Waypoint(Offboard::PositionNedYaw pos, double rad) : position(pos), radius(rad) {}
};

// Struct to hold all telemetry data
struct TelemetryData {
    Telemetry::Position position;
    Telemetry::VelocityNed velocity;
    //Telemetry::Imu imu;
    Telemetry::EulerAngle euler;
};

// Class to manage and store telemetry data
class TelemetryDataCollector {
private:
    TelemetryData data;
    std::mutex data_mutex; // Mutex to ensure thread-safe access to data
public:
    // Start subscribing to telemetry data and storing it
    void start_collecting(Telemetry &telemetry) {

        telemetry.subscribe_position([this](Telemetry::Position position) {
            std::lock_guard<std::mutex> lock(data_mutex);
            data.position = position;
        });

        // Subscribe to velocity updates (in NED frame)
        telemetry.subscribe_velocity_ned([this](Telemetry::VelocityNed velocity) {
            std::lock_guard<std::mutex> lock(data_mutex);
            data.velocity = velocity;
        });

        /*telemetry.subscribe_attitude_euler([this](mavsdk::Telemetry::Imu imu) {
            std::lock_guard<std::mutex> lock(data_mutex);
            data.imu = imu;
        });*/

        telemetry.subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle euler) {
            std::lock_guard<std::mutex> lock(data_mutex);
            data.euler = euler;
        });
    }

    // Function to safely get the latest telemetry data
    TelemetryData get_latest_data() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return data;
    }
};

void usage(const std::string& bin_name){
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udpin://0.0.0.0:14540\n";
}


int resolvehelper(const char* hostname, int family, const char* service, sockaddr_storage* pAddr){
    int result;
    addrinfo* result_list = NULL;
    addrinfo hints = {};
    hints.ai_family = family;
    hints.ai_socktype = SOCK_DGRAM; // without this flag, getaddrinfo will return 3x the number of addresses (one for each socket type).
    result = getaddrinfo(hostname, service, &hints, &result_list);
    if (result == 0)
    {
        //ASSERT(result_list->ai_addrlen <= sizeof(sockaddr_in));
        memcpy(pAddr, result_list->ai_addr, result_list->ai_addrlen);
        freeaddrinfo(result_list);
    }

    return result;
}

int main(int argc, char** argv){
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    std::optional<std::shared_ptr<mavsdk::System>> system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n"; 
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = Telemetry{system.value()};
    auto action = Action{system.value()};
    auto offboard = Offboard{system.value()};

    TelemetryDataCollector data_collector;

    // We want to listen to the altitude of the drone at 1 Hz.
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return 1;
    }

    // Check until vehicle is ready to arm
    bool is_gyrometer_calibration_ok;
    bool is_accelerometer_calibration_ok;
    bool is_magnetometer_calibration_ok;
    bool is_local_position_ok;
    bool is_global_position_ok;
    bool is_home_position_ok;
    bool is_armable;
    while (!telemetry.health_all_ok()) {
        std::cout << "Vehicle not ready to arm. Checking individual systems:\n";

        is_gyrometer_calibration_ok = telemetry.health().is_gyrometer_calibration_ok;
        is_accelerometer_calibration_ok = telemetry.health().is_accelerometer_calibration_ok;
        is_magnetometer_calibration_ok = telemetry.health().is_magnetometer_calibration_ok;
        is_local_position_ok = telemetry.health().is_local_position_ok;
        is_global_position_ok = telemetry.health().is_global_position_ok;
        is_home_position_ok = telemetry.health().is_home_position_ok;
        is_armable = telemetry.health().is_armable;

        std::cout << "Gyrometer calibration: " << (is_gyrometer_calibration_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Accelerometer calibration: " << (is_accelerometer_calibration_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Magnetometer calibration: " << (is_magnetometer_calibration_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Local position: " << (is_local_position_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Global position: " << (is_global_position_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Home position: " << (is_home_position_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Armable: " << (is_armable ? "YES" : "NO") << '\n';

   	std::this_thread::sleep_for(1s);
    }
    std::cout << "Vehicle ready to arm.\n";
	
    //initialize the UDP port
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);                // Port number
    servaddr.sin_addr.s_addr = inet_addr(DEST_IP);  // Receiver IP
    
    // Start collecting telemetry data
    data_collector.start_collecting(telemetry);

    // Set the position using the waypoint's position member
    TelemetryData latest_data;
    bool arrived = false;
    char* msg;
    size_t msg_length;

    while(true){
        latest_data = data_collector.get_latest_data();
        msg_length = sizeof(latest_data);
        sendto(sock, &latest_data, msg_length, 0, (struct sockaddr*)&servaddr, sizeof(servaddr));
        std::cout << "Sent Data\n";

        sleep_for(seconds(1));
    }
    return EXIT_SUCCESS;
}
