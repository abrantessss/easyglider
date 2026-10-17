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
#include <ifaddrs.h>
#include <net/if.h>
#include <errno.h>
#include <stdlib.h>
#include "json.hpp"

using namespace mavsdk;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define DEST_IP "192.168.50.195"

// Struct to hold all telemetry data
struct TelemetryData {
    std::optional<Telemetry::Position> position;
    std::optional<Telemetry::VelocityNed> velocity;
    std::optional<Telemetry::Imu> imu;
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

        // Subscribe to IMU updates (including accelerometer data)
        telemetry.subscribe_imu([this](Telemetry::Imu imu) {
            std::lock_guard<std::mutex> lock(data_mutex);
            data.imu = imu;
        });
    }

    // Function to safely get the latest telemetry data
    TelemetryData get_latest_data() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return data;
    }

    nlohmann::json to_json() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return nlohmann::json{
            {"position", {
                {"latitude_deg", data.position->latitude_deg},
                {"longitude_deg", data.position->longitude_deg},
                {"absolute_altitude_m", data.position->absolute_altitude_m},
                {"relative_altitude_m", data.position->relative_altitude_m}
            }},
            {"velocity", {
                {"north_m_s", data.velocity->north_m_s},
                {"east_m_s", data.velocity->east_m_s},
                {"down_m_s", data.velocity->down_m_s}
            }},
            {"imu", {
                {"acceleration_frd", {
                    {"forward_m_s2", data.imu->acceleration_frd.forward_m_s2},
                    {"right_m_s2", data.imu->acceleration_frd.right_m_s2},
                    {"down_m_s2", data.imu->acceleration_frd.down_m_s2}
                }}
            }}
        };
    }

    // Send JSON data
    ssize_t send_JSON_data(int sock, sockaddr_in &addrDest) {
        nlohmann::json telemetry_json = to_json();
        std::string msg = telemetry_json.dump();  // Convert JSON to string
        return sendto(sock, msg.c_str(), msg.size(), 0, (sockaddr*)&addrDest, sizeof(addrDest));
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
    while (telemetry.health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm\n";
        sleep_for(seconds(1));
    }

    //initialize the UDP port
    int result = 0;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addrListen = {}; // zero-int, sin_port is 0, which picks a random port for bind.
    addrListen.sin_family = AF_INET;
    result = bind(sock, (sockaddr*)&addrListen, sizeof(addrListen));
    if (result == -1){
       int lasterror = errno;
       std::cout << "error: " << lasterror;
       exit(1);
    }

    sockaddr_storage addrDest = {};
    result = resolvehelper(DEST_IP, AF_INET, "9000", &addrDest);
    if (result != 0){
       int lasterror = errno;
       std::cout << "error: " << lasterror;
       exit(1);
    }
    
    // Start collecting telemetry data
    data_collector.start_collecting(telemetry);


    // Set the position using the waypoint's position member
    TelemetryData latest_data;
    bool arrived = false;
    char* msg;
    size_t msg_length;

    while(true){
        latest_data = data_collector.get_latest_data();
        msg = reinterpret_cast<char*>(&latest_data); 
        msg_length = sizeof(latest_data);
        result = data_collector.send_JSON_data(sock, *(reinterpret_cast<sockaddr_in*>(&addrDest)));
        if (latest_data.position) {
            std::cout << "Latitude: " << latest_data.position->latitude_deg << " degrees\n";
            std::cout << "Longitude: " << latest_data.position->longitude_deg << " degrees\n";
            std::cout << "Absolute Altitude: " << latest_data.position->absolute_altitude_m << " meters\n";
            std::cout << "Relative Altitude: " << latest_data.position->relative_altitude_m << " meters\n";
        }

        if (latest_data.velocity) {
            std::cout << "Velocity NED (North): " << latest_data.velocity->north_m_s << " m/s\n";
            std::cout << "Velocity NED (East): " << latest_data.velocity->east_m_s << " m/s\n";
            std::cout << "Velocity NED (Down): " << latest_data.velocity->down_m_s << " m/s\n";
        }

        if (latest_data.imu) {
            std::cout << "IMU Accelerometer (X): " << latest_data.imu->acceleration_frd.forward_m_s2 << " m/s^2\n";
            std::cout << "IMU Accelerometer (Y): " << latest_data.imu->acceleration_frd.right_m_s2 << " m/s^2\n";
            std::cout << "IMU Accelerometer (Z): " << latest_data.imu->acceleration_frd.down_m_s2 << " m/s^2\n";
        }
        sleep_for(seconds(1));
    }

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished...\n";

    return 0;
}
