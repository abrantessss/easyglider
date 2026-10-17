#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
//#include "simulate_rc_input.h"

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

struct Waypoint {
    Offboard::PositionNedYaw position; // Position data
    double radius;                     // Radius for arrival condition

    // Constructor for Waypoint
    Waypoint(Offboard::PositionNedYaw pos, double rad) : position(pos), radius(rad) {}
};

void usage(const std::string& bin_name){
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udpin://0.0.0.0:14540\n";
}

double calculate_distance(double current_north, double current_east, double obj_north, double obj_east) {
    return sqrt(pow((current_north-obj_north), 2) + pow((current_east-obj_east), 2));
}

void navigate_waypoints(Offboard& offboard, Telemetry& telemetry, const std::vector<Waypoint>& waypoints) {
    if (waypoints.empty()) return;

    std::cout << "Going to waypoint 0\n";
    
    // Set the position using the waypoint's position member
    offboard.set_position_ned(waypoints[0].position);
    
    auto offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
        return;
    }

    while (true) {
        double distance_to_destiny = calculate_distance(
            telemetry.position_velocity_ned().position.north_m,
            telemetry.position_velocity_ned().position.east_m,
            waypoints[0].position.north_m,   
            waypoints[0].position.east_m    
        );

        if (distance_to_destiny <= waypoints[0].radius) {  // Use the radius from waypoint
            std::cout << "*Arrived at waypoint 0*\n";
            break;
        } else {
            std::cout << "Not yet at waypoint 0, distance: " << distance_to_destiny << " meters\n";
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));  // Correct sleep usage
    }

    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "Going to waypoint " << i << "\n";
        offboard.set_position_ned(waypoints[i].position);

        while (true) {
            double distance_to_destiny = calculate_distance(
                telemetry.position_velocity_ned().position.north_m,
                telemetry.position_velocity_ned().position.east_m,
                waypoints[i].position.north_m,
                waypoints[i].position.east_m
            );

            if (distance_to_destiny <= waypoints[i].radius) {
                std::cout << "*Arrived at waypoint " << i << "*\n";
                break;
            } else {
                std::cout << "Not yet at waypoint " << i << ", distance: " << distance_to_destiny << " meters\n";
            }
            sleep_for(seconds(1));
        }
    }

    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return;
    }
    std::cout << "Offboard stopped\n";
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

    // We want to listen to the altitude of the drone at 1 Hz.
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return 1;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    /*telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
    });*/

    // Subscribe to flight mode changes
    /*telemetry.subscribe_flight_mode([&](Telemetry::FlightMode flight_mode) {
        std::cout << "Flight mode: " << flight_mode << std::endl;
    });*/

    // Check until vehicle is ready to arm
    while (telemetry.health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm\n";
        sleep_for(seconds(1));
    }

    /*while(telemetry.flight_mode() != Telemetry::FlightMode::Offboard){
        std::cout << "Waiting for offboard flight mode\n";
    }*/

    // Arm vehicle
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }

    // Take off to 6 meters
    action.set_takeoff_altitude(6.0);
    std::cout << "Taking off to 6 meters...\n";
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    // Wait until the drone reaches 6 meters
    while (telemetry.position().relative_altitude_m < 5.0) {
        std::cout << "Ascending to 6 meters...\n";
        sleep_for(seconds(1));
    }

    const double radius = 10.0;
    //NED coordinate use negative height values
    const double height = -10.0;
    const double square = 200.0;
    std::vector<Waypoint> waypoints_vec;

    // Create a position using Offboard::PositionNedYaw
    Offboard::PositionNedYaw position{0, 0, height, 0};  // North=0, East=0, Down=height, Yaw=0
    
    // Add the waypoint to the vector
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {square, 0, height, 0}, radius));
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {square, square, height, 90}, radius));
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {0, square, height, 180}, radius));
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {0, 0, height, 270}, radius));
    navigate_waypoints(offboard, telemetry, waypoints_vec);

    //land
    std::cout << "Landing...\n";
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed: " << land_result << '\n';
        return 1;
    }

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }
    std::cout << "Landed!\n";

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished...\n";

    return 0;
}
