#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

namespace pf {
struct AirData {
    double calibrated=std::numeric_limits<double>::quiet_NaN();
    double air_time=-100,health_time=-100,wind_time=-100;
    bool sensor_healthy=false,wind_estimated=false;
    Eigen::Vector2d wind=Eigen::Vector2d::Zero();
    // VFR_HUD carries PX4 calibrated airspeed. No density conversion.
    double air_age(double now) const {return now-air_time;}
    bool air_valid(double now) const {
        return air_age(now)>=0 && air_age(now)<=1.0 && now-health_time>=0 && now-health_time<=3.0 &&
               sensor_healthy && std::isfinite(calibrated) && calibrated>=0;
    }
    bool wind_valid(double now) const {
        return wind_estimated && wind.allFinite() && now-wind_time>=0 && now-wind_time<=2.0;
    }
};
}
