#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

namespace pf {
struct AirData {
    double calibrated=std::numeric_limits<double>::quiet_NaN();
    double pressure_pa=0,temperature_c=0;
    double air_time=-100,pressure_time=-100,health_time=-100,wind_time=-100;
    bool sensor_healthy=false,wind_estimated=false;
    Eigen::Vector2d wind=Eigen::Vector2d::Zero();
    // PX4 low-Mach CAS/EAS -> TAS density correction. Ambient temperature
    // comes from HIGHRES_IMU, not the IMU's internal sensor temperature.
    double true_airspeed() const {
        if(!std::isfinite(calibrated) || calibrated<0 || !std::isfinite(pressure_pa) ||
           pressure_pa<=0 || !std::isfinite(temperature_c) || temperature_c<=-273.15)
            return std::numeric_limits<double>::quiet_NaN();
        const double density=pressure_pa/(287.05*(temperature_c+273.15));
        return calibrated*std::sqrt(1.225/density);
    }
    double air_age(double now) const {return std::max(now-air_time,now-pressure_time);}
    bool air_valid(double now) const {
        return air_age(now)>=0 && air_age(now)<=1.0 && now-health_time>=0 && now-health_time<=3.0 &&
               sensor_healthy && std::isfinite(true_airspeed()) && true_airspeed()>=0;
    }
    bool wind_valid(double now) const {
        return wind_estimated && wind.allFinite() && now-wind_time>=0 && now-wind_time<=2.0;
    }
};
}
