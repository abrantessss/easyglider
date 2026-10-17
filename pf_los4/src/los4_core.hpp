#pragma once
#include "los4_config.hpp"
#include <Eigen/Geometry>
#include <algorithm>

namespace pf {
struct PathPoint { Eigen::Vector3d p,d1,d2; };
inline PathPoint path_point(const Config& c,double gamma) {
    if(c.path=="line") return {c.p0+gamma*(c.p1-c.p0),c.p1-c.p0,Eigen::Vector3d::Zero()};
    const double a=c.path=="circle" ? c.radius : c.scale, s=std::sin(gamma), t=std::cos(gamma);
    if(c.path=="circle") return {c.center+Eigen::Vector3d(a*t,a*s,0),{-a*s,a*t,0},{-a*t,-a*s,0}};
    return {c.center+Eigen::Vector3d(a*t,a*s*t,0),{-a*s,a*std::cos(2*gamma),0},
            {-a*t,-2*a*std::sin(2*gamma),0}};
}
struct Output {
    double gamma=0,gamma_dot=0,throttle=0,roll=0,pitch=0,heading=0,ep_norm=0;
    Eigen::Vector3d pd=Eigen::Vector3d::Zero(), derivative=Eigen::Vector3d::Zero(),
                    ep=Eigen::Vector3d::Zero(), body_rates=Eigen::Vector3d::Zero();
};
class Los4 {
    Config c;
    double gamma=0,zref=0,ei=0,bi=0,previous_roll=0,previous_pitch=0;
    bool initialized=false;
    static double pi(double e,double dt,double& integral,double kp,double ki,double lo,double hi) {
        const double candidate=integral+e*dt, raw=kp*e+ki*candidate;
        if((raw>=lo && raw<=hi) || (raw>hi && e<0) || (raw<lo && e>0)) integral=candidate;
        return std::clamp(raw,lo,hi);
    }
public:
    explicit Los4(Config config):c(std::move(config)),gamma(c.gamma0) {}
    void reset() { gamma=c.gamma0; zref=ei=bi=previous_roll=previous_pitch=0; initialized=false; }
    Output test_step(double dt) {
        if(!std::isfinite(dt) || dt<=0) throw std::runtime_error("Invalid timestep");
        Output out;const auto point=path_point(c,gamma);
        out.gamma=gamma;out.gamma_dot=1;out.pd=point.p;out.derivative=point.d1;gamma+=dt;
        return out;
    }
    Output step(double dt,const Eigen::Vector3d& p,const Eigen::Vector3d& v,const Eigen::Vector3d& eta) {
        if(!std::isfinite(dt) || dt<=0 || dt>0.1 || !p.allFinite() || !v.allFinite() || !eta.allFinite())
            throw std::runtime_error("Invalid or nonfinite control input/timestep");
        if(v.head<2>().norm()<c.min_speed) throw std::runtime_error("Horizontal speed below controller minimum");
        Output out; const auto point=path_point(c,gamma);
        out.gamma=gamma;out.pd=point.p;out.derivative=point.d1;out.ep=p-point.p;out.ep_norm=out.ep.norm();
        const auto tangent=(point.d1/point.d1.norm()).eval();
        const Eigen::Vector3d aux=-c.k1*(Eigen::Matrix3d::Identity()-tangent*tangent.transpose())*out.ep+c.k2*tangent;
        if(aux.norm()<1e-6 || point.d1.norm()<1e-6) throw std::runtime_error("Degenerate guidance");
        const Eigen::Vector3d h=aux/aux.norm();
        out.gamma_dot=c.speed*(c.k1*tangent.dot(out.ep)+c.k2)/(aux.norm()*point.d1.norm());
        const double hdot=-c.speed*h.z();
        if(!initialized) zref=-p.z();
        zref+=hdot*dt;
        const double kinetic=0.5*c.mass*(c.speed*c.speed-v.squaredNorm());
        const double potential=c.mass*9.81*(-p.z()-zref), energy=potential+kinetic,balance=kinetic-potential;
        const double candidate=ei+energy*dt;
        const double raw=(c.kiE*candidate+(c.mass*9.81*hdot-c.kpE*energy)/c.speed)/c.tmax;
        const double lo=c.throttle_model=="quadratic" ? c.throttle_min*c.throttle_min : c.throttle_min;
        const double hi=c.throttle_model=="quadratic" ? c.throttle_max*c.throttle_max : c.throttle_max;
        if((raw>=lo && raw<=hi) || (raw>hi && energy<0) || (raw<lo && energy>0)) ei=candidate;
        out.throttle=std::clamp(raw,lo,hi);
        if(c.throttle_model=="quadratic") out.throttle=std::sqrt(out.throttle);
        out.pitch=std::clamp(hdot/c.speed+pi(balance,dt,bi,c.kpB,c.kiB,c.pitch_min,c.pitch_max),c.pitch_min,c.pitch_max);
        const double hnorm=h.head<2>().norm();
        if(hnorm<1e-6) throw std::runtime_error("Vertical guidance singularity");
        const double ez=(v.x()*h.y()-v.y()*h.x())/(v.head<2>().norm()*hnorm);
        const double curvature=(point.d1.x()*point.d2.y()-point.d1.y()*point.d2.x())/
                               std::pow(point.d1.head<2>().squaredNorm(),1.5);
        out.roll=std::atan(std::clamp(c.speed*c.speed*curvature+c.ka*ez,c.accel_min,c.accel_max)/9.81);
        if(!initialized) {previous_roll=out.roll;previous_pitch=out.pitch;initialized=true;}
        const double rd=(out.roll-previous_roll)/dt+c.kphi*(out.roll-eta.x());
        const double pd=(out.pitch-previous_pitch)/dt+c.ktheta*(out.pitch-eta.y());
        const double yd=9.81*std::tan(out.roll)/(c.speed*std::cos(out.pitch));
        previous_roll=out.roll;previous_pitch=out.pitch;
        out.body_rates={rd-yd*std::sin(eta.y()),pd*std::cos(eta.x())+yd*std::sin(eta.x())*std::cos(eta.y()),
                        -pd*std::sin(eta.x())+yd*std::cos(eta.x())*std::cos(eta.y())};
        out.heading=std::atan2(h.y(),h.x());
        if(!out.body_rates.allFinite() || !std::isfinite(out.throttle) || !std::isfinite(out.gamma_dot))
            throw std::runtime_error("Nonfinite controller output");
        for(int i=0;i<3;++i) out.body_rates[i]=std::clamp(out.body_rates[i],-c.rate_limit,c.rate_limit);
        gamma+=out.gamma_dot*dt;
        if(!std::isfinite(gamma)) throw std::runtime_error("Nonfinite gamma");
        return out;
    }
};
}
