#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H


#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <string>
#include <optional>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// 自定义clamp函数

template <typename T>

T clamp(T value, T min_val, T max_val) {

    return std::max(min_val, std::min(value, max_val));

}

class ElectricVehicleDynamicsModel {
public:
    // 车辆状态结构体
    struct VehicleState {
        double x;
        double y;
        double yaw;
        double beta;//侧滑角
        double vx;
        double vy;
        double vp;
        double yaw_rate;
        double wheel_speed;
        double steering_angle;
        std::vector<std::vector<double>> DWA_pre_traj;
        VehicleState():
        x(0.0),y(0.0),yaw(0.0),beta(0.0),vx(0.0),vy(0.0),vp(0.0),yaw_rate(0.0),wheel_speed(0.0),steering_angle(0.0),DWA_pre_traj({}){}
    };

    // 车辆参数结构体
    struct VehicleParams {
        double mass;
        double lf;
        double lr;
        double iz;
        double cf;
        double cr;
        double drag_coeff;
        double rr_coeff;
        double max_steer;
        double max_motor_torque;
        double max_brake_torque;
        double wheel_radius;
        double gear_ratio;
        double motor_efficiency;
        double wheel_width;
        double track_width;
        double max_accel;
        double max_decel;
        double Cd;      // 空气阻力系数
        double damping; // 横摆角速度阻尼
        VehicleParams() :
            mass(1200.0),
            lf(1.1),
            lr(1.4),
            iz(2000.0),
            cf(70000.0),
            cr(65000.0),
            drag_coeff(0.32),
            rr_coeff(0.016),
            max_steer(M_PI / 4),
            max_motor_torque(250.0),
            max_brake_torque(800.0),
            wheel_radius(0.3),
            gear_ratio(9.0),
            motor_efficiency(0.92),
            wheel_width(0.2),
            track_width(1.5),
            max_accel(2.5),
            max_decel(7.0),
            Cd(0.3),
            damping(0.95) {}
    };

    // 构造函数
    ElectricVehicleDynamicsModel();

    void update(double steering_angle, double desired_accel, double dt,bool model=true);

    VehicleState getState() const;

    void reset(const VehicleState& new_state);

    void plot_vehicle(const std::string& color="blue");

    void reset_for_planning_only(std::tuple<double,double,double>& new_state,

    std::vector<std::vector<double>> DWA_p_t = {});

    void plot_planning(std::vector<std::vector<double>> planning_data);

    static ElectricVehicleDynamicsModel::VehicleState kinematics(double steering_angle,double desired_accel,double dt,VehicleState current_state);

    static ElectricVehicleDynamicsModel::VehicleState dynamic(double steering_angle,double desired_accel,double dt,VehicleState current_state);

    std::vector<double> traj_x, traj_y;

    double get_vehicle_radias();

private:
    VehicleState state_;
    VehicleParams params_;
};

#endif