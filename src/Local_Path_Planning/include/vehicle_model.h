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
        double yaw_rate;
        double wheel_speed;
        std::tuple<std::vector<double>, std::vector<double>,std::vector<double>> DWA_pre_traj;
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

    };

    // 构造函数
    ElectricVehicleDynamicsModel();
    void update(double steering_angle, double desired_accel, double dt);
    VehicleState getState() const;
    void reset(const VehicleState& new_state);
    void plot_vehicle(const std::string& color="blue");
    void reset_for_planning_only(std::tuple<double,double,double>& new_state,
    std::optional<std::tuple<std::vector<double>, std::vector<double>,std::vector<double>>> DWA_p_t = std::nullopt);
    void plot_planning(std::vector<std::vector<double>> planning_data);

    void kinematics_update_State(double v,double w,double dt);
    std::vector<double> traj_x, traj_y;

private:
    VehicleState state_;
    VehicleParams params_;
    double steering_angle;//方向盘转角
    double desired_accel;//输入加速度

};

#endif