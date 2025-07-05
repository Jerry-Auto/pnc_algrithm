#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <string>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// 自定义clamp函数
template <typename T>
T clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

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
};

// 车辆状态结构体
struct VehicleState {
    double x;
    double y;
    double yaw;
    double vx;
    double vy;
    double yaw_rate;
    double wheel_speed;
};

// 电动车辆动力学模型
class ElectricVehicleDynamicsModel {
public:
    ElectricVehicleDynamicsModel(const VehicleParams& params) : params_(params) {
        state_ = {0, 0, 0, 0, 0, 0, 0};
    }

    void update(double steering_angle, double motor_torque, double brake_torque, double dt) {
        // 限制输入范围
        steering_angle = clamp(steering_angle, -params_.max_steer, params_.max_steer);
        motor_torque = clamp(motor_torque, -params_.max_motor_torque, params_.max_motor_torque);
        brake_torque = clamp(brake_torque, 0.0, params_.max_brake_torque);

        // 计算轮胎力
        double alpha_f = steering_angle - std::atan2(state_.vy + params_.lf * state_.yaw_rate, 
                                                    std::max(state_.vx, 0.1));
        double alpha_r = -std::atan2(state_.vy - params_.lr * state_.yaw_rate, 
                                   std::max(state_.vx, 0.1));

        double F_fy = params_.cf * alpha_f;
        double F_ry = params_.cr * alpha_r;

        // 计算电机产生的车轮扭矩
        double wheel_torque = (motor_torque * params_.gear_ratio * params_.motor_efficiency) - 
                             (brake_torque * (state_.wheel_speed < 0 ? -1 : 1));

        double F_drive = wheel_torque / params_.wheel_radius;
        double max_f = 5000.0;
        F_drive = clamp(F_drive, -max_f, max_f);

        // 计算阻力
        double F_air = params_.drag_coeff * state_.vx * std::abs(state_.vx);
        double F_rr = params_.rr_coeff * state_.vx;
        
        // 总纵向力
        double F_total_x = F_drive - F_air - F_rr;

        // 动力学方程
        double x_dot = state_.vx * std::cos(state_.yaw) - state_.vy * std::sin(state_.yaw);
        double y_dot = state_.vx * std::sin(state_.yaw) + state_.vy * std::cos(state_.yaw);
        
        double vx_dot = (F_total_x + state_.vy * state_.yaw_rate) / params_.mass;
        double vy_dot = (F_fy + F_ry - state_.vx * state_.yaw_rate) / params_.mass;
        double yaw_rate_dot = (params_.lf * F_fy - params_.lr * F_ry) / params_.iz;
        double wheel_speed_dot = F_drive * params_.wheel_radius / params_.mass;

        // 欧拉积分
        state_.x += x_dot * dt;
        state_.y += y_dot * dt;
        state_.yaw += state_.yaw_rate * dt;
        state_.vx += vx_dot * dt;
        state_.vy += vy_dot * dt;
        state_.yaw_rate += yaw_rate_dot * dt;
        state_.wheel_speed += wheel_speed_dot * dt;

        if (state_.vx < 0) {
            state_.vx = 0;
            state_.wheel_speed = 0;
        }
    }

    VehicleState getState() const {
        return state_;
    }

    void reset(const VehicleState& new_state) {
        state_ = new_state;
    }

private:
    VehicleParams params_;
    VehicleState state_;
};

// 绘制车辆函数
void plot_vehicle(const VehicleState& state, const VehicleParams& params, 
                 double steer_angle, const std::string& color="blue") {
    // 车辆轮廓点 (局部坐标系)
    std::vector<double> vehicle_x = {-params.lr, params.lf, params.lf, -params.lr, -params.lr};
    std::vector<double> vehicle_y = {-1.0, -1.0, 1.0, 1.0, -1.0};
    
    // 旋转和平移到全局坐标系
    std::vector<double> global_x, global_y;
    for (size_t i = 0; i < vehicle_x.size(); ++i) {
        double x_local = vehicle_x[i];
        double y_local = vehicle_y[i];
        
        // 旋转
        double x_rot = x_local * cos(state.yaw) - y_local * sin(state.yaw);
        double y_rot = x_local * sin(state.yaw) + y_local * cos(state.yaw);
        
        // 平移
        global_x.push_back(state.x + x_rot);
        global_y.push_back(state.y + y_rot);
    }
    
    // 绘制车辆
    std::map<std::string, std::string> keywords1;
    keywords1["color"] = color;
    keywords1["linewidth"] = "2";
    plt::plot(global_x, global_y, keywords1);
    
    // 绘制前轮转向
    double wheel_width = 0.2;
    double wheel_length = 0.6;
    
    // 前轮
    std::vector<double> front_wheel_x = {params.lf - wheel_length/2, params.lf + wheel_length/2, 
                                        params.lf + wheel_length/2, params.lf - wheel_length/2, 
                                        params.lf - wheel_length/2};
    std::vector<double> front_wheel_y = {-wheel_width/2, -wheel_width/2, 
                                        wheel_width/2, wheel_width/2, -wheel_width/2};
    
    std::vector<double> front_wheel_global_x, front_wheel_global_y;
    for (size_t i = 0; i < front_wheel_x.size(); ++i) {
        double x_local = front_wheel_x[i];
        double y_local = front_wheel_y[i];
        
        // 考虑转向角
        double x_rot = x_local * cos(state.yaw + steer_angle) - y_local * sin(state.yaw + steer_angle);
        double y_rot = x_local * sin(state.yaw + steer_angle) + y_local * cos(state.yaw + steer_angle);
        
        front_wheel_global_x.push_back(state.x + x_rot);
        front_wheel_global_y.push_back(state.y + y_rot);
    }
    
    std::map<std::string, std::string> keywords2;
    keywords2["color"] = "black";
    keywords2["linewidth"] = "1.5";
    plt::plot(front_wheel_global_x, front_wheel_global_y, keywords2);
    
    // 后轮
    std::vector<double> rear_wheel_x = {-params.lr - wheel_length/2, -params.lr + wheel_length/2, 
                                       -params.lr + wheel_length/2, -params.lr - wheel_length/2, 
                                       -params.lr - wheel_length/2};
    std::vector<double> rear_wheel_y = {-wheel_width/2, -wheel_width/2, 
                                       wheel_width/2, wheel_width/2, -wheel_width/2};
    
    std::vector<double> rear_wheel_global_x, rear_wheel_global_y;
    for (size_t i = 0; i < rear_wheel_x.size(); ++i) {
        double x_local = rear_wheel_x[i];
        double y_local = rear_wheel_y[i];
        
        double x_rot = x_local * cos(state.yaw) - y_local * sin(state.yaw);
        double y_rot = x_local * sin(state.yaw) + y_local * cos(state.yaw);
        
        rear_wheel_global_x.push_back(state.x + x_rot);
        rear_wheel_global_y.push_back(state.y + y_rot);
    }
    plt::plot(rear_wheel_global_x, rear_wheel_global_y, keywords2);
    
    // 绘制速度箭头（替换 quiver）

    // 1. 将局部速度 (vx, vy) 旋转到全局坐标系

    double vx_global = state.vx * cos(state.yaw) - state.vy * sin(state.yaw);

    double vy_global = state.vx * sin(state.yaw) + state.vy * cos(state.yaw);

 

    // 2. 计算箭头终点（缩放避免过长）

    double arrow_scale = 0.5;  // 缩放因子

    double end_x = state.x + vx_global * arrow_scale;

    double end_y = state.y + vy_global * arrow_scale;

 

    // 3. 绘制箭头（全局坐标系）

    plt::arrow(

        state.x, state.y,       // 起点（车辆重心）

        end_x, end_y,           // 终点（全局坐标系速度方向）

        "red", "red",           // 颜色

        0.3, 0.4                // 箭头大小

    );
}

int main() {
    // 设置车辆参数
    VehicleParams params;
    params.mass = 1500.0;
    params.lf = 1.2;
    params.lr = 1.5;
    params.iz = 2500.0;
    params.cf = 80000.0;
    params.cr = 80000.0;
    params.drag_coeff = 0.1;
    params.rr_coeff = 30.0;
    params.max_steer = M_PI / 4;
    params.max_motor_torque = 300.0;
    params.max_brake_torque = 1000.0;
    params.wheel_radius = 0.3;
    params.gear_ratio = 10.0;
    params.motor_efficiency = 0.9;

    // 创建车辆模型
    ElectricVehicleDynamicsModel vehicle(params);

    // 模拟参数
    double dt = 0.05;  // 时间步长
    double sim_time = 10.0;  // 模拟时间
    int steps = static_cast<int>(sim_time / dt);

    // 输入序列 (转向角, 电机扭矩, 刹车扭矩)
    std::vector<std::tuple<double, double, double>> inputs = {
        {0.1, 200.0, 0.0},    // 轻微转向，200Nm电机扭矩
        {0.2, 150.0, 0.0},    // 增加转向，减少电机扭矩
        {0.0, 0.0, 500.0},    // 直行，500Nm刹车扭矩
        {-0.1, -100.0, 0.0},  // 反向转向，100Nm反向电机扭矩(再生制动)
        {0.0, 0.0, 0.0}       // 释放所有输入
    };

    // 存储轨迹用于绘制
    std::vector<double> traj_x, traj_y;
    std::vector<double> speeds;
    std::vector<double> times;



    // 模拟循环
    for (int i = 0; i < steps; ++i) {
        // 获取当前状态
        auto state = vehicle.getState();
        
        // 存储轨迹和速度
        traj_x.push_back(state.x);
        traj_y.push_back(state.y);
        speeds.push_back(state.vx);
        times.push_back(i * dt);
        
        // 获取当前时间对应的输入
        int input_idx = (i / 200) % inputs.size();
        // 替换结构化绑定为传统tuple访问方式
        double steer = std::get<0>(inputs[input_idx]);
        double motor_torque = std::get<1>(inputs[input_idx]);
        double brake_torque = std::get<2>(inputs[input_idx]);
        
        // 更新车辆状态
        vehicle.update(steer, motor_torque, brake_torque, dt);
        
        // 每10步更新一次图形
        if (i % 10 == 0) {
            // 更新轨迹图
            plt::cla();  // 清除当前图形（避免重叠）
            
            // 绘制轨迹
            plt::plot(traj_x, traj_y, {{"color", "blue"}, {"linestyle", "-"}, {"linewidth", "1"}});
            // 绘制车辆和速度箭头
            plot_vehicle(state, params, steer);
            // 设置坐标范围（跟随车辆）
            plt::xlim(-50, 30);
            plt::ylim(-10, 80);
            
            plt::pause(0.1);  // 刷新图形
            
            // // 更新速度图
            // plt::subplot(2, 1, 2);
            // plt::cla();
            
            // std::map<std::string, std::string> speed_keywords;
            // speed_keywords["color"] = "red";
            // speed_keywords["linestyle"] = "-";
            // speed_keywords["linewidth"] = "1";
            // plt::plot(times, speeds, speed_keywords);
            
            // double max_speed = *std::max_element(speeds.begin(), speeds.end()) * 1.2;
            // // 确保xlim/ylim参数类型一致
            // plt::xlim(0.0, sim_time);  // 使用double而不是int
            // plt::ylim(0.0, max_speed); // 使用double而不是int
            
            plt::pause(0.001);  // 短暂暂停以更新图形
        }
    }
    
    // 保持图形打开
    plt::show();

    return 0;
}