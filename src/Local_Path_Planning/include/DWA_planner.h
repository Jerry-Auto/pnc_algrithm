#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>
#include "World_map.h"

class DWAPlanner {
public:
    // 配置参数结构体
    // 配置参数结构体
    struct Config {
        // 动态窗口参数
        double max_speed;           // 最大速度 (m/s)
        double min_speed;          // 最小速度 (m/s) (允许倒车)
        double max_yaw_rate;        // 最大偏航率 (rad/s)
        double max_accel;           // 最大加速度 (m/s^2)
        double max_decel;           // 最大减速度 (m/s^2)
        double max_delta_yaw_rate;  // 最大偏航率变化率 (rad/s^2)
        
        // 轨迹预测参数
        double dt;                  // 时间步长 (s)
        double predict_time;        // 预测时间 (s)
        
        // 代价函数权重
        double goal_cost_weight;    // 目标代价权重
        double speed_cost_weight;   // 速度代价权重
        double obstacle_cost_weight;// 障碍物代价权重
        double path_cost_weight;    // 路径跟踪代价权重
        
        // 机器人参数
        double robot_radius;       // 机器人半径 (m)
        
        // 障碍物阈值
        double obstacle_threshold; // 障碍物安全距离 (m)

        // 构造函数提供默认值
        Config() :
            max_speed(5.0),
            min_speed(-1.0),
            max_yaw_rate(1.0),
            max_accel(2.5),
            max_decel(7.0),
            max_delta_yaw_rate(0.5),
            dt(0.1),
            predict_time(3.0),
            goal_cost_weight(1.0),
            speed_cost_weight(0.1),
            obstacle_cost_weight(1.0),
            path_cost_weight(0.5),
            robot_radius(1.5),
            obstacle_threshold(2.0) {}
    };

    // 构造函数
    //explicit DWAPlanner(const Config& config = Config());  // 声明
    
    explicit DWAPlanner(const Config& config = Config()) : config_(config) {}
    
    std::tuple<double, double, std::vector<std::vector<double>>> plan(
        const ElectricVehicleDynamicsModel::VehicleState& current_state,
        const std::pair<double, double>& goal,
        const std::vector<WorldMap::Obstacle>& obstacles,
        ElectricVehicleDynamicsModel& vehicle_model);

private:
    std::tuple<double, double, double, double> calc_dynamic_window(
        const ElectricVehicleDynamicsModel::VehicleState& state) const;
    
    std::vector<std::vector<double>> predict_trajectory(
        double v, double yaw_rate, 
        const ElectricVehicleDynamicsModel::VehicleState& state,
        ElectricVehicleDynamicsModel& vehicle_model) const;
    
    double calc_trajectory_cost(
        const std::vector<std::vector<double>>& trajectory,
        const std::pair<double, double>& goal,
        const std::vector<WorldMap::Obstacle>& obstacles) const;
    
    double calc_goal_cost(
        const std::vector<std::vector<double>>& trajectory,
        const std::pair<double, double>& goal) const;
    
    double calc_speed_cost(double v) const;
    
    double calc_obstacle_cost(
        const std::vector<std::vector<double>>& trajectory,
        const std::vector<WorldMap::Obstacle>& obstacles) const;
    
    double calc_path_cost(
        const std::vector<std::vector<double>>& trajectory,
        const std::pair<double, double>& goal) const;
    
    bool check_collision(
        const std::vector<std::vector<double>>& trajectory,
        const std::vector<WorldMap::Obstacle>& obstacles) const;
    
    Config config_;
    std::vector<WorldMap::Obstacle> obstacles_;
};

#endif // DWA_PLANNER_H