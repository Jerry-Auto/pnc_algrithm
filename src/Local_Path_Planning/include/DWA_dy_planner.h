#ifndef DWA_PLANNER_DY_H
#define DWA_PLANNER_DY_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>
#include "World_map.h"
#include "vehicle_model.h"

#define PI 3.14159
/// @brief 动力学模型的DWA算法，输入是
class DWADyPlanner {
public:
    // 配置参数结构体
    // 配置参数结构体
    struct Config {
        // 动态窗口参数
        //速度变成了方向盘转角变化率，角速度变成了纵向的加速度
        double max_speed;           // 最大方向盘转角变化率(rad/s)
        double min_speed;          // 最小方向盘转角变化率(rad/s) (允许倒车)
        double max_yaw_rate;        // 最大加速度 (rad/s)
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
        double head_cost_weight;    // 航向代价
        
        // 机器人参数
        double robot_radius;       // 机器人半径 (m)
        
        // 障碍物阈值
        double obstacle_threshold; // 障碍物安全距离 (m)

        double speed_resolution;//速度采样分辨率

        double yaw_rate_resolution;//航向角采样分辨率

        // 构造函数提供默认值
        Config() :
            max_speed(45*PI/180.0),
            min_speed(-45*PI/180.0),
            max_yaw_rate(1.0),
            max_accel(20*PI/180.0),
            max_decel(20*PI/180.0),
            max_delta_yaw_rate(1.0),
            dt(0.1),
            predict_time(3.0),
            goal_cost_weight(1.0),
            speed_cost_weight(2.0),
            obstacle_cost_weight(50.0),
            head_cost_weight(1.0),
            robot_radius(2.5),
            speed_resolution(0.2*PI/180),
            yaw_rate_resolution(0.1),
            obstacle_threshold(2.0) {}
    };

    // 构造函数
    DWADyPlanner(const Config& config = Config());  // 声明
    
    std::pair<std::vector<std::vector<double>>,
    std::vector<std::vector<std::vector<double>>>> 
    planning_series();

    void read_map_data(WorldMap& map,ElectricVehicleDynamicsModel& car);

    void plot_planning(WorldMap* map,ElectricVehicleDynamicsModel* car);

private:
    std::tuple<double, double, std::vector<std::vector<double>>> get_next_ctl_trj(ElectricVehicleDynamicsModel::VehicleState current_state);

    std::tuple<double, double, double, double> calc_dynamic_window(
        const ElectricVehicleDynamicsModel::VehicleState& state) const;
    
    std::vector<std::vector<double>> predict_trajectory(double v, double yaw_rate,
        ElectricVehicleDynamicsModel::VehicleState state) const;
    
    
    double calc_goal_cost(
        const std::vector<std::vector<double>>& trajectory,
        const std::pair<double, double>& goal) const;
    
    double calc_speed_cost(double v) const;
    
    double calc_obstacle_cost(
        const std::vector<std::vector<double>>& trajectory,
        const std::vector<WorldMap::Obstacle>& obstacles) const;
    
    double calc_head_cost(
        const std::vector<std::vector<double>>& trajectory,
        const std::pair<double, double>& goal) const;
    
    bool check_collision(
        const std::vector<std::vector<double>>& trajectory,
        const std::vector<WorldMap::Obstacle>& obstacles) const;

    Config config_;
    std::vector<WorldMap::Obstacle>  obstacle_;
    std::pair<double, double> goal_point_;
    ElectricVehicleDynamicsModel::VehicleState str_state_;
};

#endif // DWA_PLANNER_H