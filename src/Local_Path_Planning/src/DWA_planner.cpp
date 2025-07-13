#include "DWA_planner.h"
#include <iostream>
#include <cmath>

DWAPlanner::DWAPlanner(const Config& config)
    : config_(config) {}

    void DWAPlanner::read_map_data(WorldMap& map,ElectricVehicleDynamicsModel& car){
        obstacle_=map.get_Obstacle();
        goal_point_=map.get_goal();
        str_state_=car.getState();
    }

std::pair<std::vector<std::vector<double>>, 
          std::vector<std::vector<std::vector<double>>>> 
DWAPlanner::planning_series() {
    double t = 0;
    auto goal = goal_point_;    
    double goal_threshold = std::pow(config_.robot_radius * 1.5, 2); // 目标点阈值
    std::vector<double> t_data, x_data, y_data, theta_data;
    std::vector<std::vector<std::vector<double>>> pre_trj;
    
    auto current_state = str_state_;
    
    while(true) {
        // 记录当前状态
        t_data.push_back(t);
        x_data.push_back(current_state.x);
        y_data.push_back(current_state.y);
        theta_data.push_back(current_state.yaw);
        
        // 获取最佳控制指令和轨迹
        auto [best_v, best_yaw_rate, best_trajectory] = get_next_ctl_trj(current_state);
        pre_trj.push_back(best_trajectory);
        
        // 更新状态
        current_state = kinematics_update_State(best_v, best_yaw_rate, current_state);
        
        // 检查是否到达目标
        double dist_to_goal = std::pow(current_state.x - goal.first, 2) + 
                             std::pow(current_state.y - goal.second, 2);
        t += config_.dt;
        
        if(dist_to_goal < goal_threshold) {
            std::cout << "Goal reached!" << std::endl;
            break;
        }
        
        // 防止无限循环
        if(t > 10000.0) { // 100秒超时
            std::cout << "Planning timeout!" << std::endl;
            break;
        }
    }
    
    std::vector<std::vector<double>> planning_data = {x_data, y_data, theta_data};
    return {planning_data, pre_trj};
}

std::tuple<double, double, std::vector<std::vector<double>>> 
DWAPlanner::get_next_ctl_trj(ElectricVehicleDynamicsModel::VehicleState current_state) {
    auto [min_v, max_v, min_yaw_rate, max_yaw_rate] = calc_dynamic_window(current_state);
    ElectricVehicleDynamicsModel::VehicleState state = current_state;
    double best_v = 0.0;
    double best_yaw_rate = 0.0;
    double min_cost = std::numeric_limits<double>::max();
    std::vector<std::vector<double>> best_trajectory;
    
    // 归一化因子
    double sum_goal_cost = 0, sum_speed_cost = 0, sum_obs_cost = 0, sum_path_cost = 0;
    int sample_count = 0;
    
    // 第一次遍历：计算归一化因子
    for(double v = min_v; v <= max_v; v += config_.speed_resolution) {
        for(double yaw_rate = min_yaw_rate; yaw_rate <= max_yaw_rate; 
            yaw_rate += config_.yaw_rate_resolution) {
            auto trajectory = predict_trajectory(v, yaw_rate, state);
            sum_goal_cost += calc_goal_cost(trajectory, goal_point_);
            sum_speed_cost += calc_speed_cost(v);
            sum_obs_cost += calc_obstacle_cost(trajectory, obstacle_);
            sum_path_cost += calc_head_cost(trajectory, goal_point_);
            sample_count++;
        }
    }
    
    // 防止除以零
    if(sample_count == 0) {
        return {0, 0, {}};
    }
    double max_cost=0;
    //state = current_state;
    //sum_goal_cost = 1, sum_speed_cost = 1, sum_obs_cost = 1, sum_path_cost = 1;
    // 第二次遍历：寻找最优轨迹
    for(double v = min_v; v <= max_v; v += config_.speed_resolution) {
        for(double yaw_rate = min_yaw_rate; yaw_rate <= max_yaw_rate; 
            yaw_rate += config_.yaw_rate_resolution) {
            auto trajectory = predict_trajectory(v, yaw_rate, state);

            double goal_cost = calc_goal_cost(trajectory, goal_point_) / sum_goal_cost;
            double speed_cost = calc_speed_cost(v) / sum_speed_cost;
            double obs_cost = 0;
            if(sum_obs_cost!=0){
                if(std::isnan(calc_obstacle_cost(trajectory, obstacle_))){obs_cost =0;}
                else{obs_cost = calc_obstacle_cost(trajectory, obstacle_) / sum_obs_cost; }}
            double path_cost = calc_head_cost(trajectory, goal_point_) / sum_path_cost;
            
            double total_cost = config_.goal_cost_weight * goal_cost 
                              +config_.speed_cost_weight * speed_cost +
                              config_.obstacle_cost_weight * obs_cost +
                              config_.head_cost_weight * path_cost;
            
            if(total_cost < min_cost) {
                min_cost = total_cost;
                best_v = v;
                best_yaw_rate = yaw_rate;
                best_trajectory = trajectory;
            }
            if(total_cost>max_cost){max_cost=total_cost;}

        }
    }
    
    return {best_v, best_yaw_rate, best_trajectory};
}

std::tuple<double, double, double, double> 
DWAPlanner::calc_dynamic_window(const ElectricVehicleDynamicsModel::VehicleState& state) const {
    // 速度动态窗口
    double v_min = std::max(config_.min_speed, state.vx - config_.max_decel * config_.dt);
    double v_max = std::min(config_.max_speed, state.vx + config_.max_accel * config_.dt);
    // v_min = config_.min_speed;
    // v_max = config_.max_speed;
    // 角速度动态窗口
    double yaw_rate_min = std::max(-config_.max_yaw_rate, 
                                  state.yaw_rate - config_.max_delta_yaw_rate * config_.dt);
    double yaw_rate_max = std::min(config_.max_yaw_rate, 
                                  state.yaw_rate + config_.max_delta_yaw_rate * config_.dt);
    // yaw_rate_min = -config_.max_yaw_rate;
    // yaw_rate_max = config_.max_yaw_rate;
    return {v_min, v_max, yaw_rate_min, yaw_rate_max};
}

ElectricVehicleDynamicsModel::VehicleState 
DWAPlanner::kinematics_update_State(double v, double w, 
                                   ElectricVehicleDynamicsModel::VehicleState state) const {
    state.vx = v;
    state.yaw_rate = w;
    state.yaw += w * config_.dt;
    
    // 限制角度在[-π,π]范围内
    if(state.yaw > PI) state.yaw -= 2*PI;
    if(state.yaw < -PI) state.yaw += 2*PI;
    
    state.x += v * cos(state.yaw) * config_.dt;
    state.y += v * sin(state.yaw) * config_.dt;
    
    return state;
}

std::vector<std::vector<double>> 
DWAPlanner::predict_trajectory(double v, double yaw_rate,
                              ElectricVehicleDynamicsModel::VehicleState state) const {
    std::vector<double> x_pre, y_pre, yaw_pre;
    x_pre.push_back(state.x);
    y_pre.push_back(state.y);
    yaw_pre.push_back(state.yaw);
    
    for(double t = config_.dt; t <= config_.predict_time; t += config_.dt) {
        state = kinematics_update_State(v, yaw_rate, state);
        x_pre.push_back(state.x);
        y_pre.push_back(state.y);
        yaw_pre.push_back(state.yaw);
    }
    
    return {x_pre, y_pre, yaw_pre};
}

double DWAPlanner::calc_goal_cost(const std::vector<std::vector<double>>& trajectory,
                                const std::pair<double, double>& goal) const {
    if(trajectory[0].empty()) return std::numeric_limits<double>::max();
    
    // 使用轨迹终点到目标的距离
    double dx = goal.first - trajectory[0].back();
    double dy = goal.second - trajectory[1].back();
    return dx*dx+dy*dy;
}

double DWAPlanner::calc_speed_cost(double v) const {
    // 鼓励接近最大速度
    return config_.max_speed - v;
}

double DWAPlanner::calc_obstacle_cost(
    const std::vector<std::vector<double>>& trajectory,
    const std::vector<WorldMap::Obstacle>& obstacles) const 
{
    if(obstacles.empty()) return 0.0;
    
    double min_dist = std::numeric_limits<double>::max();
    double cost=0;

    for(size_t i = 0; i < trajectory[0].size(); ++i) {
        const double x = trajectory[0][i];
        const double y = trajectory[1][i];
        
        for(const auto& obs : obstacles) {
            // 计算障碍物等效半径（矩形对角线的一半）
            const double obs_radius = sqrt(obs.width*obs.width + obs.height*obs.height)/2;
            
            // 计算点到障碍物圆心的距离
            const double dx = x - obs.x;
            const double dy = y - obs.y;
            const double dist_to_center = sqrt(dx*dx + dy*dy);
            
            // 有效距离 = 圆心距离 - 障碍物半径-车辆半径
            const double effective_dist = dist_to_center - obs_radius-config_.robot_radius;

            if(effective_dist < config_.obstacle_threshold)
            {cost+=1.0/std::pow((effective_dist+1e-6),1);}
             
            // 碰撞检测（考虑机器人半径）
            if(check_collision(trajectory,obstacles)) {
                return std::numeric_limits<double>::max();
            }
        }
    }
    
    // 安全距离内按距离倒数计算代价
    return cost;
}

bool DWAPlanner::check_collision(
    const std::vector<std::vector<double>>& trajectory,
    const std::vector<WorldMap::Obstacle>& obstacles) const 
{
    if(obstacles.empty()) return false;
    
    for(size_t i = 0; i < trajectory[0].size(); ++i) {
        double x = trajectory[0][i];
        double y = trajectory[1][i];
        
        for(const auto& obs : obstacles) {
            // 将点转换到障碍物局部坐标系
            double dx = x - obs.x;
            double dy = y - obs.y;
            
            // 旋转逆变换
            double local_x = dx * cos(-obs.rotation) - dy * sin(-obs.rotation);
            double local_y = dx * sin(-obs.rotation) + dy * cos(-obs.rotation);
            
            // 检查是否在矩形内（考虑机器人半径）
            bool inside_x = std::abs(local_x) <= (obs.width/2 + config_.robot_radius);
            bool inside_y = std::abs(local_y) <= (obs.height/2 + config_.robot_radius);
            
            if(inside_x && inside_y) {
                return true;
            }
        }
    }
    return false;
}
/// @brief 计算航向角度误差
/// @param trajectory 
/// @param goal 
/// @return 
double DWAPlanner::calc_head_cost(const std::vector<std::vector<double>>& trajectory,
                               const std::pair<double, double>& goal) const {
    if(trajectory.size()!=0)
    {
        double dx=goal.first-trajectory[0].back();
        double dy=goal.second-trajectory[1].back();
        double heading_err=std::abs(trajectory[2].back()-std::atan2(dy,dx));
        return heading_err;
    }
    else{
        return std::numeric_limits<double>::max();
    }
}


void DWAPlanner::plot_planning(WorldMap* map,ElectricVehicleDynamicsModel* car) {
    int k=3,i=0;
    double t=0;  
    double goal_threshold = std::pow(config_.robot_radius * 1.5, 2); // 目标点阈值  
    auto current_state = car->getState();
    while(true) {
        // 获取下一步最佳控制指令和轨迹
        auto [best_v, best_yaw_rate, best_trajectory] = get_next_ctl_trj(current_state);
        std::cout<<"当前最优控制:V:"<<best_v<<" W:"<<best_yaw_rate<<std::endl;       
        // 绘制当前状态
        std::tuple<double,double,double> state={current_state.x,current_state.y,current_state.yaw};
        car->reset_for_planning_only(state,best_trajectory);
        if (i % k == 0) {
            // 重新可视化（非阻塞模式）
            map->visualize(true,true,false);
        }
        // 更新状态
        current_state = kinematics_update_State(best_v, best_yaw_rate, current_state);
        
        // 检查是否到达目标
        double dist_to_goal = std::pow(current_state.x - goal_point_.first, 2) + 
                             std::pow(current_state.y - goal_point_.second, 2);
        if(dist_to_goal < 0.1*goal_threshold) {
            //到达了把到达的这一步绘制出来
            std::tuple<double,double,double> state={current_state.x,current_state.y,current_state.yaw};
            car->reset_for_planning_only(state);
            if (i % k == 0) {
                // 重新可视化（非阻塞模式）
                map->visualize(true,true,false);
            }
            std::cout << "Goal reached!" << std::endl;
            break;
        }
        t+=config_.dt;
        // 防止无限循环
        if(t > 10000.0) { // 100秒超时
            std::cout << "Planning timeout!" << std::endl;
            break;
        }
    }
    map->visualize(true,true,true);
}