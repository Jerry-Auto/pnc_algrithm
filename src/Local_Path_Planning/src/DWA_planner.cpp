#include <chrono>
#include <thread>
#include "DWA_planner.h"

std::tuple<double, double, std::vector<std::vector<double>>> DWAPlanner::plan(
    const ElectricVehicleDynamicsModel::VehicleState& current_state,
    const std::pair<double, double>& goal,
    const std::vector<WorldMap::Obstacle>& obstacles,
    ElectricVehicleDynamicsModel& vehicle_model) {
    
    obstacles_ = obstacles;
    auto [min_v, max_v, min_yaw_rate, max_yaw_rate] = calc_dynamic_window(current_state);
    
    double best_v = 0.0;
    double best_yaw_rate = 0.0;
    double min_cost = std::numeric_limits<double>::max();
    std::vector<std::vector<double>> best_trajectory;
    
    const int v_samples = 10;
    const int yaw_rate_samples = 10;
    
    for (int i = 0; i < v_samples; ++i) {
        double v = min_v + (max_v - min_v) * i / (v_samples - 1);
        
        for (int j = 0; j < yaw_rate_samples; ++j) {
            double yaw_rate = min_yaw_rate + (max_yaw_rate - min_yaw_rate) * j / (yaw_rate_samples - 1);
            
            auto trajectory = predict_trajectory(v, yaw_rate, current_state, vehicle_model);
            double cost = calc_trajectory_cost(trajectory, goal, obstacles);
            
            if (cost < min_cost) {
                min_cost = cost;
                best_v = v;
                best_yaw_rate = yaw_rate;
                best_trajectory = trajectory;
            }
        }
    }
    
    return {best_v, best_yaw_rate, best_trajectory};
}

std::tuple<double, double, double, double> DWAPlanner::calc_dynamic_window(
    const ElectricVehicleDynamicsModel::VehicleState& state) const {
    
    double v_min = std::max(config_.min_speed, state.vx - config_.max_decel * config_.dt);
    double v_max = std::min(config_.max_speed, state.vx + config_.max_accel * config_.dt);
    
    double yaw_rate_min = -config_.max_yaw_rate;
    double yaw_rate_max = config_.max_yaw_rate;
    
    yaw_rate_min = std::max(yaw_rate_min, state.yaw_rate - config_.max_delta_yaw_rate * config_.dt);
    yaw_rate_max = std::min(yaw_rate_max, state.yaw_rate + config_.max_delta_yaw_rate * config_.dt);
    
    return {v_min, v_max, yaw_rate_min, yaw_rate_max};
}

std::vector<std::vector<double>> DWAPlanner::predict_trajectory(
    double v, double yaw_rate, 
    const ElectricVehicleDynamicsModel::VehicleState& state,
    ElectricVehicleDynamicsModel& vehicle_model) const {
    
    std::vector<std::vector<double>> trajectory(4);
    double time = 0.0;
    
    auto original_state = vehicle_model.getState();
    ElectricVehicleDynamicsModel::VehicleState temp_state = state;
    vehicle_model.reset(temp_state);
    
    for (int i = 0; time <= config_.predict_time; ++i) {
        time += config_.dt;
        
        double steering_angle = 0.0;
        if (std::abs(v) > 0.1) {
            double L = 2.5;
            steering_angle = std::atan2(yaw_rate * L, v);
            steering_angle = std::clamp(steering_angle, -M_PI/4.0, M_PI/4.0);
        }
        
        double accel = (v - state.vx) / config_.dt;
        accel = std::clamp(accel, -config_.max_decel, config_.max_accel);
        
        vehicle_model.update(steering_angle, accel, config_.dt);
        auto new_state = vehicle_model.getState();
        
        trajectory[0].push_back(time);
        trajectory[1].push_back(new_state.x);
        trajectory[2].push_back(new_state.y);
        trajectory[3].push_back(new_state.yaw);
        
        if (check_collision(trajectory, obstacles_)) break;
    }
    
    vehicle_model.reset(original_state);
    return trajectory;
}

double DWAPlanner::calc_trajectory_cost(
    const std::vector<std::vector<double>>& trajectory,
    const std::pair<double, double>& goal,
    const std::vector<WorldMap::Obstacle>& obstacles) const {
    
    return config_.goal_cost_weight * calc_goal_cost(trajectory, goal) +
           config_.speed_cost_weight * calc_speed_cost(trajectory[1].back()) +
           config_.obstacle_cost_weight * calc_obstacle_cost(trajectory, obstacles) +
           config_.path_cost_weight * calc_path_cost(trajectory, goal);
}
    
    double DWAPlanner::calc_goal_cost(
        const std::vector<std::vector<double>>& trajectory,
        const std::pair<double, double>& goal) const {
        
        if (trajectory[1].empty()) return std::numeric_limits<double>::max();
        double dx = goal.first - trajectory[1].back();
        double dy = goal.second - trajectory[2].back();
        return std::hypot(dx, dy);
    }
    
    double DWAPlanner::calc_speed_cost(double v) const { return config_.max_speed - v; }
    
    double DWAPlanner::calc_obstacle_cost(
        const std::vector<std::vector<double>>& trajectory,
        const std::vector<WorldMap::Obstacle>& obstacles) const {
        
        if (obstacles.empty()) return 0.0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < trajectory[1].size(); ++i) {
            double x = trajectory[1][i];
            double y = trajectory[2][i];
            
            for (const auto& obs : obstacles) {
                double dist = std::hypot(x - obs.x, y - obs.y);
                if (dist < min_dist) min_dist = dist;
                if (dist < config_.robot_radius + std::min(obs.width, obs.height)/2.0) {
                    return std::numeric_limits<double>::max();
                }
            }
        }
        return 1.0 / (min_dist + 1e-6);
    }

        double DWAPlanner::calc_path_cost(
        const std::vector<std::vector<double>>& trajectory,
        const std::pair<double, double>& goal) const {
        
        if (trajectory[1].empty()) return std::numeric_limits<double>::max();
        
        double x0 = trajectory[1].front(), y0 = trajectory[2].front();
        double x1 = goal.first, y1 = goal.second;
        double A = y1 - y0, B = x0 - x1, C = x1*y0 - x0*y1;
        double norm = std::hypot(A, B);
        double total_deviation = 0.0;
        
        for (size_t i = 0; i < trajectory[1].size(); ++i) {
            double dist = std::abs(A*trajectory[1][i] + B*trajectory[2][i] + C) / norm;
            total_deviation += dist;
        }
        return total_deviation / trajectory[1].size();
    }
    
    bool DWAPlanner::check_collision(
        const std::vector<std::vector<double>>& trajectory,
        const std::vector<WorldMap::Obstacle>& obstacles) const {
        
        if (obstacles.empty()) return false;
        
        for (size_t i = 0; i < trajectory[1].size(); ++i) {
            double x = trajectory[1][i], y = trajectory[2][i];
            
            for (const auto& obs : obstacles) {
                double dist = std::hypot(x - obs.x, y - obs.y);
                if (dist < config_.robot_radius + std::min(obs.width, obs.height)/2.0) {
                    return true;
                }
               
        }
        return false;
    }
    
    Config config_;
};