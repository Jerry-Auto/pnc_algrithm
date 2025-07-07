#include"vehicle_model.h"    
    
    // 构造函数 - 调整参数更适合前驱车
ElectricVehicleDynamicsModel::ElectricVehicleDynamicsModel() {
    params_.mass = 1200.0;       // 前驱车通常较轻
    params_.lf = 1.1;            // 前轴到质心距离（稍短）
    params_.lr = 1.4;            // 后轴到质心距离
    params_.iz = 2000.0;         // 减小转动惯量
    params_.cf = 70000.0;        // 前轮侧偏刚度
    params_.cr = 65000.0;        // 后轮侧偏刚度（通常小于前轮）
    params_.drag_coeff = 0.32;
    params_.rr_coeff = 0.016;
    params_.max_steer = M_PI / 4;
    params_.max_motor_torque = 250.0; // 前驱电机扭矩
    params_.max_brake_torque = 800.0;
    params_.wheel_radius = 0.3;
    params_.gear_ratio = 9.0;
    params_.motor_efficiency = 0.92;
    params_.wheel_width = 0.2;
    params_.track_width = 1.5;
    params_.max_accel = 2.5;
    params_.max_decel = 7.0;
    state_ = {0, 0, 0, 0, 0, 0, 0};
}
 
void ElectricVehicleDynamicsModel::update(double steering_angle, double desired_accel, double dt) {
    // 限制输入范围
    steering_angle = clamp(steering_angle, -params_.max_steer, params_.max_steer);
    desired_accel = clamp(desired_accel, -params_.max_decel, params_.max_accel);
 
    // 安全速度计算
    double vx_safe = std::max(state_.vx, 0.1);
 
    // 轮胎侧偏角计算
    double alpha_f = steering_angle - std::atan2(state_.vy + params_.lf * state_.yaw_rate, vx_safe);
    double alpha_r = -std::atan2(state_.vy - params_.lr * state_.yaw_rate, vx_safe);
 
    // 轮胎力计算
    double F_fy = -params_.cf * alpha_f;
    double F_ry = -params_.cr * alpha_r;
 
    // 限制最大侧向力（防止侧滑）
    double max_lat_force = 0.35 * params_.mass * 9.81;
    F_fy = clamp(F_fy, -max_lat_force, max_lat_force);
    F_ry = clamp(F_ry, -max_lat_force, max_lat_force);
 
    // 阻力计算
    double F_air = params_.drag_coeff * state_.vx * std::abs(state_.vx) * 0.5;
    double F_rr = params_.rr_coeff * (state_.vx < 0 ? -state_.vx : state_.vx);
 
    // 前驱车特性：只有前轮提供驱动力
    double F_total_required = desired_accel * params_.mass + F_air + F_rr;
    
    // 计算前轮最大可用牵引力（考虑侧向力）
    double F_long_max_front = std::sqrt(std::pow(max_lat_force, 2) - std::pow(F_fy, 2));
    double F_drive_front = clamp(F_total_required, -F_long_max_front, F_long_max_front);
 
    // 后轮没有驱动力，只有侧向力
    double F_drive_rear = 0.0;
 
    // 计算所需电机扭矩（前驱）
    double wheel_torque_front = F_drive_front * params_.wheel_radius;
    double motor_torque = wheel_torque_front / (params_.gear_ratio * params_.motor_efficiency);
    motor_torque = clamp(motor_torque, -params_.max_motor_torque, params_.max_motor_torque);
 
    // 重新计算实际前轮驱动力
    wheel_torque_front = motor_torque * params_.gear_ratio * params_.motor_efficiency;
    F_drive_front = wheel_torque_front / params_.wheel_radius;
 
    // 动力学方程
    double x_dot = state_.vx * std::cos(state_.yaw) - state_.vy * std::sin(state_.yaw);
    double y_dot = state_.vx * std::sin(state_.yaw) + state_.vy * std::cos(state_.yaw);
    
    // 前驱车：总纵向力来自前轮
    double vx_dot = (F_drive_front * std::cos(steering_angle) - F_air - F_rr) / params_.mass;
    
    // 侧向力（前后轮贡献）
    double vy_dot = (F_fy * std::sin(steering_angle) + F_ry) / params_.mass - state_.yaw_rate * state_.vx;
    
    // 修正的yaw力矩计算（前驱车特性）
    double torque_yaw = params_.lf * F_fy * std::cos(steering_angle) - params_.lr * F_ry;
    
    // 考虑前轮驱动力产生的附加力矩
    double torque_drive = F_drive_front * std::sin(steering_angle) * params_.wheel_radius;
    double yaw_rate_dot = (torque_yaw + torque_drive) / params_.iz;
 
    // 车轮动力学（前轮驱动）
    double wheel_inertia = 0.5 * params_.mass * std::pow(params_.wheel_radius, 2);
    double front_wheel_speed_dot = (wheel_torque_front - F_drive_front * params_.wheel_radius) / wheel_inertia;
    
    // 欧拉积分
    state_.x += x_dot * dt;
    state_.y += y_dot * dt;
    state_.yaw += state_.yaw_rate * dt;
    state_.vx += vx_dot * dt;
    state_.vy += vy_dot * dt;
    state_.yaw_rate += yaw_rate_dot * dt;
    
    // 前轮速度更新（只有前轮有驱动）
    state_.wheel_speed += front_wheel_speed_dot * dt;
 
    // 防止低速不稳定
    if (state_.vx < 0.1 && desired_accel <= 0) {
        state_.vx = 0;
        state_.wheel_speed = 0;
    }
    
    // 规范化yaw角度到[-π, π]
    state_.yaw = std::atan2(std::sin(state_.yaw), std::cos(state_.yaw));
}
 
// 其余函数保持不变...
ElectricVehicleDynamicsModel::VehicleState ElectricVehicleDynamicsModel::getState() const {
    return state_;
}
 
void ElectricVehicleDynamicsModel::reset(const VehicleState& new_state) {
    state_ = new_state;
}
 
void ElectricVehicleDynamicsModel::plot_vehicle(const VehicleState& state, double steer_angle, const std::string& color) {
    // 车辆轮廓点 (局部坐标系)
    std::vector<double> vehicle_x = {-params_.lr, params_.lf, params_.lf, -params_.lr, -params_.lr};
    std::vector<double> vehicle_y = {-1.0, -1.0, 1.0, 1.0, -1.0};
    
    // 旋转和平移到全局坐标系
    std::vector<double> global_x, global_y;
    for (size_t i = 0; i < vehicle_x.size(); ++i) {
        double x_local = vehicle_x[i];
        double y_local = vehicle_y[i];
        
        double x_rot = x_local * std::cos(state.yaw) - y_local * std::sin(state.yaw);
        double y_rot = x_local * std::sin(state.yaw) + y_local * std::cos(state.yaw);
        
        global_x.push_back(state.x + x_rot);
        global_y.push_back(state.y + y_rot);
    }
    
    // 绘制车辆主体
    plt::plot(global_x, global_y, {{"color", color}, {"linewidth", "2"}});
    
    // 绘制车轮（前驱车强调前轮）
    double half_track = params_.track_width / 2.0;
    double wheel_len = 0.25;  // 前轮稍长表示驱动轮
    double wheel_width = 0.1;
    
    auto draw_wheel = [&](double x, double y, double steer, bool is_front) {
        std::vector<double> wx = {-wheel_len/2, wheel_len/2, wheel_len/2, -wheel_len/2, -wheel_len/2};
        std::vector<double> wy = {-wheel_width/2, -wheel_width/2, wheel_width/2, wheel_width/2, -wheel_width/2};
        
        std::vector<double> gx, gy;
        for (size_t i = 0; i < wx.size(); ++i) {
            double x_rel = wx[i];
            double y_rel = wy[i];
            
            // 车轮旋转（仅前轮转向）
            double x_rot = x_rel * std::cos(steer) - y_rel * std::sin(steer);
            double y_rot = x_rel * std::sin(steer) + y_rel * std::cos(steer);
            
            // 车辆坐标系 -> 全局坐标系
            double x_global = state.x + (x * std::cos(state.yaw) - y * std::sin(state.yaw)) + 
                             (x_rot * std::cos(state.yaw) - y_rot * std::sin(state.yaw));
            double y_global = state.y + (x * std::sin(state.yaw) + y * std::cos(state.yaw)) + 
                             (x_rot * std::sin(state.yaw) + y_rot * std::cos(state.yaw));
            
            gx.push_back(x_global);
            gy.push_back(y_global);
        }
        
        // 前轮用不同颜色表示
        if (is_front) {
            plt::plot(gx, gy, {{"color", "red"}, {"linewidth", "1.5"}});
        } else {
            plt::plot(gx, gy, {{"color", "black"}, {"linewidth", "1.5"}});
        }
    };
    
    // 绘制四个车轮（前轮用红色强调）
    draw_wheel(params_.lf, half_track, steer_angle, true);   // 左前（驱动轮）
    draw_wheel(params_.lf, -half_track, steer_angle, true);  // 右前（驱动轮）
    draw_wheel(-params_.lr, half_track, 0, false);           // 左后
    draw_wheel(-params_.lr, -half_track, 0, false);          // 右后
}
 
void ElectricVehicleDynamicsModel::reset_for_planning_only(std::tuple<double,double,double>& new_state) {
    state_.x = std::get<0>(new_state);
    state_.y = std::get<1>(new_state);
    state_.yaw = std::get<2>(new_state);
}
 
void ElectricVehicleDynamicsModel::plot_planning(std::vector<std::vector<double>> planning_data) {
    auto min_it = std::min_element(planning_data[1].begin(), planning_data[1].end());
    double min_val = *min_it;
    auto max_it = std::max_element(planning_data[1].begin(), planning_data[1].end());
    double max_val = *max_it;
    
    int steps = static_cast<int>(planning_data[0].size());
    std::vector<double> traj_x, traj_y;
    
    for (int i = 0; i < steps; ++i) {
        std::tuple<double,double,double> new_state = 
            {planning_data[1][i], planning_data[2][i], planning_data[3][i]};
        reset_for_planning_only(new_state);
        auto state = getState();
        traj_x.push_back(state.x);
        traj_y.push_back(state.y);
        
        if (i % 50 == 0) {
            plt::cla();
            plt::plot(traj_x, traj_y, {{"color", "blue"}, {"linestyle", "-"}, {"linewidth", "1"}});
            plot_vehicle(state);
            plt::xlim(min_val-10, max_val+10);
            plt::ylim(min_val-10, max_val+10);
            plt::pause(0.01);
        }
    }
    plt::show();
}