#include"vehicle_model.h"
#include <chrono>    
    
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
    params_.Cd = 0.3;        // 空气阻力系数
    params_.damping = 0.95;  // 横摆角速度阻尼
    state_ = {0, 0, 0, 0, 0, 0, 0};
}
 
void ElectricVehicleDynamicsModel::update(double steering_angle, double desired_accel, double dt) {
    // 更新车辆状态

        // 1. 计算轮胎侧偏角（小角度近似）
        //steering_angle=-steering_angle;
        this->steering_angle=steering_angle;
        this->desired_accel=desired_accel;
        double alpha_f = steering_angle - (state_.vy + params_.lf * state_.yaw_rate) / (state_.vx + 1e-6);
        double alpha_r = -(state_.vy - params_.lr * state_.yaw_rate) / (state_.vx + 1e-6);

        // 2. 限制侧偏角范围（非线性保护）
        alpha_f = std::max(-0.5, std::min(0.5, alpha_f));
        alpha_r = std::max(-0.5, std::min(0.5, alpha_r));

 

        // 3. 计算轮胎力（线性模型）
        double Fyf = 2 * params_.cf * alpha_f;
        double Fyr = 2 * params_.cr * alpha_r;

        // 4. 计算纵向力（简化模型）
        double Fx = desired_accel * params_.mass;
        // 5. 更新横向速度（博客公式14）
        double ay = (-(params_.cf + params_.cr) * state_.vy 
                    - (params_.lf * params_.cf - params_.lr * params_.cr) * state_.yaw_rate 
                    + params_.cf * steering_angle * state_.vx) / params_.mass;
        state_.vy += ay * dt;

        // 6. 更新横摆角速度（博客公式17+阻尼）
        double yaw_accel = (params_.lf * Fyf - params_.lr * Fyr) / params_.iz;
        state_.yaw_rate = params_.damping * state_.yaw_rate + yaw_accel * dt;

        // 7. 更新纵向速度（考虑空气阻力）
        double air_drag = 0.5 * 1.225 * params_.Cd * 2.0 * (state_.vx > 0 ? state_.vx * state_.vx : 0); // 假设迎风面积2m²
        state_.vx += (Fx - air_drag) / params_.mass * dt;

        // 8. 更新航向角
        state_.yaw += state_.yaw_rate * dt;
        // 规范化yaw角度到[-π, π]
        state_.yaw = std::atan2(std::sin(state_.yaw), std::cos(state_.yaw));
        // 9. 更新全局位置
        state_.x += (state_.vx * cos(state_.yaw) - state_.vy * sin(state_.yaw)) * dt;
        state_.y += (state_.vx * sin(state_.yaw) + state_.vy * cos(state_.yaw)) * dt;
 
    // 防止低速不稳定
    if (state_.vx < 0.1 && desired_accel <= 0) {
        state_.vx = 0;
        state_.wheel_speed = 0;
    }
    

}
 
// 其余函数保持不变...
ElectricVehicleDynamicsModel::VehicleState ElectricVehicleDynamicsModel::getState() const {
    return state_;
}
 
void ElectricVehicleDynamicsModel::reset(const VehicleState& new_state) {
    state_ = new_state;
}
 
void ElectricVehicleDynamicsModel::plot_vehicle(const std::string& color) {
    // 车辆轮廓点 (局部坐标系)
    std::vector<double> vehicle_x = {-params_.lr, params_.lf, params_.lf, -params_.lr, -params_.lr};
    std::vector<double> vehicle_y = {-1.0, -1.0, 1.0, 1.0, -1.0};
    
    // 旋转和平移到全局坐标系
    std::vector<double> global_x, global_y;
    for (size_t i = 0; i < vehicle_x.size(); ++i) {
        double x_local = vehicle_x[i];
        double y_local = vehicle_y[i];
        
        double x_rot = x_local * std::cos(state_.yaw) - y_local * std::sin(state_.yaw);
        double y_rot = x_local * std::sin(state_.yaw) + y_local * std::cos(state_.yaw);
        
        global_x.push_back(state_.x + x_rot);
        global_y.push_back(state_.y + y_rot);

    }
    
    // 绘制车辆主体
    plt::plot(global_x, global_y, {{"color", color}, {"linewidth", "2"}});
    
    // 绘制车轮（前驱车强调前轮）
    double half_track = params_.track_width / 2.0;
    double wheel_len = params_.wheel_radius*2;  // 前轮稍长表示驱动轮
    double wheel_width = params_.wheel_width;
    
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
            double x_global = state_.x + (x * std::cos(state_.yaw) - y * std::sin(state_.yaw)) + 
                             (x_rot * std::cos(state_.yaw) - y_rot * std::sin(state_.yaw));
            double y_global = state_.y + (x * std::sin(state_.yaw) + y * std::cos(state_.yaw)) + 
                             (x_rot * std::sin(state_.yaw) + y_rot * std::cos(state_.yaw));
            
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
    
    draw_wheel(params_.lf, half_track, this->steering_angle, true);   // 左前（驱动轮）
    draw_wheel(params_.lf, -half_track, this->steering_angle, true);  // 右前（驱动轮）
    draw_wheel(-params_.lr, half_track, 0, false);           // 左后
    draw_wheel(-params_.lr, -half_track, 0, false);          // 右后
}
 
void ElectricVehicleDynamicsModel::reset_for_planning_only(std::tuple<double,double,double>& new_state) {
    state_.x = std::get<0>(new_state);
    state_.y = std::get<1>(new_state);
    state_.yaw = std::get<2>(new_state);
    traj_x.push_back(state_.x);
    traj_y.push_back(state_.y);
}
 
void ElectricVehicleDynamicsModel::plot_planning(std::vector<std::vector<double>> planning_data) {

        // 开始计时
    auto start = std::chrono::high_resolution_clock::now();

    auto min_it = std::min_element(planning_data[1].begin(), planning_data[1].end());
    double min_val = *min_it;
    auto max_it = std::max_element(planning_data[1].begin(), planning_data[1].end());
    double max_val = *max_it;

    int steps = static_cast<int>(planning_data[0].size());
    double dt=planning_data[0][1]-planning_data[0][0];
    int k=3;
    double time_factor=0.8;

    
    for (int i = 0; i < steps; i++) {
        std::tuple<double,double,double> new_state = 
            {planning_data[1][i], planning_data[2][i], planning_data[3][i]};
        reset_for_planning_only(new_state);        
        if (i % k == 0) {
            plt::cla();
            plt::plot(traj_x, traj_y, {{"color", "blue"}, {"linestyle", "-"}, {"linewidth", "1"}});
            plot_vehicle();
            plt::xlim(state_.x-10, state_.x+10);
            plt::ylim(state_.y-10, state_.y+10);
            plt::title("XOY coordinate");
            plt::xlabel("World X (m)");
            plt::ylabel("World Y (m)");
            plt::grid(true);
            plt::pause(dt*time_factor*k);
        }
    }
              // 结束计时
    auto end = std::chrono::high_resolution_clock::now();
    // 计算耗时（单位：毫秒）
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "耗时: " << duration.count() << " 毫秒" << std::endl;

    plt::show();
}