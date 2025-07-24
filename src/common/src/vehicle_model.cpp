#include"vehicle_model.h"
#include <chrono>    
    
    // 构造函数 - 调整参数更适合前驱车
ElectricVehicleDynamicsModel::ElectricVehicleDynamicsModel(): params_(), state_() {

}
/// @brief 更新车辆状态
/// @param deta_steering_angle 方向盘转角变化率
/// @param desired_accel 期望的加速度
/// @param dt 
/// @param model 默认用动力学模型，false用运动学模型
void ElectricVehicleDynamicsModel::update(double steering_angle, double desired_accel, double dt,bool model) {
    // 更新车辆状态
    if(model)
    {
        state_=dynamic(steering_angle,desired_accel,dt,state_);
    }
    else
    {
        state_=kinematics(steering_angle,desired_accel,dt,state_);
    }
}


// 车辆运动学模型函数,采样时间（秒）
ElectricVehicleDynamicsModel::VehicleState ElectricVehicleDynamicsModel::kinematics(double steering_angle,double desired_accel,double dt,VehicleState current_state){
    // 更新速度
    ElectricVehicleDynamicsModel::VehicleParams params;
    ElectricVehicleDynamicsModel::VehicleState next_state;
    next_state.steering_angle=steering_angle;
    next_state.vx=current_state.vx+desired_accel*dt;
    next_state.yaw_rate=tan(next_state.steering_angle)*next_state.vx/(params.lf+params.lr);
    next_state.yaw=current_state.yaw+next_state.yaw_rate*dt;
    next_state.x=current_state.x+next_state.vx*cos(next_state.yaw)*dt;
    next_state.y=current_state.y+next_state.vx*sin(next_state.yaw)*dt;
    return next_state;
}

//动力学模型
ElectricVehicleDynamicsModel::VehicleState 
ElectricVehicleDynamicsModel::dynamic(double steering_angle, double desired_accel, double dt, 
                                     VehicleState current_state)
{
    ElectricVehicleDynamicsModel::VehicleParams params;
    VehicleState next_state = current_state;
    // 处理静止状态
if (std::abs(current_state.vx) < 0.01 && std::abs(desired_accel) < 0.01) {
    next_state.vx = 0;
    next_state.vy = 0;
    next_state.yaw_rate = 0;
    next_state.wheel_speed = 0;
    // 可选：重置侧滑角和轮胎力
    next_state.beta = 0;
}
    // 1. 限制转向角在物理范围内
    next_state.steering_angle = std::clamp(steering_angle, -params.max_steer, params.max_steer);
    
    // 2. 计算轮胎侧偏角（小角度近似）
    const double epsilon = 1e-6;
    double vx_eff = current_state.vx + epsilon;
    
    // 前轮和后轮侧偏角（公式6和7）
    double alpha_f = 0;
    double alpha_r = 0;
    if (!(std::abs(current_state.vx) < 0.01 && std::abs(desired_accel) < 0.01)) {
        alpha_f = next_state.steering_angle - (current_state.vy + params.lf * current_state.yaw_rate) / vx_eff;
        alpha_r = -(current_state.vy - params.lr * current_state.yaw_rate) / vx_eff;
    }
    // 3. 平滑限制侧偏角范围（使用tanh函数平滑过渡）
    auto smooth_clamp = [](double x, double limit) {
        return limit * std::tanh(x / limit);
    };
    alpha_f = smooth_clamp(alpha_f, 0.5);  // 限制在±0.5rad(≈±28.6°)内
    alpha_r = smooth_clamp(alpha_r, 0.5);
    
    // 4. 计算轮胎侧向力（线性模型，公式8和9）
    double Fyf = 2*params.cf * alpha_f;
    double Fyr = 2*params.cr * alpha_r;
    
    // 5. 计算纵向力
    double Fx = desired_accel * params.mass;
    
    // 6. 计算横向加速度
    if (std::abs(current_state.vx) < 0.1 && std::abs(desired_accel) < 0.01) {
        next_state.vy = 0;}
        else{
    double ay = (-(params.cf+params.cr)*current_state.vy
                -(params.lf*params.cr-params.lr*params.cf)*current_state.yaw_rate
                +params.cf*next_state.steering_angle*current_state.vx) / params.mass;
    next_state.vy += ay * dt;
    }
    // 7. 计算横摆角加速度（公式17）
    if (std::abs(current_state.vx) < 0.1 && std::abs(desired_accel) < 0.01) {
            std::cout<<"静止"<<std::endl;
        next_state.yaw_rate = 0;}
    else{
        double yaw_accel = (params.lf * Fyf * std::cos(next_state.steering_angle) - params.lr * Fyr) / params.iz;
        next_state.yaw_rate = params.damping * current_state.yaw_rate + yaw_accel * dt;
    }
    // 8. 计算纵向加速度（考虑空气阻力和滚动阻力）

    if (std::abs(current_state.vx) < 0.1 && std::abs(desired_accel) < 0.01) {
        next_state.vx = 0;}
    else{
        double air_drag = 0.5 * 1.225 * params.Cd * 2.0 * current_state.vx * std::abs(current_state.vx);
        double rolling_resistance = params.rr_coeff * params.mass * 9.81 * (current_state.vx > 0 ? 1 : -1);
        next_state.vx += (Fx - air_drag - rolling_resistance) / params.mass * dt;
    }
    // 9. 更新航向角（规范化到[-π, π]）
    next_state.yaw = current_state.yaw+current_state.yaw_rate * dt;
    next_state.yaw = std::atan2(std::sin(next_state.yaw),std::cos(next_state.yaw));
    //打印输出状态
    //std::cout<<next_state.yaw_rate<<"  "<<next_state.yaw<<"  "<<next_state.vx<<std::endl;
    // 10. 计算侧滑角（公式5）
    next_state.beta = (std::abs(vx_eff) > 0.1) ? std::atan2(next_state.vy, next_state.vx) : 0.0;
    
    // 11. 更新全局位置（公式1和2）
    double cos_yaw = std::cos(next_state.yaw);
    double sin_yaw = std::sin(next_state.yaw);
    next_state.x += (next_state.vx * cos_yaw - next_state.vy * sin_yaw) * dt;
    next_state.y += (next_state.vx * sin_yaw + next_state.vy * cos_yaw) * dt;
    
    // 12. 计算综合速度
    next_state.vp = std::hypot(next_state.vx, next_state.vy);
    
    // 13. 更新车轮速度
    next_state.wheel_speed = next_state.vx / params.wheel_radius;
    
    // 14. 处理静止状态
    if (std::abs(next_state.vx) < 0.1 && std::abs(desired_accel) < 0.01) {
        next_state.wheel_speed = 0;
    }

    
    return next_state;
}
 
ElectricVehicleDynamicsModel::VehicleState ElectricVehicleDynamicsModel::getState() const {
    return state_;
}
 
void ElectricVehicleDynamicsModel::reset(const VehicleState& new_state) {
    state_ = new_state;
}


void ElectricVehicleDynamicsModel::plot_vehicle(const std::string& color) {

    //绘制车辆历史轨迹
    plt::plot(traj_x,traj_y, {{"color", "blue"}, {"linestyle", "-"}, {"linewidth", "1"}});

    //绘制DWA预测轨迹
    if(state_.DWA_pre_traj.size()!=0)
    {
        plt::plot(state_.DWA_pre_traj[0],state_.DWA_pre_traj[1], {{"color", "cyan"}, {"linestyle", "-"},{"linewidth", "1"}});            
    }
   
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
    draw_wheel(params_.lf, half_track, this->state_.steering_angle, true);   // 左前（驱动轮）
    draw_wheel(params_.lf, -half_track, this->state_.steering_angle, true);  // 右前（驱动轮）
    draw_wheel(-params_.lr, half_track, 0, false);           // 左后
    draw_wheel(-params_.lr, -half_track, 0, false);          // 右后

    // 添加车辆方向指示箭头
    double arrow_length = 1.5 * std::max(params_.lf, params_.lr);  // 箭头长度
    double arrow_start_x = state_.x;
    double arrow_start_y = state_.y;
    double arrow_end_x = state_.x + arrow_length * std::cos(state_.yaw);
    double arrow_end_y = state_.y + arrow_length * std::sin(state_.yaw);
    
    // 绘制箭头主体 - 使用正确的参数格式
    plt::arrow(arrow_start_x, arrow_start_y, 
               arrow_end_x - arrow_start_x, arrow_end_y - arrow_start_y,
               "red",    // 填充颜色 (字符串)
               "r",       // 边缘颜色 (字符串)
               0.2,       // 头部长度 (数值)
               0.15);     // 头部宽度 (数值)

    // 可选：在车辆中心添加一个小圆点
    plt::plot({state_.x}, {state_.y}, "go");  // 蓝色圆点
}
 
void ElectricVehicleDynamicsModel::reset_for_planning_only(std::tuple<double,double,double>& new_state,
    std::vector<std::vector<double>> DWA_p_t) {
    state_.x = std::get<0>(new_state);
    state_.y = std::get<1>(new_state);
    state_.yaw = std::get<2>(new_state);
    if(DWA_p_t.size()!=0)
    {
        state_.DWA_pre_traj=DWA_p_t;      
    }
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

double ElectricVehicleDynamicsModel::get_vehicle_radias()
{
    return sqrt(pow((params_.lf+params_.lr)/2,2)+1);
}