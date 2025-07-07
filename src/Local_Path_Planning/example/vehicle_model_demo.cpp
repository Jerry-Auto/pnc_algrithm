#include"vehicle_model.h" 

int main() {
    // 创建模型对象（正确方式）
    ElectricVehicleDynamicsModel vehicle;
    
    // 模拟参数
    double dt = 0.01;  // 适当增大步长提高性能
    double sim_time = 20.0;
    int steps = static_cast<int>(sim_time / dt);
 
    // 输入序列
    std::vector<std::tuple<double, double>> inputs = {
        {0.0, 1.0},    // 直行加速
        {-0.1, 0.5},   // 左转向加速
        {0.0, 0.0},    // 匀速
        {0.0, -2.0},   // 减速
        {0.0, 0.0}     // 停止
    };
    // 存储轨迹
    std::vector<double> traj_x, traj_y;
    // 模拟循环
    for (int i = 0; i < steps; ++i) {
        // 切换输入（每4秒切换一次）
        int input_idx = (i / 400) % inputs.size();
        double steer = std::get<0>(inputs[input_idx]);
        double accel = std::get<1>(inputs[input_idx]);
        // 更新状态
        vehicle.update(steer, accel, dt);
        auto state = vehicle.getState();
        // 存储轨迹
        traj_x.push_back(state.x);
        traj_y.push_back(state.y);
        // 绘图
        if (i % 50 == 0) {
            plt::cla();
            plt::plot(traj_x, traj_y, {{"color", "blue"}, {"linestyle", "-"}, {"linewidth", "1"}});
            vehicle.plot_vehicle(state, steer);
            plt::xlim(state.x-10, state.x+10);
            plt::ylim(state.y-10, state.y+10);
            plt::pause(0.5);
        }
    }
    
    plt::show();
    return 0;
}