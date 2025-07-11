#include"DWA_planner.h"

// 主函数
int main() {
    WorldMap world_map(-10, 50, -10, 50);
    world_map.addObstacle({15, 15, 3, 3, 0, "red"});
    world_map.addObstacle({25, 20, 4, 4, 0, "red"});
    world_map.addObstacle({35, 15, 3, 3, 0, "red"});
    
    ElectricVehicleDynamicsModel vehicle;
    ElectricVehicleDynamicsModel::VehicleState initial_state = {0, 0, 0, 0, 0, 0, 0};
    vehicle.reset(initial_state);
    world_map.addVehicle(&vehicle,"green");
    DWAPlanner::Config config;
    DWAPlanner planner(config);
    
    std::pair<double, double> goal = {40, 40};
    world_map.set_goal(goal);
    world_map.visualize();
    auto current_state = vehicle.getState();
    auto [best_v, best_yaw_rate, best_traj] = planner.plan(
        current_state, goal, world_map.get_Obstacle(), vehicle);
    
    // 使用reset_for_planning_only更新状态
    std::tuple<double, double, double> new_state = {
        best_traj[1].back(), 
        best_traj[2].back(), 
        best_traj[3].back()
    };
        
    // for (int i = 0; i < 100; ++i) {
    //     auto current_state = vehicle.getState();
    //     std::tuple<double, double, double> state = {
    //         current_state.x, 
    //         current_state.y, 
    //         current_state.yaw
    //     };
    //     auto [best_v, best_yaw_rate, best_traj] = planner.plan(
    //         current_state, goal, world_map.get_Obstacle(), vehicle);
        
    //     // 使用reset_for_planning_only更新状态
    //     std::tuple<double, double, double> pre_state = {
    //         best_traj[1].back(), 
    //         best_traj[2].back(), 
    //         best_traj[3].back()
    //     };
    //     vehicle.reset_for_planning_only(new_state,pre_state);
        
    //     // 使用plot_planning进行可视化
    //     world_map.plot_planning(&vehicle, best_traj, "dwa_path.png");
        
    //     double dist_to_goal = std::hypot(current_state.x - goal.first, 
    //                                     current_state.y - goal.second);
    //     if (dist_to_goal < 1.0) {
    //         std::cout << "Goal reached!" << std::endl;
    //         break;
    //     }
        
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    
    return 0;
}