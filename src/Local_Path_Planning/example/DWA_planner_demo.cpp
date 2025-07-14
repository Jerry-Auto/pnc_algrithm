#include"DWA_planner.h"

// 主函数
int main() {
    //创建地图并设置障碍物
    WorldMap world_map(-10, 50, -10, 50);
    world_map.addObstacle({15, 10, 3, 3, 0, "red"});
    world_map.addObstacle({25, 20, 4, 4, 0, "red"});
    world_map.addObstacle({40, 15, 3, 3, 0, "red"});
        // 添加障碍物（同前）
    WorldMap::Obstacle obs1 = {5, 10, 5, 3, 0, "red"};
    WorldMap::Obstacle obs2 = {40, 30, 4, 6, M_PI/4, "green"};
    WorldMap::Obstacle obs3 = {35, 5, 2, 8, 0, "purple"};
    world_map.addObstacle(obs1);  
    world_map.addObstacle(obs2);
    world_map.addObstacle(obs3);

    //设置目标点
    std::pair<double, double> goal = {40, 40};
    world_map.set_goal(goal);
    //world_map.visualize(true, true, true);
    //创建地图里运动的车辆
    ElectricVehicleDynamicsModel vehicle;
    world_map.addVehicle(&vehicle,"green");
    //world_map.visualize(true, true, true);

    //DWA求解器创建

    DWAPlanner DWA_solver;
    DWA_solver.read_map_data(world_map,vehicle);

    auto plan_data=DWA_solver.planning_series();
    world_map.plot_planning(&vehicle,plan_data.first,"DWA.png",plan_data.second);

    //DWA_solver.plot_planning(&world_map,&vehicle);
        
    return 0;
}