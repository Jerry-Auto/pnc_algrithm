#include"Artificial_potential_field.h"

int main(){
    //创建地图并设置障碍物
    WorldMap world_map(-10, 50, -10, 50);
    world_map.addObstacle({20, 0, 3, 3, 0, "red"});
    world_map.addObstacle({15, 16, 4, 4, 0, "red"});
    world_map.addObstacle({8, 20, 3, 3, 0, "red"});
    world_map.addObstacle({0, 10, 5, 3, 0, "red"});  
    world_map.addObstacle({35, 30, 4, 6, M_PI/4, "green"});
    world_map.addObstacle({35, 18, 2, 8, 0, "purple"});
    world_map.addObstacle({25, 35, 3, 3, 0, "purple"});
    world_map.addObstacle({25, 25, 5, 5, 0, "purple"});
    //设置目标点
    std::pair<double, double> goal = {40, 40};

    world_map.set_goal(goal);

    ElectricVehicleDynamicsModel vehicle;

    world_map.addVehicle(&vehicle,"green");

    //world_map.visualize();

    APF apf_solver(world_map);

    auto planning_data=apf_solver.plan_series();
    world_map.plot_planning(&vehicle,planning_data,"apf.png");

    //apf_solver.plot_planning(&world_map,&vehicle);
    return 0;
}