//#include "PolynomialAlgorithm.h"
#include "World_map.h"

int main() {
    WorldMap world(-50, 50, -50, 50);
    
    // 添加障碍物（同前）
    WorldMap::Obstacle obs1 = {10, 10, 5, 3, 0, "red"};
    WorldMap::Obstacle obs2 = {-15, 20, 4, 6, M_PI/4, "green"};
    WorldMap::Obstacle obs3 = {0, -10, 8, 2, 0, "purple"};
    world.addObstacle(obs1);  
    world.addObstacle(obs2);
    world.addObstacle(obs3);

    // 创建车辆
    ElectricVehicleDynamicsModel vehicle1;
    ElectricVehicleDynamicsModel vehicle2;
    
    // 设置车辆初始状态
    ElectricVehicleDynamicsModel::VehicleState state2;
    state2.x=30.0;
    state2.y=-15.0;
    state2.yaw=-M_PI/6;

    //vehicle1->reset(state1);
    vehicle2.reset(state2);
    
    // 添加车辆到世界
    world.addVehicle(&vehicle1, "blue");
    world.addVehicle(&vehicle2, "orange");

    // 首次可视化（阻塞模式）
    //world.visualize(true, true, true);

    // std::pair<std::vector<double> ,std::vector<double>> str_state,end_state;
    // double t0=0;
    // double t1=30;
    // std::vector<double> x_str={0,0,0};
    // std::vector<double> y_str={0,0,0};
    // std::vector<double> x_end={100,0,0};
    // std::vector<double> y_end={20,0,0};
    // str_state.first=x_str;
    // str_state.second=y_str;
    // end_state.first=x_end;
    // end_state.second=y_end;

    // PolynomialAlgorithm poly_solver(str_state,end_state,t0,t1);
    
    // std::vector<std::vector<double>> planning_data=poly_solver.planning_series(0.05);
    // //画图
    // world.plot_planning(&vehicle1,planning_data,"worldmap.png");

    return 0;
}
