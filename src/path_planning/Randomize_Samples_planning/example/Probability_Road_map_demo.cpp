#include "Probability_Road_map.h"


int main() {
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
    std::pair<double, double> start = {0, 0};

    world_map.set_goal(goal);
    world_map.set_start(start);

    //world_map.visualize();
    // 参数设置
    int N_SAMPLE = 500;
    int N_KNN = 10;
    double MAX_EDGE_LEN = 5.0;
    double robot_r=1.6;


    // 规划
    PRMPlanner planner(N_SAMPLE, N_KNN, MAX_EDGE_LEN,robot_r, true);

    planner.init_solver(&world_map);

    auto planning_data = planner.plan();
    if(planning_data.first.size()!=0)
    {
        std::cout<<"找到了路径"<<std::endl;
        world_map.plot_Sampling_path(planning_data,"PRM.png");
    }
    else{
        std::cout<<"没有找到路径"<<std::endl;
    }
    return 0;
}