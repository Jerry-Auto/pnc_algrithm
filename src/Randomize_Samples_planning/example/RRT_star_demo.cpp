
#include "RRT_star.h"

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
    std::pair<double, double> start = {0, 0};

    world_map.set_goal(goal);
    world_map.set_start(start);

    //world_map.visualize();

    double radius = 1.6;
    double expand_dis=3;//扩展的步长
    double goal_sample_rate=20;//采样目标点的概率，百分制.default: 5，即表示5%的概率直接采样目标点
    int max_iter=500;
    double connect_circle_dist = 5.0;
    bool search_until_max_iter = false;


    RRT_Star rrt(radius,expand_dis,goal_sample_rate, max_iter,connect_circle_dist,search_until_max_iter);

    rrt.init_solver(&world_map);

    pair<vector<double>, vector<double>>traj = rrt.planning(false);

    world_map.plot_Sampling_path(traj,"RRT_star.png");

    return 0;
}
