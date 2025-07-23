#include"ObstacleGridMap.h"

int main()
{
        //创建地图并设置障碍物
    WorldMap world_map(0, 100, 0, 100);
    world_map.addObstacle({20, 0, 3, 3, 0, "red"});
    world_map.addObstacle({15, 16, 4, 4, 0, "red"});
    world_map.addObstacle({8, 20, 3, 3, 0, "red"});
    world_map.addObstacle({0, 10, 5, 3, 0, "red"});  
    world_map.addObstacle({35, 30, 4, 6, M_PI/4, "green"});
    world_map.addObstacle({35, 18, 2, 8, 0, "purple"});
    world_map.addObstacle({25, 35, 3, 3, 0, "purple"});
    world_map.addObstacle({25, 25, 5, 5, 0, "purple"});
    //设置目标点
    world_map.set_goal({88, 76});
    world_map.set_start({12,8});
    world_map.visualize();

    ObstacleGridMap grid_map(world_map,0.5,1.5);

    grid_map.plotWorldMap();

    return 0;
}
