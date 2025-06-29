#include "AntColony.h"


int main() {
    //地图信息生成
    double world_width = 300;
    double world_height = 200;
    double resolution = 20;
    ObstacleGridMap grid_map(world_width, world_height, resolution);
    grid_map.setRectangleObstacle(0,30,110,10);
    grid_map.setRectangleObstacle(0,100,10,60);
    grid_map.setRectangleObstacle(150,20,10,55);
    grid_map.setRectangleObstacle(40,65,110,10);
    grid_map.setRectangleObstacle(40,100,155,10);
    grid_map.setRectangleObstacle(225,40,10,70);
    grid_map.setRectangleObstacle(180,130,10,50);
    grid_map.setRectangleObstacle(40,140,10,60);
    grid_map.setRectangleObstacle(50,140,60,10);
    grid_map.setRectangleObstacle(world_width*3/4,3*world_height/4,world_width/8,10);
    grid_map.setRectangleObstacle(world_width*3/4,3*world_height/4,10,world_height/4);
    grid_map.setRectangleObstacle(world_width*9/10,0,world_width*1/10,world_height*2/4);
    //const std::vector<float>& grid_data=grid_map.getGridData();
    grid_map.plotWorldMap();
    grid_map.set_robot_radius(1.5);
    grid_map.setstr(30,15);
    grid_map.setgoal(290,195);

    AntColony ACO(&grid_map);
    ACO.planning();
    return 0;
}