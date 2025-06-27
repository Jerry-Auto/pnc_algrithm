#include"Dijkstra.h"
#include"ObstacleGridMap.h"

int main() {
    //地图信息生成
    double world_width = 300;
    double world_height = 200;
    double resolution = 2;
    ObstacleGridMap grid_map(world_width, world_height, resolution);
    grid_map.setRectangleObstacle(0,30,110,10);
    grid_map.setRectangleObstacle(0,100,10,60);
    grid_map.setRectangleObstacle(150,0,10,75);
    grid_map.setRectangleObstacle(40,65,110,10);
    grid_map.setRectangleObstacle(40,100,185,10);
    grid_map.setRectangleObstacle(225,40,10,70);
    grid_map.setRectangleObstacle(180,110,10,70);
    grid_map.setRectangleObstacle(40,140,10,60);
    grid_map.setRectangleObstacle(50,140,60,10);
    grid_map.setRectangleObstacle(world_width*3/4,3*world_height/4,world_width/8,10);
    grid_map.setRectangleObstacle(world_width*3/4,3*world_height/4,10,world_height/4);
    const std::vector<float>& grid_data=grid_map.getGridData();
    grid_map.plotWorldMap();
    grid_map.set_robot_radius(5);
    
    //打印栅格矩阵
    // int i=0;
    // for (const auto& row : grid_data) {  // 遍历每一行             
    //    if(i==(grid_map.getGridWidth()-1))
    //    {
    //     i=0;
    //     std::cout <<std::endl;
    //    }
    //    std::cout << row << " ";  
    //    i++;
    // }

    grid_map.plotWorldMap();


    return 0;
}