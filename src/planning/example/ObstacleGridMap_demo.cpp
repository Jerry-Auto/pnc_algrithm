#include <iostream>
#include "ObstacleGridMap.h"  // 假设 ObstacleGridMap 类的头文件是 ObstacleGridMap.h

int main() {
    // 使用正确的构造函数初始化 ObstacleGridMap 对象
    double world_width = 135.78;
    double world_height = 120.66;
    double resolution = 3.5;

    // 创建 ObstacleGridMap 对象
    ObstacleGridMap grid_map(world_width, world_height, resolution);

    // 获取栅格地图的宽度和高度
    int grid_width = grid_map.getGridWidth();
    int grid_height = grid_map.getGridHeight();

    // 初始化网格的概率数据
    for (int gy = 0; gy < grid_height; ++gy) {
        for (int gx = 0; gx < grid_width; ++gx) {
            float prob = 0.5f; // 设置为未知区域
            grid_map.setCellProbability(gx, gy, prob); // 使用 setCellProbability 而不是 setGridProbability
        }
    }

    // 设置障碍物区域
    // 假设我们想在 (2, 2) 到 (4, 4) 的区域设置障碍物
    for (int gy = 2; gy <= 4; ++gy) {
        for (int gx = 2; gx <= 4; ++gx) {
            grid_map.setCellProbability(gx, gy, 0.9f); // 高概率表示障碍物
        }
    }

    // 设置自由空间区域
    // 假设我们想在 (6, 6) 到 (8, 8) 的区域设置自由空间
    for (int gy = 6; gy <= 8; ++gy) {
        for (int gx = 6; gx <= 8; ++gx) {
            grid_map.setCellProbability(gx, gy, 0.1f); // 低概率表示自由空间
        }
    }

    // 测试世界坐标下的地图绘制
    std::cout << "Plotting world coordinates map..." << std::endl;
    grid_map.plotWorldMap();
    
        // 准备路径数据
    std::vector<double> x_coords = {10.0, 20.0, 30.0, 50.0}; // x 坐标
    std::vector<double> y_coords = {10.0, 20.0, 30.0, 40.0}; // y 坐标
    std::pair<std::vector<double>, std::vector<double>> path_data(x_coords, y_coords);
 
    // 绘制路径
    grid_map.plotpath(path_data);

    //绘制点
    std::vector<double> x_coord;
    std::vector<double> y_coord;

    for(int i=2;i<5;i++)
    {
        x_coord.push_back(i);
        y_coord.push_back(i+10);
    }
    
    for(std::vector<double>::size_type i=0;i<x_coord.size();i++)
    {
        grid_map.plotpoint(x_coord[i],y_coord[i]);
    }

    // 先画点再绘制路径
    grid_map.plotpath(path_data);


    grid_map.plotWorldMap();


    return 0;
}