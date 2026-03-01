#include "AntColony.h"
#include <iostream>

#include <fstream>

#include <vector>

#include <string>

#include <sstream> // 用于字符串流解析


int main(int argc, char* argv[]) {
    std::string arg = argv[1];
    bool read;
    if (arg == "true" || arg == "1") {
        read = true;
    } else if (arg == "false" || arg == "0") {
        read = false;
    } else {
        std::cerr << "错误: 参数必须是 'true'/'false' 或 '1'/'0'" << std::endl;
        return 1;
    }
        //地图信息生成
    double world_width = 300;
    double world_height = 200;
    double resolution = 6;
    ObstacleGridMap grid_map(world_width, world_height, resolution);
    grid_map.setRectangleObstacle(0,30,110,10);
    grid_map.setRectangleObstacle(0,100,10,60);
    grid_map.setRectangleObstacle(150,0,10,75);
    grid_map.setRectangleObstacle(40,65,110,10);
    grid_map.setRectangleObstacle(40,100,155,10);
    grid_map.setRectangleObstacle(225,40,10,70);
    grid_map.setRectangleObstacle(180,130,10,50);
    grid_map.setRectangleObstacle(180,30,10,70);
    grid_map.setRectangleObstacle(40,140,10,60);
    grid_map.setRectangleObstacle(50,140,60,10);
    grid_map.setRectangleObstacle(world_width*3/4,3*world_height/4,world_width/8,10);
    grid_map.setRectangleObstacle(world_width*3/4,3*world_height/4,10,world_height/4);
    grid_map.setRectangleObstacle(world_width*9/10,0,world_width*1/10,world_height*2/4);
    //grid_map.plotWorldMap();
    grid_map.set_robot_radius(1.5);
    grid_map.setstr(30,15);
    grid_map.setgoal(290,195);

    const std::string filename = "./image/data.csv";
    if(!read)
    {   
        AntColony ACO(&grid_map,4000,1.5,4.0,0.2,80);
        ACO.set_ant_num(300);

        ACO.planning("ant.png",true,false);

        std::vector<double> originalData =ACO.GetAllGridLength();
        // 写入 CSV
        
        grid_map.writeVectorToCSV(originalData, filename);

        // 从 CSV 读取
        std::vector<double> loadedData = grid_map.readCSVToVector(filename);
        std::vector<double> x,y;
        for (size_t i=0;i<loadedData.size();i++) {
            x.push_back(i);
            if(loadedData[i]>10000)
            {y.push_back(280);}
            else{
                y.push_back(loadedData[i]);
            }
        }
        std::pair<std::vector<double>, std::vector<double>> curve={x, y};
        
        grid_map.plot_extra_curve(curve);
    }
    else
    {
        // 从 CSV 读取
        std::vector<double> loadedData = grid_map.readCSVToVector(filename);
        std::vector<double> x,y;
        double limit=(world_width+world_height)*3/5;
        for (size_t i=0;i<loadedData.size();i++) {
            x.push_back(i);
            if(loadedData[i]>limit)
            {y.push_back(limit);}
            else{
                y.push_back(loadedData[i]);
            }
        }
        std::pair<std::vector<double>, std::vector<double>> curve={x, y};
        std::cout<<"最短距离为："<<*std::min_element(y.begin(), y.end())<<std::endl;
        grid_map.plot_extra_curve(curve);
        
    }
    
    return 0;
}