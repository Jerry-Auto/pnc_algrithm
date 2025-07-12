#ifndef WORLD_MAP_H

#define WORLD_MAP_H


#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>
#include <optional>
#include "matplotlibcpp.h"
#include "vehicle_model.h"

namespace plt = matplotlibcpp;

class WorldMap {
public:
    // 障碍物结构体
    struct Obstacle {
        double x;          // 中心x坐标
        double y;          // 中心y坐标
        double width;      // 宽度
        double height;     // 高度
        double rotation;   // 旋转角度(弧度)
        std::string color; // 显示颜色
    };

    // 构造函数
    
    WorldMap(double x_min = -100.0, double x_max = 100.0, 
             double y_min = -100.0, double y_max = 100.0);

    // 添加障碍物
    void addObstacle(const Obstacle& obstacle);

    std::vector<Obstacle> get_Obstacle();
    
    // 添加车辆
    void addVehicle(ElectricVehicleDynamicsModel* vehicle, 
                         const std::string& color) ;

    // 可视化整个世界（新增参数：是否阻塞显示）
    void visualize(bool show_grid = true, bool equal_aspect = true, bool blocking = true);

    // 更新车辆状态并重新绘制（新增方法）
    void updateAndVisualize(bool show_grid = true, bool equal_aspect = true);

    // 更新车辆状态并可视化
    void plot_planning(ElectricVehicleDynamicsModel* vehicle,std::vector<std::vector<double>> planning_data
        ,std::string filename="out.png",std::vector<std::vector<std::vector<double>>> DWA_p_t = {});
    

    // 设置边界
    void setBounds(double x_min, double x_max, double y_min, double y_max);
    
    //保存结果图
    void save_to_PNG(std::string filename);

    //添加控制点
    void add_control_point(std::pair<std::vector<double>,std::vector<double>> control_point);
    
    //设置目标点
    void set_goal(std::pair<double, double> goal);

    std::pair<double, double> get_goal();


    // 清除所有对象
    void clear();

private:
    bool is_interactive_ = false;  // 标记是否处于交互模式
    double x_min_, x_max_;
    double y_min_, y_max_;
    std::pair<double, double> goal_point={NAN, NAN};;
    std::vector<Obstacle> obstacles_;
    std::vector<std::pair<ElectricVehicleDynamicsModel*, std::string>> vehicles_;
    std::vector<std::pair<std::vector<double>,std::vector<double>>> control_point_;
};

#endif