#ifndef ANTCOLONY_H
#define ANTCOLOY_H

#include "ObstacleGridMap.h"
#include<iostream>
#include<vector>
#include<cmath>
#include<algorithm>
#include <stdlib.h>
#include <time.h>
#include <map>
#include <limits>  // 用于 std::numeric_limits
#include <random>

class AntColony
{

private:

    // 蚁群算法参数
    int num_of_iterations ;        // 迭代次数
    int num_of_ant;         // 蚂蚁数量
    double Alpha; // 信息素重要程度
    double Beta;  // 启发式信息重要程度
    double Rho;   // 信息素蒸发系数
    double Q;     // 每次旅程的信息素总量
    ObstacleGridMap* grid_map_=nullptr;//障碍物地图


    std::vector<int> bestPath;//最优路径
    double bestLength = std::numeric_limits<double>::max();//最短距离
    std::vector<std::vector<int>> All_Best_path;//历代最优距离
    std::vector<double> minPL(K, std::numeric_limits<double>::max());//历代最短距离

//邻接矩阵（节点之间的代价）vector<vector<double>> width*height，width*height
//启发式信息（节点数） vector<double> width*height,1
//信息素矩阵（节点之间的信息素）vector<vector<double>>width*height，width*height

    //辅助函数，计算可行点数量，用于初始化蚂蚁数量
    int free_pnt_num(ObstacleGridMap* map);
    double getrandm(double a,double b);//在【a,b）之间生成随机数
    double calcu_dis(int x1, int y1, int x2, int y2);//计算距离
    int rotate_wheel_slt();//轮盘赌确定移动方向


public:
    AntColony(ObstacleGridMap* grid_map,int num_i=100,double alp=1.0,double bet=7.0,double rho=0.3,double q=1);

    void set_ant_num(int n);


    ~AntColony();
};


#endif // !ANTCOLONY_H
