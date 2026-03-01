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
#include <utility>

class AntColony
{
public:
    struct Node{
            int x;
            int y;
        Node(int x, int y);
    };

private:

    struct AntState {
        int current_idx;
        int target_idx;
        int prev_idx;
        std::vector<int> segment_path;
        double segment_length;
    };

    // 蚁群算法参数
    int num_of_iterations ;        // 模拟步数
    int num_of_ant;         // 蚂蚁数量
    double Alpha; // 信息素重要程度
    double Beta;  // 启发式信息重要程度
    double Rho;   // 信息素蒸发系数
    double Q;     // 每次旅程的信息素总量
    ObstacleGridMap* grid_map_=nullptr;//障碍物地图

    int nodetoindex(Node* node);

    std::vector<int> bestPath;//最优路径
    double bestLength = std::numeric_limits<double>::max();//最短距离

    std::vector<std::vector<int>> All_Best_path;//历代最优路径

    std::vector<double> All_Best_length;//历代最短距离

//邻接矩阵（节点之间的代价）vector<vector<double>> width*height，width*height
//启发式信息（节点数） vector<double> width*height,1
//信息素矩阵（节点之间的信息素）vector<vector<double>>width*height，width*height
    std::vector<std::vector<double>>motion=
    {{1, 0, 1},
    {0, 1, 1},
    {-1, 0, 1},
    {0, -1, 1},
    {-1, -1, sqrt(2)},
    {-1, 1, sqrt(2)},
    {1, -1, sqrt(2)},
    {1, 1, sqrt(2)}};//移动方向与代价

    std::vector<std::vector<double>> Pheromone;//信息素矩阵
    
    int free_pnt_num(ObstacleGridMap* map);//辅助函数，计算可行点数量，用于初始化蚂蚁数量

    double getrandm(double a,double b);//在【a,b）之间生成随机数

    double calcu_dis_to_target(Node* node,int target_idx);//计算节点到目标点的距离

    int rotate_wheel_slt(const std::vector<double>& prob);//轮盘赌确定移动方向

    void update_pheromono(const std::vector<std::pair<int,int>>& step_edges,const std::vector<double>& step_lengths);
public:

    AntColony(ObstacleGridMap* grid_map,int num_i=100,double alp=2,double bet=4.0,double rho=0.3,double q=8);

    void set_ant_num(int n);

    bool Is_quit_map(Node* node);

    void planning(std::string PNGpath=" ",bool plotfinalpath=true,bool plotgridpath=true);

    std::pair<std::vector<int>,std::vector<int>> GetBestGridPath();

    std::vector<std::pair<std::vector<int>,std::vector<int>>> GetAllGridPath();

    std::vector<double> GetAllGridLength();

    double GetBestGridLength();
};


#endif // !ANTCOLONY_H
