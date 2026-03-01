#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include<iostream>
#include<vector>
#include<cmath>
#include<algorithm>
#include <stdlib.h>
#include <time.h>
#include <map>
#include <unordered_map>
#include <queue>
#include <limits>  // 用于 std::numeric_limits
#include"ObstacleGridMap.h"


#define err 1e-4
class Dijkstra
{
    
public:
    struct Node{
            double x;
            double y;
            float cost;//起点到该节点的最小代价
            int parent_index;//与之相连的上一节点
        Node(double x, double y, float cost, int parentIndex);
    };

private:
    std::vector<std::vector<double>>motion=
    {{1, 0, 1},
    {0, 1, 1},
    {-1, 0, 1},
    {0, -1, 1},
    {-1, -1, sqrt(2)},
    {-1, 1, sqrt(2)},
    {1, -1, sqrt(2)},
    {1, 1, sqrt(2)}};//移动方向与代价

    bool Is_quit_map(const Dijkstra::Node& node) const;

    ObstacleGridMap* grid_map_=nullptr;

    bool plotfinalpath;

    bool plotpoint;

    bool plotgridpath;

    
    std::pair<std::vector<int>,std::vector<int>> final_path;

    void cal_fina_path(const Node& node,const std::unordered_map<int,Node>& closed_set);

    double Best_grid_length;
    //把最后的最短距离表记录下来
    std::unordered_map<int,Node> C_set;

public:
    Dijkstra(ObstacleGridMap* grid_map,bool plotfinalpath=true,bool plotpoint=true,bool plotgridpath=true);

    void planning(std::string PNGpath=" ");

    std::pair<std::vector<int>,std::vector<int>> GetGridPath();

    std::pair<std::vector<double>,std::vector<double>> GetWorldPath();

    double cost_to_point(double x,double y);

};

#endif // !DIJKSTRA_H