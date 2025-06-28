#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include<iostream>
#include<vector>
#include<cmath>
#include<algorithm>
#include <stdlib.h>
#include <time.h>
#include <map>
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

    bool Is_quit_map(Dijkstra::Node* node);

    ObstacleGridMap* grid_map_=nullptr;
    std::pair<std::vector<int>,std::vector<int>> final_path;
    void cal_fina_path(Node* node,std::map<int,Node*> closed_set);

public:
    Dijkstra(ObstacleGridMap* grid_map);

    void planning();

    std::pair<std::vector<int>,std::vector<int>> GetGridPath();

    std::pair<std::vector<double>,std::vector<double>> GetWorldPath();


};

#endif // !DIJKSTRA_H