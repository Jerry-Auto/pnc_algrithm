#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include<iostream>
#include<vector>
#include<cmath>
#include<algorithm>
#include <stdlib.h>
#include <time.h>
#include <map>
#include"ObstacleGridMap.h"

class Dijkstra
{
    
public:
    struct Node{
            double x;
            double y;
            float cost;
            double parent_index;//与之相连的上一节点
        Node(double x, double y, float cost, double parentIndex);
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
    {1, 1, sqrt(2)}};;//移动方向与代价

    

};

#endif // !DIJKSTRA_H