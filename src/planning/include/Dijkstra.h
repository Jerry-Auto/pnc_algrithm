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
    vector<vector<double>>motion;//移动方向与代价

};

#endif // !DIJKSTRA_H