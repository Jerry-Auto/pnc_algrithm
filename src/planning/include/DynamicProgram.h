#ifndef DYNAMICPROGRAM_H
#define DYNAMICPROGRAM_H


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


class DynamicProgram
{
public:

    struct Node
    {
        int x;
        int y;
        double cost;
        int parent_index;
        Node(int x , int y , double cost , int parent_index);
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

    ObstacleGridMap* grid_map_;

    std::map<int,Node*> DPArray;//DP数组

    bool Is_quit_map(Node* node);

    void createnode(std::map<int,Node*>& DP_init,Node* current_node);//当前节点为中心，开辟下一阶段的状态节点

    double best_grid_length;

public:
    DynamicProgram(ObstacleGridMap* grid_map);

    void planning(std::string PNGpath=" ");






};







#endif // !DYNAMICPROGRAM_H