

#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "Rapidly_exploring_Random_Tree.h"

class RRT_Star: public RRT{
public:

    double connect_circle_dist;
    bool search_until_max_iter;


    RRT_Star(double robotRadius, double expandDis, double goalSampleRate, int maxIter, double connectCircleDist,
             bool searchUntilMaxIter);

    pair<vector<double>, vector<double>> planning(bool plot);

    vector<int>findNearInds(Node* new_node);//找出邻近节点集合

    void propagateCostToLeaves(Node* parent_node );

    double calcNewCost(Node*from_node,Node*to_node);

    void rewire(Node* new_node, vector<int>near_inds);

    int findBestGoalInd();

    Node* chooseParent(Node* new_node, vector<int>near_inds);
};


#endif 
