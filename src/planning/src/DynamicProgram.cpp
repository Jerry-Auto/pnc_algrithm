#include"DynamicProgram.h"

DynamicProgram::Node::Node(int x,int y,double cost, int parent_index):x(x),y(y),cost(cost),parent_index(parent_index){}

DynamicProgram::DynamicProgram(ObstacleGridMap* grid_map):grid_map_(grid_map){}

bool DynamicProgram::Is_quit_map(Node* node)
{
    //判断是否在地图内部
    std::pair<double,double> world_coord;
    this->grid_map_->gridToWorld(node->x,node->y,world_coord.first,world_coord.second);
    if(world_coord.first>this->grid_map_->getWorldWidth()||world_coord.first<0||world_coord.second>this->grid_map_->getWorldHeight()||world_coord.second<0)
    {
        return true;
    }
    //判断有无障碍物
    std::vector<float> grid_from_map=this->grid_map_->getGridData();
    if(grid_from_map[this->grid_map_->coordToIndex(node->x,node->y)]>0.8)
    {
        return true;
    }
    return false;
}

void DynamicProgram::planning(std::string PNGpath)
{
    Node* str_node=new Node(this->grid_map_->getstr().first,this->grid_map_->getstr().second,0,-1);
    Node* goal_node=new Node(this->grid_map_->getgoal().first,this->grid_map_->getgoal().second,0,-1);
    int g_id=this->grid_map_->coordToIndex(goal_node->x,goal_node->y);
    std::map<int,Node*> DP_current;
    std::map<int,Node*> DP_next;
    DP_next[this->grid_map_->coordToIndex(str_node->x,str_node->y)]=str_node;

    while(true)
    {
        if(DP_current.find(g_id)!=DP_current.end())
        {
            this->best_grid_length=DP_current[g_id]->cost;
            goal_node->parent_index=DP_current[g_id]->parent_index;
            goal_node->cost=DP_current[g_id]->cost;
            break;
        }
        for(auto it = this->DP_current.begin(); it != this->DP_current.end(); ++it)
        {
            this->createnode(DP_next,it->second);
        }
        this->DPArray.insert(DP_current.begin(),DP_current.end());
        DP_current.clear();
        DP_current=DP_next;
        DP_next.clear();
    }
    std::cout<<"最优距离为："<<this->best_grid_length<<std::endl;
}

void DynamicProgram::createnode(std::map<int,Node*>& DP_init,Node* current_node)
{
    int c_id=this->grid_map_->coordToIndex(current_node->x,current_node->y);
    for(auto move:this->motion)
    {
        Node*node=new Node(current_node->x+move[0],current_node->y+move[1],current_node->cost+move[2],c_id);
        int n_id=this->grid_map_->coordToIndex(node->x,node->y);
        if(this->Is_quit_map(node)){continue;};
        if(DP_init.find(n_id)!=DP_init.end())
        {
            if(DP_init[n_id]->cost>node->cost)
            {
                DP_init[n_id]=node;
            }           
        }
        else{
            DP_init[n_id]=node;
        }   
    }
}

std::pair<std::vector<int>,std::vector<int>> DynamicProgram::GetBestGridPath()
{
    int g_id=this->grid_map_->coordToIndex(this->grid_map_->getstr().first,this->grid_map_->getstr().second);
    std::pair<std::vector<int>,std::vector<int>> grids;
    std::pair<std::pair<int,int>,float> grid;
    for(size_t i=0;i<this->bestPath.size();i++)
    {
        grid=grid_map_->index_to_grid(this->bestPath[i]);
        grids.first.push_back(grid.first.first);
        grids.second.push_back(grid.first.second);
    }
    return grids;
}