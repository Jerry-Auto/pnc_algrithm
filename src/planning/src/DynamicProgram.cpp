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
    this->DPArray[this->grid_map_->coordToIndex(str_node->x,str_node->y)]=str_node;
    std::map<int,Node*> DP_init;

    while(true)
    {
        if(DPArray.find(g_id)!=DPArray.end())
        {
            this->best_grid_length=DPArray[g_id]->cost;
            break;
        }

        for(auto it = this->DPArray.begin(); it != this->DPArray.end(); ++it)
        {
            this->createnode(DP_init,it->second);
        }
        for (auto& pair : DPArray) {
            delete pair.second;
        }
        DPArray.clear();
        DPArray=DP_init;
        DP_init.clear();
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