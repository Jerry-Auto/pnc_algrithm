#include"Dijkstra.h"

Dijkstra::Node::Node(double x, double y, float cost, int parentIndex) : x(x), y(y), cost(cost), parent_index(parentIndex) {}
Dijkstra::Dijkstra(ObstacleGridMap* grid_map)
{
    this->grid_map_=grid_map;
}

bool Dijkstra::Is_quit_map(Dijkstra::Node* node)
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

void Dijkstra::planning(std::string PNGpath){
    Dijkstra::Node* str_node=new Dijkstra::Node(this->grid_map_->getstr().first,this->grid_map_->getstr().second,0,-1);
    Dijkstra::Node* goal_node=new Dijkstra::Node(this->grid_map_->getgoal().first,this->grid_map_->getgoal().second,0,-1);
    std::map<int,Node*> O_set,C_set;

    O_set[this->grid_map_->coordToIndex(str_node->x,str_node->y)]=str_node;
    Node* current=nullptr;
    while(true)
    {
        float temp_cost=std::numeric_limits<float>::max();
        int c_id=std::numeric_limits<int>::max();

        for(auto it=O_set.begin();it!=O_set.end();it++)
        {
            if(it->second->cost<temp_cost)
            {
                temp_cost=it->second->cost;
                c_id=it->first;
            }
        }
        current=O_set[c_id];

        auto iter = O_set.find(c_id);
        O_set.erase(iter);
        C_set[c_id]=current;
        
        if(err>std::abs(current->x-goal_node->x)&&err>std::abs(current->y-goal_node->y))
        {
            goal_node->parent_index=current->parent_index;
            goal_node->cost=current->cost;
            break;
        }
        std::cout<<"当前节点最小代价："<<current->cost<<std::endl;
        //this->grid_map_->plotpoint(current->x,current->y,"+b");
        for(std::vector<double>move:this->motion)
        {
            Node*node=new Node(current->x+move[0],current->y+move[1],current->cost+move[2],c_id);
            int n_id=this->grid_map_->coordToIndex(node->x,node->y);

            if(C_set.find(n_id)!=C_set.end()){continue;}

            if(Is_quit_map(node)){continue;}

            if(O_set.find(n_id)!=O_set.end())
            {
                if(node->cost<O_set[n_id]->cost)
                {
                    O_set[n_id]=node;
                }
            }
            else{O_set[n_id]=node;}
        }       
    }
    cal_fina_path(goal_node,C_set);
    std::cout<<"最短距离"<<this->Best_grid_length<<std::endl;
    if(PNGpath==" ")
    {   
       this->grid_map_->plotpath(this->GetGridPath()); 
    }
    else{
        this->grid_map_->plotpath(this->GetGridPath(),PNGpath);
    }   
}


std::pair<std::vector<int>,std::vector<int>> Dijkstra::GetGridPath()
{
    return this->final_path;
}
 void Dijkstra::cal_fina_path(Node* node,std::map<int,Node*> closed_set)
 {
    std::vector<int>rx,ry;
    rx.push_back(node->x);
    ry.push_back(node->y);
    this->Best_grid_length=node->cost;
    double parent_index=node->parent_index;
    while(parent_index!=-1)
    {
        Node* temp_node=closed_set[parent_index];
        rx.push_back(temp_node->x);
        ry.push_back(temp_node->y);
        parent_index=temp_node->parent_index;
    }
    this->final_path={rx,ry};
 }
 std::pair<std::vector<double>,std::vector<double>> Dijkstra::GetWorldPath()
{
    double wx,wy;
    std::pair<std::vector<double>,std::vector<double>> world_path;
    for(size_t i=0;i<this->final_path.first.size();i++)
    {
        this->grid_map_->gridToWorld(this->final_path.first[i],this->final_path.second[i],wx,wy);
        world_path.first.push_back(wx);
        world_path.second.push_back(wy);
    }
    return world_path;
}