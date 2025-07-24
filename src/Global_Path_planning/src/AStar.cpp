#include"AStar.h"

AStar::Node::Node(double x, double y, float cost, int parentIndex) : x(x), y(y), cost(cost), parent_index(parentIndex) {}

AStar::AStar(ObstacleGridMap* grid_map,bool plotfinalpath,bool plotpoint,bool plotgridpath)
:grid_map_(grid_map),plotfinalpath(plotfinalpath),plotpoint(plotpoint),plotgridpath(plotgridpath){

    this->str_node=new AStar::Node(this->grid_map_->getstr().first,this->grid_map_->getstr().second,0,-1);
    
    this->goal_node=new AStar::Node(this->grid_map_->getgoal().first,this->grid_map_->getgoal().second,0,-1);
}

bool AStar::Is_quit_map(AStar::Node* node)
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

void AStar::planning(std::string PNGpath){
    std::map<int,Node*> O_set,C_set;

    O_set[this->grid_map_->coordToIndex(str_node->x,str_node->y)]=str_node;
    Node* current=nullptr;
    while(true)
    {
        float temp_cost=std::numeric_limits<float>::max();
        int c_id=std::numeric_limits<int>::max();
        //开集里寻找最小代价点
        for(auto it=O_set.begin();it!=O_set.end();it++)
        {
            if(it->second->cost+cal_h_x(it->second)<temp_cost)
            {
                //开集比较f(x)=g(x)+h(x)
                temp_cost=it->second->cost+cal_h_x(it->second);
                c_id=it->first;
            }
        }
        current=O_set[c_id];
        //把当前节点从开集移动到闭集
        auto iter = O_set.find(c_id);
        O_set.erase(iter);
        C_set[c_id]=current;
        //如果到达终点
        if(err>std::abs(current->x-goal_node->x)&&err>std::abs(current->y-goal_node->y))
        {
            goal_node->parent_index=current->parent_index;
            goal_node->cost=current->cost;
            break;
        }
        std::cout<<"当前节点最小代价："<<current->cost+cal_h_x(current)<<std::endl;
        //可视化
        if(plotpoint)
        {
            this->grid_map_->plotpoint(current->x,current->y,"+b"); 
        }
        //开辟八个移动方向作为新的节点加入开集
        for(std::vector<double>move:this->motion)
        {
            //cost即f(x),现在要加上h(x)，注意node里面的cost仍然是从起点到node的最短距离，h(x)不做累加
            Node*node=new Node(current->x+move[0],current->y+move[1],current->cost+move[2],c_id);


            int n_id=this->grid_map_->coordToIndex(node->x,node->y);
            //如果在闭集里找到了该节点，不保存，跳过，闭集里的代价都是最小的
            if(C_set.find(n_id)!=C_set.end()){continue;}
            //超出地图或者碰到障碍物，跳过
            if(Is_quit_map(node)){continue;}
            //如果在开集里找到了该节点，比较代价大小，留下小的
            if(O_set.find(n_id)!=O_set.end())
            {
                //比较f(x)的大小，更新开集重复节点，取代价最小的
                if(node->cost+cal_h_x(node)<O_set[n_id]->cost+cal_h_x(O_set[n_id]))
                {
                    O_set[n_id]=node;
                }
            }
            //完全新的可行点直接放到开集
            else{O_set[n_id]=node;}
        }       
    }

    //通过闭集和终点反推路径
    cal_fina_path(goal_node,C_set);
    std::cout<<"最短距离"<<this->Best_grid_length<<std::endl;
    if(this->plotfinalpath)
    {
        if(PNGpath==" ")
        {   
        this->grid_map_->plotpath(this->GetGridPath()); 
        }
        else{
            this->grid_map_->plotpath(this->GetGridPath(),PNGpath);
        }           
    }

}


std::pair<std::vector<int>,std::vector<int>> AStar::GetGridPath()
{
    return this->final_path;
}
 void AStar::cal_fina_path(Node* node,std::map<int,Node*> closed_set)
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
 std::pair<std::vector<double>,std::vector<double>> AStar::GetWorldPath()
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

double AStar::cal_h_x(Node* node)
{
    double h_x=this->weight*sqrt(pow(node->x-this->goal_node->x,2)+pow(node->y-this->goal_node->y,2));//欧几里德距离
    //double h_x=this->weight*(abs(node->x-this->goal_node->x)+abs(node->y-this->goal_node->y))//曼哈顿距离
    return h_x;
}

void AStar::set_h_weight(double w)
{
    this->weight=w;
}