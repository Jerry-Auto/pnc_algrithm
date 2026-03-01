#include"Dijkstra.h"

Dijkstra::Node::Node(double x, double y, float cost, int parentIndex) : x(x), y(y), cost(cost), parent_index(parentIndex) {}

Dijkstra::Dijkstra(ObstacleGridMap* grid_map,bool plotfinalpath,bool plotpoint,bool plotgridpath)
:grid_map_(grid_map),plotfinalpath(plotfinalpath),plotpoint(plotpoint),plotgridpath(plotgridpath){}

bool Dijkstra::Is_quit_map(const Dijkstra::Node& node) const
{
    //判断是否在地图内部
    std::pair<double,double> world_coord;
    this->grid_map_->gridToWorld(node.x,node.y,world_coord.first,world_coord.second);
    if(world_coord.first>this->grid_map_->getWorldWidth()||world_coord.first<0||world_coord.second>this->grid_map_->getWorldHeight()||world_coord.second<0)
    {
        return true;
    }
    //判断有无障碍物
    std::vector<float> grid_from_map=this->grid_map_->getGridData();
    if(grid_from_map[this->grid_map_->coordToIndex(node.x,node.y)]>0.8)
    {
        return true;
    }
    return false;
}

void Dijkstra::planning(std::string PNGpath){
    this->C_set.clear();
    this->final_path = {{}, {}};

    const Dijkstra::Node str_node(this->grid_map_->getstr().first,this->grid_map_->getstr().second,0,-1);
    Dijkstra::Node goal_node(this->grid_map_->getgoal().first,this->grid_map_->getgoal().second,0,-1);

    struct HeapElem { Node node; int idx; };
    struct HeapCmp { bool operator()(HeapElem const&a, HeapElem const&b) const { return a.node.cost > b.node.cost; } };
    std::priority_queue<HeapElem, std::vector<HeapElem>, HeapCmp> min_heap;

    const int start_id = this->grid_map_->coordToIndex(str_node.x, str_node.y);
    min_heap.push({str_node, start_id});

    while(!min_heap.empty()){
        HeapElem top = min_heap.top();
        min_heap.pop();
        int c_id = top.idx;
        Node current = top.node;

        auto closed_it = C_set.find(c_id);
        if(closed_it != C_set.end() && closed_it->second.cost <= current.cost){
            continue;
        }

        C_set.insert_or_assign(c_id, current);

        if(err>std::abs(current.x-goal_node.x)&&err>std::abs(current.y-goal_node.y)){
            goal_node.parent_index = current.parent_index;
            goal_node.cost = current.cost;
            break;
        }

        if(plotpoint){
            this->grid_map_->plotpoint(current.x, current.y, "+b");
        }

        for(const std::vector<double>& move : this->motion){
            Node neighbor(current.x + move[0], current.y + move[1], current.cost + move[2], c_id);
            int n_id = this->grid_map_->coordToIndex(neighbor.x, neighbor.y);
            if(C_set.find(n_id) != C_set.end()) continue;
            if(Is_quit_map(neighbor)) continue;
            min_heap.push({neighbor, n_id});
        }
    }

    if(goal_node.parent_index==-1){
        std::cout<<"遍历全图未找到到达终点的路径！"<<std::endl;
    }

    if(goal_node.parent_index!=-1){
        cal_fina_path(goal_node,C_set);
        std::cout<<"最短距离"<<this->Best_grid_length<<std::endl;
        if(this->plotfinalpath){
            if(PNGpath==" "){
                this->grid_map_->plotpath(this->GetGridPath());
            } else {
                this->grid_map_->plotpath(this->GetGridPath(),PNGpath);
            }
        }
    }
}


std::pair<std::vector<int>,std::vector<int>> Dijkstra::GetGridPath()
{
    return this->final_path;
}

void Dijkstra::cal_fina_path(const Node& node,const std::unordered_map<int,Node>& closed_set)
{
    std::vector<int>rx,ry;
    rx.push_back(node.x);
    ry.push_back(node.y);
    this->Best_grid_length=node.cost;
    int parent_index=node.parent_index;
    while(parent_index!=-1)
    {
        auto it = closed_set.find(parent_index);
        if (it == closed_set.end())
        {
            break;
        }
        const Node& temp_node=it->second;
        rx.push_back(temp_node.x);
        ry.push_back(temp_node.y);
        parent_index=temp_node.parent_index;
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


double Dijkstra::cost_to_point(double x,double y)
{
    std::pair<int, int> grid_coord;
    this->grid_map_->worldToGrid(x, y, grid_coord.first, grid_coord.second); 
    auto it = C_set.find(this->grid_map_->coordToIndex(grid_coord.first,grid_coord.second));
    if (it == C_set.end())
    {
        return std::numeric_limits<double>::infinity();
    }
    return it->second.cost*this->grid_map_->getResolution();
}
