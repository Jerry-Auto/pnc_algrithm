#include "AntColony.h"

#define err 1e-4

AntColony::Node::Node(int x, int y) : x(x), y(y) {}
AntColony::AntColony(ObstacleGridMap* grid_map, int num_i, double alp, double bet, double rho, double q)
    : num_of_iterations(num_i),  // 先初始化声明顺序靠前的成员
      Alpha(alp),
      Beta(bet),
      Rho(rho),
      Q(q),
      grid_map_(grid_map) {     // 最后初始化 grid_map_
    this->num_of_ant = 1.5 * free_pnt_num(grid_map_);
    this->All_Best_length = std::vector<double>(this->num_of_iterations, std::numeric_limits<double>::max());
}

int AntColony::free_pnt_num(ObstacleGridMap* map)
{
    std::vector<float> gridmap=map->getGridData();
    int n=0;
    for(size_t i=0;i<gridmap.size();i++)
    {
        if(gridmap[i]<0.8)
        {n++;}
    }
    return n;
}

void AntColony::set_ant_num(int n)
{
    this->num_of_ant=n;
}

double AntColony::getrandm(double a,double b)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}
double AntColony::calcu_dis_to_end(Node* node)
{
    return sqrt(pow(node->x - this->grid_map_->getgoal().first, 2) + pow(node->y - this->grid_map_->getgoal().second, 2));
}
/// @brief 计算单个蚂蚁从起点走到终点的路径及距离
/// @param start 起点的一维索引
/// @param goal 终点的一维索引
/// @param size 禁忌表的尺寸即节点的个数
/// @return 
std::pair<double,std::vector<int>> AntColony::single_ant_go(Node* start,Node* goal,int size)
{
    double distance;
    std::vector<int> Path = {this->nodetoindex(start)};
    std::vector<int> TABU(size,1);
    TABU[this->nodetoindex(start)]=0;
    Node* current=start;
    while(this->nodetoindex(current)!=this->nodetoindex(goal))
    {
        std::pair<std::vector<Node*>,std::vector<double>> node_prob;
        //八个可行方向，计算各个方向的概率
        for(std::vector<double>move:this->motion)
        {
            Node*node=new Node(current->x+move[0],current->y+move[1]);
            int n_id=this->grid_map_->coordToIndex(node->x,node->y);
            if(TABU[n_id]==0){continue;}
            if(Is_quit_map(node)){continue;}
            node_prob.first.push_back(node);
            if(n_id==this->nodetoindex(goal))
            {
                node_prob.second.push_back(pow(this->Pheromone[this->nodetoindex(current)][this->nodetoindex(node)], this->Alpha) * pow(100.0, this->Beta));               
            }
            else{
                node_prob.second.push_back(pow(this->Pheromone[this->nodetoindex(current)][this->nodetoindex(node)], this->Alpha) * pow(1/calcu_dis_to_end(node), this->Beta));  
            }
        }
        // 如果没有可选节点，退出
        if (node_prob.second.empty()) {break;}
        double sumP=accumulate(node_prob.second.begin(), node_prob.second.end(), 0.0);
        for(auto&p : node_prob.second)
        {
            p /= sumP;
        }
        Node* next=node_prob.first[rotate_wheel_slt(node_prob.second)];
        distance+=(std::abs(current->x-next->x)+std::abs(current->y-next->y)==1)?1.0:std::sqrt(2.0);
        current=next;
        Path.push_back(this->nodetoindex(current));
        TABU[this->nodetoindex(current)]=0;
    }
    return {distance,Path};
}

int AntColony::rotate_wheel_slt(const std::vector<double>& prob) {
    double r =getrandm(0.0,1.0);
    double sum = 0.0;
    for (size_t i = 0; i < prob.size(); ++i) {
        sum += prob[i];
        if (r <= sum) {
            return static_cast<int>(i);
        }
    }
    return static_cast<int>(prob.size()) - 1;
}


bool AntColony::Is_quit_map(Node* node)
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


void AntColony::planning(std::string PNGpath)
{
    AntColony::Node* str_node=new AntColony::Node(this->grid_map_->getstr().first,this->grid_map_->getstr().second);
    AntColony::Node* goal_node=new AntColony::Node(this->grid_map_->getgoal().first,this->grid_map_->getgoal().second);
    int N=this->grid_map_->getGridHeight()*this->grid_map_->getGridWidth();
    this->Pheromone = std::vector<std::vector<double>>(N, std::vector<double>(N, 8.0));
    for(int i=0;i<this->num_of_iterations;i++)
    {
        std::vector<std::vector<int>> ROUTES(this->num_of_ant);
        std::vector<double> PL(this->num_of_ant, 0.0);
        for(int j=0;j<this->num_of_ant;j++)
        {
            std::pair<double,std::vector<int>> dis_path=single_ant_go(str_node,goal_node,N);
            //记录最小及路径
            ROUTES[j] = dis_path.second;
            if(dis_path.second.back()==this->nodetoindex(goal_node))
            {
                PL[j] = dis_path.first;
                if (PL[j]< this->bestLength) {
                    this->bestLength = PL[j];
                    this->bestPath = ROUTES[j];
                }
            }
            else{
                PL[j] = std::numeric_limits<double>::max();
            }
            
        }
        //更新信息素
        update_pheromono(ROUTES,PL);
        //记录当前代最优路径及距离,记录历代路径
        auto min_it = std::min_element(PL.begin(), PL.end());
        // 计算索引：距离起始迭代器的偏移量
        int min_index = std::distance(PL.begin(), min_it);
        this->All_Best_length.push_back(*min_it);
        this->All_Best_path.push_back(ROUTES[min_index]);
        std::cout <<"第"<<i+1<< "次迭代最短距离: " << *min_it << std::endl;
        
    }
        // 输出结果
    std::cout << "全局最优距离: " << bestLength << std::endl;
    //this->grid_map_->plot_iterate_path(this->GetAllGridPath());
    if(PNGpath==" ")
    {   
       this->grid_map_->plotpath(this->GetBestGridPath()); 
    }
    else
    {       
        this->grid_map_->plotpath(this->GetBestGridPath(),PNGpath);
    } 
    
}

void AntColony::update_pheromono(std::vector<std::vector<int>> ants_ROUTES_per_round,std::vector<double> ants_PL_per_round)
{

    // 更新信息素
    std::vector<std::vector<double>> Delta_Tau(this->Pheromone.size(), std::vector<double>(this->Pheromone.size(), 0.0));
    for (int m = 0; m < this->num_of_ant; ++m) {
        if (ants_PL_per_round[m] != std::numeric_limits<double>::max()) {
            for (size_t s = 0; s < ants_ROUTES_per_round[m].size() - 1; ++s) {
                int x = ants_ROUTES_per_round[m][s];
                int y = ants_ROUTES_per_round[m][s + 1];
                Delta_Tau[x][y] += this->Q / ants_PL_per_round[m];
                Delta_Tau[y][x] += this->Q / ants_PL_per_round[m];
            }
        }
    }
    // 信息素挥发和增强
    for (size_t i = 0; i < this->Pheromone.size(); ++i) {
        for (size_t j = 0; j < this->Pheromone.size(); ++j) {
            this->Pheromone[i][j] = (1 - this->Rho) * this->Pheromone[i][j] + Delta_Tau[i][j];
        }
    }
}

int AntColony::nodetoindex(Node* node)
{
    return grid_map_->coordToIndex(node->x,node->y);
}

std::pair<std::vector<int>,std::vector<int>> AntColony::GetBestGridPath()
{
    std::pair<std::vector<int>,std::vector<int>>grids;
    std::pair<std::pair<int,int>,float> grid;
    for(size_t i=0;i<this->bestPath.size();i++)
    {
        grid=grid_map_->index_to_grid(this->bestPath[i]);
        grids.first.push_back(grid.first.first);
        grids.second.push_back(grid.first.second);
    }
    return grids;
}

std::vector<std::pair<std::vector<int>,std::vector<int>>> AntColony::GetAllGridPath()
{
    std::vector<std::pair<std::vector<int>,std::vector<int>>> all_grids;
    std::pair<std::pair<int,int>,float> grid_1;
    std::pair<std::vector<int>,std::vector<int>> grid_coord;
    for(size_t i=0;i<this->All_Best_path.size();i++)
    {
        for(size_t j=0;j<this->All_Best_path[i].size();j++)
        {
            grid_1=grid_map_->index_to_grid(this->All_Best_path[i][j]);
            grid_coord.first.push_back(grid_1.first.first);
            grid_coord.second.push_back(grid_1.first.second);      
        }
        all_grids.push_back(grid_coord);
        grid_coord.first=std::vector<int>();
        grid_coord.second=std::vector<int>();
    }
    return all_grids;
}
std::vector<double> AntColony::GetAllGridLength()
{
    return this->All_Best_length;
}

double AntColony::GetBestGridLength()
{
    return this->bestLength;
}
