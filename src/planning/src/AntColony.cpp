#include "AntColony.h"




#define err 1e-4

AntColony::Node::Node(double x, double y) : x(x), y(y) {
    this->index=AntColony::grid_map_->coordToIndex(x,y);
}
AntColony::AntColony(ObstacleGridMap* grid_map,int num_i,double alp,double bet,double rho,double q)
:grid_map_(grid_map),num_of_iterations(num_i),Alpha(alp),Beta(bet),Rho(rho),Q(q)
{
    this->num_of_ant=1.5*free_pnt_num(grid_map_);

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
    std::vector<int> Path = {start->index};
    std::vector<int> TABU(size,1);
    TABU[start->index]=0;
    Node* current=start;
    while(current->index!=goal->index)
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
            if(n_id=goal->index)
            {
                node_prob.second.push_back(pow(this->Pheromone[current->index][node->index], this->Alpha) * pow(100.0, this->Beta));               
            }
            else{
                node_prob.second.push_back(pow(this->Pheromone[current->index][node->index], this->Alpha) * pow(1/calcu_dis_to_end(node), this->Beta));  
            }
        }
        // 如果没有可选节点，退出
        if (node_prob.second.empty()) {break;}
        double sumP=accumulate(node_prob.second.begin(), node_prob.second.end(), 0.0);
        for(auto&p : node_prob.second)
        {
            p /= sumP;
        }
        Node* next=node_index.first[rotate_wheel_slt(ode_prob.second)];
        distance+=(std::abs(current->x-next->x)+std::abs(current->y-next->y)==1)?1.0:std::sqrt(2.0);
        current=next;
        Path.push_back(current->index);
        TABU[current->index]=0;
    }
    return {distance,path};
}

int AntColony::rotate_wheel_slt(const vector<double>& prob) {
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


void AntColony::planning()
{
    AntColony::Node* str_node=new AntColony::Node(this->grid_map_->getstr().first,this->grid_map_->getstr().second);
    AntColony::Node* goal_node=new AntColony::Node(this->grid_map_->getgoal().first,this->grid_map_->getgoal().second);
    int N=this->grid_map_->getGridHeigh()*this->grid_map_->getGridWidth();
    this->Pheromone = std::vector<std::vector<double>>(N, vector<double>(N, 8.0));
    for(int i=0;i<this->num_of_iterations;i++)
    {
        std::vector<std::vector<int>> ROUTES(this->num_of_ant);
        std::vector<double> PL(this->num_of_ant, 0.0);
        for(int j=0;j<this->num_of_ant;j++)
        {
            std::pair<double,std::vector<int>> dis_path=single_ant_go(str_node,goal_node,N);
            //记录历代路径
            this->All_Best_path.push_back(dis_path.second);
            this->All_Best_length.push_back(dis_path.first);
            //记录最小及路径
            ROUTES[j] = dis_path.second;
            if(dis_path.second.back()==goal_node->index)
            {
                PL[j] = dis_path.first;
                if (PL[j]< this->bestLength) {
                    this->bestLength = PL[j];
                    this->bestPath = ROUTES[j];
                }
            }
            else{
                PL[j] = numeric_limits<double>::max();
            }
            
        }
        //更新信息素

        //记录当前代最优路径及距离

    }


}
