#include "AntColony.h"
#include <iostream>
#define err 1e-4
using namespace std;
// 栅格节点
AntColony::Node::Node(int x, int y) : x(x), y(y) {}

// 构造函数：num_of_iterations 现在作为“模拟步数”使用
AntColony::AntColony(ObstacleGridMap* grid_map, int num_i, double alp, double bet, double rho, double q)
    : num_of_iterations(num_i),  // 先初始化声明顺序靠前的成员
      Alpha(alp),
      Beta(bet),
      Rho(rho),
      Q(q),
      grid_map_(grid_map) {     // 最后初始化 grid_map_
    this->num_of_ant = 1.5 * free_pnt_num(grid_map_);
}

int AntColony::free_pnt_num(ObstacleGridMap* map)
{
    // 统计可通行栅格数量，用于初始化蚂蚁数量
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
    // 线程局部随机数引擎，避免每次重复构造随机引擎
    static thread_local std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dist(a, b);
    return dist(gen);
}

double AntColony::calcu_dis_to_target(Node* node,int target_idx)
{
    // 计算当前节点到“动态目标点（起点或终点）”的欧式距离
    std::pair<std::pair<int,int>,float> target_grid = this->grid_map_->index_to_grid(target_idx);
    return std::sqrt(std::pow(node->x - target_grid.first.first, 2) + std::pow(node->y - target_grid.first.second, 2));
}

int AntColony::rotate_wheel_slt(const std::vector<double>& prob) {
    // 轮盘赌：根据概率分布选择下一个候选节点
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


void AntColony::planning(std::string PNGpath,bool plotfinalpath,bool plotgridpath)
{
    // 每次规划前清空历史记录
    this->bestPath.clear();
    this->bestLength = std::numeric_limits<double>::max();
    this->All_Best_path.clear();
    this->All_Best_length.clear();

    AntColony::Node str_node(this->grid_map_->getstr().first,this->grid_map_->getstr().second);
    AntColony::Node goal_node(this->grid_map_->getgoal().first,this->grid_map_->getgoal().second);
    int N=this->grid_map_->getGridHeight()*this->grid_map_->getGridWidth();
    int start_idx = this->nodetoindex(&str_node);
    int goal_idx = this->nodetoindex(&goal_node);

    // 全图共用信息素矩阵：边(i,j)与边(j,i)共享更新
    this->Pheromone = std::vector<std::vector<double>>(N, std::vector<double>(N, 8.0));

    // 初始化所有蚂蚁：起点与终点各一半，双向同时搜索
    std::vector<AntState> ants(this->num_of_ant);
    int split_idx = this->num_of_ant / 2;
    for(int i=0;i<this->num_of_ant;i++)
    {
        if(i < split_idx)
        {
            ants[i].current_idx = start_idx;
            ants[i].target_idx = goal_idx;
            ants[i].segment_path = {start_idx};
        }
        else
        {
            ants[i].current_idx = goal_idx;
            ants[i].target_idx = start_idx;
            ants[i].segment_path = {goal_idx};
        }
        ants[i].prev_idx = -1;
        ants[i].segment_length = 0.0;
    }

    // 时间步主循环：每一步都让“全部蚂蚁”同步决策、同步移动
    for(int step=0;step<this->num_of_iterations;step++)
    {
        // 暂存本时间步的决策结果（先决策，后统一执行）
        std::vector<int> next_idx(this->num_of_ant, -1);
        std::vector<double> move_len(this->num_of_ant, 0.0);
        std::vector<std::pair<int,int>> step_edges;
        std::vector<double> step_lengths;

        // 阶段1：所有蚂蚁并行“决策”（本实现按循环模拟并行语义）
        for(int ant_id=0;ant_id<this->num_of_ant;ant_id++)
        {
            AntState& ant = ants[ant_id];
            // 到达目标立即切换方向：终点->起点，起点->终点
            if(ant.current_idx==ant.target_idx)
            {
                ant.target_idx = (ant.target_idx==goal_idx)?start_idx:goal_idx;
                ant.segment_path = {ant.current_idx};
                ant.segment_length = 0.0;
            }

            std::pair<std::pair<int,int>,float> current_grid = this->grid_map_->index_to_grid(ant.current_idx);
            Node current_node(current_grid.first.first,current_grid.first.second);
            std::vector<int> candidates;
            std::vector<double> probs;
            std::vector<double> lengths;
            std::vector<int> fallback_candidates;
            std::vector<double> fallback_probs;
            std::vector<double> fallback_lengths;
            bool has_direct_target = false;
            int direct_target_idx = -1;
            double direct_target_len = 0.0;

            // 枚举8邻域候选，并计算转移概率
            for(const auto& move : this->motion)
            {
                Node node(current_node.x + static_cast<int>(move[0]), current_node.y + static_cast<int>(move[1]));
                if(Is_quit_map(&node))
                {
                    // cout<<"蚂蚁"<<ant_id<<"在("<<node.x<<","<<node.y<<")遇障碍或越界，跳过该邻居"<<endl;
                    continue;
                }
                int n_id = this->grid_map_->coordToIndex(node.x, node.y);
                double step_cost = (std::abs(current_node.x - node.x) + std::abs(current_node.y - node.y) == 1) ? 1.0 : std::sqrt(2.0);

                // 若目标格就在一步邻域，优先直接到达，避免“差一步反复游走”
                if(n_id == ant.target_idx)
                {
                    has_direct_target = true;
                    direct_target_idx = n_id;
                    direct_target_len = step_cost;
                    break;
                }

                // 启发项：靠近当前目标点概率更大；到目标附近时给予高启发值
                double dist_to_target = calcu_dis_to_target(&node, ant.target_idx);
                double eta = (dist_to_target < err) ? 100.0 : (1.0 / std::max(dist_to_target, 1e-6));
                double tau = std::max(this->Pheromone[ant.current_idx][n_id], 1e-9);
                // ACO转移权重：tau^Alpha * eta^Beta
                double p = std::pow(tau, this->Alpha) * std::pow(eta, this->Beta);
                if(n_id == ant.prev_idx)
                {
                    p *= 0.05;
                }
                if(!std::isfinite(p) || p<=0)
                {
                    continue;
                }

                bool visited_in_segment = std::find(ant.segment_path.begin(), ant.segment_path.end(), n_id) != ant.segment_path.end();
                if(visited_in_segment)
                {
                    fallback_candidates.push_back(n_id);
                    fallback_probs.push_back(p * 0.1);
                    fallback_lengths.push_back(step_cost);
                }
                else
                {
                    candidates.push_back(n_id);
                    probs.push_back(p);
                    lengths.push_back(step_cost);
                }
            }

            if(has_direct_target)
            {
                next_idx[ant_id] = direct_target_idx;
                move_len[ant_id] = direct_target_len;
                continue;
            }

            if(candidates.empty())
            {
                candidates.swap(fallback_candidates);
                probs.swap(fallback_probs);
                lengths.swap(fallback_lengths);
                if(candidates.empty())
                {
                    continue;
                }
            }

            double sumP = 0.0;
            for(double p:probs)
            {
                sumP += p;
            }
            for(double& p:probs)
            {
                p /= sumP;
            }

            // 轮盘赌确定该蚂蚁下一步位置
            int slt = rotate_wheel_slt(probs);
            next_idx[ant_id] = candidates[slt];
            move_len[ant_id] = lengths[slt];
        }

        // 阶段2：统一执行移动，并收集“本步经过的边”用于即时信息素更新
        for(int ant_id=0;ant_id<this->num_of_ant;ant_id++)
        {
            if(next_idx[ant_id] == -1)
            {
                continue;
            }

            AntState& ant = ants[ant_id];
            int prev_idx = ant.current_idx;
            ant.prev_idx = ant.current_idx;
            ant.current_idx = next_idx[ant_id];
            ant.segment_path.push_back(ant.current_idx);
            ant.segment_length += move_len[ant_id];

            step_edges.push_back({prev_idx, ant.current_idx});
            step_lengths.push_back(move_len[ant_id]);

            // 到达任一目标都可更新全局最优：
            // - start->goal 直接更新
            // - goal->start 反转后按 start->goal 语义更新
            if(ant.current_idx==ant.target_idx)
            {
                if(ant.segment_length < this->bestLength)
                {
                    this->bestLength = ant.segment_length;
                    if(ant.target_idx==goal_idx)
                    {
                        this->bestPath = ant.segment_path;
                    }
                    else
                    {
                        this->bestPath.assign(ant.segment_path.rbegin(), ant.segment_path.rend());
                    }
                }

                ant.target_idx = (ant.target_idx==goal_idx)?start_idx:goal_idx;
                ant.prev_idx = -1;
                ant.segment_path = {ant.current_idx};
                ant.segment_length = 0.0;
            }
        }

        // 每一步都做一次信息素更新（挥发 + 增强）
        update_pheromono(step_edges, step_lengths);

        // 记录每一步的全局最优轨迹，便于可视化演化过程
        this->All_Best_length.push_back(this->bestLength);
        this->All_Best_path.push_back(this->bestPath);
        std::cout <<"第"<<step+1<< "步当前全局最短距离: " << this->bestLength << std::endl;
    }

    std::cout << "全局最优距离: " << bestLength << std::endl;
    if(plotgridpath)
    {
        this->grid_map_->plot_iterate_path(this->GetAllGridPath());
    }
    if(plotfinalpath)
    {
        if(PNGpath==" ")
        {   
        this->grid_map_->plotpath(this->GetBestGridPath()); 
        }
        else
        {       
            this->grid_map_->plotpath(this->GetBestGridPath(),PNGpath);
        }         
    }
}

void AntColony::update_pheromono(const std::vector<std::pair<int,int>>& step_edges,const std::vector<double>& step_lengths)
{
    // 1) 全局挥发
    const double evap = 1.0 - this->Rho;
    for (size_t i = 0; i < this->Pheromone.size(); ++i)
    {
        for (size_t j = 0; j < this->Pheromone.size(); ++j)
        {
            this->Pheromone[i][j] *= evap;
        }
    }

    // 2) 仅对“本时间步经过的边”进行增量强化
    for(size_t k=0;k<step_edges.size();k++)
    {
        int x = step_edges[k].first;
        int y = step_edges[k].second;
        double len = std::max(step_lengths[k], 1e-6);
        // 路径越短，单位步长信息素增量越大
        double delta = this->Q / len;
        this->Pheromone[x][y] += delta;
        this->Pheromone[y][x] += delta;
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
