#ifndef H_A_STAR_H
#define H_A_STAR_H
#include <unordered_map>
#include <algorithm>
#include <memory>
#include <queue>
#include <utility>
#include <vector>
#include <string>
#include"ObstacleGridMap.h"
#include"Dijkstra.h"
#include "ReedsShepp.h"
#include "node3d.h"
#include "math_utils.h"

using namespace math;

class H_A_Star
{
    public:
struct Cost_weight {
    double traj_forward_penalty;
    double traj_back_penalty;
    double traj_gear_switch_penalty;
    double traj_steer_penalty;
    double traj_steer_change_penalty;

    double heu_rs_forward_penalty;
    double heu_rs_back_penalty;
    double heu_rs_gear_switch_penalty;
    double heu_rs_steer_penalty;
    double heu_rs_steer_change_penalty;

    Cost_weight()
        : traj_forward_penalty(0.0),
          traj_back_penalty(10),
          traj_gear_switch_penalty(10),
          traj_steer_penalty(100),
          traj_steer_change_penalty(10),

          heu_rs_forward_penalty(0.0),
          heu_rs_back_penalty(0.5),
          heu_rs_gear_switch_penalty(10.0),
          heu_rs_steer_penalty(100.0),
          heu_rs_steer_change_penalty(0.0) {}
};


struct Params {
    double step_size;
    double grid_resolution;
    int next_node_num;
    double max_kappa;
    double min_radius;
    double robot_radius;
    double dijkstra_grid_resolution;
    double dt;

    Params()
        : step_size(0.3),
          grid_resolution(2),
          next_node_num(10),
          max_kappa(0.2),
          min_radius(5),
          robot_radius(0),
          dijkstra_grid_resolution(2),
          dt(0.1){}
};

    
//私有变量
private:
    std::shared_ptr<Dijkstra> Dijk_solver_=nullptr;
    std::shared_ptr<ReedsShepp> reedsShepp_=nullptr;
    
    std::shared_ptr<ElectricVehicleDynamicsModel::VehicleParams> vehicle_params_=nullptr;
    std::shared_ptr<Cost_weight> cost_weights_=nullptr;
    std::shared_ptr<Params> params_=nullptr;


        //存储多个障碍物，每个障碍物包含几条边界线段LineSegment2d，每个LineSegment2d里面包含两个端点坐标，长度，航向，方向向量
    std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec_;

    struct cmp {
        bool operator()(const std::pair<std::string, double>& left,
                        const std::pair<std::string, double>& right) const {
        return left.second >= right.second;
        }
    };
    //优先队列，存储从小到大排列的pair
    //记录的是节点的字符串索引和当前节点最小代价
    std::priority_queue<std::pair<std::string, double>,
                        std::vector<std::pair<std::string, double>>, cmp>
        open_pq_;
    
    //存放节点的共享指针的开集和闭集
    std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;
    std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;

    std::shared_ptr<Node3d> start_node_;
    std::shared_ptr<Node3d> end_node_;
    std::shared_ptr<Node3d> final_node_;
      // 地图边界 xmin, xmax, ymin, ymax
    std::vector<double> XYbounds_;
    //标志是否已进行初始化
    bool init_check_=true;
    

public:
    H_A_Star(std::shared_ptr<ElectricVehicleDynamicsModel::VehicleParams> vehicle_param = nullptr,
             std::shared_ptr<Params> solver_param = nullptr,
             std::shared_ptr<Cost_weight> cost_param = nullptr);
             
    std::vector<std::vector<double>> planning(WorldMap& map);


//私有函数
private:
    bool init_plan(WorldMap& map);

    void obstacle_import(WorldMap& map);

    bool ValidityCheck(std::shared_ptr<Node3d> node);

    std::shared_ptr<Node3d> Next_node_generator(std::shared_ptr<Node3d> current_node, size_t next_node_index);
    
    bool AnalyticExpansion(std::shared_ptr<Node3d> current_node);

    bool IsPathVaild(const std::vector<Pos3d>& curve_path);

    void CalculateNodeCost(std::shared_ptr<Node3d> current_node,std::shared_ptr<Node3d> next_node) ;

    double TrajCost(std::shared_ptr<Node3d> current_node,std::shared_ptr<Node3d> next_node) ;
                             
    double Heuristic_cost(std::shared_ptr<Node3d> next_node);

    std::shared_ptr<Node3d>  GenerateFinalNode( const std::vector<Pos3d>& curve_path,std::shared_ptr<Node3d> current_node);

    void plot_collision(Box2d bounding_box,LineSegment2d linesegment);

    std::vector<Pos3d> RS_generate_path(Pos3d current_pos);

    std::vector<std::vector<double>> Path_Backtracking();

};

#endif