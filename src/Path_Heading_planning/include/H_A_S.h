#ifndef H_A_STAR_H
#define H_A_STAR_H
#include"ObstacleGridMap.h"
#include"Dijkstra.h"
#include "ReedsShepp.h"
#include "node3d.h"

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
          traj_back_penalty(0.0),
          traj_gear_switch_penalty(0.0),
          traj_steer_penalty(0.0),
          traj_steer_change_penalty(0.0),
          heu_rs_forward_penalty(0.0),
          heu_rs_back_penalty(0.0),
          heu_rs_gear_switch_penalty(0.0),
          heu_rs_steer_penalty(0.0),
          heu_rs_steer_change_penalty(0.0) {}
};


struct Params {
    double step_size;
    double grid_resolution;
    int next_node_num;
    double max_kappa;
    double min_radius;
    double robot_radius;

    Params()
        : step_size(0.0),
          grid_resolution(0.0),
          next_node_num(0),
          max_kappa(0.0),
          min_radius(0.0),
          robot_radius(0.0) {}
};

    
    
private:
    std::shared_ptr<Dijkstra> Dijk_solver_=nullptr;
    std::shared_ptr<ReedsShepp> reedsShepp_=nullptr;
    std::shared_ptr<ElectricVehicleDynamicsModel::VehicleParams> vehicle_params_=nullptr;
    std::shared_ptr<Cost_weight> params_=nullptr;
    std::shared_ptr<Params> cost_weights_=nullptr;

public:
    H_A_Star(std::shared_ptr<ElectricVehicleDynamicsModel::VehicleParams> vehicle_param = nullptr,
             std::shared_ptr<Params> solver_param = nullptr,
             std::shared_ptr<Cost_weight> cost_param = nullptr);
             
    std::vector<std::vector<double>> planning(WorldMap& map);


private:
    void init
};




#endif