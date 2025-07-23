#ifndef H_A_STAR_H
#define H_A_STAR_H
#include"ObstacleGridMap.h"
#include"Dijkstra.h"
#include "ReedsShepp.h"
#include "node3d.h"

class H_A_Star
{
    public:
    struct Cost_weight
    {
            double traj_forward_penalty= 0.0;
            double traj_back_penalty= 0.0;
            double traj_gear_switch_penalty= 0.0;
            double traj_steer_penalty= 0.0;
            double traj_steer_change_penalty= 0.0;

            double heu_rs_forward_penalty= 0.0;
            double heu_rs_back_penalty= 0.0;
            double heu_rs_gear_switch_penalty= 0.0;
            double heu_rs_steer_penalty= 0.0;
            double heu_rs_steer_change_penalty = 0.0;
    };
    struct Params
    {
            double step_size= 0.0;
            double grid_resolution= 0.0;
            int next_node_num= 0;
            double max_kappa = 0.0;
            double min_radius= 0.0;
            double robot_radius;
    };
    
    
private:
    std::shared_ptr<Dijkstra> Dijk_solver_=nullptr;
    std::shared_ptr<ReedsShepp> reedsShepp_=nullptr;
    std::shared_ptr<ElectricVehicleDynamicsModel::VehicleParams> vehicle_params=nullptr;
    std::shared_ptr<Cost_weight> params_=nullptr;
    std::shared_ptr<Params> cost_weights_=nullptr;

public:
    H_A_Star(WorldMap* map,ElectricVehicleDynamicsModel::VehicleParams* vehicle_param,Params* solver_param,Cost_weight* cost_param);



private:
    void init
};




#endif