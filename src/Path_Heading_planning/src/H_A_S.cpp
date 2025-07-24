#include "H_A_S.h"


H_A_Star::H_A_Star(std::shared_ptr<ElectricVehicleDynamicsModel::VehicleParams> vehicle_param,
                         std::shared_ptr<Params> solver_param,
                         std::shared_ptr<Cost_weight> cost_param)
    : vehicle_params_(vehicle_param ? vehicle_param : std::make_shared<ElectricVehicleDynamicsModel::VehicleParams>()),
      params_(solver_param ? solver_param : std::make_shared<Params>()),
      cost_weights_(cost_param ? cost_param : std::make_shared<Cost_weight>()) {

    init();  // 调用初始化方法

}

    std::vector<std::vector<double>> H_A_Star::planning(WorldMap& map);