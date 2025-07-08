#include "PolynomialAlgorithm.h"
#include "vehicle_model.h"


int main()
{
    std::pair<std::vector<double> ,std::vector<double>> str_state,end_state;
    double t0=0;
    double t1=30;
    std::vector<double> x_str={0,0,0};
    std::vector<double> y_str={0,0,0};
    std::vector<double> x_end={100,0,0};
    std::vector<double> y_end={20,0,0};
    str_state.first=x_str;
    str_state.second=y_str;
    end_state.first=x_end;
    end_state.second=y_end;
    PolynomialAlgorithm poly_solver(str_state,end_state,t0,t1);

    std::vector<std::vector<double>> planning_data=poly_solver.planning_series(0.05);
    //画图
    ElectricVehicleDynamicsModel vehicle;

    vehicle.plot_planning(planning_data);
    
    //poly_solver.plotallcurve(planning_data);
    //poly_solver.plotpositioncurve(planning_data);
    //poly_solver.plotthetacurve(planning_data);
    //poly_solver.plotvpcurve(planning_data);
    return 0;
}
