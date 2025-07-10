
#include "B_spline.h"
#include"World_map.h"

int main()
{
    std::vector<Eigen::Vector2d> control_point{
        Eigen::Vector2d(0.0,0.0),
        Eigen::Vector2d(5.0,10.0),
        Eigen::Vector2d(10.0,-5.0),
        Eigen::Vector2d(15.0,5.0),
        Eigen::Vector2d(20.0,-5.0),
        Eigen::Vector2d(25.0,10.0),
        Eigen::Vector2d(30.0,0.0)
    };
    // 提取x和y坐标用于绘制控制多边形
    std::vector<double> x, y;
    for (const auto& p : control_point) {
        x.push_back(p.x());
        y.push_back(p.y());
    }
    WorldMap world(-50, 50, -50, 50);
    world.add_control_point({x,y});
    // 创建 B 样条对象（显式提供所有参数）
    B_spline B_spline_solver(
        std::make_optional(control_point), // control_point
        std::make_optional(3),             // k (曲线阶数)
        std::nullopt,                      // node_vector (不提供，自动计算)
        0                                  // type (默认值)
    );

    std::vector<std::vector<double>> planning_data=B_spline_solver.planning_series(1000);

        //画图
    ElectricVehicleDynamicsModel vehicle;
    //vehicle.plot_planning(planning_data);
    world.addVehicle(&vehicle, "orange");
    world.plot_planning(&vehicle,planning_data,"B_spline.png");

    return 0;
}
