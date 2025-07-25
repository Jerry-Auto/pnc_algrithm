#include "rs_curve.h"
#include <iostream>
#include <vector>
#include <cmath>

int main() {
    // 1. 定义起点和终点（格式：((x, y), theta)）
    point_type start_pos(std::make_pair(12, 15), 0.5235988); // 起点 (0,0)，朝向 0 弧度
    point_type end_pos(std::make_pair(95, 53), -M_PI_2); // 终点 (4,2)，朝向 π/2 弧度

    // 2. 创建 Reeds-Shepp 曲线对象（转弯半径设为 1.0）
    double turning_radius = 5;
    RSCurve rs_planner;

    // 3. 计算所有可能的 RS 路径
    auto all_curves = rs_planner.GetRSCurve(turning_radius, start_pos, end_pos);

    // 4. 输出路径信息并选择最短路径
    double min_length = INFINITY;
    curve_type best_curve;

    // std::cout << "Available Reeds-Shepp Paths:" << std::endl;
    // for (const auto& curve : all_curves) {
    //     if (curve.second.second < INFINITY) {
    //         std::cout << "Type: " << curve.first.second 
    //                   << ", Length: " << curve.second.second * turning_radius 
    //                   << std::endl;
    //         if (curve.second.second < min_length) {
    //             min_length = curve.second.second;
    //             best_curve = curve;
    //         }
    //     }
    // }

    best_curve =rs_planner.GetBestRSCurve(turning_radius, start_pos, end_pos);
    min_length=10;

    // 5. 插值生成路径点
    if (min_length != INFINITY) {
        auto path_points = rs_planner.GetRSPoint(best_curve,0.5);
        std::cout << "\nBest Path has " << path_points.size() << " points." << std::endl;

        std::cout << "Type: " << best_curve.first.second 
        << " , Length: " << best_curve.second.second
        << "\n ({t, u, v, w, x}: " ;
        for(int i=0;i<5;++i)
        {
           std::cout<< best_curve.second.first[i] << ",";
        }
        std::cout<<std::endl;
                  

        // 6. 输出前 5 个路径点（示例）
        std::cout << "Sample Path Points:" << std::endl;
        for (size_t i = 0; i < path_points.size(); ++i) {
            std::cout << "Point " << i << ": (" 
                      << path_points[i].first.first << ", " 
                      << path_points[i].first.second << "), theta: " 
                      << path_points[i].second << std::endl;
        }
    } else {
        std::cerr << "No valid Reeds-Shepp path found!" << std::endl;
    }

    return 0;
}

