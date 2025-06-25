#include "ObstacleGridMap.h"

int main() {
    // 创建地图 (10米×10米，分辨率0.1米/格)
    ObstacleGridMap map(10.0, 10.0, 0.1);

    // 添加障碍物
    map.setRectangleObstacle(2.0, 2.0, 3.0, 2.0);  // 矩形障碍
    map.setCircleObstacle(7.0, 7.0, 1.5);          // 圆形障碍

    // 模拟路径 (栅格坐标)
    std::vector<std::pair<double, double>> path = {
        {1,1}, {2,2}, {3,3}, {4,4}, {5,5}, {6,6}, {7,7}, {8,8}
    };

    // 绘制地图
    map.plotMapAdvanced(
        ObstacleGridMap::PlotConfig(),  // 使用默认配置
        &path,                          // 路径
        1, 1,                           // 起点(栅格坐标)
        8, 8,                           // 终点(栅格坐标)
        nullptr,                        // 无额外点
        "Path Planning Result"          // 标题
    );
    
    return 0;
}
