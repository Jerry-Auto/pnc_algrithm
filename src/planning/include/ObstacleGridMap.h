#ifndef OBSTACLE_GRID_MAP_H
#define OBSTACLE_GRID_MAP_H

#include <vector>
#include <string>
#include <stdexcept>
#include <cmath>
#include <memory>
#include <functional>
#include <map>

// 前置声明绘图函数所需的类
namespace matplotlibcpp {
    class Plot;
}

class ObstacleGridMap {
public:
    // 坐标类型枚举
    enum class CoordType {
        WORLD_COORDS,   // 世界坐标 (米)
        GRID_COORDS     // 栅格坐标
    };

    // 绘图颜色配置结构体
    struct PlotConfig {
        float occupied_color[3];       // 占据区域颜色 (RGB)
        float free_color[3];           // 空闲区域颜色 (RGB)
        float unknown_color[3];        // 未知区域颜色 (RGB)
        float grid_color[3];           // 网格线颜色 (RGB)
        float path_color[3];           // 路径颜色 (RGB)
        float start_color[3];          // 起点颜色 (RGB)
        float goal_color[3];           // 终点颜色 (RGB)
        float point_color[3];          // 普通点颜色 (RGB)
        bool show_grid;                // 是否显示网格
        bool show_values;              // 是否显示概率值
        float grid_line_width;         // 网格线宽
        float cell_margin;             // 单元格边距
        CoordType coord_type;          // 坐标类型

        PlotConfig() :
            occupied_color{0, 0, 0},
            free_color{1, 1, 1},
            unknown_color{0.7f, 0.7f, 0.7f},
            grid_color{0.8f, 0.8f, 0.8f},
            path_color{0, 0.5f, 1},
            start_color{0, 1, 0},
            goal_color{1, 0, 0},
            point_color{1, 0, 1},
            show_grid(true),
            show_values(false),
            grid_line_width(0.5f),
            cell_margin(0.05f),
            coord_type(CoordType::WORLD_COORDS) {}
    };

    /**
     * 创建障碍物地图
     * @param world_width 实际世界宽度（米）
     * @param world_height 实际世界高度（米）
     * @param resolution 栅格分辨率（米/栅格）
     */
    ObstacleGridMap(double world_width, double world_height, double resolution);
    
    // 从文件加载地图的构造函数
    ObstacleGridMap(const std::string& filename);
    
    // 获取世界宽度（米）
    double getWorldWidth() const;
    
    // 获取世界高度（米）
    double getWorldHeight() const;
    
    // 获取栅格分辨率（米/栅格）
    double getResolution() const;
    
    // 获取栅格地图宽度（栅格数）
    int getGridWidth() const;
    
    // 获取栅格地图高度（栅格数）
    int getGridHeight() const;
    
    // 世界坐标转栅格坐标
    void worldToGrid(double wx, double wy, int& gx, int& gy) const;
    
    // 栅格坐标转世界坐标（返回栅格中心点坐标）
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;
    
    // 检查世界坐标是否在地图范围内
    bool isInWorldBounds(double wx, double wy) const;
    
    // 获取栅格占据概率 [0.0, 1.0]
    float getCellProbability(int gx, int gy) const;
    
    // 设置栅格占据概率
    void setCellProbability(int gx, int gy, float probability);
    
    // 检查栅格是否被占据 (概率 > 阈值)
    bool isOccupied(int gx, int gy, float threshold = 0.65f) const;
    
    // 检查栅格是否是空闲的 (概率 < 阈值)
    bool isFree(int gx, int gy, float threshold = 0.35f) const;
    
    // 检查栅格是否未知
    bool isUnknown(int gx, int gy) const;
    
    // 在世界坐标设置障碍物
    void setObstacle(double wx, double wy, float probability = 0.9f);
    
    // 在世界坐标设置矩形障碍物
    void setRectangleObstacle(double world_x, double world_y, 
                             double width, double height, 
                             float probability = 0.9f);
    
    // 在世界坐标设置圆形障碍物
    void setCircleObstacle(double center_x, double center_y, 
                          double radius, 
                          float probability = 0.9f);
    
    // 在世界坐标设置线形障碍物
    void setLineObstacle(double start_x, double start_y, 
                        double end_x, double end_y, 
                        float probability = 0.9f);
    
    // 重置地图 (所有栅格设为未知)
    void reset();
    
    // 保存地图到文件
    void saveToFile(const std::string& filename) const;
    
    // 从文件加载地图
    void loadFromFile(const std::string& filename);

    // 直接访问栅格数据 (用于路径规划算法)
    const std::vector<float>& getGridData() const { return grid_; }

    // 绘制地图
    void plotMap(const std::string& title = "Obstacle Grid Map") const;
    
    // 绘制地图并添加路径 (世界坐标)
    void plotMapWithPath(const std::vector<std::pair<double, double>>& path,
                        double start_x, double start_y,
                        double goal_x, double goal_y,
                        const std::string& title = "Path Planning") const;
    
    // 高级绘图方法（使用自定义配置）
    void plotMapAdvanced(
        const PlotConfig& config = PlotConfig(),
        const std::vector<std::pair<double, double>>* path = nullptr,
        double start_x = -1, double start_y = -1,
        double goal_x = -1, double goal_y = -1,
        const std::vector<std::pair<double, double>>* points = nullptr,
        const std::string& title = "Obstacle Grid Map"
    ) const;

private:
    double world_width_;     // 世界宽度（米）
    double world_height_;    // 世界高度（米）
    double resolution_;      // 分辨率（米/栅格）
    int grid_width_;         // 栅格地图宽度（栅格数）
    int grid_height_;        // 栅格地图高度（栅格数）
    std::vector<float> grid_;// 栅格数据 (行优先存储)

    // 内部辅助函数
    int coordToIndex(int gx, int gy) const;
    void initializeGrid();
    
    // 绘图辅助函数
    void drawCell(int gx, int gy, const PlotConfig& config) const;
    
    // 坐标转换辅助函数
    void convertCoords(CoordType coord_type, 
                      double& x, double& y) const;
    void convertPath(CoordType coord_type,
                    std::vector<std::pair<double, double>>& path) const;
};

#endif // OBSTACLE_GRID_MAP_H