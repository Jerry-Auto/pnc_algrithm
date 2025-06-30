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


    // 声明 plotWorldMap 函数
    void plotWorldMap() const;

    void plotpath(std::pair<std::vector<int>, std::vector<int>> path_data,std::string outPNGname="out.png");

    void plot_iterate_path(const std::vector<std::pair<std::vector<int>, std::vector<int>>>& path_data);

    void plotpoint(int x_index, int y_index,std::string point_type="xb");

    void setgoal(double x_goal,double y_goal);

    void setstr(double x_str,double y_str);

    std::pair<int ,int> getgoal();

    std::pair<int ,int> getstr();

    void set_robot_radius(double radius);

    std::pair<std::pair<int,int>,float> index_to_grid(int index);

    int coordToIndex(int gx, int gy) const;

    void save_to_PNG(std::string filename);

private:
    double world_width_;     // 世界宽度（米）
    double world_height_;    // 世界高度（米）
    double resolution_;      // 分辨率（米/栅格）
    int grid_width_;         // 栅格地图宽度（栅格数）
    int grid_height_;        // 栅格地图高度（栅格数）
    std::vector<float> grid_;// 栅格数据 (行优先存储)
    double robot_radius=1.5;//移动机器人半径
    std::pair<std::vector<int> ,std::vector<int> > obstacle_grid;


    //起点和终点信息,栅格坐标
    std::pair<int ,int> strpoint;
    std::pair<int ,int> goalpoint;
    double point_size=resolution_*5;//栅格点尺寸
    double plot_pause_time=0.001;


    void initializeGrid();

    bool show_plot_=true; // 标志变量，控制是否在 plotWorldMap() 中调用 plt::show()
    bool point_plot=false;// 标志变量，控制是否在 plotWorldMap() 中新建图纸
    // 坐标转换辅助函数
    void convertCoords(CoordType coord_type, double& x, double& y) const;

    void update_grid_();

};

#endif // OBSTACLE_GRID_MAP_H