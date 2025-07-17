#ifndef PROBABILITY_ROAD_MAP_H
#define PROBABILITY_ROAD_MAP_H

#include <vector>
#include <random>
#include <Eigen/Dense>
#include <nanoflann.hpp>
#include <iostream>
#include"World_map.h"

class PRMPlanner {
public:
    /**
     * @brief 构造函数
     * @param n_sample 采样点数
     * @param n_knn 最近邻搜索数量
     * @param max_edge_len 最大边长度（超过则认为无效）
     * @param show_animation 是否显示可视化动画
     */
    PRMPlanner(int n_sample = 100, int n_knn = 10, double max_edge_len = 30.0,double robot_radius=1.6, bool show_animation = false);

    /**
     * @brief 主规划函数
     * @param start_x 起点x坐标
     * @param start_y 起点y坐标
     * @param goal_x 终点x坐标
     * @param goal_y 终点y坐标
     * @param obstacle_x 障碍物x坐标列表
     * @param obstacle_y 障碍物y坐标列表
     * @param robot_radius 机器人半径（用于碰撞检测）
     * @return 路径的x和y坐标列表
     */
    void init_solver(WorldMap* map);
    std::pair<std::vector<double>, std::vector<double>> plan();

private:
    // 参数
    const int N_SAMPLE;      // 采样点数
    const int N_KNN;         // 最近邻数量
    const double MAX_EDGE_LEN; // 最大边长度
    const bool show_animation; // 是否可视化
    std::pair<double,double> start_point,goal_point;
    std::vector<std::vector<double>> obstcles_;
    std::vector<std::vector<double>> obs_to_vector(std::vector<WorldMap::Obstacle> obs);

    std::vector<std::vector<double>> getRotatedRectCorners(WorldMap::Obstacle& rect) ;
    std::vector<std::vector<double>> rasterizeRotatedRectEdges(WorldMap::Obstacle& rect, double point_spacing); 
    double robot_radius_;
    // 随机数生成器
    std::mt19937 gen;

    /**
     * @brief 采样点集（避开障碍物）
     * @param sx 起点x
     * @param sy 起点y
     * @param gx 终点x
     * @param gy 终点y
     * @param rr 机器人半径（安全距离）
     * @param ox 障碍物x列表
     * @param oy 障碍物y列表
     * @return 采样点的x和y坐标列表
     */
    std::pair<std::vector<double>, std::vector<double>> samplePoints(
        double sx, double sy, double gx, double gy, double rr,
        const std::vector<double>& ox, const std::vector<double>& oy);

    /**
     * @brief 生成路图（概率路线图）
     * @param sample_x 采样点x列表
     * @param sample_y 采样点y列表
     * @param rr 机器人半径（安全距离）
     * @param obstacle_x 障碍物x列表
     * @param obstacle_y 障碍物y列表
     * @return 邻接表表示的路图
     */
    std::vector<std::vector<int>> generateRoadMap(
        const std::vector<double>& sample_x, const std::vector<double>& sample_y,
        double rr, const std::vector<double>& obstacle_x, const std::vector<double>& obstacle_y);

    /**
     * @brief Dijkstra路径规划
     * @param sx 起点x
     * @param sy 起点y
     * @param gx 终点x
     * @param gy 终点y
     * @param road_map 邻接表路图
     * @param sample_x 采样点x列表
     * @param sample_y 采样点y列表
     * @return 路径的x和y坐标列表
     */
    std::pair<std::vector<double>, std::vector<double>> dijkstraPlanning(
        double sx, double sy, double gx, double gy,
        const std::vector<std::vector<int>>& road_map,
        const std::vector<double>& sample_x, const std::vector<double>& sample_y);

    /**
     * @brief 碰撞检测
     * @param p1 线段起点
     * @param p2 线段终点
     * @param rr 机器人半径（安全距离）
     * @param obstacles 障碍物矩阵 (Nx2)
     * @return 是否发生碰撞
     */
    bool isCollision(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double rr,
                    const Eigen::MatrixXd& obstacles);

};

#endif // PROBABILITY_ROAD_MAP_H