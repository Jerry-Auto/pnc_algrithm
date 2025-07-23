#include "Probability_Road_map.h"
#include <iostream>
#include <queue>
#include <cmath>
#include <nanoflann.hpp>
#include <unordered_map>
#include <algorithm>
#include <random>
#include <Eigen/Dense> // 确保 Eigen 已正确安装


// 节点结构体（用于 Dijkstra）
struct Node {
    Eigen::Vector2d pos;
    double cost;
    int parent_index;

    Node(double x, double y, double c, int p) 
        : pos(x, y), cost(c), parent_index(p) {}
};

PRMPlanner::PRMPlanner(int n_sample, int n_knn, double max_edge_len,double robot_radius, bool show_animation)
    : N_SAMPLE(n_sample), N_KNN(n_knn), MAX_EDGE_LEN(max_edge_len),robot_radius_(robot_radius), show_animation(show_animation) {
    // 初始化随机数生成器
    std::random_device rd;
    gen = std::mt19937(rd());
}

// 采样点集
std::pair<std::vector<double>, std::vector<double>> PRMPlanner::samplePoints(
    double sx, double sy, double gx, double gy, double rr,
    const std::vector<double>& ox, const std::vector<double>& oy) {
    
    std::vector<double> sample_x, sample_y;
    
    // 计算边界（扩展 10% 以避免边界问题）
    double min_x = *std::min_element(ox.begin(), ox.end()) - 10.0;
    double max_x = *std::max_element(ox.begin(), ox.end()) + 10.0;
    double min_y = *std::min_element(oy.begin(), oy.end()) - 10.0;
    double max_y = *std::max_element(oy.begin(), oy.end()) + 10.0;

    std::uniform_real_distribution<double> dist_x(min_x, max_x);
    std::uniform_real_distribution<double> dist_y(min_y, max_y);

    // 构建障碍物的 KD-Tree
    Eigen::MatrixXd obstacles(ox.size(), 2);
    for (size_t i = 0; i < ox.size(); ++i) {
        obstacles(i, 0) = ox[i];
        obstacles(i, 1) = oy[i];
    }
    nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> kd_tree(2, obstacles, 10);

    while (static_cast<int>(sample_x.size()) < N_SAMPLE) {
        double tx = dist_x(gen);
        double ty = dist_y(gen);

        // 查询最近障碍物距离
        double query_pt[2] = {tx, ty};
        size_t ret_index;
        double out_dist_sqr;
        nanoflann::KNNResultSet<double> resultSet(1);
        resultSet.init(&ret_index, &out_dist_sqr);
        kd_tree.index->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

        if (std::sqrt(out_dist_sqr) >= rr) {
            sample_x.push_back(tx);
            sample_y.push_back(ty);
        }
    }

    // 添加起点和终点
    sample_x.push_back(sx); sample_y.push_back(sy);
    sample_x.push_back(gx); sample_y.push_back(gy);

    return {sample_x, sample_y};
}

// 碰撞检测（修复后的版本）
bool PRMPlanner::isCollision(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double rr,
                           const Eigen::MatrixXd& obstacles) {
    Eigen::Vector2d dir = p2 - p1;
    double length = dir.norm();
    if (length >= MAX_EDGE_LEN) return true;

    int n_step = static_cast<int>(length / rr) + 1; // 确保检查所有中间点
    dir.normalize();

    // 构建 KD-Tree 用于障碍物查询
    nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> kd_tree(2, obstacles, 10);

    for (int i = 0; i <= n_step; ++i) {
        Eigen::Vector2d p = p1 + dir * i * (length / n_step);
        double query_pt[2] = {p.x(), p.y()};
        size_t ret_index;
        double out_dist_sqr;
        nanoflann::KNNResultSet<double> resultSet(1);
        resultSet.init(&ret_index, &out_dist_sqr);
        kd_tree.index->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

        if (std::sqrt(out_dist_sqr) < rr) {
            return true; // 碰撞
        }
    }
    return false; // 无碰撞
}

// 生成路图
std::vector<std::vector<int>> PRMPlanner::generateRoadMap(
    const std::vector<double>& sample_x, const std::vector<double>& sample_y,
    double rr, const std::vector<double>& obstacle_x, const std::vector<double>& obstacle_y) {
    
    std::vector<std::vector<int>> road_map(sample_x.size());
    Eigen::MatrixXd samples(sample_x.size(), 2);
    for (size_t i = 0; i < sample_x.size(); ++i) {
        samples(i, 0) = sample_x[i];
        samples(i, 1) = sample_y[i];
    }

    // 构建障碍物矩阵（用于碰撞检测）
    Eigen::MatrixXd obstacles(obstacle_x.size(), 2);
    for (size_t i = 0; i < obstacle_x.size(); ++i) {
        obstacles(i, 0) = obstacle_x[i];
        obstacles(i, 1) = obstacle_y[i];
    }

    nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> kd_tree(2, samples, 10);

    for (size_t i = 0; i < sample_x.size(); ++i) {
        Eigen::Vector2d p(sample_x[i], sample_y[i]);
        size_t ret_indexes[N_KNN + 1]; // +1 因为包含自身
        double out_dists_sqr[N_KNN + 1];
        nanoflann::KNNResultSet<double> resultSet(N_KNN + 1);
        resultSet.init(ret_indexes, out_dists_sqr);
        kd_tree.index->findNeighbors(resultSet, p.data(), nanoflann::SearchParams(10));

        for (int k = 1; k <= N_KNN; ++k) { // 从 1 开始跳过自身
            size_t j = ret_indexes[k];
            Eigen::Vector2d neighbor(sample_x[j], sample_y[j]);
            if (!isCollision(p, neighbor, rr, obstacles)) {
                road_map[i].push_back(static_cast<int>(j));
            }
        }
    }
    return road_map;
}

// Dijkstra 路径规划

std::pair<std::vector<double>, std::vector<double>> PRMPlanner::dijkstraPlanning(
    double sx, double sy, double gx, double gy,
    const std::vector<std::vector<int>>& road_map,
    const std::vector<double>& sample_x, const std::vector<double>& sample_y) {
    
    // 1. 检查 road_map 和 sample_x/y 的大小是否一致
    if (road_map.size() != sample_x.size() || road_map.size() != sample_y.size()) {
        return {{}, {}}; // 无效输入
    }

    // 2. 找到起点和目标点的索引（如果它们不在 sample_x/y 中，需要额外处理）
    // 这里假设 sx,sy 和 gx,gy 已经在 sample_x,sample_y 中
    int start_idx = -1, goal_idx = -1;
    for (size_t i = 0; i < sample_x.size(); ++i) {
        if (sample_x[i] == sx && sample_y[i] == sy) start_idx = i;
        if (sample_x[i] == gx && sample_y[i] == gy) goal_idx = i;
    }
    if (start_idx == -1 || goal_idx == -1) {
        return {{}, {}}; // 起点或目标点不存在
    }

    // 3. 初始化优先队列、代价和父节点
    using Node = std::pair<double, int>; // {cost, node_idx}
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::vector<double> costs(sample_x.size(), INFINITY);
    std::vector<int> parent(sample_x.size(), -1);

    costs[start_idx] = 0.0;
    open_set.push({0.0, start_idx});

    while (!open_set.empty()) {
        auto current = open_set.top();
        open_set.pop();
        int current_idx = current.second;

        // 如果到达目标点，回溯路径
        if (current_idx == goal_idx) {
            std::vector<double> rx, ry;
            int idx = goal_idx;
            while (idx != -1) {
                rx.push_back(sample_x[idx]);
                ry.push_back(sample_y[idx]);
                idx = parent[idx];
            }
            std::reverse(rx.begin(), rx.end());
            std::reverse(ry.begin(), ry.end());
            return {rx, ry};
        }

        // 如果当前代价大于已知最小代价，跳过
        if (current.first > costs[current_idx]) continue;

        // 遍历所有邻居
        for (int neighbor_idx : road_map[current_idx]) {
            if (neighbor_idx < 0 || neighbor_idx >= static_cast<int>(sample_x.size())) {
                continue; // 防止越界
            }

            double dx = sample_x[neighbor_idx] - sample_x[current_idx];
            double dy = sample_y[neighbor_idx] - sample_y[current_idx];
            double new_cost = costs[current_idx] + std::hypot(dx, dy);

            if (new_cost < costs[neighbor_idx]) {
                costs[neighbor_idx] = new_cost;
                parent[neighbor_idx] = current_idx;
                open_set.push({new_cost, neighbor_idx});
            }
        }
    }

    return {{}, {}}; // 未找到路径
}


// 主规划函数
std::pair<std::vector<double>, std::vector<double>> PRMPlanner::plan() {
    double robot_radius=robot_radius_;
    double start_x=start_point.first;
    double start_y=start_point.second; 
    double goal_x=goal_point.first; 
    double goal_y=goal_point.second;
    std::vector<double> obstacle_x,obstacle_y;
    for(size_t i=0;i<obstcles_.size();i++)
    {
        obstacle_x.push_back(obstcles_[i][0]);
        obstacle_y.push_back(obstcles_[i][1]);
    }

    auto [sample_x, sample_y] = samplePoints(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_x, obstacle_y);
    auto road_map = generateRoadMap(sample_x, sample_y, robot_radius, obstacle_x, obstacle_y);
    auto [path_x, path_y] = dijkstraPlanning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y);
      
    return {path_x, path_y};
}

void PRMPlanner::init_solver(WorldMap* map)
{
    std::vector<WorldMap::Obstacle> obs=map->get_Obstacle();
    this->goal_point=map->get_goal();
    this->start_point=map->get_start();
    obstcles_=obs_to_vector(obs);
}

std::vector<std::vector<double>> PRMPlanner::obs_to_vector(std::vector<WorldMap::Obstacle> obs)
{
    std::vector<std::vector<double>> obstacles;
    
    double resolution=0.5;
    for(size_t i=0;i<obs.size();i++)
    {
        std::vector<std::vector<double>> obs_resulotion=rasterizeRotatedRectEdges(obs[i],resolution);
        obstacles.insert(obstacles.end(), 
        obs_resulotion.begin(), obs_resulotion.end());
    }
    return obstacles;
}


// 计算旋转后的顶点
std::vector<std::vector<double>> PRMPlanner::getRotatedRectCorners(WorldMap::Obstacle& rect) {
    double half_width = rect.width / 2.0;
    double half_height = rect.height / 2.0;
    double cos_rot = cos(rect.rotation);
    double sin_rot = sin(rect.rotation);

    return {
        {rect.x + (-half_width * cos_rot - -half_height * sin_rot), 
         rect.y + (-half_width * sin_rot + -half_height * cos_rot)},
        {rect.x + (half_width * cos_rot - -half_height * sin_rot), 
         rect.y + (half_width * sin_rot + -half_height * cos_rot)},
        {rect.x + (half_width * cos_rot - half_height * sin_rot), 
         rect.y + (half_width * sin_rot + half_height * cos_rot)},
        {rect.x + (-half_width * cos_rot - half_height * sin_rot), 
         rect.y + (-half_width * sin_rot + half_height * cos_rot)}
    };
}

// 栅格化旋转矩形的边缘
std::vector<std::vector<double>> PRMPlanner::rasterizeRotatedRectEdges(WorldMap::Obstacle& rect, double point_spacing) {
    std::vector<std::vector<double>> points;
    auto corners = getRotatedRectCorners(rect);

    // 插值每条边
    for (int i = 0; i < 4; ++i) {
        const auto& p1 = corners[i];
        const auto& p2 = corners[(i + 1) % 4];
        double dx = p2[0] - p1[0];
        double dy = p2[1] - p1[1];
        double length = sqrt(dx * dx + dy * dy);
        int steps = static_cast<int>(length / point_spacing);

        for (int j = 0; j <= steps; ++j) {
            double t = static_cast<double>(j) / steps;
            points.push_back({p1[0] + t * dx, p1[1] + t * dy});
        }
    }

    return points;
}