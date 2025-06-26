#include "ObstacleGridMap.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

ObstacleGridMap::ObstacleGridMap(double world_width, double world_height, double resolution)
    : world_width_(world_width), world_height_(world_height), resolution_(resolution) {
    if (world_width <= 0 || world_height <= 0 || resolution <= 0) {
        throw std::invalid_argument("World dimensions and resolution must be positive");
    }
    
    grid_width_ = static_cast<int>(std::ceil(world_width / resolution));
    grid_height_ = static_cast<int>(std::ceil(world_height / resolution));
    
    initializeGrid();
}

ObstacleGridMap::ObstacleGridMap(const std::string& filename) {
    loadFromFile(filename);
}

double ObstacleGridMap::getWorldWidth() const {
    return world_width_;
}

double ObstacleGridMap::getWorldHeight() const {
    return world_height_;
}

double ObstacleGridMap::getResolution() const {
    return resolution_;
}

int ObstacleGridMap::getGridWidth() const {
    return grid_width_;
}

int ObstacleGridMap::getGridHeight() const {
    return grid_height_;
}

void ObstacleGridMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(wx / resolution_);
    gy = static_cast<int>(wy / resolution_);
    
    // Clamp to grid bounds
    gx = std::max(0, std::min(gx, grid_width_ - 1));
    gy = std::max(0, std::min(gy, grid_height_ - 1));
}

void ObstacleGridMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
    wx = (gx + 0.5) * resolution_;  // Return center of cell
    wy = (gy + 0.5) * resolution_;
}

bool ObstacleGridMap::isInWorldBounds(double wx, double wy) const {
    return wx >= 0 && wx < world_width_ && wy >= 0 && wy < world_height_;
}

float ObstacleGridMap::getCellProbability(int gx, int gy) const {
    if (gx < 0 || gx >= grid_width_ || gy < 0 || gy >= grid_height_) {
        throw std::out_of_range("Grid coordinates out of bounds");
    }
    return grid_[coordToIndex(gx, gy)];
}

void ObstacleGridMap::setCellProbability(int gx, int gy, float probability) {
    if (gx < 0 || gx >= grid_width_ || gy < 0 || gy >= grid_height_) {
        throw std::out_of_range("Grid coordinates out of bounds");
    }
    if (probability < 0.0f || probability > 1.0f) {
        throw std::invalid_argument("Probability must be in [0.0, 1.0]");
    }
    grid_[coordToIndex(gx, gy)] = probability;
}

bool ObstacleGridMap::isOccupied(int gx, int gy, float threshold) const {
    return getCellProbability(gx, gy) > threshold;
}

bool ObstacleGridMap::isFree(int gx, int gy, float threshold) const {
    return getCellProbability(gx, gy) < threshold;
}

bool ObstacleGridMap::isUnknown(int gx, int gy) const {
    float prob = getCellProbability(gx, gy);
    return prob >= 0.35f && prob <= 0.65f;
}

void ObstacleGridMap::setObstacle(double wx, double wy, float probability) {
    if (!isInWorldBounds(wx, wy)) return;
    
    int gx, gy;
    worldToGrid(wx, wy, gx, gy);
    setCellProbability(gx, gy, probability);
}

void ObstacleGridMap::setRectangleObstacle(double world_x, double world_y, 
                                         double width, double height, 
                                         float probability) {
    int start_gx, start_gy, end_gx, end_gy;
    worldToGrid(world_x, world_y, start_gx, start_gy);
    worldToGrid(world_x + width, world_y + height, end_gx, end_gy);
    
    for (int gy = start_gy; gy <= end_gy; ++gy) {
        for (int gx = start_gx; gx <= end_gx; ++gx) {
            if (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_) {
                setCellProbability(gx, gy, probability);
            }
        }
    }
}

void ObstacleGridMap::setCircleObstacle(double center_x, double center_y, 
                                      double radius, 
                                      float probability) {
    int center_gx, center_gy;
    worldToGrid(center_x, center_y, center_gx, center_gy);
    
    int radius_cells = static_cast<int>(std::ceil(radius / resolution_));
    int min_gx = std::max(0, center_gx - radius_cells);
    int max_gx = std::min(grid_width_ - 1, center_gx + radius_cells);
    int min_gy = std::max(0, center_gy - radius_cells);
    int max_gy = std::min(grid_height_ - 1, center_gy + radius_cells);
    
    double radius_sq = radius * radius;
    
    for (int gy = min_gy; gy <= max_gy; ++gy) {
        for (int gx = min_gx; gx <= max_gx; ++gx) {
            double wx, wy;
            gridToWorld(gx, gy, wx, wy);
            double dx = wx - center_x;
            double dy = wy - center_y;
            if (dx*dx + dy*dy <= radius_sq) {
                setCellProbability(gx, gy, probability);
            }
        }
    }
}

void ObstacleGridMap::setLineObstacle(double start_x, double start_y, 
                                    double end_x, double end_y, 
                                    float probability) {
    if (!isInWorldBounds(start_x, start_y) || !isInWorldBounds(end_x, end_y)) {
        return;
    }
    
    int start_gx, start_gy, end_gx, end_gy;
    worldToGrid(start_x, start_y, start_gx, start_gy);
    worldToGrid(end_x, end_y, end_gx, end_gy);
    
    // Bresenham's line algorithm
    int dx = std::abs(end_gx - start_gx);
    int dy = std::abs(end_gy - start_gy);
    int sx = (start_gx < end_gx) ? 1 : -1;
    int sy = (start_gy < end_gy) ? 1 : -1;
    int err = dx - dy;
    
    int x = start_gx;
    int y = start_gy;
    
    while (true) {
        if (x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_) {
            setCellProbability(x, y, probability);
        }
        
        if (x == end_gx && y == end_gy) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

void ObstacleGridMap::reset() {
    std::fill(grid_.begin(), grid_.end(), 0.5f);  // Set to unknown
}

void ObstacleGridMap::saveToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filename);
    }
    
    file << world_width_ << " " << world_height_ << " " << resolution_ << "\n";
    for (int gy = 0; gy < grid_height_; ++gy) {
        for (int gx = 0; gx < grid_width_; ++gx) {
            file << grid_[coordToIndex(gx, gy)] << " ";
        }
        file << "\n";
    }
}

void ObstacleGridMap::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for reading: " + filename);
    }
    
    file >> world_width_ >> world_height_ >> resolution_;
    
    grid_width_ = static_cast<int>(std::ceil(world_width_ / resolution_));
    grid_height_ = static_cast<int>(std::ceil(world_height_ / resolution_));
    
    initializeGrid();
    
    for (int gy = 0; gy < grid_height_; ++gy) {
        for (int gx = 0; gx < grid_width_; ++gx) {
            file >> grid_[coordToIndex(gx, gy)];
        }
    }
}


int ObstacleGridMap::coordToIndex(int gx, int gy) const {
    return gy * grid_width_ + gx;
}

void ObstacleGridMap::initializeGrid() {
    grid_.resize(grid_width_ * grid_height_, 0.5f);  // Initialize to unknown
}

void ObstacleGridMap::convertCoords(CoordType coord_type, double& x, double& y) const {
    if (coord_type == CoordType::GRID_COORDS) {
        double wx, wy;
        gridToWorld(static_cast<int>(x), static_cast<int>(y), wx, wy);
        x = wx;
        y = wy;
    }
    // For world coordinates, no conversion needed
}

void ObstacleGridMap::convertPath(CoordType coord_type,
                                std::vector<std::pair<double, double>>& path) const {
    if (coord_type == CoordType::GRID_COORDS) {
        for (auto& p : path) {
            double wx, wy;
            gridToWorld(static_cast<int>(p.first), static_cast<int>(p.second), wx, wy);
            p.first = wx;
            p.second = wy;
        }
    }
    // For world coordinates, no conversion needed
}

 void ObstacleGridMap::plotWorldMap() const {
    // 创建图形
    plt::figure_size(1000, 800);
    plt::title("Obstacle Grid Map (World Coordinates)");
    
    // 设置坐标轴范围
    plt::xlim(-3.0, world_width_+5);
    plt::ylim(-3.0, world_height_+5);
    plt::xlabel("World X (m)");
    plt::ylabel("World Y (m)");
    plt::grid(true);
    
    // 绘制地图边界
    std::vector<double> boundary_x = {0.0, world_width_, world_width_, 0.0, 0.0};
    std::vector<double> boundary_y = {0.0, 0.0, world_height_, world_height_, 0.0};
    plt::plot(boundary_x, boundary_y, "k-"); // 使用黑色线条绘制边界
    
    // 准备数据
    std::vector<double> obstacle_x, obstacle_y;
    std::vector<double> free_space_x, free_space_y;
    std::vector<double> unknown_x, unknown_y;
    
    // 绘制内部点
    for (int gy = 0; gy < grid_height_; ++gy) {
        for (int gx = 0; gx < grid_width_; ++gx) {
            float prob = grid_[coordToIndex(gx, gy)];
            double wx, wy;
            gridToWorld(gx, gy, wx, wy);
            
            // 根据概率决定颜色
            if (prob > 0.65f) { // 障碍物
                obstacle_x.push_back(wx);
                obstacle_y.push_back(wy);
            } else if (prob < 0.35f) { // 自由空间
                free_space_x.push_back(wx);
                free_space_y.push_back(wy);
            } else { // 未知区域
                unknown_x.push_back(wx);
                unknown_y.push_back(wy);
            }
        }
    }
    
    // 绘制障碍物点
    if (!obstacle_x.empty() && !obstacle_y.empty()) {
        plt::scatter(obstacle_x, obstacle_y, 10, {{"color", "red"}});
    }
    
    // 绘制自由空间点
    if (!free_space_x.empty() && !free_space_y.empty()) {
        plt::scatter(free_space_x, free_space_y, 10, {{"color", "green"}});
    }
    
    // 绘制未知区域点
    if (!unknown_x.empty() && !unknown_y.empty()) {
        plt::scatter(unknown_x, unknown_y, 10, {{"color", "gray"}});
    }
    
    // 添加图例项
    std::vector<double> dummy_x = {0, 1};
    std::vector<double> dummy_y = {0, 0};
    
    std::map<std::string, std::string> obstacle_kwargs = {
        {"label", "Obstacle"},
        {"color", "red"}
    };
    plt::scatter(dummy_x, dummy_y, 10, obstacle_kwargs);
    
    std::map<std::string, std::string> free_space_kwargs = {
        {"label", "Free Space"},
        {"color", "green"}
    };
    plt::scatter(dummy_x, dummy_y, 10, free_space_kwargs);
    
    std::map<std::string, std::string> unknown_kwargs = {
        {"label", "Unknown"},
        {"color", "gray"}
    };
    plt::scatter(dummy_x, dummy_y, 10, unknown_kwargs);
    
    // 确保图例可见
    plt::legend({{"loc", "upper right"}});
    
    // 显示图形
    plt::show();
}

void ObstacleGridMap::plotGridMap() const {
    // 创建图形
    plt::figure_size(1000, 800);
    plt::title("Obstacle Grid Map (Grid Coordinates)");
    
    // 设置坐标轴范围
    plt::xlim(-3.0, static_cast<double>(grid_width_+5));
    plt::ylim(-3.0, static_cast<double>(grid_height_+5));
    plt::xlabel("Grid X");
    plt::ylabel("Grid Y");
    plt::grid(true);
    
    // 绘制地图边界
    std::vector<double> boundary_x = {0.0, static_cast<double>(grid_width_), static_cast<double>(grid_width_), 0.0, 0.0};
    std::vector<double> boundary_y = {0.0, 0.0, static_cast<double>(grid_height_), static_cast<double>(grid_height_), 0.0};
    plt::plot(boundary_x, boundary_y, "k-"); // 使用黑色线条绘制边界
    
    // 准备数据
    std::vector<double> obstacle_x, obstacle_y;
    std::vector<double> free_space_x, free_space_y;
    std::vector<double> unknown_x, unknown_y;
    
    // 绘制内部点
    for (int gy = 0; gy < grid_height_; ++gy) {
        for (int gx = 0; gx < grid_width_; ++gx) {
            float prob = grid_[coordToIndex(gx, gy)];
            
            // 根据概率决定颜色
            if (prob > 0.65f) { // 障碍物
                obstacle_x.push_back(gx);
                obstacle_y.push_back(gy);
            } else if (prob < 0.35f) { // 自由空间
                free_space_x.push_back(gx);
                free_space_y.push_back(gy);
            } else { // 未知区域
                unknown_x.push_back(gx);
                unknown_y.push_back(gy);
            }
        }
    }
    
    // 绘制障碍物点
    if (!obstacle_x.empty() && !obstacle_y.empty()) {
        plt::scatter(obstacle_x, obstacle_y, 10, {{"color", "red"}});
    }
    
    // 绘制自由空间点
    if (!free_space_x.empty() && !free_space_y.empty()) {
        plt::scatter(free_space_x, free_space_y, 10, {{"color", "green"}});
    }
    
    // 绘制未知区域点
    if (!unknown_x.empty() && !unknown_y.empty()) {
        plt::scatter(unknown_x, unknown_y, 10, {{"color", "gray"}});
    }
    
    // 添加图例项
    std::vector<double> dummy_x = {0, 1};
    std::vector<double> dummy_y = {0, 0};
    
    std::map<std::string, std::string> obstacle_kwargs = {
        {"label", "Obstacle"},
        {"color", "red"}
    };
    plt::scatter(dummy_x, dummy_y, 10, obstacle_kwargs);
    
    std::map<std::string, std::string> free_space_kwargs = {
        {"label", "Free Space"},
        {"color", "green"}
    };
    plt::scatter(dummy_x, dummy_y, 10, free_space_kwargs);
    
    std::map<std::string, std::string> unknown_kwargs = {
        {"label", "Unknown"},
        {"color", "gray"}
    };
    plt::scatter(dummy_x, dummy_y, 10, unknown_kwargs);
    
    // 确保图例可见
    plt::legend({{"loc", "upper right"}});
    
    // 显示图形
    plt::show();
}