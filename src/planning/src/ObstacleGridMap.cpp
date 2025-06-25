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

void ObstacleGridMap::plotMap(const std::string& title) const {
    PlotConfig config;
    plotMapAdvanced(config, nullptr, -1, -1, -1, -1, nullptr, title);
}

void ObstacleGridMap::plotMapWithPath(const std::vector<std::pair<double, double>>& path,
                                     double start_x, double start_y,
                                     double goal_x, double goal_y,
                                     const std::string& title) const {
    PlotConfig config;
    plotMapAdvanced(config, &path, start_x, start_y, goal_x, goal_y, nullptr, title);
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

void ObstacleGridMap::plotMapAdvanced(
    const PlotConfig& config,
    const std::vector<std::pair<double, double>>* path,
    double start_x, double start_y,
    double goal_x, double goal_y,
    const std::vector<std::pair<double, double>>* points,
    const std::string& title) const {
    
    // Create figure
    plt::figure_size(1000, static_cast<int>(1000 * (world_height_ / world_width_)));
    plt::title(title);
    plt::xlim(0.0, world_width_);
    plt::ylim(0.0, world_height_);
    
    // Draw all cells
    for (int gy = 0; gy < grid_height_; ++gy) {
        for (int gx = 0; gx < grid_width_; ++gx) {
            drawCell(gx, gy, config);
        }
    }
    
    // Draw grid lines
    if (config.show_grid) {
        // Vertical lines
        for (int x = 0; x <= grid_width_; x++) {
            double wx = x * resolution_;
            std::vector<double> xs = {wx, wx};
            std::vector<double> ys = {0, world_height_};
            plt::plot(xs, ys, {{"color", std::to_string(config.grid_color[0]) + "," + 
                               std::to_string(config.grid_color[1]) + "," + 
                               std::to_string(config.grid_color[2])},
                              {"linewidth", std::to_string(config.grid_line_width)}});
        }
        
        // Horizontal lines
        for (int y = 0; y <= grid_height_; y++) {
            double wy = y * resolution_;
            std::vector<double> xs = {0, world_width_};
            std::vector<double> ys = {wy, wy};
            plt::plot(xs, ys, {{"color", std::to_string(config.grid_color[0]) + "," + 
                               std::to_string(config.grid_color[1]) + "," + 
                               std::to_string(config.grid_color[2])},
                              {"linewidth", std::to_string(config.grid_line_width)}});
        }
    }
    
    // Convert path coordinates if needed
    std::vector<std::pair<double, double>> converted_path;
    if (path) {
        converted_path = *path;
        convertPath(config.coord_type, converted_path);
    }
    
    // Draw path
    if (!converted_path.empty()) {
        std::vector<double> path_x, path_y;
        for (const auto& point : converted_path) {
            path_x.push_back(point.first);
            path_y.push_back(point.second);
        }
        
        // Path line
        std::map<std::string, std::string> line_keywords;
        line_keywords["color"] = std::to_string(config.path_color[0]) + "," + 
                                std::to_string(config.path_color[1]) + "," + 
                                std::to_string(config.path_color[2]);
        line_keywords["linewidth"] = "2";
        line_keywords["linestyle"] = "-";
        plt::plot(path_x, path_y, line_keywords);
        
        // Path points
        std::map<std::string, std::string> point_keywords;
        point_keywords["color"] = std::to_string(config.path_color[0]) + "," + 
                                 std::to_string(config.path_color[1]) + "," + 
                                 std::to_string(config.path_color[2]);
        point_keywords["marker"] = "o";
        point_keywords["markersize"] = "4";
        point_keywords["linestyle"] = "none";
        plt::plot(path_x, path_y, point_keywords);
    }
    
    // Convert start and goal coordinates if needed
    double start_x_draw = start_x;
    double start_y_draw = start_y;
    double goal_x_draw = goal_x;
    double goal_y_draw = goal_y;
    
    if (start_x >= 0 && start_y >= 0) {
        convertCoords(config.coord_type, start_x_draw, start_y_draw);
        std::vector<double> start_x_vec = {start_x_draw};
        std::vector<double> start_y_vec = {start_y_draw};
        
        std::map<std::string, std::string> start_keywords;
        start_keywords["color"] = std::to_string(config.start_color[0]) + "," + 
                                 std::to_string(config.start_color[1]) + "," + 
                                 std::to_string(config.start_color[2]);
        start_keywords["marker"] = "o";
        start_keywords["markersize"] = "10";
        start_keywords["linestyle"] = "none";
        plt::plot(start_x_vec, start_y_vec, start_keywords);
        plt::text(start_x_draw, start_y_draw, "Start");
    }
    
    if (goal_x >= 0 && goal_y >= 0) {
        convertCoords(config.coord_type, goal_x_draw, goal_y_draw);
        std::vector<double> goal_x_vec = {goal_x_draw};
        std::vector<double> goal_y_vec = {goal_y_draw};
        
        std::map<std::string, std::string> goal_keywords;
        goal_keywords["color"] = std::to_string(config.goal_color[0]) + "," + 
                                std::to_string(config.goal_color[1]) + "," + 
                                std::to_string(config.goal_color[2]);
        goal_keywords["marker"] = "x";
        goal_keywords["markersize"] = "10";
        goal_keywords["linewidth"] = "3";
        goal_keywords["linestyle"] = "none";
        plt::plot(goal_x_vec, goal_y_vec, goal_keywords);
        plt::text(goal_x_draw, goal_y_draw, "Goal");
    }
    
    // Draw additional points
    if (points && !points->empty()) {
        std::vector<double> points_x, points_y;
        for (const auto& p : *points) {
            double x = p.first;
            double y = p.second;
            convertCoords(config.coord_type, x, y);
            points_x.push_back(x);
            points_y.push_back(y);
        }
        
        std::map<std::string, std::string> points_keywords;
        points_keywords["color"] = std::to_string(config.point_color[0]) + "," + 
                                  std::to_string(config.point_color[1]) + "," + 
                                  std::to_string(config.point_color[2]);
        points_keywords["marker"] = "o";
        points_keywords["markersize"] = "6";
        points_keywords["linestyle"] = "none";
        plt::plot(points_x, points_y, points_keywords);
    }
    
    // Add axis labels
    plt::xlabel("X (meters)");
    plt::ylabel("Y (meters)");
    
    // Show the plot
    plt::show();
}

void ObstacleGridMap::drawCell(int gx, int gy, const PlotConfig& config) const {
    // Get world coordinates of cell boundaries
    double left = gx * resolution_;
    double right = left + resolution_;
    double bottom = gy * resolution_;
    double top = bottom + resolution_;
    
    // Add margin
    double margin = config.cell_margin * resolution_;
    left += margin;
    right -= margin;
    bottom += margin;
    top -= margin;
    
    // Determine color
    float color[3];
    float prob = getCellProbability(gx, gy);
    
    if (isOccupied(gx, gy)) {
        color[0] = config.occupied_color[0];
        color[1] = config.occupied_color[1];
        color[2] = config.occupied_color[2];
    } else if (isFree(gx, gy)) {
        color[0] = config.free_color[0];
        color[1] = config.free_color[1];
        color[2] = config.free_color[2];
    } else {
        color[0] = config.unknown_color[0];
        color[1] = config.unknown_color[1];
        color[2] = config.unknown_color[2];
    }
    
    // Draw rectangle
    std::vector<double> x = {left, right, right, left, left};
    std::vector<double> y = {bottom, bottom, top, top, bottom};
    
    std::map<std::string, std::string> fill_keywords;
    fill_keywords["color"] = std::to_string(color[0]) + "," + 
                            std::to_string(color[1]) + "," + 
                            std::to_string(color[2]);
    plt::fill(x, y, fill_keywords);
    
    // Show probability value
    if (config.show_values) {
        double center_x = (left + right) / 2;
        double center_y = (bottom + top) / 2;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << prob;
        plt::text(center_x, center_y, ss.str());
    }
}    