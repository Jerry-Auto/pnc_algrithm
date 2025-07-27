#include "ObstacleGridMap.h"


ObstacleGridMap::ObstacleGridMap(double world_width, double world_height, double resolution)
    :resolution_(resolution),strpoint(-1, -1), goalpoint(-1, -1){
    if (world_width <= 0 || world_height <= 0 || resolution <= 0) {
        throw std::invalid_argument("World dimensions and resolution must be positive");
    }
    this->grid_width_ = static_cast<int>(std::ceil(world_width / resolution));
    this->grid_height_ = static_cast<int>(std::ceil(world_height / resolution));
    this->world_width_=this->grid_width_*resolution;
    this->world_height_=this->grid_height_*resolution;
    initializeGrid();
}
/// @brief 
/// @param map 
/// @param resolution 
/// @param robot_radius 
ObstacleGridMap::ObstacleGridMap(WorldMap & map, double resolution,double robot_radius)
:resolution_(resolution),strpoint(-1, -1), goalpoint(-1, -1)
{
    std::vector<double> bounds=map.getBounds();
    this->world_height_=bounds[3]-bounds[2];
    this->world_width_=bounds[1]-bounds[0];
    this->grid_width_ = static_cast<int>(std::ceil(this->world_width_ / resolution));
    this->grid_height_ = static_cast<int>(std::ceil(this->world_height_ / resolution));
    this->world_width_=this->grid_width_*resolution;
    this->world_height_=this->grid_height_*resolution;
    initializeGrid();
    std::vector<WorldMap::Obstacle> obs=map.get_Obstacle();
    for(auto obstacle:obs)
    {
        setRotatedRectangleObstacle(obstacle.x,obstacle.y,obstacle.width,obstacle.height,obstacle.rotation);
    }

    this->setstr(map.get_start().first,map.get_start().second);
    this->setgoal(map.get_goal().first,map.get_goal().second);
    set_robot_radius(robot_radius);
    
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
    this->obstacle_grid.first.push_back(gx);  
    this->obstacle_grid.second.push_back(gy); 
    setCellProbability(gx, gy, probability);
    update_grid_();
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
                this->obstacle_grid.first.push_back(gx);  
                this->obstacle_grid.second.push_back(gy); 
                setCellProbability(gx, gy, probability);
            }
        }
    }
    update_grid_();
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
                this->obstacle_grid.first.push_back(gx);  
                this->obstacle_grid.second.push_back(gy); 
                setCellProbability(gx, gy, probability);
            }
        }
    }
    update_grid_();
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
            this->obstacle_grid.first.push_back(x);  
            this->obstacle_grid.second.push_back(y); 
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
    update_grid_();
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

/// @brief 栅格坐标变换成一维索引
/// @param gx 
/// @param gy 
/// @return 
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


 void ObstacleGridMap::plotWorldMap() const {

    if(!(this->point_plot))
    {
        // 创建图形
        plt::figure_size(1000, 800);
        plt::title("Obstacle Grid Map (World Coordinates)");
        // 设置坐标轴范围（添加一些边距）
        plt::xlim(-resolution_*2, world_width_ + 2*resolution_);
        plt::ylim(-resolution_*2, world_height_ + 2*resolution_);
        plt::xlabel("World X (m)");
        plt::ylabel("World Y (m)");
        plt::grid(true);
        // 绘制地图边界
        std::vector<double> boundary_x = {0.0, resolution_*grid_width_, resolution_*grid_width_, 0.0, 0.0};
        std::vector<double> boundary_y = {0.0, 0.0, resolution_*grid_height_, resolution_*grid_height_, 0.0};
        plt::plot(boundary_x, boundary_y, "k-"); // 使用黑色线条绘制边界
    
        //绘制起点和终点
        std::map<std::string, std::string> str_point_kwargs = {
            {"c", "green"},       
            {"marker", "*"},  
            {"label", "start"}
        };
        std::map<std::string, std::string> goal_point_kwargs = {
            {"c", "red"},       
            {"marker", "p"},  // 标记为五角星
            {"label", "goal"}
        };
        if(this->strpoint.first!=-1&&this->goalpoint.second!=-1)
        {      
            std::pair<std::vector<double> ,std::vector<double>> world_str({-1},{-1});
            gridToWorld(this->strpoint.first,this->strpoint.second, world_str.first[0], world_str.second[0]);
            plt::scatter(world_str.first, world_str.second,point_size*10,str_point_kwargs);
            gridToWorld(this->goalpoint.first,this->goalpoint.second, world_str.first[0], world_str.second[0]);
            plt::scatter(world_str.first, world_str.second,point_size*10,goal_point_kwargs);
        } 

    }
    // 准备数据 - 绘制栅格点
    std::vector<double> obstacle_x, obstacle_y;
    std::vector<double> free_space_x, free_space_y;
    std::vector<double> unknown_x, unknown_y;
    
    // 绘制栅格中心点
    for (int gy = 0; gy < grid_height_; ++gy) {
        for (int gx = 0; gx < grid_width_; ++gx) {
            float prob = grid_[coordToIndex(gx, gy)];
            double wx, wy;
            gridToWorld(gx, gy, wx, wy); // 这已经返回栅格中心点
            
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
    
    
    // 绘制障碍物点（使用较大的点显示）
        std::map<std::string, std::string> obstacle_kwargs = {
        {"c", "r"},       // 颜色为红色
        {"marker", "x"},  // 标记为圆点
        {"label", "obstacle"}
    };
            std::map<std::string, std::string> free_space_kwargs = {
        {"c", "g"},       // 颜色为绿色
        {"marker", "o"},  // 标记为圆点
        {"label", "free_space"}
    };
            std::map<std::string, std::string> unknown_kwargs = {
        {"c", "gray"},       // 颜色为蓝色
        {"marker", "o"},  // 标记为圆点
         {"label", "Unknown"}
    };
    if (!obstacle_x.empty() && !obstacle_y.empty()) {
        plt::scatter(obstacle_x, obstacle_y,point_size,obstacle_kwargs);
    }
    
    // 绘制自由空间点
    if (!free_space_x.empty() && !free_space_y.empty()) {
        plt::scatter(free_space_x, free_space_y,point_size/3, free_space_kwargs);
    }
    
    // 绘制未知区域点
    if (!unknown_x.empty() && !unknown_y.empty()) {
        plt::scatter(unknown_x, unknown_y,point_size/3,unknown_kwargs);
    }


    if(!(this->point_plot))
    {
        plt::legend({{"loc", "upper left"}});
        plt::pause(plot_pause_time);
    }

    // 显示图形
    if (show_plot_) {
        plt::show();
    }
}
/// @brief 使用栅格坐标下的路径数据进行路径绘制
/// @param path_data 
void ObstacleGridMap::plotpath(std::pair<std::vector<int>, std::vector<int>> path_data,std::string outPNGname) {
    show_plot_=false;
    plotWorldMap();
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    // 对 x 坐标进行缩放和偏移
    for (double x : path_data.first) {
        x_coords.push_back(x * resolution_ + 0.5 * resolution_);
    }
    // 对 y 坐标进行缩放和偏移
    for (double y : path_data.second) {
        y_coords.push_back(y * resolution_ + 0.5 * resolution_);
    }
    // 绘制路径
    plt::plot(x_coords, y_coords, "-r");
    plt::pause(plot_pause_time);
    save_to_PNG(outPNGname);
    plt::show();
    show_plot_=true;
    this->point_plot=false;
}

void ObstacleGridMap::plot_extra_curve(std::pair<std::vector<double>, std::vector<double>> path_data,std::string outPNGname) {
    plt::figure();
    plt::title("curve");
    plt::xlabel("iteration");
    plt::ylabel("length(m)");
    plt::grid(true);
    // 绘制路径
    plt::plot(path_data.first, path_data.second, "g-");
    plt::pause(plot_pause_time);
    save_to_PNG(outPNGname);
    plt::show();
}


void ObstacleGridMap::plotpoint(int x_index, int y_index,std::string point_type) {
    show_plot_ = false; // 防止 plotWorldMap() 内部调用 plt::show()   
    plotWorldMap();
    this->point_plot=true;
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    // 对 x 坐标进行缩放和偏移
    x_coords.push_back((x_index+0.5)* resolution_);
    // 对 y 坐标进行缩放和偏移
    y_coords.push_back((y_index+0.5)* resolution_);

    std::map<std::string, std::string> obstacle_kwargs = {
        {"c", point_type.substr(1, 1)},       // 颜色为蓝色
        {"marker", point_type.substr(0, 1)},     // 标记为 x
    };

    // 绘制点
    if (!x_coords.empty() && !y_coords.empty()) {
        plt::scatter(x_coords, y_coords, this->point_size*3, obstacle_kwargs);
    }
    plt::pause(plot_pause_time);
    show_plot_ = true;
}

    // 设置终点
void ObstacleGridMap::setgoal(double x_goal, double y_goal) {
    std::pair<int, int> grid_coord;
    worldToGrid(x_goal, y_goal, grid_coord.first, grid_coord.second);        
        
    this->goalpoint = grid_coord;
    // if (isInWorldBounds(x_goal, y_goal) && (getCellProbability(grid_coord.first, grid_coord.second) < 0.8)) {
    //     this->goalpoint = grid_coord;
    // } else {
    //     throw std::invalid_argument("不在地图内或该处有障碍物，无效");
    // }
}
 
// 设置起点
void ObstacleGridMap::setstr(double x_str, double y_str) {
    std::pair<int, int> grid_coord;
    worldToGrid(x_str, y_str, grid_coord.first, grid_coord.second); 
    if (isInWorldBounds(x_str, y_str) && (getCellProbability(grid_coord.first, grid_coord.second) < 0.8)) {
        this->strpoint = grid_coord;
    } else {
        throw std::invalid_argument("起点不在栅格地图内或该处有障碍物，无效");
    }
}
 
// 获取终点
std::pair<int, int> ObstacleGridMap::getgoal() {
    if (strpoint.first != -1 && goalpoint.second != -1) {
        return this->goalpoint;
    } else {
        throw std::invalid_argument("还未设置终点");
    }
}
 
// 获取起点
std::pair<int, int> ObstacleGridMap::getstr() {
    if (strpoint.first != -1 && goalpoint.second != -1) {
        return this->strpoint;
    } else {
        throw std::invalid_argument("还未设置起点");
    }
}

void ObstacleGridMap::set_robot_radius(double radius){
    this->robot_radius=radius;
    update_grid_();
}
void ObstacleGridMap::update_grid_(){
    double R_2=this->robot_radius*this->robot_radius;
    double wx,wy,owx,owy;
    double ob_proba=0.9;
    for(int i=0;i<this->grid_width_;i++)
    {
        for(int j=0;j<this->grid_height_;j++)
        {
            for(size_t k=0;k<this->obstacle_grid.first.size();k++)
            {
                gridToWorld(i,j,wx,wy);
                gridToWorld(this->obstacle_grid.first[k],this->obstacle_grid.second[k],owx,owy);
                double d=pow(wx-owx,2)+pow(wy-owy,2);
                if(d<=R_2)
                {
                    setCellProbability(i,j,ob_proba);
                }                
            }
        }
    }
}

std::pair<std::pair<int,int>,float> ObstacleGridMap::index_to_grid(int index)
{
    std::pair<int,int> grid;
    //gy * grid_width_ + gx
    grid.first=index%this->grid_width_;
    grid.second=index/this->grid_width_;
    std::pair<std::pair<int,int>,float> grid_data={grid,this->grid_[index]};
    return grid_data;
}

void ObstacleGridMap::save_to_PNG(std::string filename)
{
    std::string folderPath = "./image";
    std::cout << "结果截图保存至：" << filename << std::endl;
    plt::save(folderPath+"/"+filename);
}

void ObstacleGridMap::plot_iterate_path(const std::vector<std::pair<std::vector<int>, std::vector<int>>>& path_data) {
    show_plot_=false;
    plotWorldMap();
    plt::Plot* line;
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    for(size_t i=0;i<path_data.size();i++)
    {
        std::cout<<"共"<<path_data.size()<<"张，第"<<i+1<<"张"<<std::endl;       
        // 对 x 坐标进行缩放和偏移
        for (double x : path_data[i].first) {
            x_coords.push_back(x * resolution_ + 0.5 * resolution_);
        }
        // 对 y 坐标进行缩放和偏移
        for (double y : path_data[i].second) {
            y_coords.push_back(y * resolution_ + 0.5 * resolution_);
        }
        // 绘制路径
        line=new plt::Plot("path",x_coords, y_coords,"g-");
        plt::pause(plot_pause_time*800);
        line->remove();       
        delete line;
        x_coords=std::vector<double>();
        y_coords=std::vector<double>();
    }
    this->point_plot=true;
}

// 写入 CSV 文件（带标题行）
void ObstacleGridMap::writeVectorToCSV(const std::vector<double>& data, const std::string& filename) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }
 
    // 写入标题行
    outFile << "Index,Value\n";
 
    // 写入数据（索引 + 值）
    for (size_t i = 0; i < data.size(); ++i) {
        outFile << i << "," << data[i] << "\n";
    }
 
    outFile.close();
    std::cout << "Data saved to " << filename << std::endl;
}
 

// 读取 CSV 文件到 vector
std::vector<double> ObstacleGridMap::readCSVToVector(const std::string& filename) {
    std::vector<double> data;
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for reading." << std::endl;
        return data;
    }
 
    std::string line;
    bool isHeader = true; // 跳过标题行
 
    while (std::getline(inFile, line)) {
        if (isHeader) {
            isHeader = false;
            continue;
        }
 
        std::istringstream iss(line);
        std::string indexStr, valueStr;
 
        // 解析 CSV 行（格式：Index,Value）
        if (std::getline(iss, indexStr, ',') && std::getline(iss, valueStr)) {
            try {
                double value = std::stod(valueStr); // 字符串转 double
                data.push_back(value);
            } catch (const std::invalid_argument& e) {
                std::cerr << "Warning: Invalid data in line '" << line << "'" << std::endl;
            }
        }
    }
 
    inFile.close();
    return data;
}


// 在 ObstacleGridMap.cpp 中添加实现

// 辅助函数：判断点是否在矩形内（射线法）
bool isPointInRotatedRectangle(const std::vector<std::pair<double, double>>& vertices, double px, double py) {
    int n = vertices.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const auto& p1 = vertices[i];
        const auto& p2 = vertices[j];
        if (((p1.second > py) != (p2.second > py)) &&
            (px < (p2.first - p1.first) * (py - p1.second) / (p2.second - p1.second) + p1.first)) {
            inside = !inside;
        }
    }
    return inside;
}

// 辅助函数：判断两条线段是否相交
bool doLinesIntersect(double a1x, double a1y, double a2x, double a2y,
                     double b1x, double b1y, double b2x, double b2y) {
    auto ccw = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax);
    };
    return ccw(a1x, a1y, a2x, a2y, b1x, b1y) != ccw(a1x, a1y, a2x, a2y, b2x, b2y) &&
           ccw(b1x, b1y, b2x, b2y, a1x, a1y) != ccw(b1x, b1y, b2x, b2y, a2x, a2y);
}

// 辅助函数：判断矩形边是否与栅格边相交
bool doesRectangleIntersectGridCell(const std::vector<std::pair<double, double>>& rect_vertices,
                                  double grid_min_x, double grid_min_y,
                                  double grid_max_x, double grid_max_y) {
    // 栅格的4条边 - 正确定义：每条边由两个点(pair)表示
    std::pair<std::pair<double, double>, std::pair<double, double>> grid_edges[4] = {
        {{grid_min_x, grid_min_y}, {grid_max_x, grid_min_y}}, // 下边
        {{grid_max_x, grid_min_y}, {grid_max_x, grid_max_y}}, // 右边
        {{grid_max_x, grid_max_y}, {grid_min_x, grid_max_y}}, // 上边
        {{grid_min_x, grid_max_y}, {grid_min_x, grid_min_y}}  // 左边
    };

    // 检查矩形的每条边是否与栅格的任意一条边相交
    for (int i = 0; i < 4; ++i) {
        const auto& rect_p1 = rect_vertices[i];
        const auto& rect_p2 = rect_vertices[(i + 1) % 4];
        for (int j = 0; j < 4; ++j) {
            // 获取栅格边的两个端点
            const auto& grid_p1 = grid_edges[j].first;  // 第一个端点
            const auto& grid_p2 = grid_edges[j].second; // 第二个端点
            
            if (doLinesIntersect(
                rect_p1.first, rect_p1.second,  // 矩形边的起点和终点
                rect_p2.first, rect_p2.second,
                grid_p1.first, grid_p1.second,  // 栅格边的起点和终点
                grid_p2.first, grid_p2.second)) {
                return true;
            }
        }
    }

    return false;
}

void ObstacleGridMap::setRotatedRectangleObstacle(double cx, double cy, double width, double height, double theta, float probability) {
    // 1. 计算矩形的4个顶点（世界坐标系）
    double half_width = width / 2.0;
    double half_height = height / 2.0;

    std::vector<std::pair<double, double>> local_vertices = {
        {-half_width, -half_height},  // 左下
        { half_width, -half_height},  // 右下
        { half_width,  half_height},  // 右上
        {-half_width,  half_height}   // 左上
    };

    std::vector<std::pair<double, double>> world_vertices;
    for (const auto& vertex : local_vertices) {
        double x = vertex.first;
        double y = vertex.second;
        double wx = cx + x * cos(theta) - y * sin(theta);
        double wy = cy + x * sin(theta) + y * cos(theta);
        world_vertices.emplace_back(wx, wy);
    }

    // 2. 计算矩形的AABB（减少栅格遍历范围）
    double x_min = world_vertices[0].first, x_max = world_vertices[0].first;
    double y_min = world_vertices[0].second, y_max = world_vertices[0].second;
    for (const auto& vertex : world_vertices) {
        x_min = std::min(x_min, vertex.first);
        x_max = std::max(x_max, vertex.first);
        y_min = std::min(y_min, vertex.second);
        y_max = std::max(y_max, vertex.second);
    }

    // 3. 遍历AABB范围内的栅格
    int gx_min, gx_max, gy_min, gy_max;
    worldToGrid(x_min, y_min, gx_min, gy_min);
    worldToGrid(x_max, y_max, gx_max, gy_max);

    for (int gy = gy_min; gy <= gy_max; ++gy) {
        for (int gx = gx_min; gx <= gx_max; ++gx) {
            if (gx < 0 || gx >= grid_width_ || gy < 0 || gy >= grid_height_) {
                continue;
            }

            // 计算当前栅格的边界（世界坐标）
            double grid_min_x = gx * resolution_;
            double grid_min_y = gy * resolution_;
            double grid_max_x = (gx + 1) * resolution_;
            double grid_max_y = (gy + 1) * resolution_;

            // 情况1：栅格的中心点在矩形内
            double center_x = (grid_min_x + grid_max_x) / 2.0;
            double center_y = (grid_min_y + grid_max_y) / 2.0;
            if (isPointInRotatedRectangle(world_vertices, center_x, center_y)) {
                setCellProbability(gx, gy, probability);
                obstacle_grid.first.push_back(gx);
                obstacle_grid.second.push_back(gy);
                continue;
            }

            // // 情况2：矩形的边与栅格的边相交
            // if (doesRectangleIntersectGridCell(world_vertices, grid_min_x, grid_min_y, grid_max_x, grid_max_y)) {
            //     setCellProbability(gx, gy, probability);
            //     obstacle_grid.first.push_back(gx);
            //     obstacle_grid.second.push_back(gy);
            // }
        }
    }

    update_grid_(); // 更新机器人半径影响的栅格
}