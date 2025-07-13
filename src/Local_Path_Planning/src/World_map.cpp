
#include"World_map.h"

// 构造函数实现
WorldMap::WorldMap(double x_min, double x_max, double y_min, double y_max) 
    : x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max) {}

// 添加障碍物
void WorldMap::addObstacle(const Obstacle& obstacle) {
    obstacles_.push_back(obstacle);
}

//获取障碍物信息
std::vector<WorldMap::Obstacle>  WorldMap::get_Obstacle()
{
    return obstacles_;
}

// 添加车辆
void WorldMap::addVehicle(ElectricVehicleDynamicsModel* vehicle, 
                         const std::string& color) {
    vehicles_.emplace_back(vehicle, color);
}
//添加样条曲线的控制点来绘制折线
void WorldMap::add_control_point(std::pair<std::vector<double>,std::vector<double>> control_point)
{
    this->control_point_.push_back(control_point);
}

//添加目标点
void WorldMap::set_goal(std::pair<double, double> goal)
{
    this->goal_point=goal;
}
std::pair<double, double> WorldMap::get_goal()
{
    return this->goal_point;
}

void WorldMap::visualize(bool show_grid, bool equal_aspect, bool blocking) {
    if (!is_interactive_) {
        plt::figure_size(1200, 800);
        is_interactive_ = !blocking;
    } else {
        plt::clf();  // 清除当前图形（交互模式下）
    }

    // 设置坐标轴
    plt::xlim(x_min_, x_max_);
    plt::ylim(y_min_, y_max_);
    
    if (equal_aspect) plt::axis("equal");
    if (show_grid) plt::grid(true);

    //绘制控制点
    for(size_t i=0;i<control_point_.size();i++)
    {
        plt::plot(control_point_[i].first, control_point_[i].second, {{"color", "green"}}); 
    }
    //绘制目标点
    if (!std::isnan(goal_point.first))  // 正确写法
    {
        std::map<std::string, std::string> goal_point_kwargs = {
            {"c", "red"},       
            {"marker", "p"}  // 标记为五角星
        };
        std::vector<double> goal_x={goal_point.first};
        std::vector<double> goal_y={goal_point.second};
        plt::scatter(goal_x,goal_y,300.0,goal_point_kwargs);       
    }
    
    // 绘制障碍物（提取为独立方法更佳）
    for (const auto& obstacle : obstacles_) {
        std::vector<double> local_x = {-obstacle.width/2, obstacle.width/2, 
                                     obstacle.width/2, -obstacle.width/2, -obstacle.width/2};
        std::vector<double> local_y = {-obstacle.height/2, -obstacle.height/2, 
                                     obstacle.height/2, obstacle.height/2, -obstacle.height/2};
        
        std::vector<double> global_x, global_y;
        for (size_t i = 0; i < local_x.size(); ++i) {
            double x_rot = local_x[i] * cos(obstacle.rotation) - local_y[i] * sin(obstacle.rotation);
            double y_rot = local_x[i] * sin(obstacle.rotation) + local_y[i] * cos(obstacle.rotation);
            global_x.push_back(obstacle.x + x_rot);
            global_y.push_back(obstacle.y + y_rot);
        }
        plt::plot(global_x, global_y, {{"color", obstacle.color}, {"linewidth", "2"}});
    }

    // 绘制所有车辆
    for (const auto& vehicle_pair : vehicles_) {
        auto vehicle = vehicle_pair.first;
        auto color = vehicle_pair.second;
        vehicle->plot_vehicle(color);

    }

    plt::title("World Map with Vehicles and Obstacles");
    plt::xlabel("X coordinate (m)");
    plt::ylabel("Y coordinate (m)");

    if (blocking) {
        plt::show();
        is_interactive_ = false;
    } else {
        plt::pause(0.01);  // 短暂暂停以刷新图形
    }
}

void WorldMap::updateAndVisualize(bool show_grid, bool equal_aspect) {


    // 重新可视化（非阻塞模式）
    visualize(show_grid, equal_aspect, false);
}

    // 更新车辆状态并可视化
    void WorldMap::plot_planning(ElectricVehicleDynamicsModel* vehicle,
        std::vector<std::vector<double>> planning_data,
        std::string filename,
        std::vector<std::vector<std::vector<double>>> DWA_p_t)
    {
        int steps = static_cast<int>(planning_data[0].size());
        std::vector<double> traj_x, traj_y;
        int k=3;
        if(DWA_p_t.size()!=0)
        {
            for (int i = 0; i < steps; i++) {
                std::tuple<double,double,double> new_state = 
                    {planning_data[1][i], planning_data[2][i], planning_data[3][i]};
                vehicle->reset_for_planning_only(new_state,DWA_p_t[i]);
                if (i % k == 0) {
                    // 重新可视化（非阻塞模式）
                    visualize(true,true,false);
                }
            }
        }
        else
        {
            for (int i = 0; i < steps; i++) {
                std::tuple<double,double,double> new_state = 
                    {planning_data[1][i], planning_data[2][i], planning_data[3][i]};
                vehicle->reset_for_planning_only(new_state);
                if (i % k == 0) {
                    // 重新可视化（非阻塞模式）
                    visualize(true,true,false);
                }
            }
        }
        save_to_PNG(filename);
        plt::show();
    }




// 设置边界
void WorldMap::setBounds(double x_min, double x_max, double y_min, double y_max) {
    x_min_ = x_min;
    x_max_ = x_max;
    y_min_ = y_min;
    y_max_ = y_max;
}

// 清除所有对象
void WorldMap::clear() {
    obstacles_.clear();
    vehicles_.clear();
}

void WorldMap::save_to_PNG(std::string filename)
{
    std::string folderPath = "./image";
    std::cout << "结果截图保存至：" << filename << std::endl;
    plt::save(folderPath+"/"+filename);
}