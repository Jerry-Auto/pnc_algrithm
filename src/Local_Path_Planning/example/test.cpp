#include "matplotlibcpp.h"
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;

class MapEnvironment {
public:
    // 构造函数
    MapEnvironment(double road_width = 3.5, double map_length = 100.0) 
        : d(road_width), len_line(map_length) {
        // 初始化灰色区域坐标（道路边界）
        greyZone_x = {-5, -5, len_line, len_line};
        greyZone_y = {-d - 0.5, d + 0.5, d + 0.5, -d - 0.5};
    }

    // 添加障碍物
    void addObstacle(double x, double y) {
        obstacle_pos.emplace_back(x, y);
    }

    // 添加多个障碍物
    void addObstacles(const vector<Vector2d>& obstacles) {
        obstacle_pos.insert(obstacle_pos.end(), obstacles.begin(), obstacles.end());
    }

    // 获取所有障碍物
    const vector<Vector2d>& getObstacles() const {
        return obstacle_pos;
    }

    // 设置目标点
    void setTarget(double x, double y) {
        target << x, y;
    }

    // 获取目标点
    Vector2d getTarget() const {
        return target;
    }

    // 设置车辆初始状态
    void setVehicleInitState(double x, double y, double heading = 0.0) {
        vehicle_state << x, y, heading;
        init_pos << x, y;
    }

    // 获取车辆当前状态
    Vector3d getVehicleState() const {
        return vehicle_state;
    }

    // 生成矩形顶点（用于绘制）
    static vector<double> generateRectangleVertices(double x, double y, double heading, 
                                                   double length, double width) {
        double half_len = length / 2.0;
        double half_wid = width / 2.0;

        // 矩形的四个顶点（局部坐标系）
        vector<Vector2d> local_vertices = {
            Vector2d(-half_len, -half_wid),  // 左后
            Vector2d(half_len, -half_wid),   // 右后
            Vector2d(half_len, half_wid),    // 右前
            Vector2d(-half_len, half_wid)    // 左前
        };

        // 旋转矩阵
        Matrix2d rot;
        rot << cos(heading), -sin(heading),
               sin(heading), cos(heading);

        // 旋转并平移顶点
        vector<double> x_verts, y_verts;
        for (const auto& v : local_vertices) {
            Vector2d rotated = rot * v;
            x_verts.push_back(x + rotated.x());
            y_verts.push_back(y + rotated.y());
        }

        // 闭合矩形
        x_verts.push_back(x_verts[0]);
        y_verts.push_back(y_verts[0]);

        return x_verts; // 返回x顶点，y顶点需要类似处理
    }

    // 更新车辆状态并模拟运动
    void updateVehicle(double step_size = 0.5, double heading_change = 0.02) {
        vehicle_state(0) += step_size * cos(vehicle_state(2)); // x更新
        vehicle_state(1) += step_size * sin(vehicle_state(2)); // y更新
        vehicle_state(2) += heading_change; // 航向角变化

        // 记录轨迹
        trajectory_x.push_back(vehicle_state(0));
        trajectory_y.push_back(vehicle_state(1));
    }

    // 绘制整个环境
    void drawEnvironment(bool show_plot = true, const string& filename = "") {
        plt::clf();

        // 画分界线
        map<string, string> keywords;
        keywords["color"] = "grey";
        plt::fill(greyZone_x, greyZone_y, keywords);
        plt::plot({-5, len_line}, {0, 0}, "w--");
        plt::plot({-5, len_line}, {d, d}, "w--");
        plt::plot({-5, len_line}, {-d, -d}, "w--");

        // 画障碍物和目标
        for (const auto& obs : obstacle_pos) {
            plt::plot(vector<double>{obs(0)}, vector<double>{obs(1)}, "ro");
        }
        plt::plot(vector<double>{target(0)}, vector<double>{target(1)}, "gv");
        plt::plot(vector<double>{init_pos(0)}, vector<double>{init_pos(1)}, "bs");

        // 画轨迹
        if (!trajectory_x.empty()) {
            plt::plot(trajectory_x, trajectory_y, "y");
        }

        // 画车辆（矩形）
        if (trajectory_x.size() > 0) {
            auto vertices = generateRectangleVertices(vehicle_state(0), vehicle_state(1), 
                                                     vehicle_state(2), vehicle_length, vehicle_width);
            // 这里需要正确处理x和y坐标
            vector<double> rect_x, rect_y;
            double heading = vehicle_state(2);
            
            // 四个角点
            vector<Vector2d> corners = {
                Vector2d(-vehicle_length/2, -vehicle_width/2),
                Vector2d(vehicle_length/2, -vehicle_width/2),
                Vector2d(vehicle_length/2, vehicle_width/2),
                Vector2d(-vehicle_length/2, vehicle_width/2)
            };
            
            for (const auto& corner : corners) {
                double x_rot = corner.x() * cos(heading) - corner.y() * sin(heading) + vehicle_state(0);
                double y_rot = corner.x() * sin(heading) + corner.y() * cos(heading) + vehicle_state(1);
                rect_x.push_back(x_rot);
                rect_y.push_back(y_rot);
            }
            // 闭合矩形
            rect_x.push_back(rect_x[0]);
            rect_y.push_back(rect_y[0]);
            
            plt::plot(rect_x, rect_y, "b-");
        }

        plt::grid(true);
        plt::ylim(-100.0, 100.0);
        plt::xlim(-100.0, 100.0);
        
        if (show_plot) {
            plt::pause(0.01);
        }
        
        if (!filename.empty()) {
            cout << "Saving result to " << filename << endl;
            plt::save(filename);
        }
    }

    // 设置车辆参数
    void setVehicleParameters(double length = 4.7, double width = 1.8) {
        vehicle_length = length;
        vehicle_width = width;
    }

    // 模拟车辆运动
    void simulate(int num_iterations = 300, double step_size = 0.5, 
                 double heading_change = 0.02, const string& save_filename = "") {
        for (int i = 0; i < num_iterations; ++i) {
            updateVehicle(step_size, heading_change);
            drawEnvironment(true, (i == num_iterations - 1) ? save_filename : "");
        }
    }

private:
    // 地图参数
    double d;           // 道路标准宽度
    double len_line;    // 地图长度
    vector<double> greyZone_x, greyZone_y; // 灰色区域坐标

    // 环境元素
    Vector2d target;                     // 目标点
    vector<Vector2d> obstacle_pos;       // 障碍物位置
    Vector2d init_pos;                   // 车辆初始位置

    // 车辆状态
    Vector3d vehicle_state;              // x, y, heading
    vector<double> trajectory_x, trajectory_y; // 车辆轨迹

    // 车辆参数
    double vehicle_length = 4.7;         // 车长
    double vehicle_width = 1.8;          // 车宽
};
void testBasicFunctions() {
    cout << "=== 基础功能测试 ===" << endl;
    
    // 创建地图环境
    MapEnvironment env(3.5, 100.0);
    
    // 测试目标点设置
    env.setTarget(90, 5);
    cout << "目标点设置为: (" << env.getTarget().x() << ", " << env.getTarget().y() << ")" << endl;
    
    // 测试障碍物添加
    env.addObstacle(10, 2);
    env.addObstacle(20, -2);
    env.addObstacle(30, 1);
    
    vector<Vector2d> obstacles = {
        Vector2d(40, 3),
        Vector2d(50, -1),
        Vector2d(60, 0)
    };
    env.addObstacles(obstacles);
    
    cout << "已添加障碍物数量: " << env.getObstacles().size() << endl;
    
    // 测试车辆设置
    env.setVehicleParameters(4.7, 1.8);
    env.setVehicleInitState(0, 0, 0.5); // 初始位置(0,0)，初始朝向0.5弧度
    
    // 测试单步更新
    env.updateVehicle(1.0, 0.1);
    auto state = env.getVehicleState();
    cout << "车辆更新后状态 - X: " << state[0] 
         << ", Y: " << state[1] 
         << ", 朝向: " << state[2] << endl;
    
    // 绘制但不显示（保存到文件）
    env.drawEnvironment(false, "basic_test.png");
    cout << "基础测试完成，结果已保存到 basic_test.png" << endl;
}

void testObstacleLayouts() {
    cout << "\n=== 障碍物布局测试 ===" << endl;
    
    vector<string> layouts = {"单列", "双列", "之字形", "随机"};
    
    for (const auto& layout : layouts) {
        cout << "测试布局: " << layout << endl;
        
        MapEnvironment env(4.0, 120.0); // 稍宽的道路
        env.setTarget(110, 0);
        env.setVehicleParameters(4.5, 1.7);
        env.setVehicleInitState(0, 0, 0);
        
        // 根据布局类型添加障碍物
        if (layout == "单列") {
            for (int i = 0; i < 10; ++i) {
                env.addObstacle(15 + i*10, 2.5);
            }
        } 
        else if (layout == "双列") {
            for (int i = 0; i < 8; ++i) {
                env.addObstacle(15 + i*12, 3.0);
                env.addObstacle(15 + i*12, -3.0);
            }
        } 
        else if (layout == "之字形") {
            for (int i = 0; i < 15; ++i) {
                double y = (i % 2 == 0) ? 2.0 : -2.0;
                env.addObstacle(10 + i*7, y);
            }
        } 
        else if (layout == "随机") {
            srand(42); // 固定随机种子以便复现
            for (int i = 0; i < 20; ++i) {
                double x = 10 + rand() % 100;
                double y = -3.5 + (rand() % 70) / 10.0;
                env.addObstacle(x, y);
            }
        }
        
        // 模拟运动
        string filename = "obstacle_layout_" + layout + ".png";
        env.simulate(400, 0.4, 0.015, filename);
        cout << layout << "布局测试完成，结果已保存到 " << filename << endl;
    }
}

void testVehicleControl() {
    cout << "\n=== 车辆控制参数测试 ===" << endl;
    
    vector<pair<double, double>> controlParams = {
        {0.3, 0.01},   // 小步长，小转向
        {0.8, 0.03},   // 大步长，中等转向
        {0.5, 0.05},   // 中等步长，大转向
        {1.0, 0.005}   // 大步长，微小转向
    };
    
    for (const auto& params : controlParams) {
        double stepSize = params.first;
        double headingChange = params.second;
        
        cout << "测试参数 - 步长: " << stepSize 
             << ", 转向变化: " << headingChange << endl;
        
        MapEnvironment env(3.5, 100.0);
        env.setTarget(90, 0);
        
        // 添加一些障碍物
        for (int i = 0; i < 5; ++i) {
            env.addObstacle(20 + i*15, (i % 2 == 0) ? 2.0 : -2.0);
        }
        
        env.setVehicleParameters(4.7, 1.8);
        env.setVehicleInitState(0, 0, 0);
        
        // 模拟运动
        string paramStr = "step_" + to_string((int)(stepSize*100)) + 
                         "_turn_" + to_string((int)(headingChange*1000));
        string filename = "control_params_" + paramStr + ".png";
        
        env.simulate(300, stepSize, headingChange, filename);
        cout << "参数测试完成，结果已保存到 " << filename << endl;
    }
}

void testNarrowPassage() {
    cout << "\n=== 狭窄通道测试 ===" << endl;
    
    MapEnvironment env(2.0, 80.0); // 非常窄的道路
    env.setTarget(75, 0);
    env.setVehicleParameters(4.0, 1.5); // 稍小的车辆
    
    // 创建狭窄通道，两侧有障碍物
    for (int i = 0; i < 10; ++i) {
        // 左侧障碍物
        env.addObstacle(10 + i*7, 1.1);
        // 右侧障碍物
        env.addObstacle(10 + i*7, -1.1);
        
        // 在通道中间添加一些随机障碍物增加难度
        if (i % 3 == 0 && i > 2 && i < 8) {
            env.addObstacle(10 + i*7, (i % 2 == 0) ? 0.5 : -0.5);
        }
    }
    
    // 设置车辆初始位置和朝向（稍微偏向一侧）
    env.setVehicleInitState(0, -0.8, 0.1);
    
    // 模拟运动 - 使用较小的步长和转向变化以获得更精确的控制
    env.simulate(500, 0.3, 0.008, "narrow_passage_test.png");
    cout << "狭窄通道测试完成，结果已保存到 narrow_passage_test.png" << endl;
}
void testDynamicObstacles() {
    cout << "\n=== 动态障碍物测试 ===" << endl;
    
    MapEnvironment env(4.0, 100.0);
    env.setTarget(90, 0);
    env.setVehicleParameters(4.7, 1.8);
    env.setVehicleInitState(0, 0, 0);
    
    // 添加静态障碍物
    env.addObstacle(30, 3);
    env.addObstacle(30, -3);
    env.addObstacle(60, 2);
    env.addObstacle(60, -2);
    
    // 模拟过程中动态添加障碍物
    vector<Vector2d> dynamicObstacles = {
        Vector2d(45, 0),   // 会在车辆路径上
        Vector2d(75, 1.5), // 右侧
        Vector2d(75, -1.5) // 左侧
    };
    
    // 手动模拟过程，动态添加障碍物
    for (int i = 0; i < 400; ++i) {
        // 在特定迭代次数添加动态障碍物
        if (i == 150) {
            for (const auto& obs : dynamicObstacles) {
                env.addObstacle(obs.x(), obs.y());
            }
            cout << "在第 " << i << " 次迭代添加了动态障碍物" << endl;
        }
        
        // 更新车辆状态
        double stepSize = (i < 200) ? 0.5 : 0.4; // 后期减速
        double turn = (i < 100) ? 0.01 : ((i < 300) ? 0.015 : 0.008);
        env.updateVehicle(stepSize, turn);
        
        // 绘制
        bool show = (i % 10 == 0); // 每10帧显示一次
        string filename = (i == 399) ? "dynamic_obstacles_test.png" : "";
        env.drawEnvironment(show, filename);
    }
    
    cout << "动态障碍物测试完成，结果已保存到 dynamic_obstacles_test.png" << endl;
}

int main() {
    // 运行所有测试
    //testBasicFunctions();
    //testObstacleLayouts();
    testVehicleControl();
    //testNarrowPassage();
    //testDynamicObstacles();
    
    cout << "\n所有测试完成！" << endl;
    return 0;
}