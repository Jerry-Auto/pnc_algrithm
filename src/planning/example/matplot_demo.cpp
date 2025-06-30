#include "matplotlibcpp.h"
#include <vector>
#include <map>
#include <sstream>

namespace plt = matplotlibcpp;

int main() {
    // 图形标题
    std::string title = "My Plot";

    // 世界坐标范围
    double world_width_ = 10.0;
    double world_height_ = 5.0;

    // 填充多边形的坐标
    std::vector<double> x = {0, 1, 1, 0};
    std::vector<double> y = {0, 0, 1, 1};

    // 填充多边形的关键字参数
    // 尝试直接传递数值类型的 alpha（如果 matplotlibcpp 支持）
    std::map<std::string, std::string> fill_keywords = {
        {"facecolor", "blue"},   // 填充颜色为蓝色
        {"edgecolor", "black"}   // 边框颜色为黑色
    };
    // 如果 matplotlibcpp 支持，可以通过其他方式传递 alpha（如修改 matplotlibcpp 的实现）
    // ============================

    // 1. 折线图（Line Plot）

    // ============================

    std::vector<double> x1 = {1, 2, 3, 4, 5};

    std::vector<double> y1 = {1, 4, 9, 16, 25};

    std::vector<double> y2 = {1, 2, 3, 4, 5};

 

    // 绘制折线图

    // plt::plot(x1, y1, "r-"); // 红色实线

    // plt::plot(x1, y2, "g--"); // 绿色虚线

    plt::Plot* line=new plt::Plot("My Line",x1, y1,"g-");
    plt::pause(1);
    line->remove();
    delete line;
    line=new plt::Plot("My Line",x1, y2,"g-");
    //plt::draw(); 
    plt::show(); // 显示空线条（不可见）


        // ============================

    // 2. 散点图（Scatter Plot）

    // ============================

    std::vector<double> scatter_x = {1, 2, 3, 4, 5};

    std::vector<double> scatter_y = {5, 4, 3, 2, 1};

 

    // 准备散点图的关键字参数

    std::map<std::string, std::string> scatter_kwargs = {

        {"c", "b"},       // 颜色为蓝色

        {"marker", "o"},  // 标记为圆点

        {"label", "Scatter Points"} // 图例标签

    };
    plt::scatter(scatter_x, scatter_y, 50.0, scatter_kwargs);
    
        // 确保图例可见
    plt::legend({{"loc", "upper right"}});

    // 文本位置和内容
    double center_x = 0.5;
    double center_y = 0.5;
    std::ostringstream ss;
    ss << "Center";



    // 绘制图形
    plt::figure(); // 创建一个新的图形窗口
    plt::title(title);
    plt::xlim(0.0, world_width_);
    plt::ylim(0.0, world_height_);
    plt::fill(x, y, fill_keywords); // 暂时移除 alpha 参数
    plt::xlabel("X-axis");
    plt::ylabel("Y-axis");
    // 文本的关键字参数（如果需要）
    // 由于 plt::text 不支持直接传递关键字参数，这里只传递基本参数
    plt::text(center_x, center_y, ss.str());

    // 显示图形
    plt::show();

 
    return 0;
}