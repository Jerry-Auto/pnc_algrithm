#ifndef MATH_UTILS
#define MATH_UTILS

#include <memory>
#include <utility>
#include <vector>


#define M_PI 3.14159265358979323846

constexpr double kMathEpsilon = 1e-10;

namespace math {
/// @brief 定义了一个点，或者以原点为起点的向量
struct Vec2d {
  double x = 0.0;
  double y = 0.0;
  Vec2d() = default;
  Vec2d(double x_, double y_) : x(x_), y(y_) {}
};

/// @brief 定义了一个位姿
struct Pos3d {
  double x;
  double y;
  double phi;
  Pos3d() = default; 
  Pos3d(double x_,double y_,double phi_):x(x_),y(y_),phi(phi_){}
  
};

/// @brief 定义了一条线段
struct LineSegment2d {
  Vec2d start;
  Vec2d end;
  Vec2d unit_direction;
  double heading = 0.0;
  double length = 0.0;

};

/// @brief 定义了一条曲线，起点位姿，曲率半径和弧长
struct CurvePath {
  double x;
  double y;
  double phi;
  double dist;
  double radius;
};

/// @brief 定义了一个有任意朝向的矩形框
struct Box2d {
  Vec2d center;
  double length = 0.0;
  double width = 0.0;
  double half_length = 0.0;
  double half_width = 0.0;
  double heading = 0.0;
  double cos_heading = 1.0;
  double sin_heading = 0.0;
};

double sign(double x);
double P_t_l_Distance(double query_x, double query_y, double start_x,double start_y, double end_x, double end_y);

double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);
double NormalizeAngle(const double angle);
std::pair<double, double> Cartesian2Polar(double x, double y);
double DistanceTo(const LineSegment2d &line_segment2d, const Vec2d &point);
double GetVecAngle(Vec2d v);
double DistanceTo(const Box2d &bounding_box, const Vec2d &point);
double DistanceTo(const Box2d &bounding_box, const LineSegment2d &line_segment);
bool IsPointIn(const Box2d &bounding_box, const Vec2d &point);
bool HasOverlap(const Box2d &bounding_box, const LineSegment2d &line_segment);
double CalProjectInX(const Pos3d &pos, const Vec2d &point);
double CalProjectInX(const Pos3d &pos1, const Pos3d &pos2);
double CalProjectInY(const Pos3d &pos, const Vec2d &point);
double CalProjectInY(const Pos3d &pos1, const Pos3d &pos2);
void ConnectByLineCircle(Pos3d current_pos, Pos3d end_pos);
Pos3d CalEndPosWithACurvePath(const Pos3d &start_pos, double dist,
                              double radius);
Pos3d CalEndPosWithACurvePath(CurvePath curve_path);
std::vector<CurvePath> CalCurvePathConnectTwoPose(const Pos3d &start_pos,
                                                  const Pos3d &end_pos,
                                                  double r_min);
std::vector<Pos3d> GetTrajFromCurvePathsConnect(const Pos3d &start_pos,
                                                const Pos3d &end_pos,
                                                double r_min,
                                                double resolution);

Vec2d rotatePoint(const Vec2d& point, double rotation);

    std::vector<LineSegment2d> computeRectangleEdges(
    double center_x, double center_y,  // 矩形中心坐标
    double length, double width,       // 矩形的长和宽
    double rotation                    // 旋转角度（弧度）
);

}  // namespace math

#endif
