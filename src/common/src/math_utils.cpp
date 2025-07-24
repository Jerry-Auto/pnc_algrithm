#include <cmath>
#include <limits>

#include "math_utils.h"

namespace math {
/// @brief 返回一个数的正负符号
/// @param x 
/// @return +1，-1
double sign(double x) { return x >= 0 ? 1 : -1; }

/// @brief 计算点query到以start为起点，end为终点的向量的最短距离
/// @param query_x 
/// @param query_y 
/// @param start_x 
/// @param start_y 
/// @param end_x 
/// @param end_y 
/// @return 
double P_t_l_Distance(double query_x, double query_y, double start_x,
                     double start_y, double end_x, double end_y) {
  const double x0 = query_x - start_x;
  const double y0 = query_y - start_y;
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double length=hypot(dx,dy);
  const double proj = x0 * dx + y0 * dy;

  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length * length) {
    return hypot(x0 - dx, y0 - dy);
  }
  return std::abs(x0 * dy - y0 * dx) / length;
}
/// @brief 以pos的点为向量起点，计算终点在point的向量在pos射线上的投影长度，有正负
/// @param pos pos表示一个有起点的射线
/// @param point 
/// @return 
double CalProjectInX(const Pos3d &pos, const Vec2d &point) {
  return (point.x - pos.x) * std::cos(pos.phi) +
         (point.y - pos.y) * std::sin(pos.phi);
}
/// @brief 以pos1的点为向量起点，计算终点在pos2的点的向量在pos1射线上的投影长度，有正负
/// @param pos1 
/// @param pos2 
/// @return 
double CalProjectInX(const Pos3d &pos1, const Pos3d &pos2) {
  return (pos2.x - pos1.x) * std::cos(pos1.phi) +
         (pos2.y - pos1.y) * std::sin(pos1.phi);
}
/// @brief 以pos的点为向量起点，计算终点在point的向量到pos射线的最短距离（垂直距离），有正负，pos-pot向量到pos射线，逆时针角度为正
/// @param pos 
/// @param point 
/// @return 
double CalProjectInY(const Pos3d &pos, const Vec2d &point) {
  return (point.y - pos.y) * std::cos(pos.phi) -
         (point.x - pos.x) * std::sin(pos.phi);
}
/// @brief 以pos1的点为向量起点，计算终点在pos2的向量到pos1射线的最短距离（垂直距离），有正负，pos1-pos2向量到pos1射线，逆时针角度为正
/// @param pos1 
/// @param pos2 
/// @return 
double CalProjectInY(const Pos3d &pos1, const Pos3d &pos2) {
  return (pos2.y - pos1.y) * std::cos(pos1.phi) -
         (pos2.x - pos1.x) * std::sin(pos1.phi);
}
/// @brief 计算两个有共同起点的向量之间的叉积，v1叉乘v2
/// @param start_point 两向量的共同起点
/// @param end_point_1 终点1，角度计算的起点
/// @param end_point_2 终点2
/// @return 
double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  Vec2d v1{end_point_1.x - start_point.x, end_point_1.y - start_point.y};
  Vec2d v2{end_point_2.x - start_point.x, end_point_2.y - start_point.y};

  return v1.x * v2.y - v1.y * v2.x;
}
/// @brief 把角度规范到-pi,+pi
/// @param angle 
/// @return 
double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}
/// @brief 笛卡尔坐标转换为极坐标
/// @param x 
/// @param y 
/// @return （r,theta）
std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}
/// @brief 计算点到线段之间的距离
/// @param line_segment2d 
/// @param point 
/// @return 
double DistanceTo(const LineSegment2d &line_segment2d, const Vec2d &point) {
  if (line_segment2d.length <= 1e-10) {
    return std::hypot(
        point.x - line_segment2d.start.x,
        point.y - line_segment2d.start.y);  // point.DistanceTo(start_);
  }
  const double x0 = point.x - line_segment2d.start.x;
  const double y0 = point.y - line_segment2d.start.y;
  const double proj = x0 * line_segment2d.unit_direction.x +
                      y0 * line_segment2d.unit_direction.y;
  if (proj <= 0.0) {
    return std::hypot(x0, y0);
  }
  if (proj >= line_segment2d.length) {
    return std::hypot(point.x - line_segment2d.end.x,
                      point.y - line_segment2d.end.y);
  }
  return std::abs(x0 * line_segment2d.unit_direction.y -
                  y0 * line_segment2d.unit_direction.x);
}
/// @brief 计算向量的角度
/// @param v 
/// @return 
double GetVecAngle(Vec2d v) { return std::atan2(v.y, v.x); }

/// @brief 计算点到一个有任意朝向的矩形盒子之间的最短距离，返回0，说明在盒子内部或边界上
/// @param bounding_box 
/// @param point 
/// @return 
double DistanceTo(const Box2d &bounding_box, const Vec2d &point) {
  const double x0 = point.x - bounding_box.center.x;
  const double y0 = point.y - bounding_box.center.y;
  const double dx =
      std::abs(x0 * bounding_box.cos_heading + y0 * bounding_box.sin_heading) -
      bounding_box.half_length;
  const double dy =
      std::abs(x0 * bounding_box.sin_heading - y0 * bounding_box.cos_heading) -
      bounding_box.half_width;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

/// @brief 计算线段到矩形盒子之间的最短距离
/// @param bounding_box 
/// @param line_segment 
/// @return 
double DistanceTo(const Box2d &bounding_box,
                  const LineSegment2d &line_segment) {
  if (line_segment.length <= kMathEpsilon) {
    return DistanceTo(bounding_box, line_segment.start);
  }
  const double ref_x1 = line_segment.start.x - bounding_box.center.x;
  const double ref_y1 = line_segment.start.y - bounding_box.center.y;
  double x1 =
      ref_x1 * bounding_box.cos_heading + ref_y1 * bounding_box.sin_heading;
  double y1 =
      ref_x1 * bounding_box.sin_heading - ref_y1 * bounding_box.cos_heading;
  double box_x = bounding_box.half_length;
  double box_y = bounding_box.half_width;
  int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
  int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
  if (gx1 == 0 && gy1 == 0) {
    return 0.0;
  }
  const double ref_x2 = line_segment.end.x - bounding_box.center.x;
  const double ref_y2 = line_segment.end.y - bounding_box.center.y;
  double x2 =
      ref_x2 * bounding_box.cos_heading + ref_y2 * bounding_box.sin_heading;
  double y2 =
      ref_x2 * bounding_box.sin_heading - ref_y2 * bounding_box.cos_heading;
  int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
  int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));
  if (gx2 == 0 && gy2 == 0) {
    return 0.0;
  }
  if (gx1 < 0 || (gx1 == 0 && gx2 < 0)) {
    x1 = -x1;
    gx1 = -gx1;
    x2 = -x2;
    gx2 = -gx2;
  }
  if (gy1 < 0 || (gy1 == 0 && gy2 < 0)) {
    y1 = -y1;
    gy1 = -gy1;
    y2 = -y2;
    gy2 = -gy2;
  }
  if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2)) {
    std::swap(x1, y1);
    std::swap(gx1, gy1);
    std::swap(x2, y2);
    std::swap(gx2, gy2);
    std::swap(box_x, box_y);
  }
  if (gx1 == 1 && gy1 == 1) {
    switch (gx2 * 3 + gy2) {
      case 4:
        return P_t_l_Distance(box_x, box_y, x1, y1, x2, y2);
      case 3:
        return (x1 > x2) ? (x2 - box_x)
                         : P_t_l_Distance(box_x, box_y, x1, y1, x2, y2);
      case 2:
        return (x1 > x2) ? P_t_l_Distance(box_x, -box_y, x1, y1, x2, y2)
                         : P_t_l_Distance(box_x, box_y, x1, y1, x2, y2);
      case -1:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) >= 0.0
                   ? 0.0
                   : P_t_l_Distance(box_x, -box_y, x1, y1, x2, y2);
      case -4:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) <= 0.0
                   ? P_t_l_Distance(box_x, -box_y, x1, y1, x2, y2)
                   : (CrossProd({x1, y1}, {x2, y2}, {-box_x, box_y}) <= 0.0
                          ? 0.0
                          : P_t_l_Distance(-box_x, box_y, x1, y1, x2, y2));
    }
  } else {
    switch (gx2 * 3 + gy2) {
      case 4:
        return (x1 < x2) ? (x1 - box_x)
                         : P_t_l_Distance(box_x, box_y, x1, y1, x2, y2);
      case 3:
        return std::min(x1, x2) - box_x;
      case 1:
      case -2:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, box_y}) <= 0.0
                   ? 0.0
                   : P_t_l_Distance(box_x, box_y, x1, y1, x2, y2);
      case -3:
        return 0.0;
    }
  }
  return 0.0;
}


/// @brief 判断一个点是否在一个矩形盒子里
/// @param bounding_box 
/// @param point 
/// @return 
bool IsPointIn(const Box2d &bounding_box, const Vec2d &point) {
  const double x0 = point.x - bounding_box.center.x;
  const double y0 = point.y - bounding_box.center.y;
  const double dx =
      std::abs(x0 * bounding_box.cos_heading + y0 * bounding_box.sin_heading);
  const double dy =
      std::abs(-x0 * bounding_box.sin_heading + y0 * bounding_box.cos_heading);
  return dx <= bounding_box.half_length + 1e-10 &&
         dy <= bounding_box.half_width + 1e-10;
}


/// @brief 判断一条线段与矩形盒子是否有重叠
/// @param bounding_box 
/// @param line_segment 
/// @return 
bool HasOverlap(const Box2d &bounding_box, const LineSegment2d &line_segment) {
  if (line_segment.length <= 1e-10) {
    return IsPointIn(bounding_box, line_segment.start);
  }

  std::vector<Vec2d> corners;
  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();

  const double dx1 = bounding_box.cos_heading * bounding_box.half_length;
  const double dy1 = bounding_box.sin_heading * bounding_box.half_length;
  const double dx2 = bounding_box.sin_heading * bounding_box.half_width;
  const double dy2 = -bounding_box.cos_heading * bounding_box.half_width;

  corners.emplace_back(Vec2d{bounding_box.center.x + dx1 + dx2,
                             bounding_box.center.y + dy1 + dy2});
  corners.emplace_back(Vec2d{bounding_box.center.x + dx1 - dx2,
                             bounding_box.center.y + dy1 - dy2});
  corners.emplace_back(Vec2d{bounding_box.center.x - dx1 - dx2,
                             bounding_box.center.y - dy1 - dy2});
  corners.emplace_back(Vec2d{bounding_box.center.x - dx1 + dx2,
                             bounding_box.center.y - dy1 + dy2});
  for (auto &corner : corners) {
    max_x_ = std::fmax(corner.x, max_x_);
    min_x_ = std::fmin(corner.x, min_x_);
    max_y_ = std::fmax(corner.y, max_y_);
    min_y_ = std::fmin(corner.y, min_y_);
  }

  if (std::fmax(line_segment.start.x, line_segment.end.x) < min_x_ ||
      std::fmin(line_segment.start.x, line_segment.end.x) > max_x_ ||
      std::fmax(line_segment.start.y, line_segment.end.y) < min_y_ ||
      std::fmin(line_segment.start.y, line_segment.end.y) > max_y_) {
    return false;
  }
  return DistanceTo(bounding_box, line_segment) <= 1e-10;
}

/// @brief 给定起点位置航向，移动的弧长，曲率半径，计算终点的位姿
/// @param start_pos 移动起点位姿
/// @param dist 移动距离，弧长
/// @param radius 曲率半径，右转<0,左转>0
/// @return 
Pos3d CalEndPosWithACurvePath(const Pos3d &start_pos, double dist,
                              double radius) {
  Pos3d end_pos;

  if (std::abs(radius) < kMathEpsilon) {
    end_pos.x = start_pos.x + std::cos(start_pos.phi) * dist;
    end_pos.y = start_pos.y + std::sin(start_pos.phi) * dist;
    end_pos.phi = start_pos.phi;
  } else {
    double theta = dist/radius;
    double beta=start_pos.phi - sign(radius) * M_PI / 2.0;
    end_pos.x = start_pos.x +(std::cos(beta + theta)-std::cos(beta)) * std::abs(radius);
    end_pos.y = start_pos.y +(std::sin(beta + theta)-std::sin(beta)) * std::abs(radius);
    end_pos.phi = start_pos.phi + theta;
  }

  return end_pos;

}

/// @brief 通过一条弧线计算终点位姿
/// @param curve_path 定义的一条弧线
/// @return 
Pos3d CalEndPosWithACurvePath(CurvePath curve_path) {
  return CalEndPosWithACurvePath(
      Pos3d{curve_path.x, curve_path.y, curve_path.phi}, curve_path.dist,
      curve_path.radius);
}

/// @brief 计算连接两个位姿的中间曲线
/// @param start_pos 
/// @param end_pos 
/// @param r_min 允许的最小转弯半径
/// @return 
std::vector<CurvePath> CalCurvePathConnectTwoPose(const Pos3d &start_pos,
                                                  const Pos3d &end_pos,
                                                  double r_min) {
  const double k_min_value = 1e-4;
  double rx_start_pos_end_pos = CalProjectInX(start_pos, end_pos);//prjs
  double ry_start_pos_end_pos = CalProjectInY(start_pos, end_pos);//ds
  double ry_end_pos_start_pos = CalProjectInY(end_pos, start_pos);//de
  double theta = NormalizeAngle(end_pos.phi - start_pos.phi);

  std::vector<CurvePath> curve_paths;
  //prjs=0,无法规划
  if (std::abs(rx_start_pos_end_pos) < k_min_value) {
    return curve_paths;
  }

  // a line path，起点终点位姿共线
  if (std::abs(ry_start_pos_end_pos) < k_min_value &&
      std::abs(theta) < k_min_value) {
        
    CurvePath line_path{start_pos.x, start_pos.y, start_pos.phi,
                        rx_start_pos_end_pos, 0};
    curve_paths.push_back(line_path);
    return curve_paths;
  }

  // a circle path
  if (std::abs(std::atan(ry_start_pos_end_pos / rx_start_pos_end_pos) -
               theta / 2.0) < k_min_value) {
    double r = rx_start_pos_end_pos / std::sin(theta);
    if (std::abs(r) < r_min) {
      return curve_paths;
    }

    CurvePath circle_path{start_pos.x, start_pos.y, start_pos.phi, r * theta,
                          r};
    curve_paths.push_back(circle_path);
    return curve_paths;
  }

  if (std::abs(1.0 - std::cos(theta)) <= k_min_value) {
    return curve_paths;
  }

  if (std::abs(ry_end_pos_start_pos) > std::abs(ry_start_pos_end_pos)) {
    double r = ry_start_pos_end_pos / (1.0 - std::cos(theta));
    double line_length =
        ry_end_pos_start_pos / std::sin(theta) - r * std::tan(theta / 2.0);

    if (std::abs(r) < r_min) {
      return curve_paths;
    }

    CurvePath line_path{start_pos.x, start_pos.y, start_pos.phi, line_length,
                        0};
    Pos3d min_pos = CalEndPosWithACurvePath(line_path);
    CurvePath circle_path{min_pos.x, min_pos.y, min_pos.phi, r * theta, r};
    curve_paths.push_back(line_path);
    curve_paths.push_back(circle_path);
  } else {
    double r = ry_end_pos_start_pos / (1.0 - std::cos(theta));
    double line_length =
        ry_start_pos_end_pos / std::sin(theta) - r * std::tan(theta / 2.0);

    if (std::abs(r) < r_min) {
      return curve_paths;
    }

    CurvePath circle_path{start_pos.x, start_pos.y, start_pos.phi, r * theta,
                          r};
    Pos3d min_pos = CalEndPosWithACurvePath(circle_path);
    CurvePath line_path{min_pos.x, min_pos.y, min_pos.phi, line_length, 0};
    curve_paths.push_back(circle_path);
    curve_paths.push_back(line_path);
  }

  return curve_paths;
}

/// @brief 计算连接两个位姿的中间轨迹
/// @param start_pos 
/// @param end_pos 
/// @param r_min 
/// @param resolution 
/// @return 位姿序列
std::vector<Pos3d> GetTrajFromCurvePathsConnect(const Pos3d &start_pos,
                                                const Pos3d &end_pos,
                                                double r_min,
                                                double resolution) {
  std::vector<Pos3d> traj_points;
  std::vector<CurvePath> curve_paths =
      CalCurvePathConnectTwoPose(start_pos, end_pos, r_min);
  if (curve_paths.empty()) {
    return traj_points;
  }

  for (const auto &path : curve_paths) {
    int traj_point_num = std::abs(path.dist) / resolution;
    for (int i = 0; i < traj_point_num; ++i) {
      const Pos3d path_start_pos{path.x, path.y, path.phi};
      Pos3d temp_pos = CalEndPosWithACurvePath(
          path_start_pos, sign(path.dist) * i * resolution, path.radius);
      traj_points.push_back(temp_pos);
    }
  }

  return traj_points;
}

}  // namespace math