#ifndef B_SPLINE_H
#define B_SPLINE_H

#include<iostream>
#include<Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
#include <optional>

class B_spline
{
private:
    int node_vect_type=0;
    std::vector<Eigen::Vector2d> control_point;//控制点信息
    int curve_order;//曲线阶次；
    double factor=1;//节点区间的最大值
    std::vector<double> node_vector_;//节点向量
    double calcul_base(int i,int k,double u);

    Eigen::Vector2d calcul_B_U(double u);//求样条曲线在u点的取值
    void calcu_node_vect();
    std::pair<std::vector<double>, std::vector<double>> calculateAngleAndCurvature(const std::vector<Eigen::Vector2d>& points); 
    // // 多项式拟合函数
    // Eigen::VectorXd polyfit(const Eigen::VectorXd& x, const Eigen::VectorXd& y, int degree);
    // std::vector<Eigen::VectorXd> getSegmentYXPolynomials();
    // Eigen::Vector2d calcul_B_l_u(int l,double u);//求样条曲线在u点的l阶导数


public:
    B_spline(
    std::optional<std::vector<Eigen::Vector2d>> control_point,
    std::optional<int> k,
    std::optional<std::vector<double>> node_vector,
    int type=0);
    std::vector<std::vector<double>> planning_series(int n);
};

#endif