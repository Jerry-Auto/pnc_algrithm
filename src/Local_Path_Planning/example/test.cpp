#include<iostream>
#include<Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
#include <map>  // 必须包含此头文件
#include <string>

#ifndef DWA_H
#define DWA_H

struct DWA_Parameter
{
    std::map<std::string,std::pair<double,double>> Parameter_range;
    std::pair<double,double> sample_resolution;
    double dt;//采样时间
    double predict_time;//轨迹推算时间长度
    double alpha;
    double beta;
    double gamma; //轨迹评价函数系数
    double radius; // 用于判断是否到达目标点
    double judge_distance; //若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值
    DWA_Parameter(std::map<std::string,std::pair<double,double>> Parameter_range,
    std::pair<double,double> sample_resolution,
    double dt,
    double predict_time,
    double alpha,
    double beta,
    double gamma,
    double radius,
    double judge_distance);
};
class DWA
{
private:
    DWA_Parameter Parameter_;
    

public:
    DWA();
};

#endif

DWA_Parameter::DWA_Parameter(std::map<std::string,std::pair<double,double>> Parameter_range,
    std::pair<double,double> sample_resolution,
    double dt,
    double predict_time,
    double alpha,
    double beta,
    double gamma,
    double radius,
    double judge_distance):
    Parameter_range(Parameter_range),sample_resolution(sample_resolution),
    dt(dt),predict_time(predict_time),alpha(alpha),
    beta(beta),gamma(gamma),radius(radius),judge_distance(judge_distance){}


int main()
{



    return 0;
}