#ifndef Artificial_potential_field_H
#define Artificial_potential_field_H
#include <iostream>
#include <Eigen/Dense>
#include <random>
#include<vector>
#include<cmath>
#include<algorithm>
#include "World_map.h"
class APF
{
    public:
    struct config
    {
        //初始化车的参数
        double road_width;  // 道路标准宽度
        double vehicle_width;  //  汽车宽度
        double vehicle_length;  // 车长
        double Eta_att;  // 引力的增益系数
        double Eta_rep_ob;  // 斥力的增益系数
        double Eta_rep_edge;   // 道路边界斥力的增益系数
        double d_max;  // 障碍影响的最大距离
        double len_step; // 步长
        double  n;//到目标点的距离的指数，启发项系数
        double dt;//时间不长
        config():
        road_width(3.5),
        vehicle_width(1.8),
        vehicle_length(4.7),
        Eta_att(2),
        Eta_rep_ob(1),
        Eta_rep_edge(10),
        d_max(5),
        len_step(0.5),
        n(1),
        dt(0.1){}
    };
    struct obstacle
    {
        Eigen::Vector2d position;
        double radius;
        obstacle(WorldMap::Obstacle obs){
            position(0)=obs.x;
            position(1)=obs.y;
            radius=sqrt(pow(obs.height/2,2)+pow(obs.width/2,2));
        }
    };
private:
    APF::config config_;

    Eigen::Vector2d target_pos;

    std::vector<obstacle> obstacles_;

    Eigen::Vector2d computeForce(Eigen::VectorXd robot_state,bool road);

    Eigen::VectorXd runAPF(Eigen::VectorXd robot_state,bool road);

    Eigen::Vector2d getrandm();//生成随机方向的单位向量

    Eigen::Vector2d Current_Move_Direction;

    bool areVectorsOpposite(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, double tolerance = 1e-6);
public:
    APF(WorldMap& map,const config& conf= config());

    void setTargetPos(const Eigen::Vector2d &targetPos);

    void setObstaclePos(const std::vector<obstacle> &obstaclePos);

    void setconfig(config& conf);

    std::vector<std::vector<double>> plan_series();

    void plot_planning(WorldMap* map,ElectricVehicleDynamicsModel* car);

};



#endif