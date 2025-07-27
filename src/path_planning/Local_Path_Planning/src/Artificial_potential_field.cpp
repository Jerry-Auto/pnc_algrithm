#include"Artificial_potential_field.h"

APF::APF(WorldMap& map,const config& conf):config_(conf)
{
    std::vector<WorldMap::Obstacle> obs=map.get_Obstacle();
    std::pair<double, double> goal_point=map.get_goal();
    target_pos[0]=goal_point.first;
    target_pos[1]=goal_point.second;
    for(size_t i=0;i<obs.size();i++)
    {
        obstacle obse(obs[i]);
        obstacles_.push_back(obse);
    }
}

void APF::setTargetPos(const Eigen::Vector2d &targetPos) {
    target_pos = targetPos;
}

void APF::setObstaclePos(const std::vector<obstacle> &obstaclePos) {
    obstacles_ = obstaclePos;
}

void APF::setconfig(config& conf)
{
    config_=conf;
}

/**
 * 计算引力斥力
 * @param robot_state 车辆状态：x,y,v
 * @return 单位合力方向
 */
Eigen::Vector2d APF::computeForce(Eigen::VectorXd robot_state,bool road) {
    // 引力势场计算
    Eigen::Vector2d delta_att = target_pos-robot_state.head(2);
    double dist_att = delta_att.norm();
    Eigen::Vector2d unite_att_vec = delta_att/dist_att;
    Eigen::Vector2d F_att = config_.Eta_att*delta_att;
    //合力
    Eigen::Vector2d F = F_att;
    //障碍物斥力势场
    //在斥力势场函数增加目标调节因子（即车辆至目标距离），以使车辆到达目标点后斥力也为0
    for(obstacle obse:obstacles_){
        Eigen::Vector2d obs=obse.position;
        double R=obse.radius;
        Eigen::Vector2d delta = robot_state.head(2)-obs;
        double dist = delta.norm();
        Eigen::Vector2d  unite_rep_vec = delta/dist;
        Eigen::Vector2d F_rep_ob;
        if(dist>=obse.radius+config_.d_max){
            F_rep_ob = Eigen::Vector2d (0,0);
        }else if(dist>=R){
            //障碍物的斥力1，方向由障碍物指向车辆
            //斥力1
            double  F_rep1_norm = config_.Eta_rep_ob*(1/pow((dist-R),3)/dist-1/pow((dist-R),2)/(config_.d_max-R)/dist)*pow(dist_att,config_.n);
            Eigen::Vector2d F_rep_ob1 = F_rep1_norm*unite_rep_vec;
            //斥力2
            double F_rep2_norm = config_.n/2*config_.Eta_rep_ob*pow(1/(dist-R)-1/(config_.d_max-R),2)*pow(dist_att,config_.n-2);
            Eigen::Vector2d F_rep_ob2 = F_rep2_norm*unite_att_vec;
            F_rep_ob = F_rep_ob1+F_rep_ob2;
        }
        else{
            double max=std::numeric_limits<double>::max();
            F_rep_ob = Eigen::Vector2d (max,max);
        }
        F += F_rep_ob;
    }
    if(road){
        //道路边界斥力势场
        Eigen::Vector2d F_rep_edge;
        double v = robot_state(2);//车辆速度
        if(robot_state(1)>-config_.road_width+config_.vehicle_width/2&&robot_state(1)<=-config_.road_width/2){
            F_rep_edge = Eigen::Vector2d (0,config_.Eta_rep_edge * v * exp(-config_.road_width / 2 - robot_state(1)));
        }else if(robot_state(1)>-config_.road_width/2&&robot_state(1)<=-config_.vehicle_width/2){
            F_rep_edge = Eigen::Vector2d (0,1/3*config_.Eta_rep_edge*pow(robot_state(1),2));
        }else if(robot_state(1)>config_.vehicle_width/2&&robot_state(1)<=config_.road_width/2){
            F_rep_edge = Eigen::Vector2d (0,-1/3*config_.Eta_rep_edge*pow(robot_state(1),2));
        }else if(robot_state(1)>config_.road_width/2&&robot_state(1)<=config_.road_width-config_.vehicle_width/2){
            F_rep_edge = Eigen::Vector2d (0,config_.Eta_rep_edge * v * exp( robot_state(1)-config_.road_width / 2 ));
        }
        F+=F_rep_edge;
    }
    Eigen::Vector2d unit_F = F/F.norm();
    return unit_F;
}

/**
 * 人工势场法控制器
 * @param robot_state
 * @return 车辆下一步位置
 */
Eigen::VectorXd APF::runAPF(Eigen::VectorXd robot_state,bool road) {
    Eigen::Vector2d unit_F = computeForce(robot_state,road);
    if(areVectorsOpposite(unit_F,Current_Move_Direction))
    {
        unit_F=getrandm();
    }
    Current_Move_Direction=unit_F;
    Eigen::Vector2d next_pos = robot_state.head(2)+config_.len_step * unit_F;
    robot_state <<next_pos(0),next_pos(1),robot_state(2);
    return robot_state;
}

std::vector<std::vector<double>> APF::plan_series()
{
    std::vector<double> t_data, x_data, y_data, theta_data;
    Eigen::VectorXd robot_state(3);
    robot_state<<0,0,2;
    Eigen::Vector2d dist_v=target_pos-robot_state.head(2);
    double t=0;
    double dist=dist_v.norm();
    while(true)
    {
        t_data.push_back(t);
        x_data.push_back(robot_state(0));
        y_data.push_back(robot_state(1));
        theta_data.push_back(0);
        std::cout<<"距离："<<dist<<std::endl;
        if(dist<config_.len_step)
        {
            std::cout<<"到达终点！"<<std::endl;
            break;
        }        
        robot_state=runAPF(robot_state,false);
        t+=config_.dt;
        dist=(target_pos-robot_state.head(2)).norm();
    }
    return {t_data,x_data,y_data,theta_data};
}

void APF::plot_planning(WorldMap* map,ElectricVehicleDynamicsModel* car)
{
    int k=3,i=0;
    Eigen::VectorXd robot_state(3);
    robot_state<<0,0,2;
    Eigen::Vector2d dist_v=target_pos-robot_state.head(2);
    double t=0;
    double dist=dist_v.norm();
    while(true)
    {
        std::tuple<double,double,double> state={robot_state[0],robot_state[1],robot_state[2]};
        car->reset_for_planning_only(state);
        if (i % k == 0) {
            // 重新可视化（非阻塞模式）
            map->visualize(true,true,false);
        }
        std::cout<<"距离："<<dist<<std::endl;
        if(dist<config_.len_step)
        {
            std::cout<<"到达终点！"<<std::endl;
            break;
        }        
        robot_state=runAPF(robot_state,false);
        t+=config_.dt;
        i++;
        dist=(target_pos-robot_state.head(2)).norm();
    }
    map->visualize(true,true,true);
}

Eigen::Vector2d APF::getrandm()//生成随机方向的单位向量
{
    // 1. 初始化随机数生成器（C++11 标准）
    static std::random_device rd;  // 随机设备
    static std::mt19937 gen(rd()); // Mersenne Twister 引擎
    static std::uniform_real_distribution<double> dist(0.0, 2.0 * M_PI); // [0, 2π] 均匀分布
    // 2. 生成随机角度 θ
    double theta = dist(gen);
    // 3. 计算单位向量 (cosθ, sinθ)
    Eigen::Vector2d unit_vector;
    unit_vector << std::cos(theta), std::sin(theta);
    return unit_vector;
}

bool APF::areVectorsOpposite(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, double tolerance) 
{
    if (v1.norm() == 0 || v2.norm() == 0) {
        return false; // 零向量无方向，视为不共线
    }
    // 归一化向量
    Eigen::Vector2d u1 = v1.normalized();
    Eigen::Vector2d u2 = v2.normalized();
    // 计算点积
    double dot_product = u1.dot(u2);
    // 判断是否接近 -1
    return dot_product <= -1.0 + tolerance;
}