#include "H_A_S.h"

int main()
{
            //创建地图并设置障碍物
    WorldMap world_map(0, 100, 0, 100);
    //边界，中心，宽，高，转动角度
    world_map.addObstacle({1, 50, 2, 100, 0, "red"});
    world_map.addObstacle({99, 50, 2, 100, 0, "red"});
    world_map.addObstacle({50, 1, 96, 2, 0, "red"});
    world_map.addObstacle({50, 99, 96, 2, 0, "red"});  

    world_map.addObstacle({42, 30, 80, 10, 0, "red"});
    world_map.addObstacle({58, 60, 80, 10, 0, "red"});
    world_map.addObstacle({50, 95, 2, 6, 0, "red"});
    world_map.addObstacle({56, 95, 2, 6, 0, "red"});
    world_map.addObstacle({66, 95, 2, 6, 0, "red"});  
    world_map.addObstacle({90, 96.5, 2, 3, 0, "red"});    

    //设置目标点
    Pos3d str_pos(12,15,M_PI/6);
    //倒车入库
    //Pos3d end_pos(53,96,-M_PI/2);
    //侧方位停车
    //Pos3d end_pos(63,96,-M_PI);
    //侧方位停车1
    Pos3d end_pos(96,96,-M_PI);

    world_map.set_goal_pos(end_pos);
    world_map.set_start_pos(str_pos);
    // world_map.visualize();

    // ObstacleGridMap grid_map(world_map,2,1.0);
    // grid_map.plotWorldMap();
    H_A_Star HAS_solver;
    auto planning_data=HAS_solver.planning(world_map);
    if(planning_data.size()!=0)
    {
        ElectricVehicleDynamicsModel vehicle;
        world_map.addVehicle(&vehicle,"green");
        world_map.plot_planning(&vehicle,planning_data,"hy_a_s.png");
    }

    return 0;
    
}
