#include "AntColony.h"




#define err 1e-4

AntColony::AntColony(ObstacleGridMap* grid_map,int num_i,double alp,double bet,double rho,double q)
:grid_map_(grid_map),num_of_iterations(num_i),Alpha(alp),Beta(bet),Rho(rho),Q(Q)
{
    this->num_of_ant=1.5*free_pnt_num(grid_map_-);
}

int AntColony::free_pnt_num(ObstacleGridMap* map)
{
    std::vector<float> gridmap=map->getGridData();
    int n=0;
    for(size_t i=0;i<gridmap.size();i++)
    {
        if(gridmap[i]<0.8)
        {n++;}
    }
    return n;
}

void AntColony::set_ant_num(int n)
{
    this->num_of_ant=n;
}

double AntColony::getrandm(double a,double b)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}
double AntColony::calcu_dis(int x1, int y1, int x2, int y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}