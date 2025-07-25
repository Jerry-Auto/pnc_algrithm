#include "ReedsShepp.h"


int main(){
    vector<double>start{33.829,23.6706,-1.8357};
    vector<double>goal{53,95,-1.570796};
    double curvature = 0.2;//1/R
    double step_size = 0.5;
    ReedsShepp reedsShepp;

    Path path = reedsShepp.reedsSheppPathPlanning(start,goal,curvature,step_size);
    plt::plot(path.x,path.y,"r");
    plt::plot(vector<double>{start[0]},vector<double>{start[1]},"og");
    plt::plot(vector<double>{goal[0]},vector<double>{goal[1]},"xb");
    plt::title("mode: "+path.modes);

    const char* filename = "./reedsShepp_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    cout<<"mode: "<<path.modes<<endl;
    return 0;
}