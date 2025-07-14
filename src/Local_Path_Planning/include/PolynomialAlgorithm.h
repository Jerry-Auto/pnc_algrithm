#ifndef POLYNOMIALALGORITHM_H
#define POLYNOMIALALGORITHM_H
#include <iostream>
#include<Eigen/Dense>
#include<vector>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#include<iostream>

class PolynomialAlgorithm
{
private:
    double t0,t1;

    Eigen::VectorXd A,B;

    Eigen::MatrixXd CalculT(double state_0,double state_t);

    Eigen::VectorXd Calculpoly(double variable);

    Eigen::VectorXd diff_factor(Eigen::VectorXd origin_factor);

public:

    PolynomialAlgorithm(std::pair<std::vector<double> ,std::vector<double>> str_state,std::pair<std::vector<double>,std::vector<double>> end_state,double t0,double t1);
    
    std::vector<double> GetState(double t);

    std::vector<std::vector<double>> planning_series(double dt);

    void plotpositioncurve(std::vector<std::vector<double>> plan_data);
    void plotthetacurve(std::vector<std::vector<double>> plan_data);
    void plotkappacurve(std::vector<std::vector<double>> plan_data);
    void plotvpcurve(std::vector<std::vector<double>> plan_data);
    void plotapcurve(std::vector<std::vector<double>> plan_data);
    void plotallcurve(std::vector<std::vector<double>> plan_data);

};




#endif