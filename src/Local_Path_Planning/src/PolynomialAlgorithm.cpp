#include "PolynomialAlgorithm.h"

PolynomialAlgorithm::PolynomialAlgorithm(std::pair<std::vector<double> ,std::vector<double>> str_state,std::pair<std::vector<double>,std::vector<double>> end_state,double t0,double t1):t0(t0),t1(t1)
{
    Eigen::VectorXd X(6),Y(6);
    X<<str_state.first[0],str_state.first[1],str_state.first[2],end_state.first[0],end_state.first[1],end_state.first[2];
    Y<<str_state.second[0],str_state.second[1],str_state.second[2],end_state.second[0],end_state.second[1],end_state.second[2];
    Eigen::MatrixXd TX=CalculT(this->t0,this->t1);
    this->A = TX.inverse()*X;
    double x0=this->A.dot(Calculpoly(this->t0));
    double xt=this->A.dot(Calculpoly(this->t1));
    Eigen::MatrixXd TY=CalculT(x0,xt);
    this->B = TY.inverse()*Y;
}

Eigen::MatrixXd PolynomialAlgorithm::CalculT(double state_0,double state_t)
{
    Eigen::MatrixXd T(6,6);
    T<<pow(state_0, 5),pow(state_0, 4),pow(state_0, 3),pow(state_0, 2),state_0,1,
        5*pow(state_0,4),4*pow(state_0,3),3*pow(state_0,2),2*state_0,1,0,
        20*pow(state_0,3),12*pow(state_0,3),6*state_0,1,0,0,
        pow(state_t, 5),pow(state_t, 4),pow(state_t, 3),pow(state_t, 2),state_t,1,
        5*pow(state_t,4),4*pow(state_t,3),3*pow(state_t,2),2*state_t,1,0,
        20*pow(state_t,3),12*pow(state_t,3),6*state_t,1,0,0;
    return T;
}

Eigen::VectorXd PolynomialAlgorithm::Calculpoly(double variable)
{
    Eigen::VectorXd P(6);
    P<<pow(variable, 5),pow(variable, 4),pow(variable, 3),pow(variable, 2),variable,1;
    return P;
}
/// @brief 输出给定时刻的规划点的x,y,theta,kappa,vp,ap
/// @param t 
/// @return 
std::vector<double> PolynomialAlgorithm::GetState(double t)
{
    Eigen::VectorXd poly_t=Calculpoly(t);
    double X_State=this->A.dot(poly_t);
    Eigen::VectorXd poly_x=Calculpoly(X_State);
    double Y_State=this->B.dot(poly_x);
    
    double dX_State=diff_factor(A).dot(poly_t);
    double dY_dx_State=diff_factor(B).dot(poly_x);
    double dY_dt_State=dY_dx_State*dX_State;
    double ddX_State=diff_factor(diff_factor(A)).dot(poly_t);
    double ddY_dx_State=diff_factor(diff_factor(B)).dot(poly_x);
    double ddY_dt_State=ddY_dx_State*pow(dX_State,2)+dY_dx_State*ddX_State;

    std::vector<double> x_state={X_State,dX_State,ddX_State};
    std::vector<double> y_state={Y_State,dY_dt_State,ddY_dt_State};
    double theta=std::atan(dY_dx_State);
    double kappa=ddY_dx_State/pow(1+pow(dY_dx_State,2),1.5);
    double vp=sqrt(pow(dY_dt_State,2)+pow(dX_State,2));
    double ap=sqrt(pow(ddX_State,2)+pow(ddY_dt_State,2));
    if(dX_State<=0)
    {ap=-ap;}
    std::vector<double> plan={X_State,Y_State,theta,kappa,vp,ap};
    return plan;
}

/// @brief 对多项式系数进行求导，最终用返回值乘以多项式基底向量即为导数值（x=A*poly(t),dx/dt=D*A*poly(t)）
/// @param origin_factor 
/// @return 
Eigen::VectorXd PolynomialAlgorithm::diff_factor(Eigen::VectorXd origin_factor)
{
    Eigen::MatrixXd D(6,6);
    D<<0,1,0,0,0,0,
    0,0,2,0,0,0,
    0,0,0,3,0,0,
    0,0,0,0,4,0,
    0,0,0,0,0,5,
    0,0,0,0,0,0;
    Eigen::VectorXd out=D*origin_factor;
    return out;
}
/// @brief 输出规划信息序列，包含时刻，x,y,theta,kappa,vp,ap
/// @param dt 采样步长
/// @return 
std::vector<std::vector<double>> PolynomialAlgorithm::planning_series(double dt)
{
    std::vector<double>x_,y_,v_p,a_p,theta,kappa;//用于保存数据，横纵向位置、横纵向速度
    std::vector<double>time;
    int cnt = 0;
    for(double t=t0;t<t1+dt;t+=dt) 
    {
        cnt++;
        time.push_back(t);
    }
    for(int i=0;i<cnt;i++)
    {
        std::vector<double> state_t=GetState(time[i]);
        x_.push_back(state_t[0]);
        y_.push_back(state_t[1]);
        theta.push_back(state_t[2]);
        kappa.push_back(state_t[3]);
        a_p.push_back(state_t[4]);
        v_p.push_back(state_t[5]);
    }
    return {time,x_,y_,theta,kappa,v_p,a_p};
}

void PolynomialAlgorithm::plotpositioncurve(std::vector<std::vector<double>> plan_data)
{
    plt::figure(1);
    plt::title("XOY coordinate");
    plt::xlabel("World X (m)");
    plt::ylabel("World Y (m)");
    plt::xlim(0, 200);
    plt::ylim(0, 200);
    plt::grid(true);
    plt::plot(plan_data[1], plan_data[2],"r");

    plt::figure(6);
    plt::title("XOt coordinate");
    plt::xlabel("time t (s)");
    plt::ylabel("World X (m)");
    plt::grid(true);
    plt::plot(plan_data[0], plan_data[1],"r");

    plt::figure(7);
    plt::title("YOt coordinate");
    plt::xlabel("time t (s)");
    plt::ylabel("World Y (m)");
    plt::grid(true);
    plt::plot(plan_data[0], plan_data[2],"r");

    plt::show();
}

void PolynomialAlgorithm::plotthetacurve(std::vector<std::vector<double>> plan_data)
{
        // theta
    plt::figure(2);
    plt::title("theta_t curve");
    plt::ylabel("theta (rad)");
    plt::xlabel("time (s)");
    plt::grid(true);
    plt::plot(plan_data[0],plan_data[3]);
    plt::show();

}
void PolynomialAlgorithm::plotkappacurve(std::vector<std::vector<double>> plan_data)
{
        // kappa
    plt::figure(3);
    plt::title("kappa_t curve");
    plt::ylabel("kappa (1/m)");
    plt::xlabel("time (s)");
    plt::grid(true);
    plt::plot(plan_data[0],plan_data[4]);
    plt::show();
}
void PolynomialAlgorithm::plotvpcurve(std::vector<std::vector<double>> plan_data)
{
        // vp
    plt::figure(4);
    plt::title("vp_t curve");
    plt::ylabel("vp (m/s)");
    plt::xlabel("time (s)");
    plt::grid(true);
    plt::plot(plan_data[0],plan_data[5]);
    plt::show();
}
void PolynomialAlgorithm::plotapcurve(std::vector<std::vector<double>> plan_data)
{
        // ap
    plt::figure(5);
    plt::title("ap_t curve");
    plt::ylabel("ap (m/s^2)");
    plt::xlabel("time (s)");
    plt::grid(true);
    plt::plot(plan_data[0],plan_data[6]);
    plt::show();
}
void PolynomialAlgorithm::plotallcurve(std::vector<std::vector<double>> plan_data)
{
    plotpositioncurve(plan_data);
    plotthetacurve(plan_data);
    plotkappacurve(plan_data);
    plotvpcurve(plan_data);
    plotapcurve(plan_data);
}
