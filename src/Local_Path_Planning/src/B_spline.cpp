#include"B_spline.h"

    B_spline::B_spline(
    std::optional<std::vector<Eigen::Vector2d>> control_point,
    std::optional<int> k,
    std::optional<std::vector<double>> node_vector,
    int type)
    {
            // 检查输入有效性（必须恰好有两个已知参数）
        if(!control_point.has_value())
        {throw std::invalid_argument("必须输入控制点");}
        int knownCount = 0;
        knownCount += (node_vector.has_value() ? 1 : 0);
        knownCount += (k.has_value() ? 1 : 0);
        if (knownCount != 1) {
            throw std::invalid_argument("输入曲线阶数或节点向量");
        }
        node_vect_type=type;

        if(!node_vector.has_value())
        {
            this->control_point=control_point.value();
            this->curve_order=k.value();
            calcu_node_vect();
        }
        else if(!k.has_value())
        {
            this->control_point=control_point.value();
            this->node_vector_=node_vector.value();
            this->curve_order=node_vector_.size()-1-this->control_point.size();
        }
        this->node_vect_type=type;
    }

    void B_spline::calcu_node_vect()
    {

        int n=this->control_point.size();
        int k=this->curve_order;
        int m=n+k+1;//节点个数m=n+k+1
        switch (this->node_vect_type)
        {
        case 0://clamped B样条 准均匀B样条
            for(int i=0;i<m;i++){
                if(i<=k)
                {
                    node_vector_.push_back(0.0);//开头相同为0
                }
                else if(i>=n)
                {
                    node_vector_.push_back(this->factor);//结尾相同
                }
                else{
                    node_vector_.push_back(factor*(double)(i-k)/(n-k));//中间均匀分布
                }
            }
            break;
        case 1://均匀B样条
            for(int i=0;i<m;i++){
                node_vector_.push_back(factor*(double)i/(m-1));
            }
            break;
        
        case 2://分段B样条
            if((n-1)%k==0&&k>0){//满足控制点数-1是曲线次数的整数倍且曲线次数为正整数
                for(int i=0;i<m;i++)
                {
                    if(i<=k)
                    {
                        node_vector_.push_back(0.0);
                    }
                    else if(i>=n)
                    {
                        node_vector_.push_back(factor);
                    }
                    else
                    {
                        int a=1+(i-k-1)/k;
                        int b=(n-1)/k;
                        node_vector_.push_back((double)(a)/b);
                    }
                }
            }else{
                std::cout<<"error!需要满足n是k-1的整数倍且k-1为正整数"<<std::endl;
            }
            break;
        }
    }



    /// @brief 计算基函数在u点的值
    /// @param i 第i个控制点对应的基函数
    /// @param k K次的基函数
    /// @param u 输入值
    /// @return 
    double B_spline::calcul_base(int i,int k,double u)
    {
        double den1=node_vector_[i+k]-node_vector_[i];
        double den2=node_vector_[i+k+1]-node_vector_[i+1];
        double num1=u-node_vector_[i];
        double num2=node_vector_[i+k+1]-u;
        if(k==0)
        {
            if(u<=node_vector_[i+1]&&u>=node_vector_[i]){return 1;}
            else{return 0;}
        }
        else if(k>0){
            double Bik=0;
            double k1=0;
            double k2=0;
            if(den1==0)
            {
                if(num1==0)
                {
                    k1=0;
                }
                else{k1=num1;}
            }
            else{k1=num1/den1;}

            if(den2==0)
            {
                if(num2==0)
                {
                    k2=0;
                }
                else{k2=num2;}
            }
            else{k2=num2/den2;}

            Bik=calcul_base(i,k-1,u)*k1+
            calcul_base(i+1,k-1,u)*k2;
            return Bik;
        }
        else{
            std::cout<<"出错,k<0"<<std::endl;
            return 0;
        }
    }



    Eigen::Vector2d B_spline::calcul_B_U(double u)//求样条曲线在u点的取值
    {
        Eigen::Vector2d B_u(0,0);
        double Bk;
        for(int i=0;i<control_point.size();i++)
        {
            Bk=calcul_base(i,curve_order,u);//计算基底
            B_u+=Bk*control_point[i];//线性组合
        }
        return B_u;
    }



    /**
 * @brief 计算离散点集的切线角度和曲率
 * @param points 输入的离散点集
 * @return 包含切线角度和曲率的pair:
 *         first - 切线角度向量(弧度)
 *         second - 曲率向量
 */
std::pair<std::vector<double>, std::vector<double>> 
B_spline::calculateAngleAndCurvature(const std::vector<Eigen::Vector2d>& points) 
{
    std::vector<double> angles;
    std::vector<double> curvatures;
    
    const size_t n = points.size();
    if(n < 3) {
        return {angles, curvatures}; // 返回空结果
    }
    
    // 预分配空间
    angles.reserve(n);
    curvatures.reserve(n);
    
    // 处理第一个点 (前向差分)
    {
        Eigen::Vector2d d = points[1] - points[0];
        angles.push_back(atan2(d.y(), d.x()));
        curvatures.push_back(0.0); // 起始点曲率设为0
    }
    
    // 中间点 (中心差分)
    for(size_t i = 1; i < n-1; ++i) {
        // 切线角度计算
        Eigen::Vector2d d1 = points[i] - points[i-1];
        Eigen::Vector2d d2 = points[i+1] - points[i];
        Eigen::Vector2d tangent = d1 + d2; // 两侧差分平均
        
        angles.push_back(atan2(tangent.y(), tangent.x()));
        
        // 曲率计算 (基于三点圆拟合)
        const Eigen::Vector2d& p0 = points[i-1];
        const Eigen::Vector2d& p1 = points[i];
        const Eigen::Vector2d& p2 = points[i+1];
        
        double dx1 = p1.x() - p0.x();
        double dy1 = p1.y() - p0.y();
        double dx2 = p2.x() - p1.x();
        double dy2 = p2.y() - p1.y();
        
        double cross = dx1*dy2 - dy1*dx2;
        double dot = dx1*dx2 + dy1*dy2;
        
        double ds1 = sqrt(dx1*dx1 + dy1*dy1);
        double ds2 = sqrt(dx2*dx2 + dy2*dy2);
        
        if(fabs(cross) > 1e-6 && ds1 > 1e-6 && ds2 > 1e-6) {
            double curvature = 2 * cross / (ds1 * ds2 * (ds1 + ds2));
            curvatures.push_back(curvature);
        } else {
            curvatures.push_back(0.0);
        }
    }
    
    // 处理最后一个点 (后向差分)
    {
        Eigen::Vector2d d = points[n-1] - points[n-2];
        angles.push_back(atan2(d.y(), d.x()));
        curvatures.push_back(0.0); // 结束点曲率设为0
    }
    
    return {angles, curvatures};
}

    std::vector<std::vector<double>> B_spline::planning_series(int n)
    {
        std::vector<std::vector<double>> curve_data;
        std::vector<double> t_data,x_data,y_data,theta_data,v_p,a_p,kappa;

        double u_str=node_vector_[curve_order];
        double u_end=node_vector_[control_point.size()];
        double dt=(u_end-u_str)/n;
        // 采样点
        std::vector<Eigen::Vector2d> points;

        for(double u=u_str;u<=u_end;u+=dt)
        {
            Eigen::Vector2d B_u=calcul_B_U(u);
            t_data.push_back(u);
            x_data.push_back(B_u(0));
            y_data.push_back(B_u(1));
            points.push_back(B_u);
        }
        // 计算角度和曲率
        auto [angles, curvatures] = calculateAngleAndCurvature(points);
        theta_data = std::move(angles);
        kappa = std::move(curvatures);

        return {t_data,x_data,y_data,theta_data,kappa};
    }


    // Eigen::VectorXd B_spline::polyfit(const Eigen::VectorXd& x, const Eigen::VectorXd& y, int degree) {
    //     assert(x.size() == y.size());
    //     assert(degree >= 1 && degree <= x.size() - 1);

    //     Eigen::MatrixXd A(x.size(), degree + 1);
    //     for (int i = 0; i < x.size(); i++) {
    //         for (int j = 0; j <= degree; j++) {
    //             A(i, j) = pow(x(i), j);
    //         }
    //     }
    //     // 使用最小二乘法求解
    //     return A.householderQr().solve(y);
    // }

    // std::vector<Eigen::VectorXd> B_spline::getSegmentYXPolynomials() {
    //     int poly_degree = curve_order;
    //     std::vector<Eigen::VectorXd> all_coeffs;
    //     int degree = curve_order;
        
    //     // 获取所有有效节点区间
    //     std::vector<std::pair<double, double>> intervals;
    //     for (int i = degree; i < node_vector_.size() - degree - 1; i++) {
    //         if (node_vector_[i] != node_vector_[i+1]) { // 跳过重复节点
    //             intervals.emplace_back(node_vector_[i], node_vector_[i+1]);
    //         }
    //     }
        
    //     // 对每个区间计算y关于x的多项式
    //     for (const auto& interval : intervals) {
    //         double u_start = interval.first;
    //         double u_end = interval.second;
            
    //         // 采样区间内的点用于拟合
    //         const int sample_points = std::max(20, 2*(poly_degree+1)); // 足够多的采样点
    //         Eigen::VectorXd x_vals(sample_points);
    //         Eigen::VectorXd y_vals(sample_points);
            
    //         for (int i = 0; i < sample_points; i++) {
    //             double u = u_start + (u_end - u_start) * i / (sample_points - 1);
    //             Eigen::Vector2d point = calcul_B_U(u);
    //             x_vals(i) = point(0);
    //             y_vals(i) = point(1);
    //         }

    //         // 使用类内polyfit方法进行拟合
    //         Eigen::VectorXd coeffs = polyfit(x_vals, y_vals, poly_degree);
    //         all_coeffs.push_back(coeffs);
    //     }
        
    //     return all_coeffs;
    // }


            // std::vector<std::vector<double>> B_spline::planning_series(int n)
    // {
    //     std::vector<std::vector<double>> curve_data;
    //     std::vector<double> t_data,x_data,y_data,theta_data,v_p,a_p,kappa;

    //     double u_str=node_vector_[curve_order];
    //     double u_end=node_vector_[control_point.size()];
    //     double dt=(u_end-u_str)/n;
        
    //     for(double u=u_str;u<=u_end;u+=dt)
    //     {
    //         Eigen::Vector2d B_u=calcul_B_U(u);
    //         Eigen::Vector2d B_u_dot=calcul_B_l_u(1,u);//一阶导
    //         Eigen::Vector2d B_u_dot2=calcul_B_l_u(2,u);//二阶导
    //         t_data.push_back(u);
    //         x_data.push_back(B_u(0));
    //         y_data.push_back(B_u(1));
    //         // double dy_dx=B_u_dot(1)/B_u_dot(0);
    //         // double d2y_dx2=(B_u_dot2(1)*B_u_dot(0)-B_u_dot2(0)*B_u_dot(1))/pow(B_u_dot(0),3);
    //         theta_data.push_back(std::atan2(B_u_dot(1),B_u_dot(0)));
    //         // kappa.push_back(d2y_dx2/pow(1+pow(dy_dx,2),1.5));
    //     }

    //     return {t_data,x_data,y_data,theta_data,kappa};
    // }
    
    // //求样条曲线在u点的l阶导数
    // Eigen::Vector2d B_spline::calcul_B_l_u(int l, double u)
    // {
    //     if (l == 0)
    //     {
    //         return calcul_B_U(u);
    //     }
        
    //     int n = control_point.size() - 1;
    //     int k = curve_order;
        
    //     // 如果导数阶数大于曲线阶数，则导数为零
    //     if (l > k)
    //     {
    //         return Eigen::Vector2d(0, 0);
    //     }
        
    //     // 计算递推控制点
    //     std::vector<Eigen::Vector2d> temp_points = control_point;
    //     for (int r = 1; r <= l; r++)
    //     {
    //         for (int i = 0; i <= n - r; i++)
    //         {
    //             double denom = node_vector_[i + k + 1] - node_vector_[i + r];
    //             if (denom != 0)
    //             {
    //                 temp_points[i] = (k - r + 1) / denom * (temp_points[i + 1] - temp_points[i]);
    //             }
    //             else
    //             {
    //                 temp_points[i] = Eigen::Vector2d(0, 0);
    //             }
    //         }
    //     }
    //     // 计算导数
    //     Eigen::Vector2d result(0, 0);
    //     for (int i = 0; i <= n - l; i++)
    //     {
    //         result += calcul_base(i, k - l, u) * temp_points[i];
    //     }
        
    //     return result;
    // }