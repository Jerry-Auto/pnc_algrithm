/**
 * @file rs_curve.hpp
 * @author Yongmeng He
 * @brief 
 * @version 0.1
 * @date 2023-12-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef RS_CURVE_H
#define RS_CURVE_H

#include <cmath>
#include <limits>
#include <vector>
#include <array>
#include <utility> 
#include <string>

enum RSCurveType {
    L = 1, S = 2, R = 3, N = 0
}; // left, stright, right, none
using point_type = std::pair<std::pair<double, double>, double>;
// <<type, name>, <{t, u, v, w, x}, length>>
using curve_type = std::pair<std::pair<const RSCurveType*, std::string>, 
        std::pair<std::array<double, 5>, double>>;
//这里const RSCurveType*指向一个数组的首地址，所以这里curve_type.first.first是一个可以索引的数组，
//里面每个元素都是一个枚举值，每个枚举值代表一种曲线类型，左右转或者直行
//RS曲线最多五段，curve_type.first.first：：{L,S,L,R,N,N},对应curve_type.second.first::<{t, u, v, w, x}，每一段长度
class RSCurve {
    public:
        /**
         * @brief Construct a new RSCurve object
         * 
         * @param radius 转弯半径
         * @param start_pos 
         * @param end_pos 
         */
        RSCurve(double radius, const point_type& start_pos, 
                const point_type& end_pos) : radius_(radius), 
                start_pos_(start_pos), end_pos_(end_pos) {};
        RSCurve(){};
        ~RSCurve() = default;

        /**
         * @brief 获取RS全部曲线的前置点处理函数
         * 
         * @param start_pos 
         * @param end_pos 
         * @return std::vector<curve_type> RS全部曲线
         */
        std::vector<curve_type> GetRSCurve(double radius,
                const point_type& start_pos, const point_type& end_pos);
        
        curve_type GetBestRSCurve(double radius,
        const point_type& start_pos, const point_type& end_pos);
        /**
         * @brief 获取RS所有曲线的主函数
         * 
         * @param input_pos 
         * @return std::vector<curve_type> RS全部曲线
         */
        std::vector<curve_type> GetRSCurve(const point_type& input_pos);
        /**
         * @brief 将RS曲线插值采样，获取特定间隔的点以获取完整路径
         * 
         * @param rs_curve 
         * @param step_size 采样间隔
         * @return std::vector<point_type> 包含了所有点的完整RS曲线
         */
        std::vector<point_type> GetRSPoint(const curve_type& rs_curve,double step_size);
        /**
         * @brief 工具函数，用于精简代码
         * 
         * @param type RS曲线类型
         * @param name RS曲线类型名
         * @param angle_array RS曲线结果
         * @param length RS曲线长度
         * @return curve_type 
         */
        curve_type CreateCurve(
                const RSCurveType* type, const std::string& name, 
                const std::array<double, 5>& angle_array, const double& length);
        /**
         * @brief 工具函数，用于精简代码
         * 
         * @param type RS曲线类型
         * @param name RS曲线类型名
         * @return curve_type 
         */
        curve_type CreateNoneCurve(
                const RSCurveType* type, const std::string& name);
        /**
         * @brief 计算两点的欧式距离
         * 
         * @param pos1 
         * @param pos2 
         * @return double 
         */
        double GetDistance(const point_type& pos1, const point_type& pos2);
        /**
         * @brief 获取起点为pos1，终点为pos2的向量
         * 
         * @param pos1 
         * @param pos2 
         * @return point_type 
         */
        point_type GetDiffVecPoint(
                const point_type& pos1, const point_type& pos2);

    private:
        inline double Mod2Pi(const double& theta);
        inline void Polar(
                const double& x, const double& y, double& r, double& theta);
        inline void TauOmega(double u, double v, double xi, double eta, 
                double phi, double &tau, double &omega);
        inline bool LpSpLp(const double& x, const double& y, const double& phi, 
                double& t, double& u, double& v, const std::string& type);
        inline bool LpSpRp(const double& x, const double& y, const double& phi, 
                double& t, double& u, double& v, const std::string& type);
        inline bool LpRmL(const double& x, const double& y, const double& phi, 
                double& t, double& u, double& v, const std::string& type);
        inline bool LpRupLumRm(const double& x, const double& y, 
                const double& phi, double& t, double& u, double& v, 
                const std::string& type);
        inline bool LpRupLumRp(const double& x, const double& y, 
                const double& phi, double& t, double& u, double& v, 
                const std::string& type);
        inline bool LpRmSmLm(const double& x, const double& y, 
                const double& phi, double& t, double& u, double& v, 
                const std::string& type);
        inline bool LpRmSmRm(const double& x, const double& y, 
                const double& phi, double& t, double& u, double& v, 
                const std::string& type);
        inline bool LpRmSLmRp(const double& x, const double& y, 
                const double& phi, double& t, double& u, double& v, 
                const std::string& type);
        std::vector<curve_type> CSC(
                const double& x, const double& y, const double& phi);
        std::vector<curve_type> CCC(
                const double& x, const double& y, const double& phi);
        std::vector<curve_type> CCCC(
                const double& x, const double& y, const double& phi);
        std::vector<curve_type> CCSC(
                const double& x, const double& y, const double& phi);
        std::vector<curve_type> CCSCC(
                const double& x, const double& y, const double& phi);

    private:
        double radius_;
        const RSCurveType rs_curve_type_[18][5]{
            {L, R, L, N, N},    // 0
            {R, L, R, N, N},    // 1
            {L, R, L, R, N},    // 2
            {R, L, R, L, N},    // 3
            {L, R, S, L, N},    // 4
            {R, L, S, R, N},    // 5
            {L, S, R, L, N},    // 6
            {R, S, L, R, N},    // 7
            {L, R, S, R, N},    // 8
            {R, L, S, L, N},    // 9
            {R, S, R, L, N},    // 10
            {L, S, L, R, N},    // 11
            {L, S, R, N, N},    // 12
            {R, S, L, N, N},    // 13
            {L, S, L, N, N},    // 14
            {R, S, R, N, N},    // 15
            {L, R, S, L, R},    // 16
            {R, L, S, R, L}     // 17
        };

    public:
        point_type start_pos_;
        point_type end_pos_;
};


#endif 