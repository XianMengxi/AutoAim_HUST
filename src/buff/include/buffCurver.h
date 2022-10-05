#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "ceres/ceres.h"

namespace ly
{
#define CURVE_FIT_SIZE 90
    struct SPEED_FITTING_COST
    {
        SPEED_FITTING_COST(double speed, double t) : _speed(speed), _t(t) {}
        // 残差的计算
        template <typename T>
        bool operator()(
            const T *const params, // 模型参数，有3维
            T *residual) const     // 残差
        {
            residual[0] = _speed - (params[0] * ceres::sin(params[1] * _t + params[2]) + 2.090 - params[0]);
            return true;
        }
        const double _speed, _t; // x,y,z数据
    };
    struct BuffTrajectory
    {
        // point = center + r * x_axis * cos(theta) + r * y_axis * sin(theta)
        float radius;
        Eigen::Vector3d center;
        Eigen::Vector3d x_axis;
        Eigen::Vector3d y_axis;
    };

    struct CICLE_FITTING_COST
    {
        CICLE_FITTING_COST(double x, double y, double z) : _x(x), _y(y), _z(z) {}
        // 残差的计算
        template <typename T>
        bool operator()(
            const T *const x0y0r, // 模型参数，有3维
            T *residual) const    // 残差
        {
            //(x-x0)^2+(y-y0)^2-r^2
            residual[0] = x0y0r[3] - (T(_x) - x0y0r[0]) * (T(_x) - x0y0r[0]) - (T(_y) - x0y0r[1]) * (T(_y) - x0y0r[1]) - (T(_z) - x0y0r[2]);
            return true;
        }
        const double _x, _y, _z; // x,y,z数据
    };
    struct COS_FITTING_COST
    {
        COS_FITTING_COST(double angle, double t) : _angle(angle), _t(t) {}
        // 残差的计算
        template <typename T>
        bool operator()(
            const T *const _theta_bias, // 模型参数，有3维
            T *residual) const          // 残差
        {
            T angle_diff = -0.785 / 1.884 * cos(1.884 * T(_t) + _theta_bias[0]) + 1.305 * T(_t) + _theta_bias[1] - T(_angle);
            residual[0] = angle_diff * angle_diff;
            return true;
        }
        const double _angle, _t; // x,y,z数据
    };
    struct PLANE_FITTING_COST
    {
        //AX+BY+CZ+1=0
        PLANE_FITTING_COST(double x, double y, double z) : _x(x), _y(y), _z(z) {}
        // 残差的计算
        template <typename T>
        bool operator()(
            const T *const abc_, // 模型参数，有3维
            T *residual) const   // 残差
        {
            //AX+BY+CZ+1=0
            residual[0] = abc_[0] * T(_x) + abc_[1] * T(_y) + abc_[2] * T(_z) + T(1.0);
            return true;
        }
        const double _x, _y, _z; // x,y,z数据
    };
    struct SineFitFunction
    {
        //传进来角度
        SineFitFunction(double t, double angle) : _t(t), _angle(angle) {}
        // 残差的计算
        template <typename T>
        bool operator()(
            const T *const params, // 模型参数，有4个,a,w,theta ,其中b=2.090-a,可舍去
            T *residual) const     // 残差一维
        {
            residual[0] = _angle - (-params[0] / params[1] * ceres::cos(params[1] * _t + params[2]) + (2.090 - params[0]) * _t);
            return true;
        }
        const double _t, _angle; // t，angle
    };
    struct SineFitFunction2
    {
        //传进来角度
        SineFitFunction2(double t, double angle, double w) : _t(t), _angle(angle), _w(w) {}
        // 残差的计算
        template <typename T>
        bool operator()(
            const T *const params, // 模型参数，有4个,a,w,theta ,其中b=2.090-a,可舍去
            T *residual) const     // 残差一维
        {
            residual[0] = _angle - (-params[0] / _w * ceres::cos(_w * _t + params[1]) + (2.090 - params[0]) * _t);
            return true;
        }
        const double _t, _angle, _w; // t，angle
    };
    struct LargeBuffFitFunction
    {
        //传进来角度
        LargeBuffFitFunction(double t, double speed) : _t(t), _speed(speed) {}
        // 残差的计算
        template <typename T>
        bool operator()(
            const T *const params, // 模型参数，有4个,a,w,theta ,其中b=2.090-a,可舍去
            T *residual) const     // 残差一维
        {
            residual[0] = _speed - (params[0] * ceres::sin(params[1] * _t + params[2]) + 2.090 - params[0]);
            return true;
        }
        const double _t, _speed; // t，angle
    };
    struct LargeBuffFitFunction2
    {
        //传进来角度
        LargeBuffFitFunction2(double speed,double t, double a, double w) : _t(t), _speed(speed), _a(a), _w(w) {}
        // 残差的计算
        template <typename T>
        bool operator()(
            const T *const params, // 模型参数，有4个,a,w,theta ,其中b=2.090-a,可舍去
            T *residual) const     // 残差一维
        {
            residual[0] = _speed - (_a * ceres::sin(_w * _t + params[0]) + 2.090 - _a);
            return true;
        }
        const double _t, _speed, _a, _w; // t，angle
    };

    class BuffCurver
    {
    private:
        ceres::Solver::Options options; // 这里有很多配置项可以填
        ceres::Solver::Summary summary; // 优化信息

        double estimate_params[4];
        double estimate_plane_params[3];

        Eigen::Vector3d armor_pose_set[16];
        int armor_pose_size = 0;

        BuffTrajectory buff_trajectory;
        bool is_model_start = false; //是否已经成功构建模型
        std::vector<Eigen::Vector3d> armor_points;
        std::vector<float> angle_points;
        double estimate_cos_params[3] = {1.0, 2, 0};

        //拟合圆
        const BuffTrajectory &fitCircle(const std::vector<Eigen::Vector3d> &armor_points);
        void correctAxis(BuffTrajectory &buff_traj, const std::vector<Eigen::Vector3d> &armor_points, const std::vector<float> &angle_points);

    public:
        BuffCurver(/* args */);
        ~BuffCurver();
        const double *solve();
        void addParams(const Eigen::Vector3d &armor_pose);
        const bool &isModelStart();
        const double *solvePlane();
        void correctPoints(std::vector<Eigen::Vector3d> &armor_points);

        const bool &fit(const Eigen::Vector3d &armor_pose, const float &angle);
        const BuffTrajectory &getTrajectory()
        {
            return buff_trajectory;
        }
        const bool &fit(std::vector<Eigen::Vector3d> &armor_pose_points);

        const double *fitCos(const std::vector<float> &angle, const std::vector<float> &t);
        const double *fitLargeBuffSin(const std::vector<float> &speed, const std::vector<float> &t);
        const double *fitLargeBuffSin2(const std::vector<float> &speed, const std::vector<float> &t);
    };
}
