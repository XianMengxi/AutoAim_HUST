#ifndef SINGER_EXTENDED_FILTER_H
#define SINGER_EXTENDED_FILTER_H
#include "Config.h"
#include "ExtendedKalman.hpp"
namespace ly
{
    class MeasureTool
    {
    public:
        template <class T>
        void operator()(const T xyz[9], T pyd[3]) //ms
        {
            pyd[0] = ceres::atan2(xyz[6], ceres::sqrt(xyz[0] * xyz[0] + xyz[3] * xyz[3])); // pitch
            pyd[1] = -ceres::atan2(xyz[0], xyz[3]);                                        // yaw
            pyd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[3] * xyz[3]);                       // distance
        }
    };
    class SingerExtendedFilter
    {
    private:
        /* data */
    private:
        /* data */
        ExtendedKalman<double, 9, 3> *kalman_filter; //滤波九维，测量三维
        bool is_kalman_init;
        void rebootKalman(const Eigen::Vector3d &new_armor_pose);
        void resetTransitionMatrix();
        void setUpdateTime(const double &delta_t);
        void setTransitionMatrix();
        Eigen::Vector3d correct(const Eigen::Vector3d &armor_pose);
        void setMeasureMatrix();
        void setMeasurementNoise();
        void setProcessNoise();
        void setControlVector();
        void calSigma();
        void setMaxA();

        double update_time;
        Eigen::Vector3d posteriori_speed;
        Eigen::Vector3d posteriori_pose;
        Eigen::Vector3d posteriori_accelerate;

        MeasureTool tool;

        double sigma_2[3] = {0.1, 0.1, 0.1}; //根据公式计算 = a_max^2 *(1+ 4 *p_max - p_0)/3 ,其中a_max表示最大加速度，p_max表示最大加速度概率，p_0表示非机动概率

        //需要调节的参数
        double alpha;
        double a_max[3] = {16, 0.5, 0.5};

    public:
        SingerExtendedFilter(/* args */);
        ~SingerExtendedFilter();
        Eigen::Vector3d runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t);
        Eigen::Vector3d predict(const double &predict_t);
        Eigen::Vector3d getSpeed() { return posteriori_speed; };
        Eigen::Vector3d getPose() { return posteriori_pose; };
        Eigen::Vector3d getA() { return posteriori_accelerate; };
        Eigen::Vector3d getPYD() { return measure(posteriori_pose); };
        Eigen::Vector3d measure(const Eigen::Vector3d &armor_pose);

        void resetKalman()
        {
            is_kalman_init = false;
        }
    };
}

#endif