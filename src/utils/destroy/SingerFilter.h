#ifndef _SINGER_FILTER_H
#define _SINGER_FILTER_H
#include "STF.hpp"
#include "Config.h"
namespace ly
{
    class SingerFilter
    {
    private:
        /* data */
    private:
        /* data */
        StrongTrackingFilter<double, 9, 3> *kalman_filter; //滤波6维，x,x_v,y,y_v,z,z_v ,测量三维x,y,z
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
        Eigen::Vector3d last_posteriori_pose;

        double sigma_2[3] = {0.1, 0.1, 0.1}; //根据公式计算 = a_max^2 *(1+ 4 *p_max - p_0)/3 ,其中a_max表示最大加速度，p_max表示最大加速度概率，p_0表示非机动概率

        //需要调节的参数
        double alpha;
        double a_max[3] = {16, 0.5, 0.5};

    public:
        SingerFilter(/* args */);
        ~SingerFilter();
        Eigen::Vector3d runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t);
        Eigen::Vector3d predict(const double &predict_t);
        Eigen::Vector3d getSpeed() { return posteriori_speed; };
        Eigen::Vector3d getPose() { return posteriori_pose; };
        Eigen::Vector3d getA() { return posteriori_accelerate; };
        void resetKalman()
        {
            is_kalman_init = false;
        }
    };
}

#endif