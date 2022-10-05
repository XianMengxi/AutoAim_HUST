#ifndef CV_MODEL_H
#define CV_MODEL_H
#include "STF.hpp"
#include "Config.h"
//匀加速度模型
namespace ly
{
    class CAModel
    {
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
        void setMeasurementNoise(const Eigen::Vector3d &armor_pose);

        double update_time;
        Eigen::Vector3d posteriori_speed;
        Eigen::Vector3d posteriori_pose;
        Eigen::Vector3d posteriori_accelerate;

        Eigen::Matrix3d process_noice;
        Eigen::Matrix<double, 3, 3> process_noise_matrix;

    public:
        CAModel(/* args */);
        ~CAModel();
        Eigen::Vector3d runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t);
        Eigen::Vector3d predict(const double &predict_t);
        Eigen::Vector3d getSpeed() { return posteriori_speed; };
        Eigen::Vector3d getPose() { return posteriori_pose; };
        Eigen::Vector3d getA() { return posteriori_accelerate; };
        Eigen::Vector3d NoPosePredict(const double &delta_t);

        void resetKalman()
        {
            is_kalman_init = false;
        }
        Eigen::Vector3d getResidual()
        {
            return kalman_filter->residual;
        }
    };
}

#endif