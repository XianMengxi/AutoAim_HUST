#ifndef _NORMAL_KALMAN_H
#define _NORMAL_KALMAN_H
#include "STF.hpp"
#include "Params.h"
#include "Log.h"
namespace ly
{
#define ACCELERATE_THRESH 20
    class NormalKalman
    {
    private:
        /* data */
        StrongTrackingFilter<double, 6, 3> *kalman_filter; //滤波6维，x,x_v,y,y_v,z,z_v ,测量三维x,y,z
        bool is_kalman_init;
        void rebootKalman(const Eigen::Vector3d &new_armor_pose);
        void resetTransitionMatrix();
        void setUpdateTime(const double &delta_t);
        void setTransitionMatrix();
        Eigen::Vector3d correct(const Eigen::Vector3d &armor_pose);
        void setMeasureMatrix();
        void setMeasurementNoise(const Eigen::Vector3d &armor_pose);
        void setProcessNoise();

        double update_time;
        Eigen::Vector3d posteriori_speed;
        Eigen::Vector3d posteriori_pose;

        Eigen::Matrix3d process_noice;
        Eigen::Matrix<double, 6, 3> process_noise_matrix;

        //加速度平滑设计
        double alpha = 0.95; //越大，加速度越趋于0，即越平滑(不可避免会有一定滞后)
        bool is_get_first_speed = false;
        Eigen::Vector3d accelerate;
        Eigen::Vector3d last_speed;
        void rebootKalmanWithoutV(const Eigen::Vector3d &new_armor_pose);
        bool ChiSquaredTest();

        // double chi_squared_thresh = 1000;

        double detect_param;

    public:
        NormalKalman(/* args */);
        ~NormalKalman();
        Eigen::Vector3d runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t);
        Eigen::Vector3d predict(const double &predict_t);
        Eigen::Vector3d getSpeed() { return posteriori_speed; };
        Eigen::Vector3d getPose() { return posteriori_pose; };
        void resetKalman()
        {
            is_kalman_init = false;
            detect_param = 0;
        }
        Eigen::Vector3d predict(const Eigen::Vector3d &pose, const double &predict_t);

        Eigen::Vector3d NoPosePredict(const double &delta_t);
        Eigen::Vector3d getA() { return accelerate; };
        Eigen::Vector3d getResidual() { return kalman_filter->residual; };
        void setProcessNoise(double x, double y, double z);

        bool chi_squared_test_status = false; //false表明卡方检验正常
        bool is_antirot_mode = false;
        double getChiSquareNumber() { return detect_param; };
        double chi_squared_thresh = 4;
        bool ChiSquaredTest(const Eigen::Vector3d &armor_pose); //armor_pose为测量值
    };

}

#endif