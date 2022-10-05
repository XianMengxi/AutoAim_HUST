#ifndef _NORMAN_EKF_H
#define _NORMAN_EKF_H
#include "ExtendedKalman.hpp"
#include "Config.h"
#include "Log.h"
#include <chrono>
#define VERIFY_THRESH 20
namespace ly
{
    enum ANTIROT_DIRECTION
    {
        ANTIROT_CLOCKWISE = 0, //顺时针
        ANTIROT_COUNTER_CLOCK_WISE = 1
    };
    class Xyz2Pyd
    {
    public:
        template <class T>
        //x,x_v,y,y_v,z,z_v
        void operator()(const T xyz[6], T pyd[3]) //ms
        {
            pyd[0] = ceres::atan2(xyz[4], ceres::sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2])); // pitch
            pyd[1] = ceres::atan2(xyz[0], xyz[2]);                                         // yaw
            pyd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[4] * xyz[4] + xyz[2] * xyz[2]);     // distance
        }
    };
    class NormalEKF
    {
    private:
        /* data */
        ExtendedKalman<double, 6, 3> *kalman_filter; //滤波6维，x,x_v,y,y_v,z,z_v ,测量三维x,y,z
        bool is_kalman_init;
        void rebootKalman(const Eigen::Vector3d &new_armor_pose);
        void resetTransitionMatrix();
        void setUpdateTime(const double &delta_t);
        void setTransitionMatrix();
        Eigen::Vector3d correct(const Eigen::Vector3d &armor_pose);
        void setMeasureMatrix();
        void setMeasurementNoise(const Eigen::Vector3d &armor_pose);
        void setProcessNoise();
        void rebootKalmanWithInitial(const Eigen::Vector3d &new_armor_pose);

        double update_time;
        Eigen::Vector3d posteriori_speed;
        Eigen::Vector3d posteriori_pose;

        Eigen::Matrix3d process_noice;
        Eigen::Matrix<double, 6, 3> process_noise_matrix;

        Eigen::Vector3d measure(const Eigen::Vector3d &armor_pose);
        void setIsUseSTF(bool flag);

        Xyz2Pyd xyz_to_pyd;

    public:
        NormalEKF(/* args */);
        ~NormalEKF();
        Eigen::Vector3d runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t);
        Eigen::Vector3d predict(const double &predict_t);
        Eigen::Vector3d getSpeed() { return posteriori_speed; };
        Eigen::Vector3d getPose() { return posteriori_pose; };

        void resetKalman()
        {
            is_kalman_init = false;
        }
        Eigen::Vector3d getResidual() { return kalman_filter->residual; };
        double getDetectParam() { return detect_param; };
        void setProcessNoise(double x, double y, double z);

        double detect_param = 0;

        bool is_antirot_state = false;
        double change_armor_time = 0;
        bool this_is_change_armor = false;
        double aver_rotate_time = 0.0;
        bool is_get_aver_time = false;
        int antirot_count = 0;
        Eigen::Vector3d last_armor_pose;
        int rotation = ANTIROT_CLOCKWISE;
        std::chrono::steady_clock::time_point last_antirot_time_point;
    };

}
#endif