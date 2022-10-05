#ifndef _CTMODEL_H
#define _CTMODEL_H
#include "AdaptiveEKF.hpp"
#define CHI_REST_THRESH 25
#include "lcm_module.h"
#include "Config.h"
#include "Log.h"
namespace ly
{
    struct Predict
    {
        /*
     * CT模型
     */
        template <class T>
        void operator()(const T x0[6], T x1[6], double delta_t)
        {
            //x,x_v,y,y_v,w,z
            x1[0] = x0[0] + ceres::sin(x0[4] * delta_t) / x0[4] * x0[1] - (T(1) - T(ceres::cos(x0[4] * delta_t))) / x0[4] * x0[3]; //0.1
            x1[1] = x0[1] * ceres::cos(x0[4] * delta_t) - ceres::sin(x0[4] * delta_t) * x0[3];                                                   //100
            x1[2] = x0[2] + (T(1) - T(ceres::cos(x0[4] * delta_t))) / x0[4] * x0[1] + ceres::sin(x0[4] * delta_t) / x0[4] * x0[3]; //0.1
            x1[3] = x0[3] * ceres::cos(x0[4] * delta_t) + ceres::sin(x0[4] * delta_t) * x0[1];                                                   //100
            x1[4] = x0[4];                                                                                                         //w
            x1[5] = x0[5];                                                                                                         //垂直方向不滤波                                                                                                     //0.01
        }
    };
    struct Measure
    {
        /*
     * 工具函数的类封装
     */
        template <class T>
        void operator()(const T xywz[6], T pyd[3])
        {
            pyd[0] = ceres::atan2(xywz[5], ceres::sqrt(xywz[0] * xywz[0] + xywz[2] * xywz[2])); // pitch
            pyd[1] = ceres::atan2(xywz[0], xywz[2]);                                            // yaw
            pyd[2] = ceres::sqrt(xywz[0] * xywz[0] + xywz[5] * xywz[5] + xywz[2] * xywz[2]);    //distance
        }
    };
    class CTModel
    {
    private:
        /* data */
        AdaptiveEKF<6, 3> *kalman_filter; //滤波6维，x,x_v,y,y_v,z,z_v ,测量三维x,y,z
        bool is_kalman_init;
        void rebootKalman(const Eigen::Vector3d &new_armor_pose);
        void setUpdateTime(const double &delta_t);
        Eigen::Vector3d correct(const Eigen::Vector3d &armor_pose);
        void setMeasureMatrix();
        void setMeasurementNoise(const Eigen::Vector3d &armor_pose);
        void setProcessNoise();

        double update_time;

        Eigen::Matrix3d process_noice;
        Eigen::Matrix<double, 6, 3> process_noise_matrix;

        Predict predict_fun;
        Measure measure_fun;
        // LcmDebug debug;

    public:
        CTModel(/* args */);
        ~CTModel();
        Eigen::Vector3d runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t);
        Eigen::Vector3d predict(const double &predict_t);
        Eigen::Vector3d getSpeed() { return posteriori_speed; };
        Eigen::Vector3d getPose() { return posteriori_pose; };
        void resetKalman()
        {
            is_kalman_init = false;
        }
        double getTestNumber(){return kalman_filter->ChiSquaredTest();};

        void setProcessNoise(double x, double y, double z);
        Eigen::Vector3d posteriori_speed;
        Eigen::Vector3d posteriori_pose;
        double rotate_speed;

    };

}

#endif