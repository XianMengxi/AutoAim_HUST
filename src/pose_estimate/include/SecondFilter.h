#ifndef _SECOND_FILTER_H
#define _SECOND_FILTER_H
#include "STF.hpp"
#include <fstream>
#include "Config.h"
namespace ly
{
    typedef Eigen::Vector3d ArmorSecondPose;
    class SecondFilter
    {
    private:
        void setTransitionMatrix();
        void updateArmorState(const ArmorSecondPose &new_armor_pose);
        void calculateSpeed(const ArmorSecondPose &new_armor_pose);
        void updateMeasurement(const ArmorSecondPose &new_armor_pose);
        void rebootKalman(const ArmorSecondPose &new_armor_pose);

        ArmorSecondPose correct();

        ArmorSecondPose last_armor_pose;

        Eigen::Matrix<double, 3, 1> measurement_;

        StrongTrackingFilter<double, 3, 3> *stf_filter;
        void setMeasurementNoise();
        void setProcessNoise();

    public:
        SecondFilter(/* args */);
        ArmorSecondPose runFilter(const ArmorSecondPose &new_armor_pose);
        ArmorSecondPose getPose() { return last_armor_pose; };
        void resetFilter();
        ~SecondFilter();
        Eigen::Vector3d predict(const float &predict_t);
        bool is_filter_init = false; //kalman init symbol
        void setMeasurementNoise(double x, double y, double z);
        void setProcessNoise(double x, double y, double z); //设置过程噪声矩阵
    };
};

#endif