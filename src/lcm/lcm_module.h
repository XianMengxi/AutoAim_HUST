#ifndef _LCM_DEBUG_H
#define _LCM_DEBUG_H

#include "lcm/lcm-cpp.hpp"
#include "exlcm/example_t.hpp"
#include <iostream>
#include <eigen3/Eigen/Core>
#include "info.hpp"

namespace ly
{
    class LcmDebug
    {
        lcm::LCM lcm;

    public:
        LcmDebug();

        ~LcmDebug()
        {
        }
        void sendData(const Eigen::Vector3d &pose, const Eigen::Vector3d &speed = Eigen::Vector3d(0, 0, 0), const Eigen::Vector3d &kf_pose = Eigen::Vector3d(0, 0, 0), const Eigen::Vector3d &kf_speed = Eigen::Vector3d(0, 0, 0), float *yaw = NULL, float *pitch = NULL);
        void sendInfo(const Eigen::Vector3d &pose1, const float &time_stamp1, const Eigen::Vector3d pose2, const float &time_stamp2);
    };
}

#endif