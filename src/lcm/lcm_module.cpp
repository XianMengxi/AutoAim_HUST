#include "lcm_module.h"

namespace ly
{
    LcmDebug::LcmDebug()
    {
        if (!lcm.good())
        {
            std::cerr << "lcm module error" << std::endl;
        }
    }
    void LcmDebug::sendData(const Eigen::Vector3d &pose, const Eigen::Vector3d &speed, const Eigen::Vector3d &kf_pose, const Eigen::Vector3d &kf_speed, float *yaw, float *pitch)
    {
        exlcm::example_t my_data;
        for (int i = 0; i < 3; i++)
        {
            my_data.pose[i] = pose[i];
            my_data.speed[i] = speed[i];
            my_data.kf_pose[i] = kf_pose[i];
            my_data.kf_speed[i] = kf_speed[i];
        }
        if (yaw != NULL && pitch != NULL)
        {
            for (int i = 0; i < 2; i++)
            {
                my_data.yaw[i] = yaw[i];
                my_data.pitch[i] = pitch[i];
            }
        }

        lcm.publish("debug", &my_data);
        // std::cout << "send lcm msg" << std::endl;
    }
    void LcmDebug::sendInfo(const Eigen::Vector3d &pose1, const float &time_stamp1, const Eigen::Vector3d pose2, const float &time_stamp2)
    {
        exlcm::info my_data;
        for (int i = 0; i < 3; i++)
        {
            my_data.predict_pos[i] = pose1[i];
            my_data.predict_pos[i + 3] = pose2[i];
        }
        my_data.time_stamp[0] = time_stamp1;
        my_data.time_stamp[1] = time_stamp2;
        lcm.publish("info", &my_data);
    }
}
