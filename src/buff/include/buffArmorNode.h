#ifndef _ARMOR_NODE_H
#define _ARMOR_NODE_H
#include <opencv2/opencv.hpp>
#include "eigen3/Eigen/Core"
#include <chrono>
namespace ly
{
    struct BuffArmor
    {
        BuffArmor()
        {
            is_useful = false;
        }
        cv::RotatedRect armor_rect;
        cv::RotatedRect armor_father_rect;
        float armor_ratio;
        cv::Point2f direction_vec; //中心指向装甲板的向量
        cv::Point2f corner[4];
        float angle;
        Eigen::Vector3d world_pose;
        bool is_useful;
        cv::Point2f buff_center;

        std::chrono::_V2::steady_clock::time_point time_stamp;
    };
}

#endif