//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_POSESOLVER_H
#define AUTOAIM_POSESOLVER_H

#include "Config.h"

#include "eigen3/Eigen/Dense"
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.h>
#include <sophus/so3.h>
#include <fstream>
#include <string>
#include "EnemyType.h"
#include "SerialPort.h"

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace Sophus;

namespace ly
{
    class PoseSolver
    {
    public:
        void undistortBuffPoints(const std::vector<cv::Point2f> input_points, std::vector<cv::Point2f> &output_points);
        void onePointCalc(SerialPortData SerialPortData_, float &yaw, float &pitch, const cv::Point2f &target);

        explicit PoseSolver();
        SE3 getPoseInCamera(const int &armor_class, const std::vector<cv::Point2f> &corners, SerialPortData imu_data);

    private:
        void setCameraMatrix(double fx, double fy, double u0, double v0);
        void setDistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3);
        void setimu(float pitch, float yaw, float roll);
        void setCameraTrans(double x, double y, double z);

        Mat camera_matrix;
        Mat distortion_coefficients;
        Mat tvec;
        Mat rvec;
        Mat m_T;
        Mat m_R;

        double camera_trans_x;
        double camera_trans_y;
        double camera_trans_z;

        Matrix3d e_R;
        Vector3d e_T;

        double yaw;

        vector<Point3f> points_large_3d;
        vector<Point3f> points_small_3d;

        Sophus::SE3 armor_to_camera;
        Sophus::SE3 camera_to_gimbal; // imu's world
        Sophus::SE3 armor_to_gimbal;
        Sophus::SE3 armor_to_world;
        Sophus::SE3 gimbal_to_world;

        Sophus::SE3 camera_to_world; // imu's world
    };
}

#endif //AUTOAIM_POSESOLVER_H
