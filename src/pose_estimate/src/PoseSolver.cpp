//
// Created by zhiyu on 2021/8/20.
//

#include "PoseSolver.h"

using namespace std;
namespace ly
{
    PoseSolver::PoseSolver()
    {
        setCameraMatrix(CameraParam::fx, CameraParam::fy, CameraParam::u0, CameraParam::v0);
        setDistortionCoefficients(CameraParam::k1, CameraParam::k2, CameraParam::p1, CameraParam::p2, CameraParam::k3);
        setCameraTrans(ShootParam::camera_trans_x, ShootParam::camera_trans_y, ShootParam::camera_trans_z);

        camera_to_gimbal = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(camera_trans_x, camera_trans_y, camera_trans_z));
    }
    void PoseSolver::setCameraTrans(double x, double y, double z)
    {
        camera_trans_x = x;
        camera_trans_y = y;
        camera_trans_z = z;
    }

    void PoseSolver::setCameraMatrix(double fx, double fy, double u0, double v0)
    {
        camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
        camera_matrix.ptr<double>(0)[0] = fx;
        camera_matrix.ptr<double>(0)[2] = u0;
        camera_matrix.ptr<double>(1)[1] = fy;
        camera_matrix.ptr<double>(1)[2] = v0;
        camera_matrix.ptr<double>(2)[2] = 1.0f;
    }
    //设置畸变系数矩阵
    void PoseSolver::setDistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3)
    {
        distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
        distortion_coefficients.ptr<double>(0)[0] = k_1;
        distortion_coefficients.ptr<double>(1)[0] = k_2;
        distortion_coefficients.ptr<double>(2)[0] = p_1;
        distortion_coefficients.ptr<double>(3)[0] = p_2;
        distortion_coefficients.ptr<double>(4)[0] = k_3;
    }

    void PoseSolver::setimu(float pitch, float yaw, float roll)
    {

        Eigen::Matrix3d rotation_matrix3;
        rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX());
        gimbal_to_world = Sophus::SE3(rotation_matrix3, Eigen::Vector3d(0, 0, 0));
    }

    Sophus::SE3 PoseSolver::getPoseInCamera(const int &armor_class, const std::vector<cv::Point2f> &corners, SerialPortData imu_data)
    {
        const static vector<Point3f> points_small_3d = {Point3f(-0.0675f, -0.0275f, 0.f),
                                                        Point3f(0.0675f, -0.0275f, 0.f),
                                                        Point3f(0.0675f, 0.0275f, 0.f),
                                                        Point3f(-0.0675f, 0.0275f, 0.f)};
        const static vector<Point3f> points_large_3d = {Point3f(-0.1125f, -0.0275f, 0.f),
                                                        Point3f(0.1125f, -0.0275f, 0.f),
                                                        Point3f(0.1125f, 0.0275f, 0.f),
                                                        Point3f(-0.1125f, 0.0275f, 0.f)};
        const static vector<Point3f> points_buff_3d = {Point3f(-0.1150f, -0.0635f, 0.f),
                                                       Point3f(0.1150f, -0.0635f, 0.f),
                                                       Point3f(0.1150f, 0.0635f, 0.f),
                                                       Point3f(-0.1150f, 0.0635f, 0.f)}; //大符四点
        const static vector<Point3f> points_buff_5point_3d = {Point3f(-0.1150f, -0.0635f, 0.f),
                                                              Point3f(0.1150f, -0.0635f, 0.f),
                                                              Point3f(0.1150f, 0.0635f, 0.f),
                                                              Point3f(-0.1150f, 0.0635f, 0.f),
                                                              Point3f(0.f, 0.7f, 0.0f)}; //大符五点

        const static Sophus::SE3 armor_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));

        if (armor_class == Hero || armor_class == Sentry)
        { // large armor
            solvePnP(points_large_3d, corners, camera_matrix, distortion_coefficients,
                     rvec, tvec, false, cv::SOLVEPNP_IPPE);
        }
        else if (armor_class == BUFF)
        {
            solvePnP(points_buff_3d, corners, camera_matrix, distortion_coefficients,
                     rvec, tvec, false, cv::SOLVEPNP_IPPE);
        }
        else if (armor_class == BUFF5)
        {
            solvePnP(points_buff_5point_3d, corners, camera_matrix, distortion_coefficients,
                     rvec, tvec, false, cv::SOLVEPNP_IPPE);
        }
        else
        { // small armor
            solvePnP(points_small_3d, corners, camera_matrix, distortion_coefficients,
                     rvec, tvec, false, cv::SOLVEPNP_IPPE);
        }

        //单点测试
        cv::Point2f high_light = corners[0] - corners[1];
        cv::Point2f low_light = corners[3] - corners[2];
        double high_length = sqrt(high_light.x * high_light.x + high_light.y * high_light.y);
        double low_length = sqrt(low_light.x * low_light.x + low_light.y * low_light.y);

        double temp = rvec.ptr<double>(0)[1];
        rvec.ptr<double>(0)[1] = rvec.ptr<double>(0)[2];
        rvec.ptr<double>(0)[2] = temp;
        temp = tvec.ptr<double>(0)[1];
        tvec.ptr<double>(0)[1] = tvec.ptr<double>(0)[2];
        tvec.ptr<double>(0)[2] = temp;
        //下面是将mat类型的旋转/平移向量转换为SE3型的TF
        cv::Mat R;
        cv::Rodrigues(rvec, R); //旋转矢量转换为旋转矩阵
        Eigen::Matrix3d Rotate_M = Eigen::Matrix3d::Identity();
        cv::cv2eigen(R, Rotate_M);

        Sophus::SO3 rotate(Rotate_M);
        Eigen::Vector3d translate(tvec.ptr<double>(0)[0], tvec.ptr<double>(0)[1], -tvec.ptr<double>(0)[2]);
        armor_to_camera = Sophus::SE3(rotate, translate);

        float rcv_yaw = (imu_data.yaw + ShootParam::shoot_right_offset) / 100.0f * M_PI / 180.0f;
        float rcv_pitch = (imu_data.pitch + ShootParam::shoot_up_offset) / 100.0f * M_PI / 180.0f;
        setimu(rcv_pitch, rcv_yaw, 0);

        Sophus::SE3 camera_ = gimbal_to_world * camera_to_gimbal;
        armor_to_world = camera_ * armor_to_camera;

        return armor_to_world;
    }
    //计算绝对角度
    void PoseSolver::onePointCalc(SerialPortData SerialPortData_, float &yaw, float &pitch, const cv::Point2f &target)
    {
        const static double fx = camera_matrix.ptr<double>(0)[0];
        const static double u0 = camera_matrix.ptr<double>(0)[2];
        const static double fy = camera_matrix.ptr<double>(1)[1];
        const static double v0 = camera_matrix.ptr<double>(1)[2];

        //转弧度
        float rcv_yaw = SerialPortData_.yaw / 100.0f * M_PI / 180.0f;
        float rcv_pitch = SerialPortData_.pitch / 100.0f * M_PI / 180.0f;

        double x = (target.x - u0) / fx;
        double y = (target.y - v0) / fy;

        float delta_yaw = atan(x); //计算相关的角度
        float delta_pitch = atan(y);

        yaw = rcv_yaw - delta_yaw;
        pitch = rcv_pitch - delta_pitch;
    }
    void PoseSolver::undistortBuffPoints(const std::vector<cv::Point2f> input_points, std::vector<cv::Point2f> &output_points)
    {
        undistortPoints(input_points, output_points, camera_matrix, distortion_coefficients, noArray(), camera_matrix);
    }

}
