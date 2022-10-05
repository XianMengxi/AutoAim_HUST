//
// Created by zhiyu on 2021/8/20.
//

#ifndef PRIDICTOR_H
#define PRIDICTOR_H

#include "Config.h"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "sophus/se3.h"
#include "sophus/so3.h"
#include "NormalKalman.h"
#include "lcm_module.h"
#include "SecondFilter.h"

#include "PoseSolver.h"
#include "NormalEKF.h"
#include <numeric>

using namespace cv;
using namespace std;

#define ANTIROT_SHOOT_DELAY 0.11
#define SHOOT_RANGE 30 //ms

namespace ly
{
    struct Params_ToPredictor
    {
        Eigen::Vector3d carPose_now;
        SerialPortData *SerialPortData_;
        float time;
        bool is_update;

        Params_ToPredictor()
        {
            is_update = false;
        }
    };
    enum SHOOT_SPEED_MODE
    {
        SHOOT_SPEED15 = 0,
        SHOOT_SPEED18,
        SHOOT_SPEED30,
        SHOOT_SPEED_UNDEFINE
    };

    typedef struct
    {
        float pitch;
        float yaw;
        float ShootSpeed = 15;
        double BeginToNowTime = 0;
    } carPose;

    //击打缓冲计算返回
    typedef struct
    {
        float pitch; //rad
        float yaw;   //rad
        float time;  //击打弹道时间(ms)
        float distance;
        /*******用作传给jscope画图*******/
        cv::Point3f carpose_now; //实际解算值
        cv::Point3f carpose_KF;  //KF的statePost
        cv::Point3f carpose_pre; //预测值
    } Angle_t;

    struct AntirotStamp //pitch的计算交给普通滤波器
    {
        std::vector<double> yaw_set;
        std::vector<double> pitch_set;
        std::vector<std::chrono::steady_clock::time_point> time_stamps;
    };
    struct HistoryStamp
    {
        double time_from_start;
        Eigen::Vector3d filte_pose;
    };

    class Predictor
    {
    public:
        Predictor();
        ~Predictor();
        Angle_t Predict(const Eigen::Vector3d &armor_trans, SerialPortData SerialPortData_, float delta_t);
        Angle_t BuffPredict(const Eigen::Vector3d &armor_trans, SerialPortData SerialPortData_);
        float calShootTime(const Eigen::Vector3d &armor_trans, SerialPortData SerialPortData_);

        void resetKalman() //重启卡尔曼滤波
        {
            normal_kalman_filter->resetKalman();
        }
        Angle_t NoPosePredict(const float &delta_t, SerialPortData SerialPortData_);
        bool judgeIsShoot(); //判断击打

        int target_id = -1; //目标id，根据id选择不同策略
        int getPitchDiff(float y);

        bool is_restart = false;

        int getBuffPitchDiff(float y);

        //小陀螺模型，待测试
        Angle_t AntirotPredict(const Eigen::Vector3d &armor_trans, SerialPortData SerialPortData_, float delta_t);

        bool runCTAntirotModel(const Eigen::Vector3d &armor_trans, std::chrono::steady_clock::time_point time_point, SerialPortData SerialPortData_, bool is_get_second_armor = false, const Eigen::Vector3d &second_armor_trans = Eigen::Vector3d(0, 0, 0));
        bool getAntirotTime(std::chrono::steady_clock::time_point time_point);
        int getBuffYawDiff(float x);
        bool CTModelTest(const Eigen::Vector3d &armor_trans, SerialPortData SerialPortData_, float delta_t);

        short getAntirotPitch(std::chrono::steady_clock::time_point time_point);
        void sortTwoArmor(Sophus::SE3 &armor_pose1, Sophus::SE3 &armor_pose2, bool is_get_second_armor);

        Angle_t getAntirotShootTarget() { return shootAngleTime_pre; };
        void checkAntirotPose(Angle_t &shootAngleTime_, SerialPortData SerialPortData_);
        void AntirotPredict(SerialPortData SerialPortData_, std::chrono::steady_clock::time_point time_point, Angle_t &raw_pre, const float &delta_t);

        double yaw;
        double pitch;
        bool is_get_shoot = false;
        double last_pitch;
        bool is_get_last_pitch = false;
        double first_yaw = 0;
        double last_yaw = 0;

    private:
        Angle_t ballistic_equation(float gim_pitch, const Eigen::Vector3d &armor_Position);
        float BulletModel(float x, float v, float angle);

        void setShootSpeed(const char &flag, const float &shoot_speed);

        //根据运动模型计算打击时间
        double calcPredictTime(const Eigen::Vector3d &armor_pose, const Eigen::Vector3d &armor_speed, const float &shoot_speed);

        Angle_t shootAngleTime_now;
        Angle_t shootAngleTime_pre;
        float ShootSpeed;

        //强跟踪模型
        NormalKalman *normal_kalman_filter;
        LcmDebug *lcm_debug; //用作观测滤波器数据

        int shoot_speed_mode = SHOOT_SPEED_UNDEFINE;

        NormalEKF *ekf_filter;

        AntirotStamp antirot_stamp;
        AntirotStamp next_antirot_stamp;

        std::chrono::steady_clock::time_point shoot_time_stamp;

        double antirot_shoot_time;

        Eigen::Vector3d last_armor_pose;

        int save_index = 0;
        std::vector<HistoryStamp> history_stamp[2];
    };

}

#endif //AUTOAIM_POSESOLVER_H
