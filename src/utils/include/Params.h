//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_PARAMS_H
#define AUTOAIM_PARAMS_H

#include <string>
#include "opencv2/opencv.hpp"

using namespace std;

namespace ly
{
    class CameraParam
    {
    public:
        static int device_type;     // 设备类型
        static string sn;           // 设备SN号
        static string video_path;   // 测试视频位置
        static string picture_path; // 测试图片位置
        static int exposure_time;   //曝光时间
        static double gain;         //增益
        static double gamma;        // 数字分类使用的gamma
        static double fx;           // 相机内参
        static double fy;
        static double u0;
        static double v0;
        static double k1;
        static double k2;
        static double k3;
        static double p1;
        static double p2;
    };
    class ShootParam
    {
    public:
        static double camera_trans_x; // 相机到枪管的平移向量
        static double camera_trans_y;
        static double camera_trans_z;

        static float shoot_speed; // 子弹速度，调试使用，实际上场直接从裁判系统读取弹速

        // 三种不同弹速的空气阻力系数k
        static float k;

        // pitch轴补偿，shoot_up_offset为固有补偿，shoot_up_offsetk则为根据distance计算的一次补偿
        static int shoot_up_offset;
        static float shoot_up_offsetk;

        static int shoot_right_offset; // yaw轴补偿
        static float shoot_delay;      // 发射延迟
        static int shoot_frequency;    // 发射频率，debug使用，降低射频
        static bool is_shoot;          // 是否打子弹，debug使用，实际由操作手判断
    };
    class BuffParam
    {
    public:
        //识别大符参数
        static int binary_thresh;         //灰度图二值化阈值
        static int color_thresh;          // 颜色通道相减阈值
        static int armor_size_min;        //轮廓点数最小
        static int armor_size_max;        //轮廓点数最大
        static int armor_father_size_min; //父轮廓点数最小
        static int armor_father_size_max;
        static float armor_ratio_min; //长宽比最小
        static float armor_ratio_max; //长宽比最大

        //大符中心识别参数
        static int buff_center_size_min; // 中心的轮廓点数
        static int buff_center_size_max;
        static float buff_center_radius_max; // 中心的像素半径
        static float buff_center_radius_min;
        static float buff_rotate_radius_min; // 中心到装甲板的像素距离
        static float buff_rotate_radius_max;

        static bool test_mode;      // 测试模式，用于静止标定
        static float shoot_delay;   // 单发发射延迟
        static int shoot_frequency; // 发射频率，debug使用，实际由操作手打击
        static int shoot_up_offset; // pitch一次函数补偿
        static int shoot_up_offsetk;
        static int shoot_right_offset; // yaw一次函数补偿
        static int shoot_right_offsetk;

        static string model_path; // 分类模型位置
    };
    class LargeBuffParam
    {
    public:
        static float angle_process_noise; // 大符利用卡尔曼滤波的过程噪声
    };
    class DetectorParam
    {
    public:
        static string color;          // 颜色，已弃用，实际从裁判系统读取颜色
        static int thresh;            //二值化阈值
        static int mode;              // 大符/辅瞄模式切换 ，弃用，实际从裁判系统读取
        static bool is_save_classify; // 是否采集数字数据集

        static BuffParam buff_params;
    };

    class ArmorDetectParam
    {
    public:
        static float angle_diff_max;     // 灯条角度差
        static float lightbar_ratio_max; // 灯条长宽比
        static float lightbar_ratio_min;
        static float lightbar_area_max; // 灯条面积
        static float lightbar_area_min;
        static float armor_ratio_max; // 装甲板长宽比
        static float armor_ratio_min;
        static float lightbar_center_diff; // 两灯条中心像素差
        static float lightbar_angle_min;   // 灯条角度
        static float lightbar_angle_max;
        static float rectangle_likely;      // 多像矩形
        static float lightbar_length_ratio; // 两灯条长度比例

        static int armor_area_min; // 装甲板面积
        static int armor_area_max;

        // debug参数
        static bool show_thresh;        // 是否显示二值化图像
        static bool show_classify;      // 是否显示分类图像
        static bool debug_show;         // 是否显示原图框出装甲板的图像
        static bool show_lightbar_info; // 控制台显示灯条信息
        static bool show_armor_info;    // 控制台显示装甲板信息
    };

    class FilterParams
    {
    public:
        // 卡尔曼滤波的测量噪声，已经弃用，实际测量噪声将会根据距离发生变化，详情见对应的README文件分析
        static float measurement_noise_pose_x;
        static float measurement_noise_pose_y;
        static float measurement_noise_pose_z;

        // 卡尔曼滤波的过程噪声，通常保持x,y一致
        static float process_noise_pose_x;
        static float process_noise_pose_y;
        static float process_noise_pose_z;

        // 强跟踪滤波参数
        static float stf_beta;
        static bool is_use_stf;
    };

    class AutoExposureGain //自动曝光参数，一般不使用
    {
    public:
        static bool is_use_auto_exposure;
        static bool is_use_auto_gain;
        static int auto_exposure_min;
        static int auto_exposure_max;
        static int auto_gain_min;
        static int auto_gain_max;
        static int target_gray_min;
        static int target_gray_max;
    };

    class ThreadDelay // 三个线程的延时参数，单位us
    {
    public:
        static int pic_thread_delay;
        static int serial_thread_delay;
        static int detect_thread_delay;
    };

    class WhiteBalanceParam //自动白平衡参数，一般设定为true即可
    {
    public:
        static bool is_auto_balance;
        static float white_balance_ratio_r;
        static float white_balance_ratio_b;
        static float white_balance_ratio_g;
    };

}

#endif //AUTOAIM_PARAMS_H
