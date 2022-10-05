//
// Created by zhiyu on 2021/8/20.
//
#include "Params.h"

using namespace std;

namespace ly
{
    int CameraParam::device_type;
    string CameraParam::sn;
    string CameraParam::picture_path;
    string CameraParam::video_path;
    int CameraParam::exposure_time;
    double CameraParam::gamma;
    double CameraParam::gain;
    double CameraParam::fx;
    double CameraParam::fy;
    double CameraParam::u0;
    double CameraParam::v0;
    double CameraParam::k1;
    double CameraParam::k2;
    double CameraParam::k3;
    double CameraParam::p1;
    double CameraParam::p2;

    string DetectorParam::color;
    int DetectorParam::thresh;
    int DetectorParam::mode;
    bool DetectorParam::is_save_classify;

    BuffParam DetectorParam::buff_params;
    int BuffParam::binary_thresh;
    int BuffParam::color_thresh;
    int BuffParam::armor_size_min;
    int BuffParam::armor_size_max;
    int BuffParam::armor_father_size_min;
    int BuffParam::armor_father_size_max;
    float BuffParam::armor_ratio_min;
    float BuffParam::armor_ratio_max;
    int BuffParam::buff_center_size_min;
    int BuffParam::buff_center_size_max;
    float BuffParam::buff_center_radius_max;
    float BuffParam::buff_center_radius_min;
    float BuffParam::buff_rotate_radius_min;
    float BuffParam::buff_rotate_radius_max;
    string BuffParam::model_path;
    bool BuffParam::test_mode;
    float BuffParam::shoot_delay;
    int BuffParam::shoot_frequency;
    int BuffParam::shoot_up_offset;
    int BuffParam::shoot_up_offsetk;
    int BuffParam::shoot_right_offset;
    int BuffParam::shoot_right_offsetk;

    double ShootParam::camera_trans_x;
    double ShootParam::camera_trans_y;
    double ShootParam::camera_trans_z;
    float ShootParam::shoot_speed;
    int ShootParam::shoot_frequency;
    float ShootParam::shoot_delay;
    int ShootParam::shoot_right_offset;
    bool ShootParam::is_shoot;
    float ShootParam::k;
    int ShootParam::shoot_up_offset;
    float ShootParam::shoot_up_offsetk;

    float ArmorDetectParam::angle_diff_max;
    float ArmorDetectParam::lightbar_ratio_max;
    float ArmorDetectParam::lightbar_ratio_min;
    float ArmorDetectParam::lightbar_area_max;
    float ArmorDetectParam::lightbar_area_min;
    float ArmorDetectParam::armor_ratio_max;
    float ArmorDetectParam::armor_ratio_min;
    float ArmorDetectParam::lightbar_center_diff;
    float ArmorDetectParam::lightbar_angle_min;
    float ArmorDetectParam::lightbar_angle_max;
    float ArmorDetectParam::rectangle_likely;
    float ArmorDetectParam::lightbar_length_ratio;
    int ArmorDetectParam::armor_area_min;
    int ArmorDetectParam::armor_area_max;
    bool ArmorDetectParam::show_thresh;
    bool ArmorDetectParam::show_classify;
    bool ArmorDetectParam::debug_show;
    bool ArmorDetectParam::show_lightbar_info;
    bool ArmorDetectParam::show_armor_info;

    float FilterParams::measurement_noise_pose_x;
    float FilterParams::measurement_noise_pose_y;
    float FilterParams::measurement_noise_pose_z;
    float FilterParams::process_noise_pose_x;
    float FilterParams::process_noise_pose_y;
    float FilterParams::process_noise_pose_z;
    float FilterParams::stf_beta;
    bool FilterParams::is_use_stf;

    float LargeBuffParam::angle_process_noise;

    bool AutoExposureGain::is_use_auto_exposure;
    bool AutoExposureGain::is_use_auto_gain;
    int AutoExposureGain::auto_exposure_min;
    int AutoExposureGain::auto_exposure_max;
    int AutoExposureGain::auto_gain_min;
    int AutoExposureGain::auto_gain_max;
    int AutoExposureGain::target_gray_min;
    int AutoExposureGain::target_gray_max;

    int ThreadDelay::pic_thread_delay;
    int ThreadDelay::serial_thread_delay;
    int ThreadDelay::detect_thread_delay;

    bool WhiteBalanceParam::is_auto_balance;
    float WhiteBalanceParam::white_balance_ratio_r;
    float WhiteBalanceParam::white_balance_ratio_b;
    float WhiteBalanceParam::white_balance_ratio_g;
}