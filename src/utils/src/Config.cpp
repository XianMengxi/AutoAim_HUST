//
// Created by zhiyu on 2021/8/20.
//

#include "Config.h"

using namespace ly;
using namespace std;
using namespace Json;

Config::Config(const string &path)
{
    this->json_file_path = path;
}

void Config::parse()
{
    fstream json_file(json_file_path, std::ios::in);
    LOG_IF(ERROR, !json_file.is_open()) << "can't find json file in " << json_file_path;
    LOG_IF(INFO, json_file.is_open()) << "successfully open json file: " << json_file_path;

    JSONCPP_STRING errs;
    CharReaderBuilder builder;
    Value root;
    bool status = Json::parseFromStream(builder, json_file, &root, &errs);
    LOG_IF(ERROR, !status) << "json file parse error!";
    /*** camera param ***/
    CameraParam::device_type = root["camera"]["device_type"].asInt();
    CameraParam::sn = root["camera"]["sn"].asString();
    CameraParam::video_path = root["camera"]["video_path"].asString();
    CameraParam::picture_path = root["camera"]["picture_path"].asString();
    CameraParam::exposure_time = root["camera"]["camera_param"]["exposure_time"].asInt();
    CameraParam::gain = root["camera"]["camera_param"]["gain"].asDouble();
    CameraParam::gamma = root["camera"]["camera_param"]["gamma"].asDouble();
    CameraParam::fx = root["camera"]["camera_param"]["fx"].asDouble();
    CameraParam::fy = root["camera"]["camera_param"]["fy"].asDouble();
    CameraParam::u0 = root["camera"]["camera_param"]["u0"].asDouble();
    CameraParam::v0 = root["camera"]["camera_param"]["v0"].asDouble();
    CameraParam::k1 = root["camera"]["camera_param"]["k1"].asDouble();
    CameraParam::k2 = root["camera"]["camera_param"]["k2"].asDouble();
    CameraParam::k3 = root["camera"]["camera_param"]["k3"].asDouble();
    CameraParam::p1 = root["camera"]["camera_param"]["p1"].asDouble();
    CameraParam::p2 = root["camera"]["camera_param"]["p2"].asDouble();

    DetectorParam::color = root["detector"]["color"].asString();
    DetectorParam::thresh = root["detector"]["thresh"].asInt();
    DetectorParam::mode = root["detector"]["mode"].asInt();
    DetectorParam::is_save_classify = root["detector"]["is_save_classify"].asBool();

    BuffParam::binary_thresh = root["detector"]["buff_params"]["binary_thresh"].asInt();
    BuffParam::color_thresh = root["detector"]["buff_params"]["color_thresh"].asInt();
    BuffParam::armor_size_min = root["detector"]["buff_params"]["armor_size_min"].asInt();
    BuffParam::armor_size_max = root["detector"]["buff_params"]["armor_size_max"].asInt();
    BuffParam::armor_father_size_min = root["detector"]["buff_params"]["armor_father_size_min"].asInt();
    BuffParam::armor_father_size_max = root["detector"]["buff_params"]["armor_father_size_max"].asInt();
    BuffParam::armor_ratio_min = root["detector"]["buff_params"]["armor_ratio_min"].asFloat();
    BuffParam::armor_ratio_max = root["detector"]["buff_params"]["armor_ratio_max"].asFloat();
    BuffParam::model_path = root["detector"]["buff_params"]["model_path"].asString();
    BuffParam::test_mode = root["detector"]["buff_params"]["test_mode"].asBool();
    BuffParam::shoot_delay = root["detector"]["buff_params"]["shoot_delay"].asFloat();
    BuffParam::shoot_frequency = root["detector"]["buff_params"]["shoot_frequency"].asInt();
    BuffParam::shoot_up_offset = root["detector"]["buff_params"]["shoot_up_offset"].asInt();
    BuffParam::shoot_up_offsetk = root["detector"]["buff_params"]["shoot_up_offsetk"].asInt();

    BuffParam::shoot_right_offsetk = root["detector"]["buff_params"]["shoot_right_offsetk"].asInt();
    BuffParam::shoot_right_offset = root["detector"]["buff_params"]["shoot_right_offset"].asInt();

    BuffParam::buff_center_size_min = root["detector"]["buff_center_params"]["buff_center_size_min"].asInt();
    BuffParam::buff_center_size_max = root["detector"]["buff_center_params"]["buff_center_size_max"].asInt();
    BuffParam::buff_center_radius_max = root["detector"]["buff_center_params"]["buff_center_radius_max"].asFloat();
    BuffParam::buff_center_radius_min = root["detector"]["buff_center_params"]["buff_center_radius_min"].asFloat();
    BuffParam::buff_rotate_radius_min = root["detector"]["buff_center_params"]["buff_rotate_radius_min"].asFloat();
    BuffParam::buff_rotate_radius_max = root["detector"]["buff_center_params"]["buff_rotate_radius_max"].asFloat();

    ShootParam::camera_trans_x = root["camera"]["camera_trans"]["x"].asDouble();
    ShootParam::camera_trans_y = root["camera"]["camera_trans"]["y"].asDouble();
    ShootParam::camera_trans_z = root["camera"]["camera_trans"]["z"].asDouble();
    ShootParam::shoot_speed = root["camera"]["shoot_speed"].asFloat();

    ShootParam::k = root["detector"]["k"].asFloat();

    ShootParam::shoot_up_offset = root["detector"]["shoot_up_offset"].asInt();
    ShootParam::shoot_up_offsetk = root["detector"]["shoot_up_offsetk"].asFloat();

    ShootParam::shoot_delay = root["detector"]["shoot_delay"].asFloat();
    ShootParam::shoot_frequency = root["detector"]["shoot_frequency"].asInt();
    ShootParam::shoot_right_offset = root["detector"]["shoot_right_offset"].asInt();
    ShootParam::is_shoot = root["detector"]["is_shoot"].asBool();

    //识别装甲板参数，规则是多参数，给足宽裕度
    ArmorDetectParam::angle_diff_max = root["detector"]["armor_detect_params"]["angle_diff_max"].asFloat();
    ArmorDetectParam::lightbar_ratio_max = root["detector"]["armor_detect_params"]["lightbar_ratio_max"].asFloat();
    ArmorDetectParam::lightbar_ratio_min = root["detector"]["armor_detect_params"]["lightbar_ratio_min"].asFloat();
    ArmorDetectParam::lightbar_area_min = root["detector"]["armor_detect_params"]["lightbar_area_min"].asFloat();
    ArmorDetectParam::lightbar_area_max = root["detector"]["armor_detect_params"]["lightbar_area_max"].asFloat();
    ArmorDetectParam::armor_ratio_max = root["detector"]["armor_detect_params"]["armor_ratio_max"].asFloat();
    ArmorDetectParam::armor_ratio_min = root["detector"]["armor_detect_params"]["armor_ratio_min"].asFloat();
    ArmorDetectParam::lightbar_center_diff = root["detector"]["armor_detect_params"]["lightbar_center_diff"].asFloat();
    ArmorDetectParam::lightbar_angle_min = root["detector"]["armor_detect_params"]["lightbar_angle_min"].asFloat();
    ArmorDetectParam::lightbar_angle_max = root["detector"]["armor_detect_params"]["lightbar_angle_max"].asFloat();
    ArmorDetectParam::rectangle_likely = root["detector"]["armor_detect_params"]["rectangle_likely"].asFloat();
    ArmorDetectParam::lightbar_length_ratio = root["detector"]["armor_detect_params"]["lightbar_length_ratio"].asFloat();

    ArmorDetectParam::armor_area_min = root["detector"]["armor_detect_params"]["armor_area_min"].asInt();
    ArmorDetectParam::armor_area_max = root["detector"]["armor_detect_params"]["armor_area_max"].asInt();

    ArmorDetectParam::show_thresh = root["detector"]["armor_detect_params"]["show_thresh"].asBool();
    ArmorDetectParam::show_classify = root["detector"]["armor_detect_params"]["show_classify"].asBool();
    ArmorDetectParam::debug_show = root["detector"]["armor_detect_params"]["debug_show"].asBool();
    ArmorDetectParam::show_lightbar_info = root["detector"]["armor_detect_params"]["show_lightbar_info"].asBool();
    ArmorDetectParam::show_armor_info = root["detector"]["armor_detect_params"]["show_armor_info"].asBool();

    FilterParams::measurement_noise_pose_x = root["detector"]["filter_params"]["measurement_noise_pose_x"].asFloat();
    FilterParams::measurement_noise_pose_y = root["detector"]["filter_params"]["measurement_noise_pose_y"].asFloat();
    FilterParams::measurement_noise_pose_z = root["detector"]["filter_params"]["measurement_noise_pose_z"].asFloat();

    FilterParams::process_noise_pose_x = root["detector"]["filter_params"]["process_noise_pose_x"].asFloat();
    FilterParams::process_noise_pose_y = root["detector"]["filter_params"]["process_noise_pose_y"].asFloat();
    FilterParams::process_noise_pose_z = root["detector"]["filter_params"]["process_noise_pose_z"].asFloat();

    FilterParams::stf_beta = root["detector"]["filter_params"]["stf_beta"].asFloat();
    FilterParams::is_use_stf = root["detector"]["filter_params"]["is_use_stf"].asBool();

    LargeBuffParam::angle_process_noise = root["detector"]["buff_params"]["large_buff_params"]["angle_process_noise"].asFloat();

    AutoExposureGain::is_use_auto_exposure = root["camera"]["auto_exposure_gain"]["is_use_auto_exposure"].asBool();
    AutoExposureGain::is_use_auto_gain = root["camera"]["auto_exposure_gain"]["is_use_auto_gain"].asBool();
    AutoExposureGain::auto_exposure_min = root["camera"]["auto_exposure_gain"]["auto_exposure_min"].asInt();
    AutoExposureGain::auto_exposure_max = root["camera"]["auto_exposure_gain"]["auto_exposure_max"].asInt();
    AutoExposureGain::auto_gain_min = root["camera"]["auto_exposure_gain"]["auto_gain_min"].asInt();
    AutoExposureGain::auto_gain_max = root["camera"]["auto_exposure_gain"]["auto_gain_max"].asInt();
    AutoExposureGain::target_gray_min = root["camera"]["auto_exposure_gain"]["target_gray_min"].asInt();
    AutoExposureGain::target_gray_max = root["camera"]["auto_exposure_gain"]["target_gray_max"].asInt();

    ThreadDelay::pic_thread_delay = root["thread"]["pic_thread_delay"].asInt();
    ThreadDelay::serial_thread_delay = root["thread"]["serial_thread_delay"].asInt();
    ThreadDelay::detect_thread_delay = root["thread"]["detect_thread_delay"].asInt();

    WhiteBalanceParam::is_auto_balance = root["camera"]["white_balance"]["is_auto_balance"].asBool();
    WhiteBalanceParam::white_balance_ratio_r = root["camera"]["white_balance"]["white_balance_ratio_r"].asFloat();
    WhiteBalanceParam::white_balance_ratio_b = root["camera"]["white_balance"]["white_balance_ratio_b"].asFloat();
    WhiteBalanceParam::white_balance_ratio_g = root["camera"]["white_balance"]["white_balance_ratio_g"].asFloat();

    json_file.close();
}
