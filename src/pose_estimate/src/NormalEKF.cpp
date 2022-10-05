#include "NormalEKF.h"
#include <fstream>
namespace ly
{
    NormalEKF::NormalEKF(/* args */)
    {
        is_kalman_init = false;
        kalman_filter = new ExtendedKalman<double, 6, 3>();
        posteriori_pose = Eigen::Vector3d::Zero();
        posteriori_speed = Eigen::Vector3d::Zero();
        process_noice = Eigen::Matrix3d::Identity();
        process_noise_matrix = Eigen::Matrix<double, 6, 3>::Zero();
        setIsUseSTF(FilterParams::is_use_stf);
    }

    NormalEKF::~NormalEKF()
    {
    }
    void NormalEKF::setIsUseSTF(bool flag)
    {
        if (flag)
        {
            kalman_filter->is_use_stf = true;
            kalman_filter->is_residual_matrix_init = false;
            kalman_filter->setBeta(FilterParams::stf_beta);
            kalman_filter->setRho(0.95);
        }
        else
        {
            kalman_filter->is_use_stf = false;
        }
    }
    void NormalEKF::rebootKalman(const Eigen::Vector3d &new_armor_pose)
    {
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->posteriori_state_estimate[i * 2] = new_armor_pose[i];
            kalman_filter->posteriori_state_estimate[i * 2 + 1] = 0;

            posteriori_pose[i] = kalman_filter->posteriori_state_estimate[i * 2];
            posteriori_speed[i] = kalman_filter->posteriori_state_estimate[i * 2 + 1];
        }
        setIsUseSTF(FilterParams::is_use_stf); //如果开启了强跟踪的话，此时重启参数
        kalman_filter->error_cov_post = Eigen::Matrix<double, 6, 6>::Identity();

        resetTransitionMatrix();
    }
    void NormalEKF::rebootKalmanWithInitial(const Eigen::Vector3d &new_armor_pose)
    {

        if (rotation == ANTIROT_CLOCKWISE)
        {
            double x_v = -kalman_filter->posteriori_state_estimate[3];
            double y_v = kalman_filter->posteriori_state_estimate[1];

            kalman_filter->posteriori_state_estimate[1] = x_v;
            kalman_filter->posteriori_state_estimate[3] = y_v;
        }
        else
        {
            double x_v = kalman_filter->posteriori_state_estimate[3];
            double y_v = -kalman_filter->posteriori_state_estimate[1];

            kalman_filter->posteriori_state_estimate[1] = x_v;
            kalman_filter->posteriori_state_estimate[3] = y_v;
        }
        kalman_filter->posteriori_state_estimate[5] = 0;

        for (int i = 0; i < 3; i++)
        {
            kalman_filter->posteriori_state_estimate[i * 2] = new_armor_pose[i];
            posteriori_pose[i] = kalman_filter->posteriori_state_estimate[i * 2];

            posteriori_speed[i] = kalman_filter->posteriori_state_estimate[i * 2 + 1];
        }

        setIsUseSTF(FilterParams::is_use_stf); //如果开启了强跟踪的话，此时重启参数
        kalman_filter->error_cov_post = Eigen::Matrix<double, 6, 6>::Identity();

        resetTransitionMatrix();
    }
    void NormalEKF::resetTransitionMatrix()
    {
        //x,x_v,y,y_v,z,z_v
        kalman_filter->transition_matrix = Eigen::Matrix<double, 6, 6>::Identity();
    }
    void NormalEKF::setUpdateTime(const double &delta_t)
    {
        if (fabs(delta_t) < 1e-4) //防止时间差为0
        {
            update_time = 8.0 / 1000.0;
        }
        else
        {
            update_time = delta_t / 1000.0;
        }
    }

    Eigen::Vector3d NormalEKF::runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t) //量测有效更新
    {
        if (!is_kalman_init)
        {
            //set signal values
            is_kalman_init = true;

            //reset kalman
            rebootKalman(new_armor_pose);

            //return values
            return new_armor_pose;
        }
        else
        {
            //set update time
            setUpdateTime(delta_t);

            //update transition matrix
            setTransitionMatrix();

            return correct(new_armor_pose);
        }
    }
    void NormalEKF::setTransitionMatrix()
    {
        Eigen::Matrix2d transition;
        transition << 1, update_time, 0, 1;
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->transition_matrix.block<2, 2>(i * 2, i * 2) = transition;
        }
    }
    Eigen::Vector3d NormalEKF::correct(const Eigen::Vector3d &armor_pose)
    {
        setProcessNoise();

        setMeasurementNoise(armor_pose); //设置测量噪声
        Eigen::Vector3d pyd = measure(armor_pose);

        std::fstream ss("../pyd.txt", std::ios::app);
        ss << pyd << std::endl;
        ss.close();
        std::cout << "update_time:" << update_time << std::endl; 

        if (update_time > 0.3) //大于0.2s没有观测到数据，选择重启卡尔曼滤波
        {
            LOG(WARNING) << "RESET KALMAN DUE TO TIME";
            rebootKalman(armor_pose);
            return armor_pose;
        }

        kalman_filter->predict(xyz_to_pyd, pyd); //量测有效更新

        //检验
        // detect_param = pow(kalman_filter->residual[0], 2) + pow(kalman_filter->residual[1], 2);
        if (antirot_count > 0) //如果超过一段时间没有找到小陀螺的话，小陀螺系数清零
        {
            std::chrono::steady_clock::time_point this_get_armor_time_point = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(this_get_armor_time_point - last_antirot_time_point).count() > 1000)
            {
                change_armor_time = 0;
                //1s没有检测到装甲板切换，将小陀螺系数清零
                antirot_count = 0;
            }
        }
        detect_param = kalman_filter->ChiSquaredTest();

        /********************反小陀螺状态检测开始*******************/
        if (detect_param > VERIFY_THRESH) //检验失败
        {
            this_is_change_armor = true;
            std::cout << "CAHNGE ARMOR" << std::endl;
            antirot_count++;
            if (antirot_count >= 1)
            {
                std::chrono::steady_clock::time_point this_get_armor_time_point = std::chrono::steady_clock::now();
                change_armor_time = std::chrono::duration_cast<std::chrono::milliseconds>(this_get_armor_time_point - last_antirot_time_point).count() / 1000.0;
            }
            last_antirot_time_point = std::chrono::steady_clock::now(); //记录上一次切换时间

            if (change_armor_time > 0.1 && change_armor_time < 1) //时间有效
            {
                if (!is_get_aver_time)
                {
                    aver_rotate_time = change_armor_time;
                    is_get_aver_time = true;
                }
                else
                {
                    static double coef = 0.8;
                    aver_rotate_time = coef * aver_rotate_time + (1 - coef) * change_armor_time;
                }
                // aver_rotate_time = change_armor_time;

                double last_yaw = atan2(last_armor_pose[0], last_armor_pose[1]);
                double this_yaw = atan2(armor_pose[0], armor_pose[1]);

                double diff_yaw = this_yaw - last_yaw; //大于0说明顺时针旋转
                if ((diff_yaw > 0 && diff_yaw < M_PI / 2) || diff_yaw < -M_PI / 2)
                {
                    rotation = ANTIROT_CLOCKWISE; //顺时针
                }
                else
                {
                    rotation = ANTIROT_COUNTER_CLOCK_WISE;
                }
            }

            last_armor_pose = armor_pose;

            // if (is_antirot_state)
            // {
            //     rebootKalmanWithInitial(armor_pose);
            // }
            // else
            {
                rebootKalman(armor_pose);
            }
            return armor_pose;
        }
        else
        {
            this_is_change_armor = false;
        }

        if (antirot_count >= 3)
        {
            is_antirot_state = true;
            LOG(ERROR) << "ANTIROT STATE";
        }
        else
        {
            aver_rotate_time = 0; //平均时间清零
            is_antirot_state = false;
            is_get_aver_time = false;
            // LOG(ERROR) << "NORMAL STATE";
        }
        /********************反小陀螺状态检测结束*******************/

        kalman_filter->update();

        last_armor_pose = armor_pose; //更新上一帧的位姿

        for (int i = 0; i < 3; i++)
        {
            //update armor status and return
            posteriori_pose[i] = kalman_filter->posteriori_state_estimate[i * 2];
            posteriori_speed[i] = kalman_filter->posteriori_state_estimate[i * 2 + 1];
        }
        return posteriori_pose;
    }
    Eigen::Vector3d NormalEKF::measure(const Eigen::Vector3d &armor_pose)
    {
        Eigen::Vector3d pyd;
        pyd[2] = armor_pose.norm();
        pyd[0] = ceres::atan2(armor_pose[2], sqrt(armor_pose[0] * armor_pose[0] + armor_pose[1] * armor_pose[1])); //pitch
        pyd[1] = ceres::atan2(armor_pose[0], armor_pose[1]);
        return pyd;
    }
    void NormalEKF::setMeasurementNoise(const Eigen::Vector3d &armor_pose)
    {
        //pitch,yaw,distance的噪声
        double measurement_noise_pose_pitch = 0.0001;
        double measurement_noise_pose_yaw = 0.0001;

        double distance = armor_pose.norm();
        double measurement_noise_pose_distance;

        //4号车
        // if (distance < 1.5) //统计方法计算，分段线性
        // {
        //     measurement_noise_pose_distance = pow(distance * 0.01, 2);
        // }
        // else if (distance < 4.5)
        // {
        //     measurement_noise_pose_distance = pow(0.015 + 0.058 * (distance - 1.5), 2);
        // }
        // else
        // {
        //     measurement_noise_pose_distance = pow(0.189 + 0.03 * (distance - 4.5), 2);
        // }
        //3号车,暂时保持和四号车一致
        if (distance < 1.5) //统计方法计算，分段线性
        {
            measurement_noise_pose_distance = pow(distance * 0.01, 2);
        }
        else if (distance < 4.5)
        {
            measurement_noise_pose_distance = pow(0.015 + 0.058 * (distance - 1.5), 2);
        }
        else
        {
            measurement_noise_pose_distance = pow(0.189 + 0.03 * (distance - 4.5), 2);
        }

        kalman_filter->measurement_noise_cov.diagonal() << measurement_noise_pose_pitch,
            measurement_noise_pose_yaw,
            measurement_noise_pose_distance; //3个轴的测量噪声，感觉三个轴的噪声需要根据PNP距离来计算
    }

    void NormalEKF::setProcessNoise()
    {
        Eigen::Matrix<double, 2, 1> process_noice_vec;
        process_noice_vec << 0.5 * update_time * update_time, update_time;
        for (int i = 0; i < 3; i++)
        {
            process_noise_matrix.block<2, 1>(2 * i, i) = process_noice_vec;
        }

        process_noice.diagonal() << FilterParams::process_noise_pose_x,
            FilterParams::process_noise_pose_y,
            FilterParams::process_noise_pose_z; //3个轴的过程噪声
        kalman_filter->process_noise_cov = process_noise_matrix * process_noice * process_noise_matrix.transpose();
    }
    void NormalEKF::setProcessNoise(double x, double y, double z)
    {
        Eigen::Matrix<double, 2, 1> process_noice_vec;
        process_noice_vec << 0.5 * update_time * update_time, update_time;
        for (int i = 0; i < 3; i++)
        {
            process_noise_matrix.block<2, 1>(2 * i, i) = process_noice_vec;
        }
        process_noice.diagonal() << x, y, z;

        kalman_filter->process_noise_cov = process_noise_matrix * process_noice * process_noise_matrix.transpose();
    }
    Eigen::Vector3d NormalEKF::predict(const double &predict_t)
    {
        // if (is_antirot_state)
        // {
        //     //小陀螺状态限制速度
        //     double speed = sqrt(posteriori_speed[0] * posteriori_speed[0] + posteriori_speed[1] * posteriori_speed[1]);
        //     double max_speed = M_PI / 2 / aver_rotate_time * 0.1;
        //     LOG(WARNING) << "RATIO: " << speed / max_speed;

        //     double ratio = speed / max_speed > 1 ? 1 : speed / max_speed;
        //     posteriori_speed[0] = ratio * posteriori_speed[0];
        //     posteriori_speed[1] = ratio * posteriori_speed[1];
        // }
        // if (is_antirot_state)
        // {
        //     // double k = 1;
        //     // double ratio = 1 - predict_t * k;
        //     // posteriori_speed[0] = ratio * posteriori_speed[0];
        //     // posteriori_speed[1] = ratio * posteriori_speed[1];
        //     // //对距离作速度衰减
        //     double distance = posteriori_pose.norm();
        //     double raw_angle = atan2(0.2 * sin(M_PI / 2 / aver_rotate_time * predict_t), distance);
        //     double predict_angle = atan2(posteriori_speed[0] * predict_t, distance);
        //     LOG(WARNING) << "DIFF ANGLE" << fabs(predict_angle - raw_angle) * 180 / M_PI;
        //     // LOG(WARNING) << "DISTANCE: " << distance;
        //     // LOG(WARNING) << "ROTATE SPEED " << M_PI / 2 / aver_rotate_time;
        // }
        // double time = predict_t;
        // if (is_antirot_state)
        // {
        //     time = predict_t * FilterParams::measurement_noise_pose_z;
        // }

        return posteriori_pose + posteriori_speed * predict_t;
    }

}