#include "CTModel.h"
namespace ly
{
    CTModel::CTModel(/* args */)
    {
        is_kalman_init = false;
        kalman_filter = new AdaptiveEKF<6, 3>();
        posteriori_pose = Eigen::Vector3d::Zero();
        posteriori_speed = Eigen::Vector3d::Zero();
        process_noice = Eigen::Matrix3d::Identity();
        process_noise_matrix = Eigen::Matrix<double, 6, 3>::Zero();
    }

    CTModel::~CTModel()
    {
    }
    void CTModel::rebootKalman(const Eigen::Vector3d &new_armor_pose)
    {
        for (int i = 0; i < 2; i++)
        {
            kalman_filter->Xe[i * 2] = new_armor_pose[i];
        }
        kalman_filter->Xe[5] = new_armor_pose[2];

        if (kalman_filter->Xe[4] > 0) //速度重置
        {
            double x_v = kalman_filter->Xe[3];
            double y_v = -kalman_filter->Xe[1];

            kalman_filter->Xe[1] = x_v;
            kalman_filter->Xe[3] = y_v;
        }
        else
        {
            double x_v = -kalman_filter->Xe[3];
            double y_v = kalman_filter->Xe[1];

            kalman_filter->Xe[1] = x_v;
            kalman_filter->Xe[3] = y_v;
        }

        kalman_filter->P = Eigen::Matrix<double, 6, 6>::Identity();
    }
    void CTModel::setUpdateTime(const double &delta_t)
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

    Eigen::Vector3d CTModel::runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t) //量测有效更新
    {
        if (!is_kalman_init)
        {
            //set signal values
            is_kalman_init = true;

            //reset kalman
            rebootKalman(new_armor_pose);
            setMeasurementNoise(new_armor_pose);

            //return values
            return new_armor_pose;
        }
        else
        {
            //set update time
            setUpdateTime(delta_t);

            setMeasurementNoise(new_armor_pose);

            setProcessNoise();

            return correct(new_armor_pose);
        }
    }
    Eigen::Vector3d Xyz2Pyd(const Eigen::Vector3d &pose)
    {
        Eigen::Vector3d pyd;
        pyd[0] = ceres::atan2(pose[2], sqrt(pose[0] * pose[0] + pose[1] * pose[1])); // pitch
        pyd[1] = ceres::atan2(pose[0], pose[1]);                                     // yaw
        pyd[2] = pose.norm();                                                        //distance
        return pyd;
    }

    Eigen::Vector3d CTModel::correct(const Eigen::Vector3d &armor_pose)
    {
        if (fabs(kalman_filter->Xe[4]) < 1e-6)
        {
            kalman_filter->Xe[4] = 1e-6;
        }
        kalman_filter->predict(predict_fun, update_time);
        Eigen::Vector3d armor_pose_rotate = Xyz2Pyd(armor_pose);
        kalman_filter->update(measure_fun, armor_pose_rotate);

        // if (kalman_filter->ChiSquaredTest() > CHI_REST_THRESH) //卡方检验失败，说明装甲板切换了
        // {
        //     LOG(WARNING) << "REBOOT ROTATION KALMAN";
        //     rebootKalman(armor_pose); //卡尔曼滤波重启
        // }
        //以新的位置，角速度不变来估计敌方新位置和速度

        for (int i = 0; i < 2; i++)
        {
            //update armor status and return
            posteriori_pose[i] = kalman_filter->Xe[i * 2];
            posteriori_speed[i] = kalman_filter->Xe[i * 2 + 1];
        }
        posteriori_pose[2] = kalman_filter->Xe[5];
        rotate_speed = kalman_filter->Xe[4];
        std::cout << "w:" << rotate_speed << std::endl;

        return posteriori_pose;
    }
    void CTModel::setMeasurementNoise(const Eigen::Vector3d &armor_pose)
    {
        double measurement_noise_pose_p = 0.0001;
        double measurement_noise_pose_y = 0.0001;
        double measurement_noise_pose_d;
        double distance = armor_pose.norm();
        if (distance < 1.5) //统计方法计算，分段线性
        {
            measurement_noise_pose_d = pow(distance * 0.01, 2);
        }
        else if (distance < 4.5)
        {
            measurement_noise_pose_d = pow(0.015 + 0.058 * (distance - 1.5), 2);
        }
        else
        {
            measurement_noise_pose_d = pow(0.189 + 0.03 * (distance - 4.5), 2);
        }

        kalman_filter->R.diagonal() << measurement_noise_pose_p,
            measurement_noise_pose_y, measurement_noise_pose_d;
    }
    void CTModel::setProcessNoise()
    {
        double coef = FilterParams::measurement_noise_pose_x;   //需要调节的系数
        double z_coef = FilterParams::measurement_noise_pose_y; //需要调节的系数
        Eigen::Matrix<double, 2, 2> process_noice_vec;
        process_noice_vec << pow(update_time, 4) / 4, pow(update_time, 3) / 2, pow(update_time, 3) / 2, update_time * update_time;

        kalman_filter->Q.block<2, 2>(0, 0) = coef * process_noice_vec;
        kalman_filter->Q.block<2, 2>(2, 2) = coef * process_noice_vec;
        kalman_filter->Q(4, 4) = coef * update_time * update_time;
        kalman_filter->Q(5, 5) = z_coef * update_time; //过程噪声与时间有关
    }
    Eigen::Vector3d CTModel::predict(const double &predict_t)
    {
        Eigen::Matrix<double, 6, 1> Xe = kalman_filter->Xe;
        Eigen::Vector3d target_pos;
        target_pos[0] = Xe[0] + sin(Xe[4] * predict_t) / Xe[4] * Xe[1] - (1 - cos(Xe[4] * predict_t)) / Xe[4] * Xe[3];
        target_pos[1] = Xe[2] + (1 - cos(Xe[4] * predict_t)) / Xe[4] * Xe[1] + sin(Xe[4] * predict_t) / Xe[4] * Xe[3];
        target_pos[2] = Xe[5];

        return target_pos;
    }
}