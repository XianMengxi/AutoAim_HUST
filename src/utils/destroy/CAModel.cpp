#include "CAModel.h"

namespace ly
{
    CAModel::CAModel(/* args */)
    {
        is_kalman_init = false;
        kalman_filter = new StrongTrackingFilter<double, 9, 3>();
        posteriori_pose = Eigen::Vector3d::Zero();
        posteriori_speed = Eigen::Vector3d::Zero();
        posteriori_accelerate = Eigen::Vector3d::Zero();
        process_noice = Eigen::Matrix3d::Identity();
        setMeasureMatrix();
        setMeasurementNoise();
    }

    CAModel::~CAModel()
    {
    }
    void CAModel::setMeasureMatrix()
    {
        Eigen::Matrix<double, 1, 3> measure;
        measure << 1, 0, 0;
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->measurement_matrix.block<1, 3>(i, 3 * i) = measure;
        }
    }
    void CAModel::rebootKalman(const Eigen::Vector3d &new_armor_pose)
    {
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->posteriori_state_estimate[i * 3] = new_armor_pose[i];
            kalman_filter->posteriori_state_estimate[i * 3 + 1] = 0;
            kalman_filter->posteriori_state_estimate[i * 3 + 2] = 0;
        }
        kalman_filter->error_cov_post = Eigen::Matrix<double, 9, 9>::Identity();
        resetTransitionMatrix();
    }
    void CAModel::resetTransitionMatrix()
    {
        //x,x_v,y,y_v,z,z_v
        kalman_filter->transition_matrix = Eigen::Matrix<double, 9, 9>::Identity();
    }
    void CAModel::setUpdateTime(const double &delta_t)
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

    Eigen::Vector3d CAModel::runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t)
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

            setMeasurementNoise(new_armor_pose);

            setProcessNoise();

            return correct(new_armor_pose);
        }
    }
    void CAModel::setTransitionMatrix()
    {
        Eigen::Matrix3d transition;
        transition << 1, update_time, update_time * update_time / 2,
            0, 1, update_time,
            0, 0, 1;
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->transition_matrix.block<3, 3>(i * 3, i * 3) = transition;
        }
    }
    Eigen::Vector3d CAModel::correct(const Eigen::Vector3d &armor_pose)
    {
        kalman_filter->predict(armor_pose); //量测有效更新
        // if (!kalman_filter->ChiSquaredTest()) //true则卡方检验通过
        // {
        //     std::cout << " CHI SQUARED TEST NOT PASS" << std::endl;

        //     rebootKalman(armor_pose); //重启卡尔曼滤波器
        //     return armor_pose;
        // }
        kalman_filter->update();
        for (int i = 0; i < 3; i++)
        {
            //update armor status and return
            posteriori_pose[i] = kalman_filter->posteriori_state_estimate[i * 3];
            posteriori_speed[i] = kalman_filter->posteriori_state_estimate[i * 3 + 1];
            posteriori_accelerate[i] = kalman_filter->posteriori_state_estimate[i * 3 + 2];
        }
        return posteriori_pose;
    }
    Eigen::Vector3d CAModel::NoPosePredict(const double &delta_t)
    {
        //set update time
        setUpdateTime(delta_t);

        //update transition matrix
        setTransitionMatrix();

        setProcessNoise();

        Eigen::Matrix<double, 9, 1> predict_state = kalman_filter->predict();

        for (int i = 0; i < 3; i++)
        {
            //update armor status and return
            posteriori_pose[i] = predict_state[i * 2];
            posteriori_speed[i] = predict_state[i * 2 + 1];
            posteriori_accelerate[i] = predict_state[i * 3 + 2];
        }
        return posteriori_pose;
    }
    void CAModel::setMeasurementNoise()
    {
        kalman_filter->measurement_noise_cov.diagonal() << 0.0675,
            0.0675,
            0.0675; //3个轴的测量噪声
    }
    void CAModel::setMeasurementNoise(const Eigen::Vector3d &armor_pose)
    {
        //根据小孔成像原理，测得的距离抖动应该与距离成正比(不过多了一个坐标系转换，这里近似处理)
        //至少有3cm的测量误差,大概测得的系数为1/20
        //误差可以统计得到，3*sigma^2
        double measurement_noise_pose_x = armor_pose[0] * armor_pose[0] * 0.0075 + 0.0027;
        double measurement_noise_pose_y = armor_pose[1] * armor_pose[1] * 0.0075 + 0.0027;
        double measurement_noise_pose_z = armor_pose[2] * armor_pose[2] * 0.0075 + 0.0027;

        kalman_filter->measurement_noise_cov.diagonal() << measurement_noise_pose_x,
            measurement_noise_pose_y,
            measurement_noise_pose_z; //3个轴的测量噪声，感觉三个轴的噪声需要根据PNP距离来计算
    }
    void CAModel::setProcessNoise()
    {
        const double process_noise_xyz[3] = {50,
                                             50,
                                             0.5};
        Eigen::Matrix3d process_noise_time_related; //时间相关的噪声矩阵
        double T = update_time;
        double T_2 = update_time * update_time;
        double T_3 = update_time * T_2;
        double T_4 = update_time * T_3;
        double T_5 = update_time * T_4;
        process_noise_time_related << T_5 / 20, T_4 / 8, T_3 / 6,
            T_4 / 8, T_3 / 3, T_2 / 2,
            T_3 / 6, T_2 / 2, T;
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->process_noise_cov.block<3, 3>(3 * i, 3 * i) = process_noise_time_related * process_noise_xyz[i];
        }
    }
    Eigen::Vector3d CAModel::predict(const double &predict_t)
    {
        return posteriori_pose + posteriori_speed * predict_t + 0.5 * predict_t * predict_t * posteriori_accelerate;
    }
}