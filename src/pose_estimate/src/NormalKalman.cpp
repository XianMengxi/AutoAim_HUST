#include "NormalKalman.h"
namespace ly
{
    NormalKalman::NormalKalman(/* args */)
    {
        is_kalman_init = false;
        kalman_filter = new StrongTrackingFilter<double, 6, 3>();
        posteriori_pose = Eigen::Vector3d::Zero();
        posteriori_speed = Eigen::Vector3d::Zero();
        process_noice = Eigen::Matrix3d::Identity();
        process_noise_matrix = Eigen::Matrix<double, 6, 3>::Zero();
        kalman_filter->setIsUseSTF(FilterParams::is_use_stf); //可以试试使用
        kalman_filter->setBeta(FilterParams::stf_beta);
        setMeasureMatrix();
    }

    NormalKalman::~NormalKalman()
    {
    }
    void NormalKalman::setMeasureMatrix()
    {
        Eigen::Matrix<double, 1, 2> measure;
        measure << 1, 0;
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->measurement_matrix.block<1, 2>(i, 2 * i) = measure;
        }
    }
    void NormalKalman::rebootKalmanWithoutV(const Eigen::Vector3d &new_armor_pose)
    {
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->posteriori_state_estimate[i * 2] = new_armor_pose[i];

            posteriori_pose[i] = kalman_filter->posteriori_state_estimate[i * 2];
        }
        kalman_filter->posteriori_state_estimate[3] = -kalman_filter->posteriori_state_estimate[5]; //y方向速度设置为0，x,z速度方向不变
        posteriori_speed[1] = 0;
        kalman_filter->posteriori_state_estimate[5] = 0; //y方向速度设置为0，x,z速度方向不变
        posteriori_speed[2] = 0;

        kalman_filter->is_residual_matrix_init = false;
        //加速度参数清零
        is_get_first_speed = false;
        accelerate = Eigen::Vector3d(0, 0, 0); //加速度清零
        resetTransitionMatrix();
    }
    void NormalKalman::rebootKalman(const Eigen::Vector3d &new_armor_pose)
    {
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->posteriori_state_estimate[i * 2] = new_armor_pose[i];
            kalman_filter->posteriori_state_estimate[i * 2 + 1] = 0;

            posteriori_pose[i] = kalman_filter->posteriori_state_estimate[i * 2];
            posteriori_speed[i] = kalman_filter->posteriori_state_estimate[i * 2 + 1];
        }
        kalman_filter->is_residual_matrix_init = false;
        kalman_filter->error_cov_post = Eigen::Matrix<double, 6, 6>::Identity();

        //加速度参数清零
        is_get_first_speed = false;
        accelerate = Eigen::Vector3d(0, 0, 0); //加速度清零

        resetTransitionMatrix();
    }
    void NormalKalman::resetTransitionMatrix()
    {
        //x,x_v,y,y_v,z,z_v
        kalman_filter->transition_matrix = Eigen::Matrix<double, 6, 6>::Identity();
    }
    void NormalKalman::setUpdateTime(const double &delta_t)
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

    Eigen::Vector3d NormalKalman::runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t) //量测有效更新
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

            //update transition matrix
            setTransitionMatrix();

            setMeasurementNoise(new_armor_pose);

            return correct(new_armor_pose);
        }
    }
    Eigen::Vector3d NormalKalman::NoPosePredict(const double &delta_t)
    {
        //set update time
        setUpdateTime(delta_t);

        //update transition matrix
        setTransitionMatrix();

        kalman_filter->predict();
        for (int i = 0; i < 3; i++)
        {
            //update armor status and return
            posteriori_pose[i] = kalman_filter->prior_state_estimate[i * 2];
            posteriori_speed[i] = kalman_filter->prior_state_estimate[i * 2 + 1];
        }

        return posteriori_pose;
    }
    void NormalKalman::setTransitionMatrix()
    {
        Eigen::Matrix2d transition;
        transition << 1, update_time, 0, 1;
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->transition_matrix.block<2, 2>(i * 2, i * 2) = transition;
        }
    }
    bool NormalKalman::ChiSquaredTest(const Eigen::Vector3d &armor_pose) //armor_pose为测量值
    {
        //计算实际pitch和yaw
        double pitch = atan2(armor_pose[2], sqrt(armor_pose[0] * armor_pose[0] + armor_pose[1] * armor_pose[1]));
        double yaw = atan2(armor_pose[0], armor_pose[1]);

        Eigen::Vector2d z_k = Eigen::Vector2d(yaw, pitch);

        //计算预测pitch和yaw
        //线性检验
        Eigen::Matrix<double, 6, 1> x_speed_k = kalman_filter->prior_state_estimate;

        Eigen::Vector3d x_k = Eigen::Vector3d(x_speed_k[0], x_speed_k[2], x_speed_k[4]);
        double x_k_norm_pow = pow(x_k.norm(), 2);

        double distance = sqrt(x_k[0] * x_k[0] + x_k[1] * x_k[1]);
        //计算预测yaw和pitch
        double predict_pitch = atan2(x_k[2], distance);
        double predict_yaw = atan2(x_k[0], x_k[1]);
        Eigen::Vector2d f_xk = Eigen::Vector2d(predict_yaw, predict_pitch);

        Eigen::Matrix<double, 2, 6> H_k = Eigen::Matrix<double, 2, 6>::Zero();
        H_k(0, 0) = x_k[1] / (distance * distance);
        H_k(0, 2) = -x_k[0] / (distance * distance);
        H_k(1, 0) = -x_k[0] * x_k[2] / (distance * x_k_norm_pow);
        H_k(1, 2) = -x_k[1] * x_k[2] / (distance * x_k_norm_pow);
        H_k(1, 4) = distance / x_k_norm_pow;

        Eigen::Vector2d residual = z_k - f_xk;
        residual[0] = residual[0] - round(residual[0] / 2.0 / M_PI) * M_PI * 2;

        // std::cout << "H_k: " << H_k << std::endl;
        // std::cout << "x_speed_k" << x_speed_k << std::endl;

        // std::cout << "residual: " << residual << std::endl;
        // std::cout << "z_k: " << z_k << std::endl;
        // std::cout << "x_k: " << x_k << std::endl;
        // std::cout << "H_k * x_speed_k" << H_k * x_speed_k << std::endl;

        Eigen::Matrix2d yaw_pitch_measure_noise;
        yaw_pitch_measure_noise.diagonal() << 0.0009, 0.0001;
        Eigen::Matrix2d D_k = (H_k * kalman_filter->error_cov_post * H_k.transpose() + yaw_pitch_measure_noise).inverse();
        detect_param = residual.transpose() * D_k * residual;

        // std::cout << "detect_params: " << detect_param << std::endl;
        detect_param = residual.norm() * 100;

        return fabs(detect_param) > chi_squared_thresh;
    }
    bool NormalKalman::ChiSquaredTest()
    {
        //线性检验
        Eigen::Matrix2d measurement_cov_maxtrix_xy = kalman_filter->measurement_cov_maxtrix.block<2, 2>(0, 0);
        Eigen::Vector2d residual_xy = kalman_filter->residual.block<2, 1>(0, 0);

        detect_param = residual_xy.transpose() * measurement_cov_maxtrix_xy * residual_xy;
        return detect_param > chi_squared_thresh;
    }
    Eigen::Vector3d NormalKalman::correct(const Eigen::Vector3d &armor_pose)
    {
        kalman_filter->predict(armor_pose); //量测有效更新
        if (ChiSquaredTest())               //true则卡方检验不通过
        {
            DLOG(ERROR) << " CHI SQUARED TEST NOT PASS" << std::endl;

            //卡方检验标志位
            chi_squared_test_status = true;

            if (!is_antirot_mode)
            {
                rebootKalman(armor_pose); //重启卡尔曼滤波器
            }
            else
            {
                rebootKalmanWithoutV(armor_pose);
            }

            return armor_pose;
        }
        ChiSquaredTest(armor_pose);
        // if (ChiSquaredTest(armor_pose)) //true则卡方检验不通过
        // {
        //     DLOG(ERROR) << " CHI SQUARED TEST NOT PASS" << std::endl;

        //     //卡方检验标志位
        //     chi_squared_test_status = true;

        //     if (!is_antirot_mode)
        //     {
        //         rebootKalman(armor_pose); //重启卡尔曼滤波器
        //     }
        //     else
        //     {
        //         rebootKalmanWithoutV(armor_pose);
        //     }

        //     return armor_pose;
        // }
        kalman_filter->update();

        for (int i = 0; i < 3; i++)
        {
            //update armor status and return
            posteriori_pose[i] = kalman_filter->posteriori_state_estimate[i * 2];
            posteriori_speed[i] = kalman_filter->posteriori_state_estimate[i * 2 + 1];
        }
        return posteriori_pose;
    }
    void NormalKalman::setMeasurementNoise(const Eigen::Vector3d &armor_pose)
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
    void NormalKalman::setProcessNoise()
    {
        Eigen::Matrix<double, 2, 1> process_noice_vec;
        process_noice_vec << 0.5 * update_time * update_time, update_time;
        for (int i = 0; i < 3; i++)
        {
            process_noise_matrix.block<2, 1>(2 * i, i) = process_noice_vec;
        }
        // process_noice.diagonal() << FilterParams::process_noise_pose_x,
        //     FilterParams::process_noise_pose_y,
        //     FilterParams::process_noise_pose_z; //3个轴的过程噪声
        process_noice.diagonal() << 150,
            150,
            0.5; //3个轴的过程噪声
        kalman_filter->process_noise_cov = process_noise_matrix * process_noice * process_noise_matrix.transpose();
    }
    void NormalKalman::setProcessNoise(double x, double y, double z)
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
    Eigen::Vector3d NormalKalman::predict(const double &predict_t)
    {
        return posteriori_pose + posteriori_speed * predict_t;
    }
    Eigen::Vector3d NormalKalman::predict(const Eigen::Vector3d &pose, const double &predict_t)
    {
        return pose + posteriori_speed * predict_t;
    }

}