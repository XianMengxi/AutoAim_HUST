#ifndef STRONG_TRAKING_FILTER
#define STRONG_TRAKING_FILTER

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "Log.h"
//此类默认输入输出维度相同
template <typename T, int x, int y> //数据类型以及x滤波量，y测量量
class StrongTrackingFilter
{
private:
    Eigen::Matrix<T, x, y> kalman_gain; //卡尔曼增益

    Eigen::Matrix<T, y, 1> measurement; //测量值z_k

    Eigen::Matrix<T, y, y> residual_matrix; // 残差矩阵S

    Eigen::Matrix<T, y, y> M_matrix; //矩阵M,N,用来求lambda
    Eigen::Matrix<T, y, y> N_matrix;

    T rho_coef; //遗忘因子rho
    T lambda_coef;
    T beta_coef; //具体需要根据系统而定
    bool is_use_stf = false;

public:
    Eigen::Matrix<T, x, x> transition_matrix;         //A:状态转移矩阵
    Eigen::Matrix<T, x, 1> posteriori_state_estimate; //后验估计
    Eigen::Matrix<T, x, x> error_cov_post;            //状态估计协方差矩阵 P_k
    Eigen::Matrix<T, x, x> process_noise_cov;         //过程噪声矩阵：Q
    Eigen::Matrix<T, y, y> measurement_noise_cov;     //测量噪声矩阵 : R
    Eigen::Matrix<T, x, 1> control_vector;            //u_k：控制向量
    Eigen::Matrix<T, x, x> control_matrix;            // B： 控制矩阵
    Eigen::Matrix<T, y, x> measurement_matrix;        //测量矩阵 ： H
    Eigen::Matrix<T, x, 1> prior_state_estimate;      //先验估计

    bool is_residual_matrix_init;
    Eigen::Matrix<T, y, y> measurement_cov_maxtrix;

    Eigen::Matrix<T, y, 1> residual; //残差向量

    double ChiSquaredTest_THRESH = 10;
    double detect_param = 0;

    StrongTrackingFilter() //外部需要初始化后验估计
    {
        is_residual_matrix_init = false;
        rho_coef = 0.85;
        beta_coef = 1;
        error_cov_post = Eigen::Matrix<T, x, x>::Identity();
        control_vector = Eigen::Matrix<T, x, 1>::Zero();
        control_matrix = Eigen::Matrix<T, x, x>::Zero();
        measurement_matrix = Eigen::Matrix<T, y, x>::Zero();
        process_noise_cov = Eigen::Matrix<T, x, x>::Identity() * 0.001; //表明对预测过程的相信程度
        measurement_noise_cov = Eigen::Matrix<T, y, y>::Identity();     //证明对测量过程的相信程度
        posteriori_state_estimate = Eigen::Matrix<T, x, 1>::Zero();     //后验估计由外部进行初始化
        measurement_cov_maxtrix = Eigen::Matrix<T, y, y>::Zero();
    }
    void setBeta(const float &set_beta)
    {
        beta_coef = set_beta;
    }
    void setIsUseSTF(bool flag)
    {
        is_use_stf = flag;
    }
    ~StrongTrackingFilter()
    {
    }
    Eigen::Matrix<T, x, 1> update(const Eigen::Matrix<T, y, 1> &measure_vector)
    {
    }
    Eigen::Matrix<T, x, 1> predict() //测量无效的更新(测量无效的逻辑需要另外写)
    {
        //预测过程，更新过程协方差矩阵和先验估计
        prior_state_estimate = transition_matrix * posteriori_state_estimate + control_matrix * control_vector;

        error_cov_post = lambda_coef * transition_matrix * error_cov_post * transition_matrix.transpose() + process_noise_cov;

        return prior_state_estimate;
    }
    Eigen::Matrix<T, x, 1> predict(const Eigen::Matrix<T, y, 1> &measure_vector) //测量正确的预测，带残差，方便卡方检验
    {
        //先验估计计算
        prior_state_estimate = transition_matrix * posteriori_state_estimate + control_matrix * control_vector;

        measurement = measurement_matrix * prior_state_estimate; //计算测量值

        residual = measure_vector - measurement; //计算残差

        if (is_use_stf)
        {
            // // //残差矩阵初始化
            if (!is_residual_matrix_init)
            {
                residual_matrix = residual * residual.transpose();
                is_residual_matrix_init = true;
            }
            else //残差矩阵迭代计算
            {
                residual_matrix = (rho_coef * residual_matrix + residual * residual.transpose()) / (1 + rho_coef);
            }

            // //计算N矩阵
            N_matrix = residual_matrix - measurement_matrix * process_noise_cov * measurement_matrix.transpose() - beta_coef * measurement_noise_cov;

            //计算M矩阵，测量方差矩阵
            M_matrix = measurement_matrix * transition_matrix * error_cov_post * transition_matrix.transpose() * measurement_matrix.transpose();

            lambda_coef = N_matrix.trace() / M_matrix.trace(); //三个轴的残差变化不一致，这样用的话就将三个轴关联了起来

            if (lambda_coef > 1)
            {
                LOG(ERROR) << "STF RUN";
            }
            lambda_coef = lambda_coef > 1 ? lambda_coef : 1;
        }
        else
        {
            lambda_coef = 1;
        }

        // //计算状态估计协方差矩阵
        error_cov_post = lambda_coef * transition_matrix * error_cov_post * transition_matrix.transpose() + process_noise_cov;

        measurement_cov_maxtrix = (measurement_matrix * error_cov_post * measurement_matrix.transpose() + measurement_noise_cov).inverse();
    }
    Eigen::Matrix<T, x, 1> update()
    {
        // // //计算卡尔曼增益
        kalman_gain = error_cov_post * measurement_matrix.transpose() * measurement_cov_maxtrix;

        error_cov_post = (Eigen::Matrix<T, x, x>::Identity() - kalman_gain * measurement_matrix) * error_cov_post;

        posteriori_state_estimate = prior_state_estimate + kalman_gain * residual;

        return posteriori_state_estimate;
    }
    bool ChiSquaredTest() //小于阈值，表明卡方检验通过，测量数据有效
    {
        detect_param = residual.transpose() * measurement_cov_maxtrix * residual;

        // std::cout << "CHI SQUARED TEST" << detect_param << std::endl;
        return detect_param < ChiSquaredTest_THRESH;
    }
};
#endif
