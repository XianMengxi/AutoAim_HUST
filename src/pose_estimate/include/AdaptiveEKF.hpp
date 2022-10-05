//
// Created by xinyang on 2021/3/15.
//

// 参考自上交2021开源

#ifndef _ADAPTIVE_EKF_HPP_
#define _ADAPTIVE_EKF_HPP_

#include "ceres/jet.h"
#include "eigen3/Eigen/Dense"
#include <vector>
template <int N_X, int N_Y>
class AdaptiveEKF
{
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
    using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY = Eigen::Matrix<double, N_Y, 1>;

public:
    explicit AdaptiveEKF(const VectorX &X0 = VectorX::Zero()) //初始状态向量
        : Xe(X0), P(MatrixXX::Identity()), Q(MatrixXX::Identity()), R(MatrixYY::Identity())
    {
    }
    void resetMatrix()
    {
        P = MatrixXX::Identity();
        Q = MatrixXX::Identity();
        R = MatrixYY::Identity();
    }
    template <class Func>
    VectorX predict(Func &&func, const float &delta_t) //根据上一帧的估计值直接利用函数进行计算
    {
        ceres::Jet<double, N_X> Xe_auto_jet[N_X];
        for (int i = 0; i < N_X; i++)
        {
            Xe_auto_jet[i].a = Xe[i];
            Xe_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        func(Xe_auto_jet, Xp_auto_jet, delta_t);
        for (int i = 0; i < N_X; i++)
        {
            Xp[i] = Xp_auto_jet[i].a;
            F.block(i, 0, 1, N_X) = Xp_auto_jet[i].v.transpose();
        }
        P = F * P * F.transpose() + Q;
        return Xp;
    }

    template <class Func>
    VectorX update(Func &&func, const VectorY &Y) //根据实际值计算,并更新
    {
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        for (int i = 0; i < N_X; i++)
        {
            Xp_auto_jet[i].a = Xp[i];
            Xp_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Yp_auto_jet[N_Y];
        func(Xp_auto_jet, Yp_auto_jet);
        for (int i = 0; i < N_Y; i++)
        {
            Yp[i] = Yp_auto_jet[i].a;
            H.block(i, 0, 1, N_X) = Yp_auto_jet[i].v.transpose();
        }
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        VectorY res = (Y - Yp);

        if (fabs(res[1]) > 1.5 * M_PI) //yaw轴残差矫正
        {
            if (res[1] > 0)
            {
                res[1] -= 2 * M_PI;
            }
            else
            {
                res[1] += 2 * M_PI;
            }
        }
        Xe = Xp + K * res;
        // std::cout << "res: " << (Y - Yp).norm() << std::endl;
        P = (MatrixXX::Identity() - K * H) * P;
        return Xe;
    }
    double ChiSquaredTest()
    {
        Eigen::Matrix3d measurement_cov_maxtrix = (H * P * H.transpose() + R).inverse();
        Eigen::Vector2d residual_xy = Eigen::Vector2d(Res[0], Res[1]);
        Eigen::Matrix2d measurement_cov_maxtrix_xy;
        measurement_cov_maxtrix_xy << measurement_cov_maxtrix(0, 0), measurement_cov_maxtrix(0, 1),
            measurement_cov_maxtrix(1, 0), measurement_cov_maxtrix(1, 1);

        return residual_xy.transpose() * measurement_cov_maxtrix_xy * residual_xy;
    }

    VectorX Xe; // 估计状态变量
    VectorX Xp; // 预测状态变量
    MatrixXX F; // 预测雅克比
    MatrixYX H; // 观测雅克比
    MatrixXX P; // 状态协方差
    MatrixXX Q; // 预测过程协方差
    MatrixYY R; // 观测过程协方差
    MatrixXY K; // 卡尔曼增益
    VectorY Yp; // 预测观测量
    VectorY Res;
};

#endif /* _ADAPTIVE_EKF_HPP_ */
