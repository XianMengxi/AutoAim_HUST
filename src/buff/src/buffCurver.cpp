#include "buffCurver.h"

namespace ly
{
    BuffCurver::BuffCurver(/* args */)
    {
    }
    void BuffCurver::addParams(const Eigen::Vector3d &armor_pose)
    {
        static int i = 0;
        i %= 16;
        if (armor_pose_size < 16)
        {
            armor_pose_size++;
        }
        armor_pose_set[i] = armor_pose;
        i++;
    }
    const bool &BuffCurver::isModelStart()
    {
        return is_model_start;
    }
    const double *BuffCurver::solve()
    {
        if (armor_pose_size < 5)
        {
            is_model_start = false;
            return estimate_params;
        }
        else
        {
            is_model_start = true;
        }
        ceres::Problem problem;
        for (int i = 0; i < armor_pose_size; i++)
        {
            problem.AddResidualBlock( // 向问题中添加误差项
                                      // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<CICLE_FITTING_COST, 1, 4>(
                    new CICLE_FITTING_COST(armor_pose_set[i][0], armor_pose_set[i][1], armor_pose_set[i][2])),
                new ceres::CauchyLoss(0.5), // 核函数，这里不使用，为空
                estimate_params             // 待估计参数
            );
        }
        ceres::Solve(options, &problem, &summary); // 开始优化

        return estimate_params;
    }
    const double *BuffCurver::fitCos(const std::vector<float> &angle, const std::vector<float> &t)
    {
        //新大符
        ceres::Problem problem;
        problem.AddParameterBlock(estimate_cos_params, 3); //添加优化参数

        //设置a范围
        problem.SetParameterLowerBound(estimate_cos_params, 0, 0.780);
        problem.SetParameterUpperBound(estimate_cos_params, 0, 1.045);

        //设置w范围
        problem.SetParameterLowerBound(estimate_cos_params, 1, 1.884);
        problem.SetParameterUpperBound(estimate_cos_params, 1, 2.000);

        // problem.SetParameterLowerBound(estimate_cos_params, 2, 0);
        // problem.SetParameterUpperBound(estimate_cos_params, 2, 2 * M_PI);
        ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();
        ordering->AddElementToGroup(estimate_cos_params + 1, 0);
        ordering->AddElementToGroup(estimate_cos_params + 2, 1);
        ordering->AddElementToGroup(estimate_cos_params + 0, 2);

        for (int i = 0; i < t.size(); i++)
        {
            problem.AddResidualBlock(
                // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<SineFitFunction, 1, 3>(
                    new SineFitFunction(t[i], angle[i])),
                new ceres::CauchyLoss(0.5),
                estimate_cos_params // 待估计参数
            );
        }
        // 配置求解器
        ceres::Solver::Options options;               // 这里有很多配置项可以填
        options.linear_solver_type = ceres::DENSE_QR; // 增量方程如何求解
        options.minimizer_progress_to_stdout = true;  // 输出到cout
        options.max_num_iterations = 50;              //迭代次数
        options.linear_solver_ordering.reset(ordering);

        ceres::Solver::Summary summary;            // 优化信息
        ceres::Solve(options, &problem, &summary); // 开始优化

        // 原大符
        // ceres::Problem problem;
        // for (int i = 0; i < angle.size(); i++)
        // {
        //     problem.AddResidualBlock( // 向问题中添加误差项
        //                               // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
        //         new ceres::AutoDiffCostFunction<COS_FITTING_COST, 1, 2>(
        //             new COS_FITTING_COST(angle[i], t[i])),
        //         nullptr,            // 核函数，这里不使用，为空
        //         estimate_cos_params // 待估计参数
        //     );
        // }
        // ceres::Solve(options, &problem, &summary); // 开始优化

        return estimate_cos_params;
    }
    const double *BuffCurver::solvePlane()
    {
        if (!is_model_start)
        {
            return estimate_plane_params;
        }
        ceres::Problem problem;
        for (int i = 0; i < armor_pose_size; i++)
        {
            problem.AddResidualBlock( // 向问题中添加误差项
                                      // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<PLANE_FITTING_COST, 1, 3>(
                    new PLANE_FITTING_COST(armor_pose_set[i][0], armor_pose_set[i][1], armor_pose_set[i][2])),
                nullptr,              // 核函数，这里不使用，为空
                estimate_plane_params // 待估计参数
            );
        }
        ceres::Solve(options, &problem, &summary); // 开始优化
    }

    BuffCurver::~BuffCurver()
    {
    }
    void BuffCurver::correctAxis(BuffTrajectory &buff_traj, const std::vector<Eigen::Vector3d> &armor_points, const std::vector<float> &angle_points)
    {
        int axis_score[2][2] = {0}; //0:x,1:y 0:原，1:反
        Eigen::Vector3d axis_set[2][2] = {buff_traj.x_axis, -buff_traj.x_axis, buff_traj.y_axis, -buff_traj.y_axis};
        int index_x, index_y;
        for (int k = 0; k < armor_points.size(); k++)
        {
            index_x = 0;
            index_y = 0;
            double min_dis = 999.0;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    Eigen::Vector3d predict_point = buff_traj.center + axis_set[0][i] * buff_traj.radius * cos(angle_points[k]) + axis_set[1][j] * buff_traj.radius * sin(angle_points[k]);
                    if ((predict_point - armor_points[k]).norm() < min_dis)
                    {
                        index_x = i;
                        index_y = j;
                        min_dis = (predict_point - armor_points[k]).norm();
                    }
                }
            }
            axis_score[index_x][index_y] += 1;
        }
        if (axis_score[0][0] < axis_score[0][1])
        {
            buff_traj.x_axis *= -1;
        }
        if (axis_score[1][0] < axis_score[1][1])
        {
            buff_traj.y_axis *= -1;
        }
    }
    const BuffTrajectory &BuffCurver::fitCircle(const std::vector<Eigen::Vector3d> &armor_points)
    {
        if (armor_points.size() != CURVE_FIT_SIZE)
        {
            return buff_trajectory;
        }

        Eigen::Matrix<double, CURVE_FIT_SIZE, 3> M;

        for (int i = 0; i < CURVE_FIT_SIZE; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                M(i, j) = armor_points[i][j];
            }
        }

        Eigen::Matrix<double, CURVE_FIT_SIZE, 1> L1 = Eigen::Matrix<double, CURVE_FIT_SIZE, 1>::Ones();

        Eigen::Vector3d A = (M.transpose() * M).inverse() * M.transpose() * L1; //平面法向量

        //计算两点之间的向量
        Eigen::Matrix<double, (CURVE_FIT_SIZE - 1) * CURVE_FIT_SIZE / 2, 3> B;
        int count = 0;
        for (int i = 0; i < CURVE_FIT_SIZE - 1; i++)
        {
            for (int j = i + 1; j < CURVE_FIT_SIZE; j++)
            {
                B.row(count) = M.row(j) - M.row(i);
                count++;
            }
        }

        // //求取两两平方差
        count = 0;
        Eigen::Matrix<double, (CURVE_FIT_SIZE - 1) * CURVE_FIT_SIZE / 2, 1> L2;
        for (int i = 0; i < CURVE_FIT_SIZE - 1; i++)
        {
            for (int j = i + 1; j < CURVE_FIT_SIZE; j++)
            {
                L2(count, 0) = (pow(M.row(j).norm(), 2) - pow(M.row(i).norm(), 2)) / 2;
                count++;
            }
        }

        // Eigen::Matrix<double, 3, 3> matrix = B.transpose() * B;
        //get D Matrix
        Eigen::Matrix<double, 4, 4> D;

        D << B.transpose() * B, A, A.transpose(), 0;

        //get L3 Matrix
        Eigen::Vector4d L3;
        L3 << B.transpose() * L2, 1;

        // //式（7）
        Eigen::Vector4d C = D.transpose().inverse() * L3;

        Eigen::Vector3d center(C[0], C[1], C[2]);

        // // //式（8）
        float radius = 0;
        for (int i = 0; i < CURVE_FIT_SIZE; i++)
        {
            radius += (armor_points[i] - center).norm();
        }
        radius = radius / CURVE_FIT_SIZE;

        Eigen::Vector3d y_axis = A.cross(Eigen::Vector3d(1, 0, 0));
        Eigen::Vector3d x_axis = A.cross(y_axis);

        x_axis = x_axis / x_axis.norm();
        y_axis = y_axis / y_axis.norm();

        buff_trajectory.radius = radius;
        buff_trajectory.center = center;
        buff_trajectory.x_axis = x_axis;
        buff_trajectory.y_axis = y_axis;

        if (buff_trajectory.x_axis[0] < 0)
        {
            buff_trajectory.x_axis = -buff_trajectory.x_axis;
        }
        if (buff_trajectory.y_axis[2] < 0)
        {
            buff_trajectory.y_axis = -buff_trajectory.y_axis;
        }
        return buff_trajectory;
    }
    void BuffCurver::correctPoints(std::vector<Eigen::Vector3d> &armor_points)
    {
        if (armor_points.size() <= 0)
        {
            return;
        }
        float depth_aver = 0.0f;
        std::for_each(armor_points.begin(), armor_points.end(), [&](const Eigen::Vector3d &d)
                      { depth_aver += d[1]; });
        depth_aver /= armor_points.size();

        float depth_std = 0.0;

        std::for_each(armor_points.begin(), armor_points.end(), [&](const Eigen::Vector3d &d)
                      { depth_std += pow((d[1] - depth_aver), 2); });
        depth_std = sqrt(depth_std / armor_points.size());

        std::for_each(
            armor_points.begin(), armor_points.end(), [&](Eigen::Vector3d &armor_point)
            {
                if (armor_point[1] < depth_aver - 0.5 * depth_std || armor_point[1] > depth_aver + 0.5 * depth_std)
                {
                    armor_point = armor_point / armor_point[1] * depth_aver;
                }
            });
    }
    const bool &BuffCurver::fit(std::vector<Eigen::Vector3d> &armor_pose_points)
    {
        // if (!is_model_start)
        // {
        correctPoints(armor_pose_points);
        buff_trajectory = fitCircle(armor_pose_points);
        if (buff_trajectory.radius < 0.8 && buff_trajectory.radius > 0.65)
        {
            is_model_start = true;
        }
        else
        {
            std::vector<Eigen::Vector3d>().swap(armor_pose_points);
        }
        // }
        return isModelStart();
    }
    const bool &BuffCurver::fit(const Eigen::Vector3d &armor_pose, const float &angle)
    {
        static int count = 0;
        if (armor_points.size() < CURVE_FIT_SIZE)
        {
            if (count % 6 == 0)
            {
                armor_points.push_back(armor_pose);
                angle_points.push_back(angle);
            }
            count++;
            is_model_start = false;
            return isModelStart();
        }
        if (!is_model_start)
        {
            correctPoints(armor_points);
            buff_trajectory = fitCircle(armor_points);
            // std::vector<Eigen::Vector3d>().swap(armor_points); //重复拟合
            if (buff_trajectory.radius < 0.8 && buff_trajectory.radius > 0.65)
            {
                // correctAxis(buff_trajectory, armor_points, angle_points);
                is_model_start = true;
            }
            else
            {
                std::vector<Eigen::Vector3d>().swap(armor_points);
            }
        }
        return isModelStart();
    }
    // const bool &BuffCurver::fit(const Eigen::Vector3d &armor_pose, const float &angle)
    // {
    //     static int count = 0;
    //     float angle_180 = angle / M_PI * 180;
    //     int angle_int = round(angle_180);
    //     if (angle_int <= 0 || angle_int >= 360)
    //     {
    //         if(!watched_points[0].is_get)
    //         {

    //         }
    //     }
    //     if (armor_points.size() < CURVE_FIT_SIZE)
    //     {
    //         if (count % 6 == 0)
    //         {
    //             armor_points.push_back(armor_pose);
    //             angle_points.push_back(angle);
    //         }
    //         count++;
    //         is_model_start = false;
    //         return isModelStart();
    //     }
    //     if (!is_model_start)
    //     {
    //         // correctPoints(armor_points);
    //         buff_trajectory = fitCircle(armor_points);
    //         if (buff_trajectory.radius < 0.8 && buff_trajectory.radius > 0.65)
    //         {
    //             is_model_start = true;
    //         }
    //         else
    //         {
    //             std::vector<Eigen::Vector3d>().swap(armor_points);
    //         }
    //     }
    //     return isModelStart();
    // }
    const double *BuffCurver::fitLargeBuffSin(const std::vector<float> &speed, const std::vector<float> &t)
    {
        //新大符
        ceres::Problem problem;
        problem.AddParameterBlock(estimate_cos_params, 3); //添加优化参数

        // //设置a范围
        problem.SetParameterLowerBound(estimate_cos_params, 0, 0.780);
        problem.SetParameterUpperBound(estimate_cos_params, 0, 1.045);

        // // //设置w范围
        problem.SetParameterLowerBound(estimate_cos_params, 1, 1.884);
        problem.SetParameterUpperBound(estimate_cos_params, 1, 2.000);

        // problem.SetParameterLowerBound(estimate_cos_params, 2, 0);
        // problem.SetParameterUpperBound(estimate_cos_params, 2, 2 * M_PI);

        // ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();
        // ordering->AddElementToGroup(estimate_cos_params + 0, 0);

        // ordering->AddElementToGroup(estimate_cos_params + 1, 1);
        // ordering->AddElementToGroup(estimate_cos_params + 2, 2);

        for (int i = 0; i < t.size(); i++)
        {
            problem.AddResidualBlock(
                // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<SPEED_FITTING_COST, 1, 3>(
                    new SPEED_FITTING_COST(speed[i], t[i])),
                new ceres::CauchyLoss(0.5),
                estimate_cos_params // 待估计参数
            );
        }
        // 配置求解器
        ceres::Solver::Options options;               // 这里有很多配置项可以填
        options.linear_solver_type = ceres::DENSE_QR; // 增量方程如何求解
        options.minimizer_progress_to_stdout = true;  // 输出到cout
        options.max_num_iterations = 50;              //迭代次数
        options.num_threads = 1;

        ceres::Solver::Summary summary;            // 优化信息
        ceres::Solve(options, &problem, &summary); // 开始优化
        return estimate_cos_params;
    }
    const double *BuffCurver::fitLargeBuffSin2(const std::vector<float> &speed, const std::vector<float> &t)
    {
        //新大符
        ceres::Problem problem;
        // problem.AddParameterBlock(estimate_cos_params, 1); //添加优化参数
        double theta = 0;

        for (int i = 0; i < t.size(); i++)
        {
            problem.AddResidualBlock(
                // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<LargeBuffFitFunction2, 1, 1>(
                    new LargeBuffFitFunction2(speed[i], t[i], estimate_cos_params[0], estimate_cos_params[1])),
                new ceres::CauchyLoss(0.5),
                &theta // 待估计参数
            );
        }
        // 配置求解器
        ceres::Solver::Options options;               // 这里有很多配置项可以填
        options.linear_solver_type = ceres::DENSE_QR; // 增量方程如何求解
        options.minimizer_progress_to_stdout = true;  // 输出到cout
        options.max_num_iterations = 50;              //迭代次数
        options.num_threads = 1;

        ceres::Solver::Summary summary;            // 优化信息
        ceres::Solve(options, &problem, &summary); // 开始优化

        estimate_cos_params[2] = theta;
        return estimate_cos_params;
    }

}
