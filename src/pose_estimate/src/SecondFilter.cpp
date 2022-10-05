#include "SecondFilter.h"
namespace ly
{
    SecondFilter::SecondFilter(/* args */)
    {
        stf_filter = new StrongTrackingFilter<double, 3, 3>();
        measurement_ = Eigen::Matrix<double, 3, 1>::Zero();
        setMeasurementNoise();
        setProcessNoise();
        stf_filter->setIsUseSTF(false);
        setTransitionMatrix();
    }

    SecondFilter::~SecondFilter()
    {
        delete stf_filter;
    }

    void SecondFilter::setProcessNoise() //设置过程噪声矩阵
    {
        // double process_noise_pose = 0.001;
        stf_filter->process_noise_cov.diagonal() << FilterParams::process_noise_pose_x, FilterParams::process_noise_pose_y, FilterParams::process_noise_pose_z;
    }
    void SecondFilter::setProcessNoise(double x, double y, double z) //设置过程噪声矩阵
    {
        // double process_noise_pose = 0.001;
        stf_filter->process_noise_cov.diagonal() << x, y, z;
    }
    void SecondFilter::setMeasurementNoise(double x, double y, double z)
    {
        //效果不好
        double position_cov_x = x;
        double position_cov_y = y;
        double position_cov_z = z;

        stf_filter->measurement_noise_cov.diagonal() << position_cov_x, position_cov_y, position_cov_z;
    }
    void SecondFilter::setMeasurementNoise()
    {
        //效果不好
        double position_cov_x = FilterParams::measurement_noise_pose_x;
        double position_cov_y = FilterParams::measurement_noise_pose_y;
        double position_cov_z = FilterParams::measurement_noise_pose_z;

        stf_filter->measurement_noise_cov.diagonal() << position_cov_x, position_cov_y, position_cov_z;
    }

    void SecondFilter::setTransitionMatrix()
    {
        //x,y,z,x_v,y_v,z_v
        stf_filter->transition_matrix = Eigen::Matrix<double, 3, 3>::Identity();
    }

    void SecondFilter::rebootKalman(const ArmorSecondPose &new_armor_pose)
    {
        for (int i = 0; i < 3; i++)
        {
            stf_filter->posteriori_state_estimate[i] = new_armor_pose[i];
        }
        stf_filter->error_cov_post = Eigen::Matrix<double, 3, 3>::Identity();
        setTransitionMatrix();
    }
    void SecondFilter::resetFilter()
    {
        is_filter_init = false;
    }
    ArmorSecondPose SecondFilter::runFilter(const ArmorSecondPose &new_armor_pose)
    {
        if (!is_filter_init)
        {
            //set signal values
            is_filter_init = true;

            //reset kalman
            rebootKalman(new_armor_pose);
            updateArmorState(new_armor_pose);

            //return values
            return new_armor_pose;
        }
        else
        {
            stf_filter->measurement_matrix = Eigen::Matrix3d::Identity();
            updateMeasurement(new_armor_pose);

            return correct();
        }
    }
    void SecondFilter::updateArmorState(const ArmorSecondPose &new_armor_pose)
    {
        last_armor_pose = new_armor_pose;
    }
    ArmorSecondPose SecondFilter::correct()
    {
        stf_filter->predict(measurement_); //量测有效更新
        // if (!stf_filter->ChiSquaredTest()) //true则卡方检验通过
        // {
        //     rebootKalman(measurement_); //重启卡尔曼滤波器
        //     return measurement_;
        // }
        stf_filter->update();

        //update armor status and return
        last_armor_pose = stf_filter->posteriori_state_estimate;

        return last_armor_pose;
    }
    void SecondFilter::updateMeasurement(const ArmorSecondPose &new_armor_pose)
    {
        measurement_ << new_armor_pose[0], new_armor_pose[1],
            new_armor_pose[2];
    }
}
