#include "PitchYawFilter.h"
namespace ly
{
    PitchYawFilter::PitchYawFilter(/* args */)
    {
        is_kalman_init = false;
        kalman_filter = new StrongTrackingFilter<double, 2, 2>();
        kalman_filter->setIsUseSTF(false); //可以试试使用
        setMeasureMatrix();
        setMeasurementNoise();
        setProcessNoise();
    }

    PitchYawFilter::~PitchYawFilter()
    {
    }
    void PitchYawFilter::setMeasureMatrix()
    {
        kalman_filter->measurement_matrix = Eigen::Matrix2d::Identity();
    }
    void PitchYawFilter::rebootKalman(const float &yaw, const float &pitch)
    {

        kalman_filter->posteriori_state_estimate[0] = yaw;
        kalman_filter->posteriori_state_estimate[1] = pitch;

        kalman_filter->error_cov_post = Eigen::Matrix<double, 2, 2>::Identity();

        resetTransitionMatrix();
    }
    void PitchYawFilter::resetTransitionMatrix()
    {
        //x,x_v,y,y_v,z,z_v
        kalman_filter->transition_matrix = Eigen::Matrix<double, 2, 2>::Identity();
    }

    void PitchYawFilter::runKalman(const float &yaw, const float &pitch) //量测有效更新
    {
        if (!is_kalman_init)
        {
            //set signal values
            is_kalman_init = true;

            //reset kalman
            rebootKalman(yaw, pitch);
            resetTransitionMatrix();

            //return values
            return;
        }
        else
        {
            //update transition matrix
            resetTransitionMatrix();

            return correct(yaw, pitch);
        }
    }
    void PitchYawFilter::correct(const float &yaw, const float &pitch)
    {
        Eigen::Vector2d yaw_pitch = Eigen::Vector2d(yaw, pitch);
        kalman_filter->predict(yaw_pitch); //量测有效更新
        kalman_filter->update();

        return;
    }
    void PitchYawFilter::setMeasurementNoise()
    {

        double measurement_noise_pose_yaw = 0.1;
        double measurement_noise_pose_pitch = 0.1;

        kalman_filter->measurement_noise_cov.diagonal() << measurement_noise_pose_yaw,
            measurement_noise_pose_pitch;
    }
    void PitchYawFilter::setProcessNoise()
    {
        double process_noise_pose_yaw = 0.01;
        double process_noise_pose_pitch = 0.01;
        kalman_filter->process_noise_cov.diagonal() << process_noise_pose_yaw, process_noise_pose_pitch;
    }
}