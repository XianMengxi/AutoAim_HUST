#ifndef PITCH_YAW_FILTER
#define PITCH_YAW_FILTER
#include "STF.hpp"
namespace ly
{
    class PitchYawFilter
    {
    public:
        PitchYawFilter(/* args */);
        ~PitchYawFilter();
        void runKalman(const float &yaw, const float &pitch);

    private:
        /* data */
        StrongTrackingFilter<double, 2, 2> *kalman_filter; //滤波6维，x,x_v,y,y_v,z,z_v ,测量三维x,y,z
        bool is_kalman_init;
        void rebootKalman(const float &yaw, const float &pitch);
        void resetTransitionMatrix();
        void correct(const float &yaw, const float &pitch);
        void setMeasureMatrix();
        void setMeasurementNoise();
        void setProcessNoise();

    public:
        void resetKalman()
        {
            is_kalman_init = false;
        }
        float getYaw()
        {
            return kalman_filter->posteriori_state_estimate[0];
        }
        float getPitch()
        {
            return kalman_filter->posteriori_state_estimate[1];
        }
    };

}
#endif