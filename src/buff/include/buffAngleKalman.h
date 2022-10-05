#include "STF.hpp"
#include "Config.h"
#include "ceres/ceres.h"
namespace ly
{
    class SineFunction
    {
    public:
        SineFunction()
        {
            base_time = 0.0f;
        }
        SineFunction(double *sine_params)
        {
            base_time = 0.0f;
            setParams(sine_params);
        }
        void setBaseTime(const float &time_stamp)
        {
            base_time = time_stamp / 1000.0f;
            // std::cout << "base time:" << base_time << std::endl;
        }
        void setParams(const double *sine_params)
        {
            for (int i = 0; i < 3; i++)
            {
                params[i] = sine_params[i];
            }
        }
        double predict(const float &predict_t)
        {
            return -params[0] / params[1] * cos(params[1] * predict_t + params[2]) + (2.090 - params[0]) * predict_t;
        }
        double getSpeed(const double &t)
        {
            return params[0] * cos(params[1] * t + params[2]) + 2.090 - params[0];
        }
        template <typename T>
        void operator()(const T Xe[1], T Xp[1], const double &t) //重载小括号
        {
            std::cout << "t: " << t << "base_time" << base_time << std::endl;
            Xp[0] = Xe[0] + (-params[0] / params[1]) * (ceres::cos(T(params[1] * (base_time + t) + params[2])) - ceres::cos(T(params[1] * base_time + params[2]))) + (2.090 - params[0]) * t;
        }

    private:
        double params[3];
        float base_time;
    };
    class buffAngleKalman
    {
    private:
        /* data */
        StrongTrackingFilter<double, 2, 1> *kalman_filter; //滤波二维
        bool is_kalman_init;
        void rebootKalman(const double &angle);
        void resetTransitionMatrix();
        void setUpdateTime(const double &delta_t);
        void setTransitionMatrix();
        double correct(const double &angle);
        void setMeasureMatrix();
        void setMeasurementNoise();
        void setProcessNoise();

        double update_time;
        double posteriori_speed;
        double posteriori_angle;

    public:
        buffAngleKalman(/* args */);
        ~buffAngleKalman();
        double runKalman(const double &new_armor_angle, const double &delta_t); //量测有效更新
        double predict(const double &predict_t);
        double getSpeed() { return posteriori_speed; };
        double getAngle() { return posteriori_angle; };
        void resetKalman()
        {
            is_kalman_init = false;
        }
    };

}
