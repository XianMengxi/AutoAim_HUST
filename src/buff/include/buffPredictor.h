#ifndef _BUFF_PREDICTOR_H
#define _BUFF_PREDICTOR_H
#include "eigen3/Eigen/Core"
#include "buffArmorNode.h"
#include "buffCurver.h"
#include <vector>
#include "lcm_module.h"
#include "buffAngleKalman.h"
namespace ly
{
#define SINE_FIT_SIZE 200
#define CURVER_SECOND_THRESH 150
#define CONVERGE_COUNT 30
#define SPEED_FIT_SIZE 300 //待改

    enum ROTATION
    {
        CLOCK_WISE = 0,
        COUNTER_CLOCK_WISE = 1
    };
    enum BUFF_TYPE
    {
        UNDECIDE = -1,
        SMALL_BUFF = 0,
        LARGE_BUFF = 1,
        MIXED_BUFF
    };
    struct BuffTrajPoint
    {
        bool is_get;
        Eigen::Vector3d point;
        BuffTrajPoint()
        {
            is_get = false;
        }
    };
    // struct Compensation  //补偿计算
    // {
    //     bool is_get;
    //     int comp_yaw;
    //     int comp_pitch;
    //     Compensation()
    //     {
    //         is_get = false;
    //     }
    // };
    class BuffPredictor
    {
    private:
        //方向获取
        bool is_get_direction;
        bool is_trajectory_get = false; //是否已经获得轨迹

        BuffCurver *curver;

        //三维拟合
        BuffTrajectory buff_trajectory;

        BuffArmor last_aim_armor;
        BuffArmor this_aim_armor;

        //连续化之后的角度
        float this_armor_angle;
        float last_armor_angle;

        float time_start = 0.0f; //成功拟合的时间初始

        //使角度连续化
        float makeAngleContinous();
        float angleFilter(const float &predict_t);
        void sineFit();
        float predict(const float &predict_t);
        float fixSpeedPredict(const float &filte_angle, const float &predict_t);
        void judgeRotation(float angle_diff);
        Eigen::Vector3d ThreeDPredict(const float &predict_t);
        void correctAxis(BuffTrajectory &buff_traj, const Eigen::Vector3d &sample_point, const float &angle);
        float SinePredict(const float &filte_angle, const float &predict_t);

        float FilteSpeed(const float &delta_t); //滤出速度
        void largeBuffFit();
        void emptyTraj();

        //用于正弦拟合
        std::vector<float> buff_angle_set;
        std::vector<float> buff_time_set;
        std::vector<float> speed_set;

        BuffTrajPoint watched_points[360]; //将角度分为360度
        int get_angle_num = 0;

        float time_base;          //时间基准，可以考虑用time_point代替
        bool is_get_sine;         //是否已经得到拟合参数
        const double *cos_params; //拟合参数
        float filte_speed = 0;

        //旋转方向计算
        bool is_get_rotation = false; //是否计算好了方向
        int clockwise_count;
        float rotation = -1; //旋转方向

        //滤波计算
        float max_interval;       //间隔时间最大值.单位ms
        float max_armor_move_dis; //连续两帧之间装甲板的距离之差，如果大于这个值就说明发生了切换

        //预测
        float aim_angle;
        const float bullet_speed = 80.0f;
        const float shoot_delay = 0.2; //s

        bool is_get_first_armor = false;
        BuffArmor first_armor;

        //判断装甲板类型
        bool is_get_buff_type = false;
        int buff_type = UNDECIDE; //0:小符，1大符,-1未定

        void judgeBuffType();

        float last_last_armor_angle = 0.0f;
        float predict_angle;
        SineFunction *sine_function;

        bool is_buff_curve_not_sucess = false;

        LcmDebug debug;

        buffAngleKalman *buff_angle_kalman_filter;

        std::vector<Eigen::Vector3d> armor_pose_points;
        void getPointsFromTraj(int freq);

    public:
        BuffPredictor();
        ~BuffPredictor();
        void getTrajData(const Eigen::Vector3d &armor_pose, const float &angle);
        float getShootAngle(const float &filte_angle, const float &predict_time);

        int getRotation()
        {
            if (is_get_rotation)
            {
                return rotation;
            }
            return -1;
        }
        float getPredictAngle()
        {
            return predict_angle;
        }
        void setPredictMode(int type);

        //传进去滤波之后的位置以及需要预测的时间长度
        float runBuffPredict(BuffArmor &aim_armor);
        Eigen::Vector3d getPredictPose(const float &filte_angle, const float &shoot_time);
        float runLargeBuffPredict(BuffArmor &aim_armor);
        float OnePointPreidct(BuffArmor &aim_armor);

        bool judgeRestart(); //判断前后两帧角度差是否大于一定的阈值
        float runSmallBuffPredict(BuffArmor &aim_armor);
        Eigen::Vector3d SmallBuffPreidct(const float &delta_t);
        cv::Point2f OnePointPositonCalc(const cv::Point2f &center, const cv::Point2f &armor_center, const float &predict_time, const float &filte_angle);

        bool update_symbol = false;

        // float runBuffPredict(BuffArmor &aim_armor);
    };

};
#endif