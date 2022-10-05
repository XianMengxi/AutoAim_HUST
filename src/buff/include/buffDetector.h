#ifndef _BUFF_DETECTOR_H_
#define _BUFF_DETECTOR_H_
#include "opencv2/opencv.hpp"
#include <string>
#include <vector>
#include "buffArmorNode.h"
#include "buffClassifier.h"
#include "buffPredictor.h"
#include "Params.h"
#include "Log.h"
namespace ly
{
    // #define COUT_BUFF_INFO
    // #define SHOW_THRESH
    enum COLOR
    {
        BLUE_BUFF = 3,
        RED_BUFF = 4
    };
    class BuffDetector
    {
    private:
        //识别
        //基本图像处理工作
        void imageProcess(const cv::Mat &frame, cv::Mat &output_frame);

        //查找装甲板
        void findBuffArmor(const cv::Mat &raw_pic, const cv::Mat &img_processed);

        //svm分类，找到正确的击打装甲板
        void judgeRightArmor(const cv::Mat &img_processed);

        //查找位置
        void getBuffLocation();

        //保存上一帧的击打装甲板
        void updateArmor();

        //找圆心R
        void findCenter(const cv::Mat &raw_pic, const std::vector<std::vector<cv::Point>> &contours, std::vector<cv::Vec4i> hierarchy, const std::vector<BuffArmor> &targets);

        //解算角度，正确分辨每一个点
        void sortArmorPoints(BuffArmor &buff_armor_node);

        //params
        int buff_color; //0:blue，1:red

        //预处理参数
        int binary_thresh;
        int color_thresh;

        //装甲板识别参数
        int armor_size_min;
        int armor_size_max;
        int armor_father_size_min;
        int armor_father_size_max;
        float armor_ratio_min;
        float armor_ratio_max;
        int father_area_max;
        int father_area_min;

        //圆心检测参数
        int buff_center_size_min;
        int buff_center_size_max;
        float buff_center_radius_max;
        float buff_center_radius_min;
        float buff_rotate_radius_min;
        float buff_rotate_radius_max;

        //signal
        bool is_find_armor;        //1个或者多个装甲板
        bool is_get_one_armor;     //只有一个装甲板，可以避免分类
        bool is_get_aim_armor;     //是否获得分类之后的装甲板
        bool is_get_buff_location; //获得buff的基准向量
        bool is_get_center;        //是否获得了圆心

        //tools function
        //清空装甲板队列
        void emptyArmorQueue();

        //计算欧式距离
        float getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2);

        //存储变量
        std::vector<BuffArmor> targets;
        BuffArmor aim_target;
        BuffArmor last_target;
        std::vector<cv::Point2f> buff_center_candidate;
        cv::Point2f buff_center;

        //svm
        void getBuffDataSet(const cv::Mat &img_processed);
        BuffClassifier *buff_classifier;

        BuffPredictor *predictor;

        cv::Point2f predict_point;

        bool VerifyPitch(const cv::Point2f &center);
        bool VerifyColor(const cv::Mat &raw_pic, const std::vector<cv::Point> &countour_points);

    public:
        BuffDetector();
        BuffDetector(const BuffParam &buff_params);
        ~BuffDetector();
        void NewPredictor();

        float this_pitch;

        //识别
        const BuffArmor &runBuffDetect(const cv::Mat &frame);
        void setPredictMode(int type);

        //预测，一个版本3维拟合，另一个版本找圆心
        float runLargeBuffPredict(const Eigen::Vector3d &armor_pose, const std::chrono::_V2::steady_clock::time_point &catch_armor_time);

        float runBuffPredict(const Eigen::Vector3d &armor_pose, const std::chrono::_V2::steady_clock::time_point &catch_armor_time);
        float runSmallBuffPredict(const Eigen::Vector3d &armor_pose, const std::chrono::_V2::steady_clock::time_point &catch_armor_time);

        // const cv::Point2f &runBuffPredict(const Eigen::Vector3d &armor_pose, std::chrono::steady_clock::time_point *catch_armor_time);
        Eigen::Vector3d getPredictPose(const float &filte_angle, const float &shoot_time)
        {
            return predictor->getPredictPose(filte_angle, shoot_time);
        } //转接buff_predictor

        bool judgeRestart()
        {
            return predictor->judgeRestart();
        }
        bool VerifyAngle(const cv::RotatedRect &rect, const cv::Point2f &center);
        bool VerifyDistance(const Eigen::Vector3d &armor_pose);
        void setColor(const char &flag);
        //void setColor(int color) { buff_color = color; };
        Eigen::Vector3d SmallBuffPreidct(const float &delta_t)
        {
            return predictor->SmallBuffPreidct(delta_t);
        }
        void setRecurveSymbol()
        {
            predictor->update_symbol = true;
        };

        float OnePointPreidct(const std::chrono::_V2::steady_clock::time_point &catch_armor_time)
        {
            aim_target.time_stamp = catch_armor_time;
            return predictor->OnePointPreidct(aim_target);
        }
        cv::Point2f OnePointPositonCalc(const cv::Point2f &center, const cv::Point2f &armor_center, const float &predict_time, const float &filte_angle)
        {
            return predictor->OnePointPositonCalc(center, armor_center, predict_time, filte_angle);
        }
        float getShootAngle(const float &filte_angle, const float &predict_time)
        {
            return predictor->getShootAngle(filte_angle, predict_time);
        }

        void drawAngle(cv::Mat &frame, cv::RotatedRect rect, cv::Point2f center);

        //画出预测的位置
        void drawPredict(cv::Mat &frame, const float &angle);

        //画出目标装甲板
        void drawArmor(cv::Mat &frame, const cv::RotatedRect &rect);
        void drawArmor(cv::Mat &frame, cv::Point2f *points);
        void drawCircle(cv::Mat &frame, const cv::Point2f &armor_center);
        void drawAngle(cv::Mat &frame);
        void drawCountour(cv::Mat &frame, cv::Mat &img_processed);
        void drawAllArmor(cv::Mat &frame);
        void drawFatherRect(cv::Mat &frame, const BuffArmor &);
    };
};
#endif