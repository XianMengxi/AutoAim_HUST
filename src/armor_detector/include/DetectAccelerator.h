#ifndef _DETECT_ACCELERATOR_H
#define _DETECT_ACCELERATOR_H
#include "opencv2/opencv.hpp"
#include <vector>
#include "Config.h"
#include "EnemyType.h"
namespace ly
{
#define ROI_WIDTH 450
#define ROI_HEIGHT 256
#define ROI_LOSS_MAX_COUNT 10
    class DetectAccelerator
    {
    private:
        bool is_get_roi;

        cv::Rect roi;

        int detect_sucess_count;

        void makePointInRange(cv::Point &point);

    public:
        DetectAccelerator();
        ~DetectAccelerator();
        const cv::Rect &getROI();
        const cv::Point getRoiOffset();
        void detectFeedback(const std::vector<cv::Point2f> &corners); //检测到目标，给予一定的反馈
        void drawROI(cv::Mat &frame)
        {
            if (ArmorDetectParam::debug_show)
            {
                cv::rectangle(frame, roi, cv::Scalar(255, 0, 0), 2, 8);
            }
        }
        void resetROI() //重置ROI
        {
            is_get_roi = false;
        }
    };

}
#endif