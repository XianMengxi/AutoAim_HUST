#include "DetectAccelerator.h"
namespace ly
{
    DetectAccelerator::DetectAccelerator(/* args */)
    {
        is_get_roi = false;
        detect_sucess_count = 0; //检测到计数
    }

    DetectAccelerator::~DetectAccelerator()
    {
    }
    const cv::Rect &DetectAccelerator::getROI()
    {
        if (!is_get_roi || detect_sucess_count > ROI_LOSS_MAX_COUNT) //无ROI
        {
            detect_sucess_count = 0;
            is_get_roi = false;
            roi = cv::Rect(0, 0, 1280, 1024); //全图搜索
        }
        else
        {
            detect_sucess_count++; //理论上如果使用该ROI但长时间没有获得目标，该值会增大
        }

        return roi;
    }
    const cv::Point DetectAccelerator::getRoiOffset()
    {
        return roi.tl();
    }
    void DetectAccelerator::detectFeedback(const std::vector<cv::Point2f> &corners) //检测到目标，给予一定的反馈
    {
        //根据优先级判断
        cv::Point2f center = cv::Point2f(0, 0);
        for (int i = 0; i < corners.size(); i++)
        {
            center += corners[i];
        }
        center /= 4; //求得中心

        cv::Point2f point_diff_width = (corners[1] + corners[2] - corners[0] - corners[3]) / 2;
        cv::Point2f point_diff_height = (corners[2] + corners[3] - corners[1] - corners[0]) / 2;

        float WIDTH = sqrt(point_diff_width.x * point_diff_width.x + point_diff_width.y * point_diff_width.y);
        float HEIGHT = sqrt(point_diff_height.x * point_diff_height.x + point_diff_height.y * point_diff_height.y);

        float width_ratio = 8.0f;
        float height_ratio = 4.0f;

        if (HEIGHT < 50) //设定最低高度区域
        {
            HEIGHT = 50;
        }

        //暂时分配固定大小的ROI
        cv::Point tl_point = cv::Point((int)(center.x - WIDTH * width_ratio), (int)(center.y - HEIGHT * height_ratio));
        cv::Point br_point = cv::Point((int)(center.x + WIDTH * width_ratio), (int)(center.y + HEIGHT * height_ratio));

        makePointInRange(tl_point);
        makePointInRange(br_point);

        roi = cv::Rect(tl_point, br_point);
        is_get_roi = true;
        detect_sucess_count = 0; //漏检测清零
    }

    void DetectAccelerator::makePointInRange(cv::Point &point)
    {
        point.x = point.x > 1280 ? 1280 : (point.x < 0 ? 0 : point.x);
        point.y = point.y > 1024 ? 1024 : (point.y < 0 ? 0 : point.y);
    }
}