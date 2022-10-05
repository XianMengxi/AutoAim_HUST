//
// Created by zhiyu on 2021/8/24.
//

#include "LightBarFinder.h"

using namespace ly;
using namespace cv;

bool LightBarFinder::findLightBarBlobs(const Mat &frame, LightBarBlobs &lightBarBlobs, const cv::Point2f &roi_offset)
{
    int mode = GRAY;
    Mat mat;
    vector<Mat> channels;
    vector<vector<Point>> contours;
    cv::Mat hsv1, hsv2, color_part;
    cv::Mat gray;

    if (frame.empty())
    {
        return false;
    }

    if (mode == BGR)
    {
        split(frame, channels);
        if (enemy_color == BLUE)
        {
            subtract(channels[0], channels[2], mat);
        }
        else if (enemy_color == RED)
        {
            subtract(channels[2], channels[0], mat);
        }
        threshold(mat, mat, DetectorParam::thresh, 255, THRESH_BINARY);
        erode(mat, mat, kernel_33); //去除噪点，考虑不要腐蚀(在黑暗的环境不需要，白天环境可能需要)
        dilate(mat, mat, kernel_55);
    }
    else if (mode == GREEN) //使用绿通道，效果并不好
    {
        cv::Mat target_color;
        cv::Mat sub_mat;
        split(frame, channels);
        if (enemy_color == BLUE) //剔除白光
        {
            target_color = channels[0];
            subtract(channels[0], channels[1], sub_mat);
        }
        else if (enemy_color == RED)
        {
            target_color = channels[2];

            subtract(channels[2], channels[0], sub_mat);
        }

        threshold(sub_mat, sub_mat, DetectorParam::thresh, 255, THRESH_BINARY);
        threshold(target_color, target_color, 64, 255, THRESH_BINARY);

        mat = sub_mat & target_color;
        erode(mat, mat, kernel_33);
        dilate(mat, mat, kernel_35);
        // erode(mat, mat, kernel_33); //去除噪点，考虑不要腐蚀(在黑暗的环境不需要，白天环境可能需要)
        // dilate(mat, mat, kernel_55);
    }
    else if (mode == GRAY) //灰度图预处理
    {
        // const int color_thresh = 54;
        // cv::Mat sub_mat;
        // split(frame, channels);                //考虑可能需要使用阈值分割来求得颜色，不然的话误判数可能增加，配合自动曝光
        cvtColor(frame, gray, COLOR_BGR2GRAY); //转灰度图
        // if (enemy_color == BLUE)               //剔除白光
        // {
        //     subtract(channels[0], channels[2], sub_mat);
        // }
        // else if (enemy_color == RED)
        // {
        //     subtract(channels[2], channels[0], sub_mat);
        // }
        threshold(gray, mat, DetectorParam::thresh, 255, THRESH_BINARY); //二值化分割
        // threshold(sub_mat, sub_mat, color_thresh, 255, THRESH_BINARY);
        // mat = mat & sub_mat;

        erode(mat, mat, kernel_33); //去除噪点，考虑不要腐蚀(在黑暗的环境不需要，白天环境可能需要)
        dilate(mat, mat, kernel_35);
    }

    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mat, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() <= 10 || hierarchy[i][3] >= 0) //不能有父轮廓
        {
            continue;
        }
        const RotatedRect &rrect = minAreaRect(contours[i]);

        if (isValidLightBarBlob(rrect))
        {
            if (mode == GRAY && !isRightColor(frame, rrect))
            {
                continue;
            }
            if (ArmorDetectParam::show_lightbar_info)
            {
                std::cout << "lightbar ratio:  " << MAX(rrect.size.width, rrect.size.height) / MIN(rrect.size.width, rrect.size.height) << std::endl;
                std::cout << "lightbar area:  " << rrect.size.area() << std::endl;
                std::cout << "lightbar angle:  " << (rrect.size.width > rrect.size.height ? rrect.angle - 90 : rrect.angle) << std::endl;
            }

            RotatedRect roi_rrect = RotatedRect(rrect.center + roi_offset, rrect.size, rrect.angle);

            lightBarBlobs.emplace_back(roi_rrect);
        }
    }

    if (ArmorDetectParam::show_thresh)
    {
        DEBUG_MODE(imshow("thresh", mat));
        waitKey(1);
    }

    sort(lightBarBlobs.begin(), lightBarBlobs.end(), [](const RotatedRect &a, const RotatedRect &b) -> bool
         {
             if (a.center.x != b.center.x)
                 return a.center.x < b.center.x;
             return a.center.y > b.center.y;
         });
    return lightBarBlobs.size() >= 2;
}
bool LightBarFinder::isRightColor(const cv::Mat &frame, const RotatedRect &rrect)
{
    cv::Rect rect = rrect.boundingRect();
    rect.x = rect.x < 0 ? 0 : (rect.x > frame.cols ? frame.cols : rect.x);
    rect.y = rect.y < 0 ? 0 : (rect.y > frame.rows ? frame.rows : rect.y);
    rect.width = rect.x + rect.width > frame.cols ? frame.cols - rect.x : rect.width;
    rect.height = rect.y + rect.height > frame.rows ? frame.rows - rect.y : rect.height;
    cv::Scalar roi_color_bgr_mean = mean(frame(rect));
    if (enemy_color == BLUE)
    {
        // std::cout << "roi_color: " << roi_color_bgr_mean[0] - roi_color_bgr_mean[1] << " " << roi_color_bgr_mean[0] - roi_color_bgr_mean[2] << std::endl;

        return roi_color_bgr_mean[0] > roi_color_bgr_mean[2] &&
               roi_color_bgr_mean[0] > roi_color_bgr_mean[1];
    }
    else
    {
        // std::cout << "roi_color: " << roi_color_bgr_mean[2] - roi_color_bgr_mean[1] << " " << roi_color_bgr_mean[2] - roi_color_bgr_mean[0] << std::endl;
        return roi_color_bgr_mean[2] > roi_color_bgr_mean[0] && roi_color_bgr_mean[2] > roi_color_bgr_mean[1];
    }
}

bool LightBarFinder::isValidLightBarBlob(const RotatedRect &rrect)
{
    float angle = rrect.size.width > rrect.size.height ? rrect.angle - 90 : rrect.angle;
    double aspect_ratio = MAX(rrect.size.width, rrect.size.height) / MIN(rrect.size.width, rrect.size.height);
    if (checkAspectRatio(aspect_ratio) && checkArea(rrect.size.area()) && checkAngle(angle))
    {
        if (!checkRatioAndArea(aspect_ratio, rrect.size.area())) //面积较大时，比例会更加严格
        {
            return false;
        }
        return true;
    }
    return false;
}
bool LightBarFinder::checkRatioAndArea(const double &ratio, const double &area)
{
    // if (area > 200)
    // {
    //     return ratio > 2;
    // }
    return true;
}
//长宽比设置
bool LightBarFinder::checkAspectRatio(const double &ratio)
{
    return ratio <= ArmorDetectParam::lightbar_ratio_max && ratio >= ArmorDetectParam::lightbar_ratio_min;
}

bool LightBarFinder::checkArea(const double &area)
{
    return area >= ArmorDetectParam::lightbar_area_min && area <= ArmorDetectParam::lightbar_area_max;
}

bool LightBarFinder::checkAngle(const float &angle)
{
    // std::cout << "angle:  " << angle << std::endl;
    return angle >= ArmorDetectParam::lightbar_angle_min && angle <= ArmorDetectParam::lightbar_angle_max;
}

LightBarFinder::LightBarFinder()
{
    kernel_33 = getStructuringElement(cv::MORPH_RECT, Size(3, 3));
    kernel_55 = getStructuringElement(cv::MORPH_RECT, Size(5, 5));
    kernel_35 = getStructuringElement(cv::MORPH_RECT, Size(3, 5));
}
