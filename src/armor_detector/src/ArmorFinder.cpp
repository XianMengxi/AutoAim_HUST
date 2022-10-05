//
// Created by zhiyu on 2021/8/24.
//

#include "ArmorFinder.h"

using namespace ly;

float getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2)
{
    cv::Point2f diff = point_1 - point_2;
    return sqrt(diff.x * diff.x + diff.y * diff.y);
}

//根据角度和高度判断是否是
bool ArmorFinder::matchTwoLightBar(const RotatedRect &l, const RotatedRect &r)
{
    status = checkAngleDiff(l, r);
    // DLOG_IF(INFO, !status) << "angle not match! left: " << getAngle(l) << " right: " << getAngle(r);
    if (!status)
        return false;
    status = checkHeightDiff(l, r);
    // DLOG_IF(INFO, !status) << "height differs: " << abs((l.center - r.center).y);
    if (!status)
        return false;
    return true;
}

//判断长宽比
bool ArmorFinder::judgeArmor(const ArmorBlob &armor_blob)
{

    double high_length = getDistance(armor_blob.corners[0], armor_blob.corners[1]);
    double low_length = getDistance(armor_blob.corners[2], armor_blob.corners[3]);
    double left_height = getDistance(armor_blob.corners[0], armor_blob.corners[3]);
    double right_height = getDistance(armor_blob.corners[1], armor_blob.corners[2]);

    double armor_ratio = ((high_length + low_length) / 2) / ((left_height + right_height) / 2);

    if (ArmorDetectParam::show_armor_info)
    {
        std::cout << "armor ratio::" << armor_ratio << std::endl;
    }

    bool ret = armor_ratio <= ArmorDetectParam::armor_ratio_max && armor_ratio >= ArmorDetectParam::armor_ratio_min;
    return ret;
}

/***
 * @brief 将两个灯条拼接成一个装甲(为什么和某些OpenCV版本不兼容？)
 * @param l
 * @param r
 * @param armor
 * @return
 */
bool ArmorFinder::getArmor(const RotatedRect &l, const RotatedRect &r, ArmorBlob &armor)
{
    Point2f points_of_rrect[4];
    l.points(points_of_rrect);
    float height = fmax(l.size.width, l.size.height);
    armor.rect = Rect(l.center.x, l.center.y - height / 2, r.center.x - l.center.x, height);

    // armor
    // 0 1
    // 3 2
    if (l.angle > 45)
    {
        armor.corners[0] = (points_of_rrect[0] + points_of_rrect[1]) / 2;
        armor.corners[3] = (points_of_rrect[3] + points_of_rrect[2]) / 2;
    }
    else
    {
        armor.corners[0] = (points_of_rrect[1] + points_of_rrect[2]) / 2;
        armor.corners[3] = (points_of_rrect[0] + points_of_rrect[3]) / 2;
    }
    r.points(points_of_rrect);
    if (r.angle > 45)
    {
        armor.corners[1] = (points_of_rrect[1] + points_of_rrect[0]) / 2;
        armor.corners[2] = (points_of_rrect[2] + points_of_rrect[3]) / 2;
    }
    else
    {
        armor.corners[1] = (points_of_rrect[2] + points_of_rrect[1]) / 2;
        armor.corners[2] = (points_of_rrect[3] + points_of_rrect[0]) / 2;
    }

    //这个可能不满足吗
    if (armor.corners[0].y > armor.corners[2].y || armor.corners[1].y > armor.corners[3].y)
    {
        return false;
    }

    //尽量满足矩形
    cv::Point2f left_light = armor.corners[0] - armor.corners[3];
    cv::Point2f right_light = armor.corners[2] - armor.corners[1];
    cv::Point2f center_diff = l.center - r.center;

    float left_length = sqrt(left_light.x * left_light.x + left_light.y * left_light.y);
    float right_length = sqrt(right_light.x * right_light.x + right_light.y * right_light.y);
    float center_diff_length = sqrt(center_diff.x * center_diff.x + center_diff.y * center_diff.y);
    float rectangle_likely = (left_light.x * center_diff.x + left_light.y * center_diff.y) / (left_length * center_diff_length);
    rectangle_likely += (right_light.x * center_diff.x + right_light.y * center_diff.y) / (right_length * center_diff_length);

    float length_ratio = MAX(left_length, right_length) / MIN(left_length, right_length);

    if (length_ratio > ArmorDetectParam::lightbar_length_ratio || fabs(rectangle_likely) > ArmorDetectParam::rectangle_likely)
    {
        // std::cout << "rectangle_likly or lightbar_ratio not right" << std::endl;
        return false;
    }

    int armor_area = armor.rect.size().area();
    if (armor_area < ArmorDetectParam::armor_area_min || armor_area > ArmorDetectParam::armor_area_max)
    {
        return false;
    }
    if (ArmorDetectParam::show_armor_info)
    {
        std::cout << "armor_area" << armor_area << std::endl;
        std::cout << "rectangle_likely" << rectangle_likely << std::endl;
        std::cout << "light_bar_length_ratio" << length_ratio << std::endl;
    }
    return true;
}
int ArmorFinder::filteArmor(const RotatedRect &rect, const ArmorBlob &blob) //如果返回true，说明装甲板中间有灯条，跳过
{
    cv::Point2f points[4];
    rect.points(points);
    if (pointPolygonTest(blob.corners, rect.center, true) >= 0)
    {
        return false;
    }
    for (int i = 0; i < 4; i++)
    {
        if (pointPolygonTest(blob.corners, points[i], true) >= 0)
        {
            return false;
        }
    }
    return true;
}

bool ArmorFinder::checkAngleDiff(const RotatedRect &l, const RotatedRect &r)
{
    float angle_l = getAngle(l);
    float angle_r = getAngle(r);
    if (ArmorDetectParam::show_armor_info)
    {
        std::cout << "angle_diff" << abs(angle_l - angle_r) << std::endl;
    }
    return abs(angle_l - angle_r) < ArmorDetectParam::angle_diff_max;
}

float ArmorFinder::getAngle(const RotatedRect &rrect)
{
    return rrect.size.width > rrect.size.height ? rrect.angle - 90 : rrect.angle;
}

bool ArmorFinder::checkHeightDiff(const RotatedRect &l, const RotatedRect &r)
{
    Point2f diff = l.center - r.center;
    if (ArmorDetectParam::show_armor_info)
    {
        std::cout << "lightbar_center_diff:  " << abs(diff.y) << std::endl;
    }
    return abs(diff.y) < ArmorDetectParam::lightbar_center_diff;
}
