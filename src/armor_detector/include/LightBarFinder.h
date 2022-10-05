//
// Created by zhiyu on 2021/8/24.
//

#ifndef AUTOAIM_LIGHTBARFINDER_H
#define AUTOAIM_LIGHTBARFINDER_H

#include "opencv2/opencv.hpp"
#include <vector>

#include "Params.h"
#include "Log.h"

using namespace cv;
using namespace std;

namespace ly
{
    enum DETECTOR_METHOD
    {
        BGR = 0,
        HSV = 1,
        GREEN = 2,
        GRAY = 3,
        MIXED = 4,
        HSV_BGR_GRAY = 5
    };
    enum ENEMY_COLOR
    {
        BLUE = 1,
        RED = 0,
        WHITE = -1 //误识别
    };

    typedef vector<RotatedRect> LightBarBlobs;

    class LightBarFinder
    {
    public:
        LightBarFinder();
        bool findLightBarBlobs(const Mat &frame, LightBarBlobs &lightBarBlobs, const cv::Point2f &roi_offset = cv::Point2f(0, 0));
        static inline bool isValidLightBarBlob(const RotatedRect &);

        void setEnemyColor(int flag) { enemy_color = (flag & 0x01); };
        int enemy_color = 0;

    private:
        static inline bool checkAspectRatio(const double &);
        static inline bool checkArea(const double &);
        static inline bool checkAngle(const float &angle);
        static inline bool checkRatioAndArea(const double &ratio, const double &area);

        bool isRightColor(const cv::Mat &frame, const RotatedRect &rrect);

        Mat kernel_33;
        Mat kernel_55;
        Mat kernel_35;
    };
}

#endif //AUTOAIM_LIGHTBARFINDER_H
