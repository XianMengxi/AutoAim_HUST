//
// Created by zhiyu on 2021/8/24.
//

#ifndef AUTOAIM_ARMORFINDER_H
#define AUTOAIM_ARMORFINDER_H

#include "opencv2/opencv.hpp"
#include <vector>
#include "Params.h"

#include "Log.h"

using namespace std;
using namespace cv;

namespace ly
{
    static bool status = false;

    class ArmorBlob
    {
    public:
        ArmorBlob()
        {
            corners = vector<Point2f>(4);
        }

        Rect rect;

        vector<Point2f> corners;
        int _class;
    };

    typedef vector<ArmorBlob> ArmorBlobs;

    class ArmorFinder
    {
    public:
        bool judgeArmor(const ArmorBlob &);
        bool matchTwoLightBar(const RotatedRect &, const RotatedRect &);
        bool getArmor(const RotatedRect &, const RotatedRect &, ArmorBlob &armor);
        int filteArmor(const RotatedRect &, const ArmorBlob &);

    private:
        inline float getAngle(const RotatedRect &);
        inline bool checkAngleDiff(const RotatedRect &l, const RotatedRect &r);
        inline bool checkHeightDiff(const RotatedRect &l, const RotatedRect &r);
    };
}

#endif //AUTOAIM_ARMORFINDER_H
