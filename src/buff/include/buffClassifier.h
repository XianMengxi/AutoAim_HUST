#ifndef _BUFF_CLASSIFIER_H
#define _BUFF_CLASSIFIER_H
#include "opencv2/opencv.hpp"
#include "opencv2/ml/ml.hpp"
#include <string>
#include <iostream>
#include "Config.h"
namespace ly
{
    class BuffClassifier
    {
    private:
        cv::Ptr<cv::ml::SVM> buff_svm_model;

        //signal
        bool is_model_set;
        bool is_get_buff_roi;

        //params
        const float min_area_thresh = 4000.0f; //用于去掉较小的矩形

        bool loadModel(const std::string &model_path);

        void tranformBuffROI(const cv::Mat &buff_image, const cv::RotatedRect &father_rect);
        void tranformBuffROI(const cv::Mat &buff_image, const cv::RotatedRect &armor_rect, const cv::Point2f &r_center);

        float getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2);
        void saveImage();

        //regist
        cv::Mat buff_roi;
        cv::Mat classify_result;
        cv::Mat save;

    public:
        BuffClassifier(/* args */);
        BuffClassifier(const std::string &model_path);
        ~BuffClassifier();

        //0:未击打的大符页(锤子)，1:已经击打的大符页(大宝剑)
        //传进的是整张图片而不是ROI，旋转矩形是大符外框轮廓的minAreaRect()
        int predict(const cv::Mat &buff_image, const cv::RotatedRect &father_rect);
        int predict(const cv::Mat &buff_image, const cv::RotatedRect &armor_rect, const cv::Point2f &r_center);
    };
}

#endif