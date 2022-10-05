#ifndef NUMBER_CLASSIFIER_H
#define NUMBER_CLASSIFIER_H
#include "Config.h"
#include "opencv2/opencv.hpp"
#include "opencv2/ml/ml.hpp"
#include "ArmorFinder.h"
#include <string>
#include "EnemyType.h"
#define ROI_MEAN_THRESH 185
#define ROI_THRESH_MIN 20
namespace ly
{
    class NumberClassifier
    {
    private:
        /* data */
        void calcGammaTable(float gamma);
        bool loadModel(const std::string &model_path);

        uchar gamma_table[256];
        cv::Mat lut_table;

        bool is_model_set;
        cv::Ptr<cv::ml::SVM> number_svm_model;

        bool is_get_roi;
        cv::Mat number_roi;
        cv::Mat class_;

        bool affineNumber(Mat &frame, const std::vector<cv::Point2f> &corners);
        float getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2);
        void saveImage();
        void showNumber();

        std::string armor_classify_type[9] = {"Undefined", "Hero", "Engineer", "Infantry", "Infantry", "Infantry", "Sentry", "Outpost", "Base"};

        //debug用
        cv::Mat armor_to_show;
        cv::Mat save;

        cv::HOGDescriptor *hog_;
        void initHog();

    public:
        NumberClassifier(/* args */);
        ~NumberClassifier();
        int predict(Mat &frame, const std::vector<cv::Point2f> &corners);
        void showNumberStr(cv::Mat &drawing, int id, cv::Rect rect);
    };
}

#endif