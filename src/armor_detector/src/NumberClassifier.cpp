#include "NumberClassifier.h"

namespace ly
{
    NumberClassifier::NumberClassifier(/* args */)
    {
        loadModel("../src/utils/tools/svm_numbers.xml");
        initHog();
        calcGammaTable(CameraParam::gamma);
    }

    NumberClassifier::~NumberClassifier()
    {
        delete hog_;
    }
    void NumberClassifier::initHog()
    {
        //窗口大小,块大小，块步长，cell，180度分为几个区间
        hog_ = new HOGDescriptor(cv::Size(32, 32), cv::Size(8, 8), cv::Size(4, 4), cv::Size(4, 4), 6);
    }

    void NumberClassifier::calcGammaTable(float gamma)
    {
        for (int i = 0; i < 256; ++i)
        {
            gamma_table[i] = saturate_cast<uchar>(pow((float)(i / 255.0), gamma) * 255.0f);
        }
        lut_table = Mat(1, 256, CV_8UC1, gamma_table);
    }
    //加载模型
    bool NumberClassifier::loadModel(const std::string &model_path)
    {
        number_svm_model = cv::Algorithm::load<cv::ml::SVM>(model_path);
        if (number_svm_model.empty())
        {
            std::cerr << "Open NUMBER svm model ERROR" << std::endl;
            is_model_set = false; //模型不可用
        }
        else
        {
            is_model_set = true;
        }
        return is_model_set;
    }
    bool NumberClassifier::affineNumber(Mat &frame, const std::vector<cv::Point2f> &corners)
    {
        is_get_roi = false;
        static float classify_width_ratio = 0.2f;
        static float classify_height_ratio = 0.5f;

        //求解完全包围这个框的最大矩形
        cv::Point2f correct_points[4];
        cv::Point2f width_vec = (corners[1] - corners[0] + corners[2] - corners[3]) / 2;
        cv::Point2f height_vec = (corners[3] - corners[0] + corners[2] - corners[1]) / 2;
        correct_points[0] = corners[0] + classify_width_ratio * width_vec - classify_height_ratio * height_vec;
        correct_points[1] = corners[1] - classify_width_ratio * width_vec - classify_height_ratio * height_vec;
        correct_points[2] = corners[2] - classify_width_ratio * width_vec + classify_height_ratio * height_vec;
        correct_points[3] = corners[3] + classify_width_ratio * width_vec + classify_height_ratio * height_vec;

        int width = getDistance(correct_points[0], correct_points[1]);
        int height = getDistance(correct_points[1], correct_points[2]);
        cv::Point2f min_point = cv::Point2f(9999.0f, 9999.0f);
        cv::Point2f max_point = cv::Point2f(0.0f, 0.0f);
        for (int i = 0; i < 4; i++)
        {
            min_point.x = min_point.x < correct_points[i].x ? min_point.x : correct_points[i].x;
            min_point.y = min_point.y < correct_points[i].y ? min_point.y : correct_points[i].y;
            max_point.x = max_point.x > correct_points[i].x ? max_point.x : correct_points[i].x;
            max_point.y = max_point.y > correct_points[i].y ? max_point.y : correct_points[i].y;
        }
        min_point.x = MAX(min_point.x, 0);
        min_point.y = MAX(min_point.y, 0);
        max_point.x = MIN(max_point.x, frame.cols);
        max_point.y = MIN(max_point.y, frame.rows);

        //截取
        cv::Mat m_number_roi = frame(cv::Rect(min_point, max_point));

        for (int i = 0; i < 4; i++)
        {
            correct_points[i] -= min_point;
        }

        // 制作重映射对应点
        cv::Point2f remap_points[4];
        remap_points[0] = cv::Point2f(0, 0);
        remap_points[1] = cv::Point2f((int)width, 0);
        remap_points[2] = cv::Point2f((int)width, (int)height);
        remap_points[3] = cv::Point2f(0, (int)height);

        //进行重映射
        cv::Mat trans_matrix = cv::getPerspectiveTransform(correct_points, remap_points);
        cv::Mat output_roi;
        output_roi.create(cv::Size((int)width, (int)height), CV_8UC3);

        if (m_number_roi.empty() || output_roi.empty())
        {
            return false;
        }
        cv::warpPerspective(m_number_roi, output_roi, trans_matrix, output_roi.size());

        // //从重映射中取得目标图像
        cv::resize(output_roi, number_roi, cv::Size(32, 32)); //根据训练的数据大小来判断大小
        is_get_roi = true;
        return is_get_roi;
    }
    float NumberClassifier::getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2)
    {
        float x = (point_1 - point_2).x;
        float y = (point_1 - point_2).y;
        return sqrt(x * x + y * y);
    }
    int NumberClassifier::predict(Mat &frame, const std::vector<cv::Point2f> &corners)
    {
        if (!affineNumber(frame, corners) || number_roi.empty())
        {
            return -1;
        }
        // cv::LUT(number_roi, lut_table, number_roi);

        cvtColor(number_roi, number_roi, COLOR_BGR2GRAY);
        cv::LUT(number_roi, lut_table, number_roi);

        //不保存图片记得注释掉
        if (DetectorParam::is_save_classify)
        {
            save = number_roi.clone();
        }

        // cv::Scalar roi_mean = mean(number_roi);
        // if (roi_mean(0) > ROI_MEAN_THRESH || roi_mean(0) < ROI_THRESH_MIN)
        // {
        //     // LOG(ERROR) << "0 DUE TO ROI MEAN";
        //     return 0;
        // }
        // LOG(INFO) << "ROI MEAN" << roi_mean;

        // GaussianBlur(number_roi, number_roi, Size(3, 3), 0);
        // threshold(number_roi, number_roi, 0, 255, THRESH_BINARY | THRESH_OTSU);

        std::vector<float> hog_descriptors;
        //对图片提取hog描述子存在hog_descriptors中，hog描述子是向量，不是矩阵
        hog_->compute(number_roi, hog_descriptors);
        size_t size = hog_descriptors.size();
        cv::Mat descriptors_mat(1, size, CV_32FC1); //行向量
        for (size_t i = 0; i < hog_descriptors.size(); ++i)
        {
            descriptors_mat.at<float>(0, i) = hog_descriptors[i];
        }

        if (ArmorDetectParam::show_classify)
        {
            armor_to_show = number_roi.clone();
        }
        // number_roi.convertTo(number_roi, CV_32FC1);
        // number_roi = number_roi.reshape(0, 1);
        //把提取的本张图片的hog描述子放进svm预测器中进行预测
        number_svm_model->predict(descriptors_mat, class_);

        // number_svm_model->predict(number_roi, class_);
        // DLOG(INFO) << "armor: " << class_;
        saveImage();
        if ((int)class_.at<float>(0) > 0)
        {
            showNumber();
        }

        return (int)class_.at<float>(0);
    }
    void NumberClassifier::showNumber()
    {
        if (ArmorDetectParam::show_classify)
        {
            DEBUG_MODE(imshow("armor", armor_to_show));
            cv::waitKey(1);
        }
    }
    void NumberClassifier::showNumberStr(cv::Mat &drawing, int id, cv::Rect rect)
    {
        if (ArmorDetectParam::debug_show)
        {
            putText(drawing, to_string(id) + ":" + armor_classify_type[id], cv::Point2f(rect.x, rect.y), 5, 1, Scalar(0, 255, 0), 2);
        }
    }
    void NumberClassifier::saveImage()
    {
        if (DetectorParam::is_save_classify)
        {
            static int save_id = 0;
            static int id = 0;
            if (save_id++ % 100 == 0)
            {
                imwrite(CameraParam::picture_path + "tmp_" + to_string((int)class_.at<float>(0)) + "/" + to_string(clock()) + "_" + to_string(id++) + ".png", save);
            }
        }
    }
}