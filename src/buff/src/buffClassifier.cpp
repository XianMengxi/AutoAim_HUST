#include "buffClassifier.h"
namespace ly
{
    BuffClassifier::BuffClassifier() //也可以选取参数配置地址，只需要重载构造函数即可
    {
        loadModel("../model/buff_model.xml");
    }
    BuffClassifier::~BuffClassifier()
    {
    }
    BuffClassifier::BuffClassifier(const std::string &model_path)
    {
        loadModel(model_path);
    }

    //加载模型
    bool BuffClassifier::loadModel(const std::string &model_path)
    {
        buff_svm_model = cv::Algorithm::load<cv::ml::SVM>(model_path);
        if (buff_svm_model.empty())
        {
            std::cerr << "Open BUFF svm model ERROR" << std::endl;
            is_model_set = false; //模型不可用
        }
        else
        {
            is_model_set = true;
        }
        return is_model_set;
    }

    int BuffClassifier::predict(const cv::Mat &buff_image, const cv::RotatedRect &father_rect)
    {
        if (buff_image.empty())
        {
            return -1;
        }
        tranformBuffROI(buff_image, father_rect);
        if (!is_get_buff_roi || !is_model_set)
        {
            return -1; //errors happen
        }

        cv::Mat f_buff_roi;
        cv::Mat reshape_buff_roi;
        if (DetectorParam::is_save_classify)
        {
            save = buff_roi.clone();
        }
        buff_roi.convertTo(f_buff_roi, CV_32FC1);
        reshape_buff_roi = f_buff_roi.reshape(1, 1);
        buff_svm_model->predict(reshape_buff_roi, classify_result);
        saveImage();
        return (int)classify_result.at<float>(0);
    }
    int BuffClassifier::predict(const cv::Mat &buff_image, const cv::RotatedRect &armor_rect, const cv::Point2f &r_center)
    {
        cv::Mat copy;
        if (buff_image.empty())
        {
            return -1;
        }
        tranformBuffROI(buff_image, armor_rect, r_center);
        if (!is_get_buff_roi || !is_model_set)
        {
            return -1; //errors happen
        }
        if (ArmorDetectParam::show_classify)
        {
            cv::imshow("classify", buff_roi);
            cv::waitKey(1);
        }
        if (DetectorParam::is_save_classify)
        {
            save = buff_roi.clone();
        }

        cv::Mat f_buff_roi;
        cv::Mat reshape_buff_roi;
        buff_roi.convertTo(f_buff_roi, CV_32FC1);
        reshape_buff_roi = f_buff_roi.reshape(1, 1);
        buff_svm_model->predict(reshape_buff_roi, classify_result);
        saveImage();

        return (int)classify_result.at<float>(0);
        // return 0;
    }
    void BuffClassifier::saveImage()
    {
        if (DetectorParam::is_save_classify)
        {
            static int save_id = 0;
            static int id = 0;
            if (save_id++ % 20 == 0)
            {
                cv::imwrite(CameraParam::picture_path + "tmp_" + to_string((int)classify_result.at<float>(0) + 9) + "/" + to_string(clock()) + "_" + to_string(id++) + ".png", save);
            }
        }
    }

    //计算欧式距离
    float BuffClassifier::getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2)
    {
        float x = (point_1 - point_2).x;
        float y = (point_1 - point_2).y;
        return sqrt(x * x + y * y);
    }
    void BuffClassifier::tranformBuffROI(const cv::Mat &buff_image, const cv::RotatedRect &armor_rect, const cv::Point2f &r_center)
    {
        cv::Point2f r_2_center = armor_rect.center - r_center;
        float k = 0.15;
        float k_ = 1.6;
        float distance = getDistance(armor_rect.center, r_center);
        cv::RotatedRect father_rect;
        if (armor_rect.size.width > armor_rect.size.height)
        {
            father_rect = cv::RotatedRect((armor_rect.center + r_center) / 2 + r_2_center * k, cv::Size2f(armor_rect.size.width * k_, distance), armor_rect.angle);
        }
        else
        {
            father_rect = cv::RotatedRect((armor_rect.center + r_center) / 2 + r_2_center * k, cv::Size2f(distance, armor_rect.size.height * k_), armor_rect.angle);
        }
        tranformBuffROI(buff_image, father_rect);
    }

    void BuffClassifier::tranformBuffROI(const cv::Mat &buff_image, const cv::RotatedRect &father_rect)
    {
        //获取旋转矩形的四个点
        cv::Point2f father_points[4];
        cv::Point2f correct_points[4];
        father_rect.points(father_points);

        //对这些点进行矫正
        float width = getDistance(father_points[0], father_points[1]);
        float height = getDistance(father_points[1], father_points[2]);

        //去除面积较小的区域
        float area = width * height;
        if (area < min_area_thresh) //5000.0f
        {
            DLOG(WARNING)<<"AREA NOT RIGHT";
            is_get_buff_roi = false;
            return;
        }

        if (width > height)
        {
            for (int i = 0; i < 4; i++)
            {
                correct_points[i] = father_points[i];
            }
        }
        else
        {
            std::swap(width, height);
            for (int i = 0; i < 4; i++)
            {
                correct_points[i] = father_points[(i + 1) % 4];
            }
        }

        //求解完全包围这个框的最大矩形
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
        max_point.x = MIN(max_point.x, buff_image.cols);
        max_point.y = MIN(max_point.y, buff_image.rows);

        //截取
        cv::Mat buff_fan_roi = buff_image(cv::Rect2f(min_point, max_point));

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
        cv::Mat output_buff;
        output_buff.create(cv::Size((int)width, (int)height), CV_8UC1);
        cv::warpPerspective(buff_fan_roi, output_buff, trans_matrix, output_buff.size());

        // //从重映射中取得目标图像
        cv::resize(output_buff, buff_roi, cv::Size(120, 50)); //根据训练的数据大小来判断大小
        is_get_buff_roi = true;
    }
};