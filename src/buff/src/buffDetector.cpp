#include "buffDetector.h"
namespace ly
{
    BuffDetector::BuffDetector()
    {
        //params
        buff_color = BLUE_BUFF; //0:蓝色，可以写另外的构造函数将参数传进来

        //some params should be set
        binary_thresh = 80;
        color_thresh = 80;
        armor_size_min = 80;
        armor_size_max = 200;
        armor_father_size_min = 350;
        armor_ratio_min = 1.3;
        armor_ratio_max = 3;

        //buff_center detect
        buff_center_size_min = 40;
        buff_center_size_max = 100;
        buff_center_radius_max = 15.0;
        buff_center_radius_min = 6.0;
        buff_rotate_radius_min = 100.0f;
        buff_rotate_radius_max = 500.0f;

        is_find_armor = false;        //1个或者多个装甲板
        is_get_one_armor = false;     //只有一个装甲板，可以避免分类
        is_get_aim_armor = false;     //是否获得分类之后的装甲板
        is_get_buff_location = false; //获得buff的基准向量
        is_get_center = false;        //是否获得了圆心

        buff_classifier = new BuffClassifier();
        predictor = new BuffPredictor();
    }
    BuffDetector::BuffDetector(const BuffParam &buff_params)
    {
        if (DetectorParam::color == "red")
        {
            buff_color = RED_BUFF;
        }
        else
        {
            buff_color = BLUE_BUFF;
        }
        binary_thresh = buff_params.binary_thresh;
        color_thresh = buff_params.color_thresh;
        armor_size_min = buff_params.armor_size_min;
        armor_size_max = buff_params.armor_size_max;
        armor_father_size_min = buff_params.armor_father_size_min;
        armor_father_size_max = buff_params.armor_father_size_max;
        armor_ratio_min = buff_params.armor_ratio_min;
        armor_ratio_max = buff_params.armor_ratio_max;

        //buff center detect
        buff_center_size_min = buff_params.buff_center_size_min;
        buff_center_size_max = buff_params.buff_center_size_max;
        buff_center_radius_max = buff_params.buff_center_radius_max;
        buff_center_radius_min = buff_params.buff_center_radius_min;
        buff_rotate_radius_min = buff_params.buff_rotate_radius_min;
        buff_rotate_radius_max = buff_params.buff_rotate_radius_max;

        is_find_armor = false;
        is_get_one_armor = false;

        buff_classifier = new BuffClassifier(buff_params.model_path);
        predictor = new BuffPredictor();
    }
    BuffDetector::~BuffDetector()
    {
        delete buff_classifier;

        delete predictor;
    }
    //建系
    //做这个的原因是更加精确地分辨装甲板这几个点的位置，确保对应点正确
    /* (y指向圆心，x向右)
        |
        |
        |------->
    */
    void BuffDetector::NewPredictor()
    {
        delete predictor;
        predictor = new BuffPredictor();
    }
    void BuffDetector::sortArmorPoints(BuffArmor &buff_armor_node)
    {
        //旋转theta - pi/2 度,求得旋转角
        float theta = -atan2(buff_armor_node.direction_vec.y, buff_armor_node.direction_vec.x);
        theta -= CV_PI / 2;
        float sin_theta = sin(theta);
        float cos_theta = cos(theta);
        // std::cout << "theta" << theta << std::endl;

        cv::Point2f armor_points[4];
        buff_armor_node.armor_rect.points(armor_points);

        //旋转采点
        for (int i = 0; i < 4; i++)
        {
            cv::Point2f temp_point = armor_points[i] - buff_armor_node.armor_rect.center;

            //计算旋转后的点，根据点来判断PNP点的分布
            float point_x = temp_point.x * cos_theta - temp_point.y * sin_theta;
            float point_y = temp_point.x * sin_theta + temp_point.y * cos_theta;

            if (point_x > 0 && point_y > 0)
            {
                buff_armor_node.corner[2] = armor_points[i];
                buff_armor_node.corner[0] = armor_points[(i + 2) % 4];
            }
            else if (point_x < 0 && point_y > 0)
            {
                buff_armor_node.corner[3] = armor_points[i];
                buff_armor_node.corner[1] = armor_points[(i + 2) % 4];
            }
            // std::cout << i << ":  " << armor_points[i] << std::endl;
        }
        //用靠近装甲板的线计算角度
        cv::Point2f armor_line = buff_armor_node.corner[2] - buff_armor_node.corner[3];

        buff_armor_node.angle = atan2(armor_line.x, armor_line.y);

        // std::cout << atan2(armor_line.x, armor_line.y) << std::endl;
        //debug
        // for (int i = 0; i < 4; i++)
        // {
        //     std::cout << i << ":  " << buff_armor_node.corner[i] << std::endl;
        // }
    }
    void BuffDetector::updateArmor()
    {
        last_target = aim_target;
    }
    void BuffDetector::setColor(const char &flag)
    {
        if ((flag & 0x01) == 1)
        {
            buff_color = RED_BUFF;
        }
        else
        {
            buff_color = BLUE_BUFF;
        }
    }
    void BuffDetector::imageProcess(const cv::Mat &frame, cv::Mat &output_frame)
    {
        if (frame.empty())
        {
            return;
        }
        cv::Mat gray;
        cv::Mat mat;
        cv::Mat thresh_gray;
        std::vector<cv::Mat> channels;

        split(frame, channels);
        if (buff_color == BLUE_BUFF)
        {
            subtract(channels[0], channels[2], output_frame);
        }
        else if (buff_color == RED_BUFF)
        {
            subtract(channels[2], channels[0], output_frame);
        }
        cv::Mat kernel33 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        cv::threshold(output_frame, mat, color_thresh, 255, cv::THRESH_BINARY);
        cv::dilate(mat, mat, kernel33);

        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);                         //转灰度图
        threshold(gray, thresh_gray, binary_thresh, 255, cv::THRESH_BINARY); //二值化分割

        mat = thresh_gray & mat;

        // erode(mat, mat, kernal); //去除噪点，考虑不要腐蚀(在黑暗的环境不需要，白天环境可能需要)
        // dilate(mat, output_frame, kernal);
        output_frame = mat;

        if (output_frame.empty())
        {
            return;
        }

        if (ArmorDetectParam::show_thresh)
        {
            cv::imshow("thresh", output_frame);
            cv::waitKey(1);
        }
    }
    const BuffArmor &BuffDetector::runBuffDetect(const cv::Mat &frame)
    {
        //保存上一帧装甲板信息
        updateArmor();

        if (frame.empty())
        {
            std::cerr << "GET EMPTY FRAME" << std::endl;
            return aim_target;
        }

        //清空装甲板队列
        emptyArmorQueue();

        //作图像处理
        cv::Mat img_processed;
        imageProcess(frame, img_processed);

        // svm数据采集
        // getBuffDataSet(img_processed);

        //查找装甲板
        findBuffArmor(frame, img_processed);

        // //找到目标的装甲板
        judgeRightArmor(img_processed);

        //获得大符的象限位置
        getBuffLocation();

        //对装甲板的点进行排序
        sortArmorPoints(aim_target);

        return aim_target;
    }

    //初步方向矢量，后续还需要进行校正
    void BuffDetector::getBuffLocation()
    {
        if (!is_get_aim_armor)
        {
            is_get_buff_location = false;
            return;
        }
        //直接获得向量准确度不高
        aim_target.direction_vec = aim_target.armor_rect.center - aim_target.buff_center;

        is_get_buff_location = true;
    }
    void BuffDetector::getBuffDataSet(const cv::Mat &img_processed)
    {
        //查找轮廓
        static int j = 0;

        //保存图片位置
        static std::string buff_str = "../buff_pic/buff_";
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img_processed, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
        for (int i = 0; i < contours.size(); i++)
        {
            //筛选
            if (contours[i].size() < 350) //筛选面积比较大的轮廓
            {
                continue;
            }

            //获取旋转矩形的四个点
            cv::RotatedRect father_rect = cv::minAreaRect(contours[i]);
            cv::Point2f father_points[4];
            cv::Point2f correct_points[4];
            father_rect.points(father_points);

            //对这些点进行矫正
            float width = getDistance(father_points[0], father_points[1]);
            float height = getDistance(father_points[1], father_points[2]);

            //去除面积较小的区域
            float area = width * height;
            if (area < 5000.0f)
            {
                continue;
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

            //制作重映射对应点
            cv::Point2f remap_points[4];
            remap_points[0] = cv::Point2f(0, 0);
            remap_points[1] = cv::Point2f(width, 0);
            remap_points[2] = cv::Point2f(width, height);
            remap_points[3] = cv::Point2f(0, height);

            //进行重映射
            cv::Mat trans_matrix = cv::getPerspectiveTransform(correct_points, remap_points);
            cv::Mat output_buff;
            cv::warpPerspective(img_processed, output_buff, trans_matrix, img_processed.size());
            // cv::imshow("buff", output_buff);
            // cv::waitKey(0);

            // //从重映射中取得目标图像
            cv::Mat buff_roi;
            buff_roi = output_buff(cv::Rect(0, 0, width, height));
            cv::imwrite(buff_str + std::to_string(j) + ".jpg", buff_roi);
            j++;
            // cv::imshow("buff_roi", buff_roi);
            // cv::waitKey(0);
        }
    }
    void BuffDetector::findBuffArmor(const cv::Mat &raw_pic, const cv::Mat &img_processed)
    {
        //查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img_processed, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

        //筛选轮廓
        for (int i = 0; i < contours.size(); i++)
        {
            //子轮廓不能太小，父轮廓不能太小
            if (hierarchy[i][3] < 0 || contours[static_cast<uint>(hierarchy[i][3])].size() < 6) //父轮廓太小
            {
                continue;
            }
            if (contours[i].size() < armor_size_min || contours[i].size() > armor_size_max ||
                contours[static_cast<uint>(hierarchy[i][3])].size() < armor_father_size_min
                /*||
                contours[static_cast<uint>(hierarchy[i][3])].size() > armor_father_size_max*/
                //不带最大面积，因为可能有背景影响
                ) //装甲板轮廓和父轮廓在一个区间内
            {
                continue;
            }

            //长宽比筛选
            cv::RotatedRect contour_rect = cv::minAreaRect(contours[i]);
            float length_width_ratio = MAX(contour_rect.size.height, contour_rect.size.width) /
                                       MIN(contour_rect.size.height, contour_rect.size.width);
            if (length_width_ratio > armor_ratio_min && length_width_ratio < armor_ratio_max) //子轮廓长宽比
            {
#ifdef COUT_BUFF_INFO
                std::cout << "armor_size" << contours[i].size() << std::endl;
                std::cout << "armor_father_size" << contours[static_cast<uint>(hierarchy[i][3])].size() << std::endl;
                std::cout << "armor_ratio" << length_width_ratio << std::endl;
#endif
                // std::cout << "armor_ratio: " << length_width_ratio << std::endl;
                BuffArmor armor_candidate;
                armor_candidate.armor_rect = contour_rect;
                armor_candidate.armor_ratio = length_width_ratio;
                armor_candidate.armor_father_rect = cv::minAreaRect(contours[static_cast<uint>(hierarchy[i][3])]);
                targets.push_back(armor_candidate);
            }
            else
            {
#ifdef COUT_BUFF_INFO
                std::cout << "armor_ratio: " << length_width_ratio << std::endl;
#endif
            }
        }
        std::cout << "armor target size" << targets.size() << std::endl;
        if (targets.size() >= 1) //有目标
        {
            findCenter(raw_pic, contours, hierarchy, targets);
        }
        if (targets.size() == 1)
        {
            is_find_armor = true;
            is_get_one_armor = true;
        }
        else if (targets.size() > 1)
        {
            is_find_armor = true;
            is_get_one_armor = false;
        }
        else
        {
            LOG(ERROR) << "NOT GET ARMOR";
            is_find_armor = false;
            is_get_one_armor = false;
        }
        // std::cout << "armor size " << targets.size() << std::endl;
        // //用作选择最近的击打，在家里由于有多个目标，容易直接切换
        // float max_y = -1.0;
        // int target_index = -1;
        // for (int i = 0; i < targets.size(); i++)
        // {
        //     // float dis = getDistance(targets[i].armor_rect.center, cv::Point2f(640, 512));
        //     if (targets[i].armor_rect.center.y > max_y)
        //     {
        //         // min_distance = dis;
        //         max_y = targets[i].armor_rect.center.y;
        //         target_index = i;
        //     }
        // }
        // if (target_index > -1)
        // {
        //     aim_target = targets[target_index];
        //     aim_target.is_useful = true;
        // }
        // else
        // {
        //     aim_target.is_useful = false;
        // }
        // std::cout << "armor size " << targets.size() << std::endl;
        // std::cout << "center: " << aim_target.armor_rect.center << std::endl;
        // findCenter(contours, hierarchy);
    }
    //找圆心R
    void BuffDetector::findCenter(const cv::Mat &raw_pic, const std::vector<std::vector<cv::Point>> &contours, std::vector<cv::Vec4i> hierarchy, const std::vector<BuffArmor> &targets)
    {
        std::vector<cv::Point2f>().swap(buff_center_candidate); //清空装甲板
        for (int i = 0; i < contours.size(); i++)
        {
            bool is_center_valid = true;
            //轮廓大小满足一定的关系
            //不能有父轮廓
            if (contours[i].size() < buff_center_size_min || contours[i].size() > buff_center_size_max || hierarchy[i][3] >= 0)
            {
                // std::cout << "buff_center_size_min not right" << std::endl;
                continue;
            }

            //找圆心包围的轮廓
            cv::Point2f center;
            float radius;
            minEnclosingCircle(contours[i], center, radius);

            if (radius > buff_center_radius_max || radius < buff_center_radius_min)
            {
                // std::cout << "buff_center_radius_max not right" << std::endl;

                continue;
            }
            if (!VerifyPitch(center))
            {
                is_center_valid = false;
            }
            float distance;
            for (int i = 0; i < targets.size(); i++)
            {
                // std::cout << getDistance(center, targets[i].armor_rect.center) << std::endl;
                distance = getDistance(center, targets[i].armor_rect.center);
                if (distance < buff_rotate_radius_min || distance > buff_rotate_radius_max)
                {
                    // std::cout << "buff_rotate_radius_min not right" << std::endl;

                    is_center_valid = false;
                }
                VerifyPitch(targets[i].armor_rect.center);
                if (!VerifyAngle(targets[i].armor_rect, center))
                {
                    // std::cout << "VerifyAngle not right" << std::endl;

                    is_center_valid = false;
                }
            }
            //if (!VerifyColor(raw_pic, contours[i]))
            //{
                //is_center_valid = false;
            //}
            if (is_center_valid)
            {
                // std::cout << "distance: " << distance << std::endl;
                // std::cout << "radius: " << radius << std::endl;
                // std::cout << "contours[i].size()" << contours[i].size() << std::endl;
                buff_center_candidate.emplace_back(center);
            }
        }
        if (buff_center_candidate.size() == 1) //找到唯一的圆心
        {
            // std::cout << "get one center" << std::endl;
            is_get_center = true;
        }
        else if (buff_center_candidate.size() <= 0) //无圆心
        {
            LOG(ERROR) << "NOT GET CENTER";
            // std::cout << "not get center" << std::endl;
            is_get_center = false;
        }
        else //多个圆心，如何筛选？
        {
            LOG(ERROR) << "GET MANY CENTER";

            // std::cout << "get many center" << std::endl;
            is_get_center = false;
        }
    }
    bool BuffDetector::VerifyColor(const cv::Mat &raw_pic, const std::vector<cv::Point> &countour_points)
    {
        cv::RotatedRect rotate_rect = cv::minAreaRect(countour_points);

        cv::Rect rect = rotate_rect.boundingRect();
        rect.x = rect.x < 0 ? 0 : (rect.x > raw_pic.cols ? raw_pic.cols : rect.x);
        rect.y = rect.y < 0 ? 0 : (rect.y > raw_pic.rows ? raw_pic.rows : rect.y);
        rect.width = rect.x + rect.width > raw_pic.cols ? raw_pic.cols - rect.x : rect.width;
        rect.height = rect.y + rect.height > raw_pic.rows ? raw_pic.rows - rect.y : rect.height;
        cv::Scalar roi_color_bgr_mean = cv::mean(raw_pic(rect));
        if (buff_color == BLUE_BUFF)
        {
            float b_sub_g_margin = 20.0;
            // std::cout << "roi_color: " << roi_color_bgr_mean[0] - roi_color_bgr_mean[1] << std::endl;

            return roi_color_bgr_mean[0] > roi_color_bgr_mean[1] + b_sub_g_margin;
        }
        else
        {
            float r_sub_g_margin = 20.0;
            std::cout << "roi_color: " << roi_color_bgr_mean[2] - roi_color_bgr_mean[1] << std::endl;
            return roi_color_bgr_mean[2] > roi_color_bgr_mean[1] + r_sub_g_margin;
        }
    }
    bool BuffDetector::VerifyPitch(const cv::Point2f &center)
    {
        const static double fy = CameraParam::fy;
        const static double v0 = CameraParam::v0;
        double y = (center.y - v0) / fy;
        float delta_pitch = atan(y);
        float center_pitch = this_pitch - delta_pitch;
        // std::cout << "center_pitch: " << center_pitch << std::endl;

        return center_pitch > 0.08 && center_pitch < 0.25;
    }
    bool BuffDetector::VerifyDistance(const Eigen::Vector3d &armor_pose)
    {
        return armor_pose.norm() < 9 && armor_pose.norm() > 6;
    }

    bool BuffDetector::VerifyAngle(const cv::RotatedRect &rect, const cv::Point2f &center)
    {
        cv::Point2f r_2_center = center - rect.center;
        float angle = atan2(r_2_center.y, r_2_center.x);
        float rect_angle;
        if (rect.size.width > rect.size.height)
        {
            rect_angle = rect.angle + 90;
        }
        else
        {
            rect_angle = rect.angle;
        }
        return fabs(sin(rect_angle * M_PI / 180 - angle)) < 0.15;
    }
    void BuffDetector::emptyArmorQueue()
    {
        std::vector<BuffArmor>().swap(targets);
    }
    float BuffDetector::getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2)
    {
        float x = (point_1 - point_2).x;
        float y = (point_1 - point_2).y;
        return sqrt(x * x + y * y);
    }

    void BuffDetector::judgeRightArmor(const cv::Mat &img_processed)
    {
        int _class = -1;
        if (is_find_armor && is_get_center) //至少有两个或者以上的装甲板,而且必须得找到圆心
        {
            //对装甲板排序，离上一帧目标最近的先进行分类
            std::sort(targets.begin(), targets.end(), [&](const ly::BuffArmor &a, const ly::BuffArmor &b)
                      { return getDistance(a.armor_rect.center, last_target.armor_rect.center) < getDistance(b.armor_rect.center, last_target.armor_rect.center); });
            for (int i = 0; i < targets.size(); i++)
            {
                //直接使用父轮廓不太行
                _class = buff_classifier->predict(img_processed, targets[i].armor_rect, buff_center_candidate[0]);
                if (_class == 0)
                {
                    aim_target = targets[i];
                    aim_target.buff_center = buff_center_candidate[0];
                    aim_target.is_useful = true;
                    is_get_aim_armor = true;
                    break;
                }
                else
                {
                    LOG(WARNING) << "GET ID != 0";
                    aim_target.is_useful = false;
                }
            }
        }
        else
        {
            LOG(WARNING) << "NO TARGET || NO CENTER";
            is_get_aim_armor = false;
            aim_target.is_useful = false;
        }
    }
    void BuffDetector::drawAngle(cv::Mat &frame, cv::RotatedRect rect, cv::Point2f center)
    {
        // cv::Point2f r_2_center = center - rect.center;
        // float angle = atan2(r_2_center.y, r_2_center.x);
        // float rect_angle;
        // if (rect.size.width > rect.size.height)
        // {
        //     rect_angle = rect.angle + 90;
        // }
        // else
        // {
        //     rect_angle = rect.angle;
        // }
        // cv::putText(frame, "angle_to_center: " + to_string(angle), cv::Point(10, 60), 0, 1, cv ::Scalar(255, 255, 0));
        // cv::putText(frame, "rect angle: " + to_string(rect_angle * M_PI / 180), cv::Point(500, 60), 0, 1, cv ::Scalar(255, 255, 0));
        // cv::putText(frame, "diff_angle: " + to_string(fabs(sin(rect_angle * M_PI / 180 - angle))), cv::Point(900, 60), 0, 1, cv ::Scalar(255, 255, 0));
        cv::putText(frame, "angle: " + to_string(aim_target.angle), cv::Point(10, 60), 0, 1, cv ::Scalar(255, 255, 0));
    }
    void BuffDetector::drawFatherRect(cv::Mat &frame, const BuffArmor &aim_target_)
    {
        if (is_get_center)
        {
            cv::Point2f r_2_center = aim_target_.armor_rect.center - buff_center_candidate[0];
            float k = 0.15;
            float k_ = 1.3;
            float distance = getDistance(aim_target_.armor_rect.center, buff_center_candidate[0]);
            cv::RotatedRect father_rect;
            if (aim_target_.armor_rect.size.width > aim_target_.armor_rect.size.height)
            {
                father_rect = cv::RotatedRect((aim_target_.armor_rect.center + buff_center_candidate[0]) / 2 + r_2_center * k, cv::Size2f(aim_target_.armor_rect.size.width * k_, distance), aim_target_.armor_rect.angle);
            }
            else
            {
                father_rect = cv::RotatedRect((aim_target_.armor_rect.center + buff_center_candidate[0]) / 2 + r_2_center * k, cv::Size2f(distance, aim_target_.armor_rect.size.height * k_), aim_target_.armor_rect.angle);
            }

            cv::Point2f points[4];
            father_rect.points(points);
            for (int i = 0; i < 4; i++)
            {
                cv::line(frame, points[i], points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
            }
        }
    }

    //根据delta_t以及先验知识来计算
    //三维拟合版本
    // float BuffDetector::runBuffPredict(const Eigen::Vector3d &armor_pose, const std::chrono::_V2::steady_clock::time_point &catch_armor_time)
    // {
    //     aim_target.world_pose = armor_pose;
    //     aim_target.time_stamp = catch_armor_time;
    //     return predictor->runBuffPredict(aim_target);
    // }
    void BuffDetector::setPredictMode(int type)
    {
        predictor->setPredictMode(type);
    }
    // float BuffDetector::runLargeBuffPredict(const Eigen::Vector3d &armor_pose, const std::chrono::_V2::steady_clock::time_point &catch_armor_time)
    // {
    //     aim_target.world_pose = armor_pose;
    //     aim_target.time_stamp = catch_armor_time;
    //     return predictor->runLargeBuffPredict(aim_target);
    // }
    // float BuffDetector::runSmallBuffPredict(const Eigen::Vector3d &armor_pose, const std::chrono::_V2::steady_clock::time_point &catch_armor_time)
    // {
    //     aim_target.world_pose = armor_pose;
    //     aim_target.time_stamp = catch_armor_time;
    //     return predictor->runSmallBuffPredict(aim_target);
    // }
    //传进PNP计算好的三维位姿
    //二维找圆心版本
    // const cv::Point2f &BuffDetector::runBuffPredict(const Eigen::Vector3d &armor_pose, std::chrono::steady_clock::time_point *catch_armor_time)
    // {
    //     aim_target.world_pose = armor_pose;
    //     aim_target.time_stamp = std::chrono::steady_clock::now(); //@TODO：时间戳与外部同步
    //     // aim_target.time_stamp = *catch_armor_time;
    //     float angle = predictor->runBuffPredict(aim_target);
    //     float r = getDistance(buff_center, aim_target.armor_rect.center);
    //     predict_point.x = buff_center.x + r * cos(angle);
    //     predict_point.y = buff_center.y - r * sin(angle);
    //     return predict_point;
    // }
    void BuffDetector::drawPredict(cv::Mat &frame, const float &angle)
    {
        cv::circle(frame, predict_point, 2, cv::Scalar(255, 0, 0), 2);
    }

    void BuffDetector::drawCountour(cv::Mat &frame, cv::Mat &img_processed)
    {
        //查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img_processed, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
        cv::drawContours(frame, contours, -1, cv::Scalar(120, 120, 120));

        cv::imshow("draw coutour", frame);
        cv::waitKey(10);
    }
    void BuffDetector::drawAngle(cv::Mat &frame)
    {
        if (is_get_aim_armor)
        {
            cv::line(frame, aim_target.armor_rect.center, aim_target.armor_rect.center - aim_target.direction_vec, cv::Scalar(0, 255, 0), 2);
        }
    }
    void BuffDetector::drawArmor(cv::Mat &frame, const cv::RotatedRect &rect)
    {
        if (!is_find_armor)
        {
            return;
        }
        cv::Point2f armor_points[4];
        rect.points(armor_points);
        for (int j = 0; j < 4; j++)
        {
            line(frame, armor_points[j], armor_points[(j + 1) % 4], cv::Scalar(0, 255, 0), 2, 8);
        }
        // line(frame, armor_points[0], armor_points[1], cv::Scalar(0, 255, 0), 2, 8);
        // line(frame, armor_points[1], armor_points[2], cv::Scalar(0, 255, 255), 2, 8);
        // cv::imshow("buff", frame);
        // cv::waitKey(0);
    }
    void BuffDetector::drawArmor(cv::Mat &frame, cv::Point2f *points)
    {
        if (!is_find_armor)
        {
            return;
        }
        for (int j = 0; j < 4; j++)
        {
            line(frame, points[j], points[(j + 1) % 4], cv::Scalar(0, 255, 0), 2, 8);
            // cv::circle(frame, points[i], 2, cv::Scalar(255, 255, 0));
            cv::putText(frame, std::to_string(j), points[j], 0, 1, cv ::Scalar(255, 255, 0));
            for (int i = 0; i < buff_center_candidate.size(); i++)
            {
                cv::circle(frame, buff_center_candidate[i], 2, cv::Scalar(255, 255, 0), 2);
            }
        }
        // line(frame, points[0], points[1], cv::Scalar(0, 255, 0), 2, 8);
    }
    void BuffDetector::drawAllArmor(cv::Mat &frame)
    {
        for (int i = 0; i < targets.size(); i++)
        {
            cv::Point2f armor_points[4];
            targets[i].armor_rect.points(armor_points);
            for (int j = 0; j < 4; j++)
            {
                line(frame, armor_points[j], armor_points[(j + 1) % 4], cv::Scalar(0, 255, 0), 2, 8);
            }
        }
    }
    void BuffDetector::drawCircle(cv::Mat &frame, const cv::Point2f &armor_center)
    {
        if (targets.size() != 0)
        {
            for (int i = 0; i < buff_center_candidate.size(); i++)
            {
                cv::circle(frame, buff_center_candidate[0], getDistance(targets[0].armor_rect.center, buff_center_candidate[0]), cv::Scalar(255, 255, 0), 2);
            }
        }
    }
};