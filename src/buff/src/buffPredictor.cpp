#include "buffPredictor.h"
#include <fstream>
namespace ly
{
    BuffPredictor::BuffPredictor()
    {
        curver = new BuffCurver();
        sine_function = new SineFunction();

        time_base = 0.0f;
        clockwise_count = 0;

        //buff predict params
        max_interval = 30.0;      //连续两帧时间差，太大的话也直接重启滤波器
        max_armor_move_dis = 0.5; //单位：m

        is_get_sine = false;
        buff_angle_kalman_filter = new buffAngleKalman();
    }

    BuffPredictor::~BuffPredictor()
    {
        delete curver;
        delete sine_function;
        delete buff_angle_kalman_filter;
    }
    bool BuffPredictor::judgeRestart()
    {
        // std::cout << fabs(last_last_armor_angle - last_armor_angle) << "du" << std::endl;
        return fabs(last_last_armor_angle - last_armor_angle) > 0.3;
    }
    // void BuffPredictor::getTrajData(const Eigen::Vector3d &armor_pose, const float &angle)
    // {
    //     static float k = 0.85; //低通滤波参数
    //     float temp_angle = angle;
    //     if (temp_angle < 0)
    //     {
    //         temp_angle += 2 * M_PI;
    //     }
    //     float angle_360 = temp_angle / M_PI * 180;
    //     LOG(INFO) << "angle_360: " << angle_360;

    //     int angle_int = round(angle_360);
    //     int deal_index = -1;
    //     if (angle_int <= 0 || angle_int >= 360)
    //     {
    //         deal_index = 0;
    //     }
    //     else
    //     {
    //         deal_index = angle_int;
    //     }
    //     if (!watched_points[deal_index].is_get)
    //     {
    //         //第一次获得这个角度的数值
    //         watched_points[deal_index].is_get = true;
    //         watched_points[deal_index].point = armor_pose;
    //         get_angle_num++;
    //     }
    //     else
    //     {
    //         watched_points[deal_index].point = watched_points[deal_index].point * k + (1 - k) * armor_pose;
    //     }
    //     LOG(INFO) << "GET ANGLE NUM :" << get_angle_num;
    // }
    // void BuffPredictor::getPointsFromTraj(int freq)
    // {
    //     std::vector<Eigen::Vector3d>().swap(armor_pose_points);
    //     int count = 0;
    //     for (int i = 0; i < 360; i++)
    //     {
    //         if (watched_points[i].is_get)
    //         {
    //             count++;
    //         }
    //         if (count >= freq)
    //         {
    //             armor_pose_points.emplace_back(watched_points[i].point);
    //             count = 0;
    //         }
    //     }
    // }
    // void BuffPredictor::emptyTraj()
    // {
    //     for (int i = 0; i < 360; i++)
    //     {
    //         watched_points[i].is_get = false;
    //     }
    //     get_angle_num = 0;
    // }

    //两种混合
    // float BuffPredictor::runBuffPredict(BuffArmor &aim_armor) //返回是否已经准备好，即是否已经拟合好轨迹
    // {
    //     //update armor
    //     this_aim_armor = aim_armor;
    //     float delta_t = std::chrono::duration_cast<std::chrono::microseconds>(this_aim_armor.time_stamp - last_aim_armor.time_stamp).count() / 1000.0;

    //     std::cout << "delta_t: " << delta_t << std::endl;

    //     getTrajData(aim_armor.world_pose, aim_armor.angle);
    //     //拟合轨迹,考虑实时更新轨迹
    //     // if (!is_trajectory_get)
    //     // {
    //     //     is_trajectory_get = curver->fit(aim_armor.world_pose, aim_armor.angle);
    //     //     if (is_trajectory_get)
    //     //     {
    //     //         buff_trajectory = curver->getTrajectory();
    //     //     }
    //     //     else
    //     //     {
    //     //         is_buff_curve_not_sucess = true;
    //     //     }
    //     // }
    //     if (get_angle_num == CURVE_FIT_SIZE)
    //     {
    //         getPointsFromTraj(1); //使用原有数据，不分频
    //         is_trajectory_get = curver->fit(armor_pose_points);
    //         if (is_trajectory_get)
    //         {
    //             buff_trajectory = curver->getTrajectory();
    //         }
    //     }
    //     else if (get_angle_num == CURVE_FIT_SIZE * 2)
    //     {
    //         getPointsFromTraj(2); //使用原有数据，不分频
    //         curver->fit(armor_pose_points);

    //         is_trajectory_get = true;

    //         buff_trajectory = curver->getTrajectory();
    //     }
    //     else if (get_angle_num == CURVE_FIT_SIZE * 3)
    //     {
    //         getPointsFromTraj(3); //使用原有数据，不分频
    //         curver->fit(armor_pose_points);
    //         is_trajectory_get = true;
    //         buff_trajectory = curver->getTrajectory();
    //     }

    //     if (is_trajectory_get)
    //     {
    //         std::cout << "traj radius: " << buff_trajectory.radius << std::endl;
    //         std::cout << "traj x_axis" << buff_trajectory.x_axis << std::endl;
    //         std::cout << "traj y_axis" << buff_trajectory.y_axis << std::endl;
    //     }
    //     //@TODO 矫正轨迹，判断轨迹是否符合实际

    //     if (!is_get_first_armor) //获取第一个装甲板数据
    //     {
    //         is_get_first_armor = true;
    //         first_armor = this_aim_armor;
    //         time_base = 0.0f;
    //         last_armor_angle = aim_armor.angle;
    //     }
    //     else
    //     {
    //         time_base += delta_t / 1000.0f;
    //     }
    //     // std::cout << "time_base" << time_base << std::endl;
    //     // if (is_buff_curve_not_sucess)
    //     // {
    //     //     is_get_first_armor = false;
    //     // }

    //     //获取连续化的角度
    //     makeAngleContinous();

    //     filte_speed = FilteSpeed(delta_t);

    //     // sineFit();
    //     // fitLargeBuffSin();

    //     if (is_get_sine)
    //     {
    //         std::cout << "a: " << cos_params[0] << " w: " << cos_params[1] << " theta: " << cos_params[2] << std::endl;
    //     }

    //     //角度滤波
    //     float filte_angle;
    //     // std::cout << "delta_t:" << this_aim_armor.begin_to_now_time - last_aim_armor.begin_to_now_time << std::endl;
    //     if (is_get_buff_type && buff_type == SMALL_BUFF)
    //     {
    //         std::cout << "SMALL BUFF" << std::endl;
    //         filte_angle = kalman_filter->runKalman(this_armor_angle, delta_t);
    //     }
    //     else if (is_get_buff_type && buff_type == LARGE_BUFF && is_get_sine)
    //     {
    //         std::cout << "LARGE BUFF" << std::endl;
    //         buff_ekf_filter->setBaseTime((time_base - time_start) * 1000.0f - delta_t);
    //         // filte_angle = this_armor_angle;

    //         filte_angle = buff_ekf_filter->runKalman(this_armor_angle * pow(-1, rotation + 1), delta_t) * pow(-1, rotation + 1);

    //         // debug.sendData(Eigen::Vector3d(fmod(this_armor_angle, 2 * M_PI), fmod(filte_angle, 2 * M_PI), 0));
    //     }
    //     else
    //     {
    //         filte_angle = 999.0f;
    //     }
    //     std::cout << "filter angle : " << filte_angle << std::endl;

    //     //判断方向
    //     if (!is_get_rotation)
    //     {
    //         judgeRotation(this_armor_angle - last_armor_angle); //判断旋转方向
    //     }

    //     if (rotation == CLOCK_WISE)
    //     {
    //         std::cout << "ROTATION: CLOCK_WISE" << std::endl;
    //     }
    //     else if (rotation == COUNTER_CLOCK_WISE)
    //     {
    //         std::cout << "ROTATION: COUNTER_CLOCK_WISE" << std::endl;
    //     }

    //     // //判断正弦或者是匀速
    //     // if (!is_get_buff_type)
    //     // {
    //     //     judgeBuffType(); //判断装甲板类型
    //     // }
    //     last_last_armor_angle = last_armor_angle;
    //     last_armor_angle = this_armor_angle; //更新角度
    //     last_aim_armor = this_aim_armor;
    //     return filte_angle;

    //     // //计算角度
    //     // if (is_trajectory_get)
    //     // {
    //     //     last_aim_armor = this_aim_armor;
    //     //     return ThreeDPredict();
    //     // }
    //     // else
    //     // {
    //     //     //update armor
    //     //     last_aim_armor = this_aim_armor;
    //     //     return Eigen::Vector3d::Zero();
    //     // }
    // }

    // float BuffPredictor::runLargeBuffPredict(BuffArmor &aim_armor)
    // {
    //     //update armor
    //     is_get_buff_type = true;
    //     buff_type == LARGE_BUFF;
    //     this_aim_armor = aim_armor;
    //     float delta_t = std::chrono::duration_cast<std::chrono::microseconds>(this_aim_armor.time_stamp - last_aim_armor.time_stamp).count() / 1000.0;

    //     getTrajData(aim_armor.world_pose, aim_armor.angle);
    //     //拟合轨迹,考虑实时更新轨迹

    //     if (get_angle_num == CURVE_FIT_SIZE)
    //     {
    //         getPointsFromTraj(1); //使用原有数据，不分频
    //         is_trajectory_get = curver->fit(armor_pose_points);
    //         if (is_trajectory_get)
    //         {
    //             buff_trajectory = curver->getTrajectory();
    //         }
    //     }
    //     else if (get_angle_num == CURVE_FIT_SIZE * 2)
    //     {
    //         getPointsFromTraj(2); //使用原有数据，不分频
    //         curver->fit(armor_pose_points);

    //         is_trajectory_get = true;

    //         buff_trajectory = curver->getTrajectory();
    //     }
    //     else if (get_angle_num == CURVE_FIT_SIZE * 3)
    //     {
    //         getPointsFromTraj(3); //使用原有数据，不分频
    //         curver->fit(armor_pose_points);
    //         is_trajectory_get = true;
    //         buff_trajectory = curver->getTrajectory();
    //     }

    //     if (is_trajectory_get)
    //     {
    //         std::cout << "traj radius: " << buff_trajectory.radius << std::endl;
    //         std::cout << "traj x_axis" << buff_trajectory.x_axis << std::endl;
    //         std::cout << "traj y_axis" << buff_trajectory.y_axis << std::endl;
    //     }
    //     if (!is_get_first_armor) //获取第一个装甲板数据
    //     {
    //         is_get_first_armor = true;
    //         first_armor = this_aim_armor;
    //         time_base = 0.0f;
    //         last_armor_angle = aim_armor.angle;
    //     }
    //     else
    //     {
    //         time_base += delta_t / 1000.0f;
    //     }

    //     //获取连续化的角度
    //     makeAngleContinous();

    //     filte_speed = FilteSpeed(delta_t);

    //     largeBuffFit();

    //     float filte_angle;
    //     // if (is_get_sine)
    //     // {
    //     //     debug.sendData(Eigen::Vector3d(-filte_speed, cos_params[0] * sin(cos_params[1] * (time_base - time_start) + cos_params[2]) + 2.090 - cos_params[0], 0));
    //     // }
    //     // else
    //     // {
    //     //     debug.sendData(Eigen::Vector3d(-filte_speed, 0, 0));
    //     // }
    //     if (is_get_sine)
    //     {
    //         // std::cout
    //         //     << "LARGE BUFF" << std::endl;
    //         std::cout << "a: " << cos_params[0] << "w: " << cos_params[1] << "theta: " << cos_params[2] << std::endl;
    //         buff_ekf_filter->setBaseTime((time_base - time_start) * 1000.0f - delta_t);
    //         // filte_angle = this_armor_angle;

    //         filte_angle = buff_ekf_filter->runKalman(this_armor_angle * pow(-1, rotation + 1), delta_t) * pow(-1, rotation + 1);

    //         // debug.sendData(Eigen::Vector3d(fmod(this_armor_angle, 2 * M_PI), fmod(filte_angle, 2 * M_PI), 0));
    //     }
    //     else
    //     {
    //         filte_angle = 999.0f;
    //     }
    //     std::cout << "filter angle : " << filte_angle << std::endl;

    //     //判断方向
    //     if (!is_get_rotation)
    //     {
    //         judgeRotation(this_armor_angle - last_armor_angle); //判断旋转方向
    //     }

    //     if (rotation == CLOCK_WISE)
    //     {
    //         std::cout << "ROTATION: CLOCK_WISE" << std::endl;
    //     }
    //     else if (rotation == COUNTER_CLOCK_WISE)
    //     {
    //         std::cout << "ROTATION: COUNTER_CLOCK_WISE" << std::endl;
    //     }
    //     last_last_armor_angle = last_armor_angle;
    //     last_armor_angle = this_armor_angle; //更新角度
    //     last_aim_armor = this_aim_armor;
    //     return filte_angle;
    // }
    float BuffPredictor::FilteSpeed(const float &delta_t)
    {
        static float last_angle_temp = 0.0;
        float this_angle_temp = this_armor_angle - round((this_armor_angle - last_angle_temp) / 2 / M_PI * 5) * 2 * M_PI / 5;
        last_angle_temp = this_angle_temp;

        buff_angle_kalman_filter->runKalman(this_angle_temp, delta_t);
        return buff_angle_kalman_filter->getSpeed();
    }

    Eigen::Vector3d BuffPredictor::SmallBuffPreidct(const float &predict_t)
    {
        Eigen::Vector3d target_position;
        if (is_get_rotation)
        {
            float angle_add = 1.047 * (predict_t)*pow(-1, rotation);
            float angle_relative_to_axis = 3.0 / 2.0 * M_PI + angle_add;
            target_position = Eigen::Vector3d(0, 0.7, 0) + 0.7 * Eigen::Vector3d::UnitX() * cos(angle_relative_to_axis) + 0.7 * Eigen::Vector3d::UnitY() * sin(angle_relative_to_axis);
        }
        else
        {
            target_position = Eigen::Vector3d(-999, -999, -999);
        }

        return target_position;
    }

    float BuffPredictor::runSmallBuffPredict(BuffArmor &aim_armor)
    {
        this_aim_armor = aim_armor;
        float delta_t = std::chrono::duration_cast<std::chrono::microseconds>(this_aim_armor.time_stamp - last_aim_armor.time_stamp).count() / 1000.0;

        makeAngleContinous(); //角度连续化
        if (!is_get_rotation)
        {
            judgeRotation(this_armor_angle - last_armor_angle); //判断旋转方向
        }

        if (rotation == CLOCK_WISE)
        {
            std::cout << "ROTATION: CLOCK_WISE" << std::endl;
        }
        else if (rotation == COUNTER_CLOCK_WISE)
        {
            std::cout << "ROTATION: COUNTER_CLOCK_WISE" << std::endl;
        }
        last_last_armor_angle = last_armor_angle;
        last_armor_angle = this_armor_angle; //更新角度
        last_aim_armor = this_aim_armor;

        // float filte_angle = kalman_filter->runKalman(this_armor_angle, delta_t);
        float filte_angle = this_armor_angle;

        if (!is_get_rotation)
        {
            return -999.0f;
        }
        else
        {
            return filte_angle;
        }
    }
    // Eigen::Vector3d BuffPredictor::ThreeDPredict()
    // {
    //     if (!is_get_sine && buff_type != SMALL_BUFF) //准备拟合
    //     {
    //         sineFit();
    //         last_armor_angle = this_armor_angle;
    //         return Eigen::Vector3d::Zero();
    //     }
    //     else if (is_get_rotation && buff_type == LARGE_BUFF && is_trajectory_get) //正弦预测
    //     {
    //         last_armor_angle = this_armor_angle;
    //         float predict_angle = predict(predict_t);
    //         Eigen::Vector3d aim_loc = buff_trajectory.center + buff_trajectory.radius * buff_trajectory.x_axis * cos(predict_angle) + buff_trajectory.radius * buff_trajectory.y_axis * sin(predict_angle);
    //         return aim_loc;
    //     }
    //     else if (is_get_rotation && buff_type == SMALL_BUFF && is_trajectory_get) //匀速预测
    //     {
    //         last_armor_angle = this_armor_angle;
    //         float predict_angle = fixSpeedPredict(predict_t);
    //         Eigen::Vector3d aim_loc = buff_trajectory.center + buff_trajectory.radius * buff_trajectory.x_axis * cos(predict_angle) + buff_trajectory.radius * buff_trajectory.y_axis * sin(predict_angle);
    //         return aim_loc;
    //     }
    //     else
    //     {
    //         last_armor_angle = this_armor_angle;
    //         return Eigen::Vector3d::Zero();
    //     }
    // }
    //两帧之间的时间差由时间戳相减得到，这里的时间是预测的时间
    // float BuffPredictor::runBuffPredict(BuffArmor &aim_armor)
    // {
    //     //update armor
    //     this_aim_armor = aim_armor;

    //     //start to filte angle and calc speed
    //     float predict_t = aim_armor.world_pose.norm() / bullet_speed + shoot_delay; //s

    //     if (!is_get_first_armor)
    //     {
    //         is_get_first_armor = true;
    //         first_armor = this_aim_armor;
    //     }
    //     else
    //     {
    //         angleFilter(predict_t);
    //     }

    //     //update armor
    //     last_aim_armor = this_aim_armor;

    //     if (!is_get_sine)
    //     {
    //         return -10000.0f;
    //     }
    //     return aim_angle;
    // }
    float BuffPredictor::makeAngleContinous()
    {
        this_armor_angle = this_aim_armor.angle + round((last_armor_angle - this_aim_armor.angle) / 2 / CV_PI) * 2 * CV_PI;
        std::cout << "angle: " << this_armor_angle << std::endl;
        return this_armor_angle;
    }
    void BuffPredictor::judgeRotation(float angle_diff)
    {
        if (is_get_rotation)
        {
            return;
        }
        angle_diff > 0 ? clockwise_count++ : clockwise_count--;
        if (clockwise_count > 30)
        {
            is_get_rotation = true;
            rotation = COUNTER_CLOCK_WISE; //逆时针
            // kalman_filter->setRotation(COUNTER_CLOCK_WISE);
        }
        else if (clockwise_count < -30)
        {
            is_get_rotation = true;
            rotation = CLOCK_WISE; //顺时针
            // kalman_filter->setRotation(CLOCK_WISE);
        }
    }

    // //2d版本
    // float BuffPredictor::angleFilter(const float &predict_t)
    // {
    //     //get continous angle and angle speed
    //     makeAngleContinous();

    //     // time_base = std::chrono::duration_cast<std::chrono::milliseconds>(this_aim_armor.time_stamp - first_armor.time_stamp).count() / 1000.0f;

    //     if (!is_get_rotation)
    //     {
    //         judgeRotation(this_armor_angle - last_armor_angle); //判断旋转方向
    //     }
    //     if (!is_get_buff_type)
    //     {
    //         judgeBuffType(); //判断装甲板类型
    //     }
    //     if (!is_get_sine && buff_type != SMALL_BUFF) //准备拟合
    //     {
    //         sineFit();
    //         last_armor_angle = this_armor_angle;
    //         return -10000.0f; //-10000.0f表示还没有成功拟合曲线
    //     }
    //     else if (is_get_rotation && buff_type == LARGE_BUFF) //正弦预测
    //     {
    //         last_armor_angle = this_armor_angle;
    //         return predict(predict_t);
    //     }
    //     else if (is_get_rotation && buff_type == SMALL_BUFF) //匀速预测
    //     {
    //         last_armor_angle = this_armor_angle;
    //         return fixSpeedPredict(predict_t);
    //     }
    //     else
    //     {
    //         last_armor_angle = this_armor_angle;
    //         return -10000.0f;
    //     }
    // }
    float BuffPredictor::fixSpeedPredict(const float &filte_angle, const float &predict_t)
    {
        float predict_angle = 1.047 * (predict_t); //直接根据这个来获取角度，或者用Kalman
        if (!is_get_rotation)
        {
            return (float)-999;
        }
        if (rotation == CLOCK_WISE) //顺时针
        {
            aim_angle = filte_angle - predict_angle;
            std::cout << "filte angle" << filte_angle << std::endl;
            std::cout << "predict angle:" << aim_angle << std::endl;
        }
        else if (rotation == COUNTER_CLOCK_WISE)
        {
            aim_angle = filte_angle + predict_angle;
        }
        return aim_angle;
    }

    // float BuffPredictor::predict(const float &predict_t)
    // {
    //     float theta_start = 1.884 * time_base + cos_params[0];
    //     float theta_end = 1.884 * (time_base + predict_t) + cos_params[0];
    //     float predict_angle = -0.785 / 1.884 * (cos(theta_end) - cos(theta_start)) + 1.305 * predict_t;

    //     if (rotation == 0) //顺时针
    //     {
    //         aim_angle = this_armor_angle - predict_angle;
    //     }
    //     else
    //     {
    //         aim_angle = this_armor_angle + predict_angle;
    //     }
    //     return aim_angle;
    // }
    //判断是正常速度还是正弦运动
    void BuffPredictor::setPredictMode(int type)
    {
        buff_type = type;
        is_get_buff_type = true;
    }

    // void BuffPredictor::judgeBuffType()
    // {
    //     std::vector<float> buff_angle_set_temp = std::vector<float>(buff_angle_set.begin(), buff_angle_set.end());
    //     double sum = 0.0;
    //     int j = 0;
    //     int jump_num = 0;
    //     for (int i = 1; i < buff_angle_set_temp.size(); i++)
    //     {
    //         if (buff_time_set[i] - buff_time_set[i - 1] > 0.5) //丢帧0.5s以上
    //         {
    //             j = i; //间隔时间过长
    //             jump_num++;
    //         }
    //         else
    //         {
    //             //连续化之后的角度
    //             buff_angle_set_temp[i] = buff_angle_set_temp[i] - 2 * M_PI / 5 * round((buff_angle_set_temp[i] - buff_angle_set_temp[i - 1]) / 2 / M_PI * 5);
    //             sum += fabs(1.047 * (buff_time_set[i] - buff_time_set[j]) * pow(-1, rotation + 1) + (buff_angle_set_temp[j] - buff_angle_set_temp[i]));
    //             std::cout << "fabs" << fabs(1.047 * (buff_time_set[i] - buff_time_set[j]) * pow(-1, rotation + 1) + (buff_angle_set_temp[j] - buff_angle_set_temp[i])) << std::endl;
    //             std::cout << "angle: " << buff_angle_set_temp[i] << std::endl;
    //         }
    //     }
    //     // for (int i = 0; i < buff_angle_set.size(); i++)
    //     // {
    //     //     std::cout<<"i: "<<
    //     // }
    //     sum /= (buff_angle_set.size() - jump_num - 1);
    //     // std::cout << "sum" << std::endl;

    //     if (sum > 0.22) //大于一定比例就认为是大符号
    //     {
    //         buff_type = LARGE_BUFF;
    //         is_get_buff_type = true;
    //     }
    //     else
    //     {
    //         buff_type = SMALL_BUFF;
    //         is_get_buff_type = true;
    //     }
    // }
    // //正弦波拟合,参考西北工业大学2021，考虑速度拟合或者直接角度拟合
    // void BuffPredictor::sineFit()
    // {
    //     if (buff_type == SMALL_BUFF)
    //     {
    //         return;
    //     }
    //     static int count = 0;
    //     if (count % 4 == 0)
    //     {
    //         buff_angle_set.push_back(this_armor_angle);
    //         buff_time_set.push_back(time_base);

    //         std::fstream ss("angle2.txt", std::ios::app);
    //         ss << this_armor_angle << std::endl;
    //         ss << time_base << std::endl;
    //         ss.close();

    //         if (buff_angle_set.size() > SINE_FIT_SIZE)
    //         {
    //             buff_angle_set.erase(buff_angle_set.begin());
    //             buff_time_set.erase(buff_time_set.begin());
    //         }
    //     }
    //     count++;

    //     if (buff_angle_set.size() < SINE_FIT_SIZE || !is_get_rotation || !is_trajectory_get)
    //     {
    //         return;
    //     }

    //     if (buff_type == SMALL_BUFF)
    //     {
    //         buff_time_set.clear();
    //         buff_angle_set.clear();
    //         return;
    //     }

    //     // if (is_get_sine)
    //     // {
    //     //     static int update_count = 0;
    //     //     if (update_count % 400 == 399)
    //     //     {
    //     //         update_symbol = true;
    //     //     }
    //     //     update_count++;
    //     // }

    //     if (is_get_sine && !update_symbol) //如果没有已经获得正弦函数并且没有标志位更新的话，则跳出这个拟合过程
    //     {
    //         return;
    //     }

    //     //已经确定是大符而不是小符
    //     int lost_frame_index = -1;
    //     for (int i = 1; i < buff_time_set.size(); i++)
    //     {
    //         if (buff_time_set[i] - buff_time_set[i - 1] > 0.28) //超过0.25s之后，不能判断角度变化的趋势
    //         {
    //             LOG(ERROR) << "CURVE MAY NOT BE RIGHT";
    //             lost_frame_index = i;
    //             break;
    //         }
    //     }
    //     if (lost_frame_index > 0) //确实存在掉帧现象
    //     {
    //         if (lost_frame_index > CURVER_SECOND_THRESH) //大于这个阈值，仍然强行拟合
    //         {
    //             buff_angle_set.erase(buff_angle_set.begin() + lost_frame_index, buff_angle_set.end());
    //             buff_time_set.erase(buff_time_set.begin() + lost_frame_index, buff_time_set.end());
    //             LOG(ERROR) << "ERROR OCCUR BUT CURVE";
    //         }
    //         else //拟合帧数过少，不得已清除前面的数据
    //         {
    //             buff_angle_set.erase(buff_angle_set.begin(), buff_angle_set.begin() + lost_frame_index);
    //             buff_time_set.erase(buff_time_set.begin(), buff_time_set.begin() + lost_frame_index);
    //             LOG(ERROR) << "ABANDON CURVE";
    //             return; //放弃拟合
    //         }
    //     }

    //     std::vector<float> buff_angle_set_copy = std::vector<float>(buff_angle_set.begin(), buff_angle_set.end());
    //     std::vector<float> buff_time_set_copy = std::vector<float>(buff_time_set.begin(), buff_time_set.end());

    //     if (rotation == CLOCK_WISE)
    //     {
    //         for (int i = 0; i < buff_angle_set.size(); i++)
    //         {
    //             buff_angle_set_copy[i] = -buff_angle_set_copy[i];
    //         }
    //     }
    //     //怎么处理掉帧问题？如果掉多帧的话，角度判断会出错
    //     for (int i = 1; i < buff_angle_set_copy.size(); i++)
    //     {
    //         buff_angle_set_copy[i] = buff_angle_set_copy[i] - 2 * M_PI / 5 * round((buff_angle_set_copy[i] - buff_angle_set_copy[i - 1]) / 2 / M_PI * 5); //四舍五入
    //     }

    //     float angle_base = buff_angle_set_copy[0];
    //     time_start = buff_time_set_copy[0];
    //     for (int i = 0; i < buff_angle_set.size(); i++)
    //     {
    //         buff_angle_set_copy[i] -= angle_base;
    //         buff_time_set_copy[i] -= time_start;
    //     }

    //     cos_params = curver->fitCos(buff_angle_set_copy, buff_time_set_copy);
    //     buff_ekf_filter->init(cos_params);
    //     sine_function->setParams(cos_params);

    //     is_get_sine = true;
    //     update_symbol = false;
    // }
    float BuffPredictor::OnePointPreidct(BuffArmor &aim_armor)
    {
        this_aim_armor = aim_armor;
        float delta_t = std::chrono::duration_cast<std::chrono::microseconds>(this_aim_armor.time_stamp - last_aim_armor.time_stamp).count() / 1000.0;

        std::cout << "delta_t: " << delta_t << std::endl;

        if (!is_get_first_armor) //获取第一个装甲板数据
        {
            is_get_first_armor = true;
            first_armor = this_aim_armor;
            time_base = 0.0f;
            last_armor_angle = aim_armor.angle;
        }
        else
        {
            time_base += delta_t / 1000.0f;
        }

        makeAngleContinous();

        float filte_angle;

        if (buff_type == SMALL_BUFF)
        {
            std::cout << "SMALL BUFF" << std::endl;
            // filte_angle = kalman_filter->runKalman(this_armor_angle, delta_t);
            filte_angle = this_armor_angle;
        }
        else
        {
            std::cout << "LARGE BUFF" << std::endl;

            filte_speed = FilteSpeed(delta_t);

            if (!is_get_sine)
            {
                debug.sendData(Eigen::Vector3d(-filte_speed, 0, 0));
            }
            else
            {
                debug.sendData(Eigen::Vector3d(-filte_speed, cos_params[0] * sin(cos_params[1] * (time_base - time_start) + cos_params[2]) + 2.090 - cos_params[0], 0));
                std::cout << "send: " << cos_params[0] * sin(cos_params[1] * (time_base - time_start) + cos_params[2]) + 2.090 - cos_params[0] << std::endl;
            }

            largeBuffFit();

            if (is_get_sine)
            {
                std::cout << "a: " << cos_params[0] << "w: " << cos_params[1] << "theta: " << cos_params[2] << std::endl;
                // buff_ekf_filter->setBaseTime((time_base - time_start) * 1000.0f - delta_t);

                // filte_angle = buff_ekf_filter->runKalman(this_armor_angle * pow(-1, rotation + 1), delta_t) * pow(-1, rotation + 1);
                filte_angle = this_armor_angle; //暂时不滤波
            }
            else
            {
                filte_angle = 999.0f;
            }
            std::cout << "filter angle : " << filte_angle << std::endl;
        }

        SinePredict(filte_angle, 0.4);

        //判断方向
        if (!is_get_rotation)
        {
            judgeRotation(this_armor_angle - last_armor_angle); //判断旋转方向
        }

        if (rotation == CLOCK_WISE)
        {
            std::cout << "ROTATION: CLOCK_WISE" << std::endl;
        }
        else if (rotation == COUNTER_CLOCK_WISE)
        {
            std::cout << "ROTATION: COUNTER_CLOCK_WISE" << std::endl;
        }
        last_last_armor_angle = last_armor_angle;
        last_armor_angle = this_armor_angle; //更新角度
        last_aim_armor = this_aim_armor;
        return filte_angle;
    }
    cv::Point2f BuffPredictor::OnePointPositonCalc(const cv::Point2f &center, const cv::Point2f &armor_center, const float &predict_time, const float &filte_angle)
    {
        cv::Point2f center_to_armor = center - armor_center;
        float radius = sqrt(center_to_armor.x * center_to_armor.x + center_to_armor.y * center_to_armor.y);

        if (buff_type == SMALL_BUFF)
        {
            if (rotation == CLOCK_WISE) //顺时针
            {
                return center + cv::Point2f(radius * cos(filte_angle - 1.047 * predict_time), -radius * sin(filte_angle - 1.047 * predict_time));
            }
            else if (rotation == COUNTER_CLOCK_WISE)
            {
                return center + cv::Point2f(radius * cos(filte_angle + 1.047 * predict_time), -radius * sin(filte_angle + 1.047 * predict_time));
            }
            else
            {
                return cv::Point2f(-1, -1);
            }
        }
        else //大符模式
        {
            if (fabs(filte_angle - 999) < 1e-4 || !is_get_rotation) //说明未拟合到角度
            {
                return cv::Point2f(-1, -1);
            }
            else
            {
                float aim_angle = filte_angle + (sine_function->predict(time_base + predict_time - time_start) - sine_function->predict(time_base - time_start)) * pow(-1, rotation + 1);
                return center + cv::Point2f(radius * cos(aim_angle), -radius * sin(aim_angle));
            }
        }
    }
    float BuffPredictor::getShootAngle(const float &filte_angle, const float &predict_time)
    {
        if (buff_type == SMALL_BUFF)
        {
            if (rotation == CLOCK_WISE) //顺时针
            {
                return filte_angle - 1.047 * predict_time;
            }
            else if (rotation == COUNTER_CLOCK_WISE)
            {
                return filte_angle + 1.047 * predict_time;
            }
            else
            {
                return 0;
            }
        }
        else //大符模式
        {
            if (fabs(filte_angle - 999) < 1e-4 || !is_get_rotation) //说明未拟合到角度
            {
                return 0;
            }
            else
            {
                return filte_angle + (sine_function->predict(time_base + predict_time - time_start) - sine_function->predict(time_base - time_start)) * pow(-1, rotation + 1);
            }
        }
    }

    void BuffPredictor::largeBuffFit()
    {
        static int count = 0;
        if (count >= CONVERGE_COUNT)
        {
            // if (count % 3 == 0)
            // {
            speed_set.push_back(filte_speed);
            buff_time_set.push_back(time_base);
            // }
        }

        // if (is_get_sine)
        // {
        //     debug.sendData(Eigen::Vector3d(filte_speed - sine_function->getSpeed(time_base - time_start), 0, 0));
        // }

        count++;

        if (speed_set.size() < SPEED_FIT_SIZE || !is_get_rotation)
        {
            return;
        }

        // std::vector<float> speed_set_copy = std::vector<float>(speed_set.begin(), speed_set.end());
        // std::vector<float> time_set_copy = std::vector<float>(buff_time_set.begin(), buff_time_set.end());

        if (rotation == CLOCK_WISE) //转为正速度
        {
            for (int i = 0; i < speed_set.size(); i++)
            {
                speed_set[i] = -speed_set[i];
            }
        }

        time_start = buff_time_set[0];
        for (int i = 0; i < buff_time_set.size(); i++)
        {
            buff_time_set[i] -= time_start;
            // std::cout << "time_set_copy[i]" << time_set_copy[i] << std::endl;
        }

        if (!is_get_sine)
        {
            cos_params = curver->fitLargeBuffSin(speed_set, buff_time_set);
        }
        else
        {
            cos_params = curver->fitLargeBuffSin2(speed_set, buff_time_set);
        }

        // buff_ekf_filter->init(cos_params);
        sine_function->setParams(cos_params);
        is_get_sine = true;
        count = 0; //清空缓冲

        std::vector<float>().swap(speed_set);
        std::vector<float>().swap(buff_time_set);
    }

    float ly::BuffPredictor::SinePredict(const float &filte_angle, const float &predict_t)
    {
        if (!is_get_rotation) //未获得转动方向
        {
            return (float)-999;
        }
        // float angle_raw = sine_function->predict(time_base) * pow(-1, rotation + 1);
        // double radio = round((this_armor_angle - angle_raw) / (2 * M_PI / 5));
        aim_angle = filte_angle + (sine_function->predict(time_base + predict_t - time_start) - sine_function->predict(time_base - time_start)) * pow(-1, rotation + 1);
        // debug.sendInfo(Eigen::Vector3d(fmod(filte_angle, 2 * M_PI) * 100, 0, 0), time_base, Eigen::Vector3d(fmod(aim_angle, 2 * M_PI) * 100, 0, 0), time_base + predict_t);
        // std::cout << "aim_angle" << aim_angle << std::endl;
        // float angle_predict = sine_function->predict(time_base + predict_t) * pow(-1, rotation + 1);
        // aim_angle = angle_predict + radio * (2 * M_PI / 5);

        return aim_angle;
    }
    // Eigen::Vector3d BuffPredictor::getPredictPose(const float &filte_angle, const float &shoot_time)
    // {
    //     bool is_get_predict = false;
    //     if (is_get_rotation)
    //     {
    //         if (is_get_buff_type && buff_type == SMALL_BUFF)
    //         {
    //             predict_angle = fixSpeedPredict(filte_angle, shoot_time); //小符
    //         }
    //         else if (is_get_buff_type && buff_type == LARGE_BUFF && is_get_sine)
    //         {
    //             predict_angle = SinePredict(filte_angle, shoot_time);
    //         }
    //         else
    //         {
    //             predict_angle = -999.0f;
    //         }

    //         if (fabs(predict_angle + 999) > 1e-4)
    //         {
    //             is_get_predict = true;
    //         }
    //     }
    //     float predict_angle_180 = fmod(predict_angle, 2 * M_PI);
    //     if (predict_angle_180 < 0)
    //     {
    //         predict_angle_180 += 2 * M_PI;
    //     }
    //     predict_angle_180 = predict_angle_180 / M_PI * 180;
    //     int index = round(predict_angle_180);
    //     if (index < 0 || index >= 360)
    //     {
    //         index = 0;
    //     }
    //     if (is_get_predict && is_trajectory_get)
    //     {
    //         std::cout << "radius" << buff_trajectory.radius << std::endl;
    //         std::cout << "x_axis" << buff_trajectory.x_axis << std::endl;
    //         std::cout << "y_axis" << buff_trajectory.y_axis << std::endl;
    //         return buff_trajectory.center + buff_trajectory.x_axis * buff_trajectory.radius * cos(predict_angle) + buff_trajectory.y_axis * buff_trajectory.radius * sin(predict_angle);
    //     }
    //     // else if (is_get_predict && watched_points[index].is_get) //没有获得轨迹，那么可以i=利用拟合得到的数据进行击打
    //     // {
    //     //     return watched_points[index].point;
    //     // }
    //     else
    //     {
    //         return Eigen::Vector3d(-999, -999, -999);
    //     }
    //     // else
    //     // {
    //     //     return (Eigen::Vector3d)(-1);
    //     // }
    // }
}