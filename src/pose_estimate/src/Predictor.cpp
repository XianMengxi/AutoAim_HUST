//
// Created by Doctor-James on 2021/11/11.
//

#include "Predictor.h"
namespace ly
{
#define GRAVITY 9.79338
    Predictor::Predictor()
    {
        normal_kalman_filter = new NormalKalman();
        lcm_debug = new LcmDebug();

        ekf_filter = new NormalEKF();
    }
    Predictor::~Predictor()
    {
        delete normal_kalman_filter;
        delete lcm_debug;

        delete ekf_filter;
    }

    //预设弹速，加上低通滤波器过滤子弹速度
    void Predictor::setShootSpeed(const char &flag, const float &shoot_speed)
    {
        const float k = 0.85; //遗忘系数
        float shoot_speed_temp;
        if ((flag & 0x06) == 0) // 15m/s档
        {
            if (shoot_speed_mode != SHOOT_SPEED15) // 15M档切换
            {
                ShootSpeed = 14.0f; //需要在上场之前测平均弹速作预设
            }
            else
            {
                //低通滤波
                if (shoot_speed > 15.0 || shoot_speed < 12) //弹速度异常
                {
                    shoot_speed_temp = 14.0f;
                }
                else
                {
                    shoot_speed_temp = shoot_speed;
                }
                ShootSpeed = ShootSpeed * k + (1 - k) * shoot_speed_temp;
            }
            shoot_speed_mode = SHOOT_SPEED15;
        }
        else if ((flag & 0x06) == 2) // 18m/s挡
        {
            if (shoot_speed_mode != SHOOT_SPEED18) // 30M档切换
            {
                ShootSpeed = 17.0;
            }
            else
            {
                if (shoot_speed < 16 || shoot_speed > 18)
                {
                    shoot_speed_temp = 17.0;
                }
                else
                {
                    shoot_speed_temp = shoot_speed;
                }
                ShootSpeed = ShootSpeed * k + (1 - k) * shoot_speed_temp;
            }
            shoot_speed_mode = SHOOT_SPEED18;
        }
        else if ((flag & 0x06) == 4) // 30m/s档
        {
            if (shoot_speed_mode != SHOOT_SPEED30) // 30M档切换
            {
                ShootSpeed = 26.0f;
            }
            else
            {
                if (shoot_speed < 21 || shoot_speed > 30.5)
                {
                    shoot_speed_temp = 26.0f;
                }
                else
                {
                    shoot_speed_temp = shoot_speed;
                }
                ShootSpeed = ShootSpeed * k + (1 - k) * shoot_speed_temp;
            }
            shoot_speed_mode = SHOOT_SPEED30;
        }
        else
        {
            shoot_speed_mode = SHOOT_SPEED_UNDEFINE;
            ShootSpeed = 13.5f;
        }
    }

    //这个在敌方运动低速影响很小，不使用
    double Predictor::calcPredictTime(const Eigen::Vector3d &armor_pose, const Eigen::Vector3d &armor_speed, const float &shoot_speed)
    {
        //这个过程之中，作的假设是pitch在运动过程中变化不大，然后使所有速度归到x,y平面，利用一元二次方程求解
        Eigen::Vector2d armor_plane_pose = Eigen::Vector2d(armor_pose[0], armor_pose[1]);
        Eigen::Vector2d armor_plane_speed = Eigen::Vector2d(armor_speed[0], armor_speed[1]);

        double plane_armor_distance = armor_plane_pose.norm();
        double plane_armor_speed = armor_plane_speed.norm();

        double pitch = atan2(armor_pose[2], plane_armor_distance);
        double plane_shoot_speed = shoot_speed * cos(pitch);

        if (fabs(plane_armor_distance) < 1e-4 || fabs(plane_armor_speed) < 1e-4) //装甲板速度太小
        {
            return plane_armor_distance / plane_shoot_speed;
        }
        double theta = -armor_plane_pose.dot(armor_plane_speed) / plane_armor_distance / plane_armor_speed;
        if (fabs(plane_armor_speed - plane_shoot_speed) < 1e-4)
        {
            return plane_armor_distance / 2 / theta / plane_shoot_speed;
        }

        double plane_shoot_speed_2 = plane_shoot_speed * plane_shoot_speed;
        double plane_armor_speed_2 = plane_armor_speed * plane_armor_speed;
        double plane_armor_distance_2 = plane_armor_distance * plane_armor_distance;
        double theta_2 = theta * theta;
        if (plane_armor_speed_2 * (1 - theta_2) > plane_shoot_speed_2)
        {
            std::cerr << "TARGET RUN TOO FAST" << std::endl;
            return plane_armor_distance / plane_shoot_speed;
        }
        double ret;
        ret = 2 * plane_armor_distance * theta * plane_armor_speed;
        ret -= sqrt(4 * plane_armor_distance_2 * theta_2 * plane_armor_speed_2 - 4 * (plane_armor_speed_2 - plane_shoot_speed_2) * plane_armor_distance_2);
        ret /= 2 * (plane_armor_speed_2 - plane_shoot_speed_2);
        std::cout << "diff time" << fabs(ret - plane_armor_distance / plane_shoot_speed) << std::endl;
        return ret;
    }

    Angle_t Predictor::Predict(const Eigen::Vector3d &armor_trans, SerialPortData SerialPortData_, float delta_t)
    {
        float pitch_now = SerialPortData_.pitch / 100.0f * 3.14159f / 180.0f;

        setShootSpeed(SerialPortData_.flag, SerialPortData_.shoot_speed / 100.0f);
        shootAngleTime_now = ballistic_equation(pitch_now, armor_trans); //当前实际位置击打所需时间

        float shootTime = shootAngleTime_now.time / 1000; //需要测试各种延时(s)

        Eigen::Vector3d predict_point;
        Eigen::Vector3d filte_point;

        // if (target_id == Sentry) //哨兵
        // {
        //     //过程噪声决定了跟踪性能
        //     normal_kalman_filter->setProcessNoise(50, 50, 0.01);
        //     normal_kalman_filter->chi_squared_thresh = 100; //卡方检验阈值设置
        // }
        // else
        // {
        //     normal_kalman_filter->setProcessNoise(FilterParams::process_noise_pose_x, FilterParams::process_noise_pose_y, FilterParams::process_noise_pose_z);
        //     normal_kalman_filter->chi_squared_thresh = 10;
        // }
        // normal_kalman_filter->is_antirot_mode = true;

        // //仅使用强跟踪模型,也可以退化为普通卡尔曼滤波
        // filte_point = normal_kalman_filter->runKalman(armor_trans, delta_t);

        // predict_point = normal_kalman_filter->predict(shootTime + ShootParam::shoot_delay);

        // shootAngleTime_pre = ballistic_equation(pitch_now, predict_point);

        // EKF球面模型测试
        filte_point = ekf_filter->runKalman(armor_trans, delta_t);
        // DLOG(INFO) << "distance: " << sqrt(armor_trans[0] * armor_trans[0] + armor_trans[1] * armor_trans[1]);
        predict_point = ekf_filter->predict(shootTime + ShootParam::shoot_delay);
        shootAngleTime_pre = ballistic_equation(pitch_now, predict_point);

        // LOG(INFO) << "PREDICT POINT:" << predict_point;

        // //自动打蛋判断，打击哨兵前提之下
        // if (target_id == Sentry && normal_kalman_filter->getChiSquareNumber() > 0.9)
        // {
        //     is_restart = true;
        // }
        // else
        // {
        //     is_restart = false;
        // }

        //强跟踪滤波器debug
        //发送滤波值和观测值
        float pitch[2] = {pitch_now * 100, (shootAngleTime_pre.pitch) * 3.1415926f / 180.0f * 100};
        float yaw_temp = (float)(SerialPortData_.yaw) / 100.0f;
        yaw_temp = yaw_temp - round(yaw_temp / 180.0f) * 180.0f;
        yaw_temp = yaw_temp / 180.0f * 3.1415926f;
        float yaw[2] = {yaw_temp, (float)shootAngleTime_pre.yaw * 3.1415926f / 180.0f};
        Eigen::Vector3d filte_acc;
        filte_acc[0] = ekf_filter->change_armor_time;
        filte_acc[1] = ekf_filter->getDetectParam();
        filte_acc[2] = ekf_filter->is_antirot_state;
        lcm_debug->sendData(armor_trans, ekf_filter->getSpeed(), filte_point, filte_acc, yaw, pitch);

        //发送预测值和实际值
        if (delta_t < 50)
        {
            static float t_raw = 0.0f;
            static float t_predict = 0.0f;
            t_raw += delta_t / 1000.0f;
            t_predict = t_raw + shootTime + ShootParam::shoot_delay;

            double yaw = atan2(armor_trans[0], armor_trans[1]);
            double pitch = atan2(armor_trans[2], sqrt(armor_trans[0] * armor_trans[0] + armor_trans[1] * armor_trans[1]));
            double predict_yaw = atan2(predict_point[0], predict_point[1]);
            double predict_pitch = atan2(predict_point[2], sqrt(predict_point[0] * predict_point[0] + predict_point[1] * predict_point[1]));

            // lcm_debug->sendInfo(Eigen::Vector3d(yaw * 100, armor_trans[0], armor_trans[1]), t_raw, Eigen::Vector3d(predict_yaw * 100, predict_point[0], predict_point[1]), t_predict);
        }

        return shootAngleTime_pre;
    }

    //根据当前时间和上一帧来计计算新的估计
    void Predictor::AntirotPredict(SerialPortData SerialPortData_, std::chrono::steady_clock::time_point time_point, Angle_t &raw_pre, const float &delta_t)
    {
        //在小陀螺状态中，才处理
        if (is_get_shoot)
        {
            float pitch_now = SerialPortData_.pitch / 100.0f * 3.14159f / 180.0f;

            int from_start_to_now = std::chrono::duration_cast<std::chrono::milliseconds>(time_point - ekf_filter->last_antirot_time_point).count();
            int shootTime = raw_pre.time + (int)(ShootParam::shoot_delay * 1000); //ms

            int write_index = (save_index + 1) % 2;
            if (history_stamp[write_index].empty())
            {
                return;
            }

            Eigen::Vector3d start_pose = history_stamp[write_index][0].filte_pose;
            Eigen::Vector3d end_pose = history_stamp[write_index][0].filte_pose;

            for (int i = 0; i < history_stamp[write_index].size(); i++)
            {
                if (history_stamp[write_index][i].time_from_start < from_start_to_now)
                {
                    start_pose = history_stamp[write_index][i].filte_pose;
                }
                else if (history_stamp[write_index][i].time_from_start < from_start_to_now + shootTime)
                {
                    end_pose = history_stamp[write_index][i].filte_pose;
                }
                else
                {
                    break;
                }
            }
            Eigen::Vector3d armor_trans = ekf_filter->getPose();
            Eigen::Vector3d predict_point = ekf_filter->getPose() + end_pose - start_pose;
            raw_pre = ballistic_equation(pitch_now, predict_point);

            // lcm_debug->sendData(ekf_filter->getPose(), predict_point);
            //发送预测值和实际值
            if (delta_t < 50)
            {
                static float t_raw = 0.0f;
                static float t_predict = 0.0f;
                t_raw += delta_t / 1000.0f;
                t_predict = t_raw + shootTime / 1000 + ShootParam::shoot_delay;

                double yaw = atan2(armor_trans[0], armor_trans[1]);
                double pitch = atan2(armor_trans[2], sqrt(armor_trans[0] * armor_trans[0] + armor_trans[1] * armor_trans[1]));
                double predict_yaw = atan2(predict_point[0], predict_point[1]);
                double predict_pitch = atan2(predict_point[2], sqrt(predict_point[0] * predict_point[0] + predict_point[1] * predict_point[1]));

                lcm_debug->sendInfo(Eigen::Vector3d(yaw * 100, armor_trans[0], armor_trans[1]), t_raw, Eigen::Vector3d(predict_yaw * 100, predict_point[0], predict_point[1]), t_predict);
            }
        }
    }

    void Predictor::checkAntirotPose(Angle_t &shootAngleTime_, SerialPortData SerialPortData_)
    {
        if (ekf_filter->is_antirot_state)
        {
            bool is_get_target_pose = false;
            double yaw = SerialPortData_.yaw / 100.0;

            double this_yaw = shootAngleTime_.yaw;
            this_yaw = (int)((this_yaw + round((yaw - this_yaw) / 360.0) * 360.0) * 100);

            if (this_yaw < MIN(first_yaw, last_yaw) || this_yaw > MAX(first_yaw, last_yaw))
            {
                shootAngleTime_.yaw = (first_yaw + last_yaw) / 2;
            }
        }
    }
    bool Predictor::judgeIsShoot() //检测是否打蛋，结合残差计算，判断卡尔曼滤波收敛时刻
    {
        // true表示可以打蛋
        if (target_id == Sentry && ekf_filter->getDetectParam() > 0.25)
        {
            is_restart = false;
        }
        else
        {
            is_restart = true;
        }
        return is_restart;
    }

    // 根据物理方程来计算设定pitch和yaw
    Angle_t Predictor::ballistic_equation(float gim_pitch, const Eigen::Vector3d &armor_Position)
    {
        Angle_t shootAngleTime_;
        // 先计算yaw轴角度
        shootAngleTime_.yaw = -atan2(armor_Position[0], armor_Position[1]);
        shootAngleTime_.distance = sqrt(armor_Position[0] * armor_Position[0] + armor_Position[1] * armor_Position[1]);
        // armor 的位置进行了一定的旋转
        Eigen::Vector3d armor_new_position = Eigen::Vector3d(0, shootAngleTime_.distance, armor_Position[2]);
        // 计算pitch轴的初始角度
        shootAngleTime_.pitch = atan2(armor_new_position[2], armor_new_position[1]);

        float err = 100;

        float y_temp = armor_new_position[2];
        float dy, a, y_actual;
        for (int i = 0; i < 20; i++)
        {
            a = (float)atan2(y_temp, shootAngleTime_.distance);
            y_actual = BulletModel(shootAngleTime_.distance, ShootSpeed, a);
            dy = armor_new_position[2] - y_actual;
            y_temp = y_temp + dy;
            if (fabs(dy) < 0.001)
            {
                break;
            }
            // printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n", i + 1, a * 180 / 3.1415926535, y_temp, dy);
        }

        shootAngleTime_.pitch = (float)atan2(y_temp, shootAngleTime_.distance);

        shootAngleTime_.time = abs(shootAngleTime_.distance / (ShootSpeed * cos(shootAngleTime_.pitch)) * 1000);

        shootAngleTime_.pitch = (shootAngleTime_.pitch) / 3.1415926 * 180.0;

        shootAngleTime_.yaw = shootAngleTime_.yaw / 3.1415926 * 180.0f;

        return shootAngleTime_;
    }

    float Predictor::BulletModel(float x, float v, float angle)
    {
        float init_k_ = ShootParam::k;
        float t, y;
        t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle))); // k系数等待测量
        y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
        return y;
    }
    int Predictor::getPitchDiff(float y)
    {
        return ShootParam::shoot_up_offset + (int)(ShootParam::shoot_up_offsetk * y);
    }
    int Predictor::getBuffPitchDiff(float y) //计算大符击打的额外补偿
    {
        return BuffParam::shoot_up_offset;
    }
    int Predictor::getBuffYawDiff(float x)
    {
        return BuffParam::shoot_right_offset;
    }

    float Predictor::calShootTime(const Eigen::Vector3d &armor_trans, SerialPortData SerialPortData_) //根据锁给的三维坐标直接计算打击时间
    {
        float pitch_now = SerialPortData_.pitch / 100.0f * 3.14159f / 180.0f;
        setShootSpeed(SerialPortData_.flag, SerialPortData_.shoot_speed / 100.0f);
        shootAngleTime_now = ballistic_equation(pitch_now, armor_trans); //当前实际位置击打所需时间
        float shootTime = shootAngleTime_now.time / 1000;                //需要测试各种延时(s)
        return shootTime;
    }

    //根据锁给的三维坐标直接计算yaw和pitch
    Angle_t Predictor::BuffPredict(const Eigen::Vector3d &armor_trans, SerialPortData SerialPortData_)
    {
        //根据传进来的位置直接计算yaw和pitch
        float pitch_now = SerialPortData_.pitch / 100.0f * 3.14159f / 180.0f;
        shootAngleTime_pre = ballistic_equation(pitch_now, armor_trans);
        return shootAngleTime_pre;
    }

    //直接卡尔曼滤波预测并计算yaw和pitch，待测
    Angle_t Predictor::AntirotPredict(const Eigen::Vector3d &armor_trans, SerialPortData SerialPortData_, float delta_t)
    {
        float pitch_now = SerialPortData_.pitch / 100.0f * 3.14159f / 180.0f;

        setShootSpeed(SerialPortData_.flag, SerialPortData_.shoot_speed / 100.0f);

        shootAngleTime_now = ballistic_equation(pitch_now, armor_trans);

        float shootTime = shootAngleTime_now.time / 1000; //需要测试各种延时(s)

        Eigen::Vector3d armor_pose = normal_kalman_filter->predict(armor_trans, shootTime + ShootParam::shoot_delay);

        shootAngleTime_pre = ballistic_equation(pitch_now, armor_pose);

        return shootAngleTime_pre;
    }

    //缓冲预测，卡尔曼 滤波预测不更新
    Angle_t Predictor::NoPosePredict(const float &delta_t, SerialPortData SerialPortData_)
    {
        Eigen::Vector3d predict_pose = normal_kalman_filter->NoPosePredict(delta_t); //状态predict,不update

        float pitch_now = SerialPortData_.pitch / 100.0f * 3.14159f / 180.0f;
        setShootSpeed(SerialPortData_.flag, SerialPortData_.shoot_speed / 100.0f);
        shootAngleTime_now = ballistic_equation(pitch_now, predict_pose); //当前实际位置击打所需时间
        float shootTime = shootAngleTime_now.time / 1000;                 //需要测试各种延时(s)

        Eigen::Vector3d armor_trans = normal_kalman_filter->predict(shootTime + ShootParam::shoot_delay);

        shootAngleTime_pre = ballistic_equation(pitch_now, armor_trans);
        return shootAngleTime_pre;
    }

    bool Predictor::runCTAntirotModel(const Eigen::Vector3d &armor_trans, std::chrono::steady_clock::time_point time_point, SerialPortData SerialPortData_, bool is_get_second_armor, const Eigen::Vector3d &second_armor_trans)
    {
        if (ekf_filter->this_is_change_armor) //装甲板切换
        {
            if (antirot_stamp.time_stamps.size() > 8) //防止误触发，设置上一个周期内的数据
            {
                DLOG(WARNING) << "SIZE:" << antirot_stamp.yaw_set.size();
                //选取中间目标作为yaw
                int target_id = (antirot_stamp.yaw_set.size() / 3);
                yaw = antirot_stamp.yaw_set[target_id];

                //处理高低装甲板
                if (is_get_last_pitch) //已经获得pitch轴信息
                {
                    last_pitch = pitch;
                }
                pitch = antirot_stamp.pitch_set[target_id];
                if (!is_get_last_pitch)
                {
                    is_get_last_pitch = true;
                    last_pitch = pitch;
                }

                shoot_time_stamp = antirot_stamp.time_stamps[target_id];
                Angle_t antirot_shoot_pre = ballistic_equation(SerialPortData_.pitch, armor_trans);
                antirot_shoot_time = antirot_shoot_pre.time; // ms

                double get_yaw = SerialPortData_.yaw / 100.0; //转角度
                first_yaw = *std::max_element(antirot_stamp.yaw_set.begin(), antirot_stamp.yaw_set.end()) * 180 / M_PI;
                last_yaw = *std::min_element(antirot_stamp.yaw_set.begin(), antirot_stamp.yaw_set.end()) * 180 / M_PI;
                first_yaw = (int)((first_yaw + round((get_yaw - first_yaw) / 360.0) * 360.0) * 100);
                last_yaw = (int)((last_yaw + round((get_yaw - last_yaw) / 360.0) * 360.0) * 100);

                is_get_shoot = true;
                antirot_stamp.yaw_set.clear();
                antirot_stamp.time_stamps.clear();
                antirot_stamp.pitch_set.clear();

                save_index = (save_index + 1) % 2;
                if (!history_stamp[save_index].empty())
                {
                    history_stamp[save_index].clear();
                }
            }
        }

        //开始记录每个周期的旋转
        if (ekf_filter->antirot_count >= 2)
        {
            antirot_stamp.yaw_set.emplace_back(-atan2(armor_trans[0], armor_trans[1]));
            Angle_t antirot_shoot_pre = ballistic_equation(SerialPortData_.pitch, armor_trans);
            antirot_stamp.pitch_set.emplace_back(antirot_shoot_pre.pitch);
            antirot_stamp.time_stamps.emplace_back(time_point);

            //记录滤波位姿数据
            HistoryStamp stamp_temp;
            stamp_temp.time_from_start = std::chrono::duration_cast<std::chrono::milliseconds>(time_point - ekf_filter->last_antirot_time_point).count();
            stamp_temp.filte_pose = ekf_filter->getPose();
            history_stamp[save_index].emplace_back(stamp_temp);
        }
        else
        {
            is_get_shoot = false;
            is_get_last_pitch = false;
            //清空数据

            for (int i = 0; i < 2; i++)
            {
                if (!history_stamp[i].empty())
                {
                    history_stamp[i].clear();
                }
            }
        }

        return is_get_shoot;
    }
    bool Predictor::getAntirotTime(std::chrono::steady_clock::time_point time_point)
    {
        //已经获得
        double time_to_target = std::chrono::duration_cast<std::chrono::microseconds>(time_point - shoot_time_stamp).count();

        double shoot_range = 0.135 / 0.125 / M_PI * antirot_shoot_time;

        //根据旋转速度来决定打弹量以及通区间

        for (int i = 1; i < 5; i++)
        {
            if (fabs((time_to_target / 1000.0 + antirot_shoot_time + ANTIROT_SHOOT_DELAY * 1000) - i * ekf_filter->aver_rotate_time * 1000) <= SHOOT_RANGE) //打击区间可以根据旋转速度来确定
            {

                LOG(WARNING) << "SHOOT !!!!!!!!!!";
                return true;
            }
        }
        // lcm_debug->sendData(Eigen::Vector3d(fabs((time_to_target / 1000.0 + antirot_shoot_time + ANTIROT_SHOOT_DELAY * 1000) - ekf_filter->aver_rotate_time * 1000.0) < 30, fabs((time_to_target / 1000.0 + antirot_shoot_time + ANTIROT_SHOOT_DELAY * 1000.0) - 2.0 * ekf_filter->aver_rotate_time * 1000) < 30, (time_to_target / 1000.0 + antirot_shoot_time + ANTIROT_SHOOT_DELAY * 1000) - ekf_filter->aver_rotate_time * 1000));

        return false;
    }
    short Predictor::getAntirotPitch(std::chrono::steady_clock::time_point time_point)
    {
        double time_to_target = std::chrono::duration_cast<std::chrono::microseconds>(time_point - shoot_time_stamp).count();

        //计算从建模开始，到打击到目标位置的时间
        // LOG(WARNING) << "ANTIROT_TIME: " << antirot_shoot_time;

        double from_start_to_shoot_time = time_to_target / 1000.0 + antirot_shoot_time + ANTIROT_SHOOT_DELAY * 1000; // ms

        int index = 1;
        double shoot_range = 0.135 / 0.125 / M_PI * antirot_shoot_time;
        // LOG(WARNING) << "SHOOT RANGE" << shoot_range;
        for (int i = 5; i > 0; i--)
        {
            //超出了打击前i块装甲板的范围
            // i = 1 说明打击 前1个装甲板已经明显延迟，那么此时应该瞄第二个装甲板
            if (from_start_to_shoot_time - i * ekf_filter->aver_rotate_time * 1000 > SHOOT_RANGE)
            {
                index = i + 1; //下一个目标的索引
                break;
            }
        }
        if (index % 2 == 0)
        {
            return (short)(last_pitch * 100);
        }
        else
        {
            return (short)(pitch * 100);
        }
    }
    void Predictor::sortTwoArmor(Sophus::SE3 &armor_pose1, Sophus::SE3 &armor_pose2, bool is_get_second_armor)
    {
        if (!is_get_second_armor)
        {
            last_armor_pose = armor_pose1.translation();
            return;
        }
        double dis1 = (armor_pose1.translation() - last_armor_pose).norm();
        double dis2 = (armor_pose2.translation() - last_armor_pose).norm();

        if (dis2 < dis1)
        {
            Sophus::SE3 temp = armor_pose1;
            armor_pose1 = armor_pose2;
            armor_pose2 = temp;
        }
        last_armor_pose = armor_pose1.translation(); //取与上一帧最近的数据

        // // //反陀螺状态下的特殊处理：
        if (is_get_shoot)
        {
            if (is_get_second_armor) //获得了第二个装甲板
            {
                Eigen::Vector3d armor_trans = armor_pose1.translation();
                Eigen::Vector3d second_armor_trans = armor_pose2.translation();
                double first_yaw = atan2(armor_trans[0], armor_trans[1]);
                double second_yaw = atan2(second_armor_trans[0], second_armor_trans[1]);

                double diff_yaw = second_yaw - first_yaw; //大于0说明顺时针旋转
                LOG(WARNING) << "DIFF_YAW" << diff_yaw << "SECOND YAW" << second_yaw;

                //打击旋转方向下落后的一块装甲板
                if (ekf_filter->rotation == ANTIROT_CLOCKWISE && ((diff_yaw > 0 && diff_yaw < M_PI / 2) || diff_yaw < -M_PI / 2))
                {
                    Sophus::SE3 temp = armor_pose1;
                    armor_pose1 = armor_pose2;
                    armor_pose2 = temp;
                }
                else if (ekf_filter->rotation == ANTIROT_COUNTER_CLOCK_WISE && ((diff_yaw < 0 && diff_yaw > -M_PI / 2) || diff_yaw > M_PI / 2))
                {
                    Sophus::SE3 temp = armor_pose1;
                    armor_pose1 = armor_pose2;
                    armor_pose2 = temp;
                }
            }
        }
    }
}
