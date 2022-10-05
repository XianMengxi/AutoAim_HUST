//
// Created by zhiyu on 2021/8/20.
//

#include "Detector.h"

// #define RECORD //记录视频
#define SERIAL_COMM_DELAY 4 // ms

namespace ly
{

    ArmorBlob last_armor;
    Angle_t shootAngleTime_pre;
    extern std::chrono::steady_clock::time_point TX2_BeginTime;
    extern int MCU_BeginTime;

    float getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2)
    {
        float x = (point_1 - point_2).x;
        float y = (point_1 - point_2).y;
        return sqrt(x * x + y * y);
    }

    void Detector::setParams(const Params_ToVideo &params_to_video, const Params_ToSerialPort &params_to_serial_port)
    {

        // send params to detector thread
        _detector_thread_params.frame_queue = params_to_video.frame_queue;
        _detector_thread_params.time_stamp = params_to_video.time_stamp;
        _detector_thread_params.serial_queue = params_to_serial_port.serial_data_queue;
    }
    void Detector::setParams(const Params_ToSave &params_to_save)
    {
        _detector_thread_params.save_frame_queue = params_to_save.save_frame_queue;
    }

    void Detector::startDetect(const Params_ToDetector &params, SerialPort *SerialPort_)
    {
        // params out
        _detector_thread_params.armor = params.armor;

        // image to show
        Mat drawing = Mat();

        auto lightBarFinder = new LightBarFinder();
        auto armorFinder = new ArmorFinder();

        // nunber classify
        NumberClassifier *number_classifier = new NumberClassifier();

        //计时器
        Timer *shoot_timer = new Timer(FREQ_TIMER, ShootParam::shoot_frequency);
        Timer *buff_shoot_timer = new Timer(FREQ_TIMER, BuffParam::shoot_frequency);

        //帧率计时器
        Timer *frame_freq_timer = new Timer(FRAME_FREQ_TIMER);
        frame_freq_timer->countStart();

        // pose
        auto solver = new PoseSolver();
        auto predict_ = new Predictor();
        SE3 armor_pose, second_armor_pose;

        //缓冲器
        TargetBumper *bumper = new TargetBumper();

        //大符击打
        BuffDetector *buff_detector = new BuffDetector(DetectorParam::buff_params);
        BuffArmor aim_buff_target;

        SecondFilter *buff_filter = new SecondFilter(); //用以将预测位置进行滤波

        //仅使用上位机的时间，测试
        std::chrono::steady_clock::time_point last_time_point;

        //连续多少次不能从串口读取到数据就重启
        int serial_restart_count = 0;

        // ROI 进行加速
        DetectAccelerator *accelerator = new DetectAccelerator();

        LcmDebug debug;

        int mode = AUTO_AIM;
        int detector_mode = NOT_GET_TARGET;
        buff_filter->setMeasurementNoise(0.1, 1, 0.1);
        buff_filter->setProcessNoise(0.003, 0.001, 0.003);

        PitchYawFilter *pitch_yaw_filter = new PitchYawFilter();

        int update_symbol = 0;

        int shoot_target_id = -1;
        int not_wanted_id = -1;
        char last_change_id_symbol = 0x00;

        //测时间延迟
        std::vector<SerialPortData> serial_data_set;
        serial_data_set.reserve(100);

        buffCompensation *buff_comp = new buffCompensation();

        while (1)
        {
            //获取图像
            PicStamp frame;
            if (!_detector_thread_params.frame_queue->pop(frame) || frame.pic.empty())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(ThreadDelay::detect_thread_delay));
                continue;
            }
            // std::vector<SerialPortData> serial_data_set_temp;

            if (!_detector_thread_params.serial_queue->pop(_detector_thread_params.SerialPortData_))
            {
                serial_restart_count++;
                if (serial_restart_count >= 70)
                {
                    DLOG(FATAL) << "CAN'T GET SERIAL DATA";
                }
                std::this_thread::sleep_for(std::chrono::microseconds(ThreadDelay::detect_thread_delay));
                continue;
            }
            serial_restart_count = 0;

            // //选取第n ms之前的数据来使用，使控制值抖动程度降到最低
            // _detector_thread_params.SerialPortData_ = serial_data_set_temp[0]; //最新的数据

            // int this_mcu_time = _detector_thread_params.SerialPortData_.MCU_Time;

            // for (int i = serial_data_set_temp.size() - 1; i >= 0; i--) //遍历，将数据存进
            // {
            //     serial_data_set.emplace_back(serial_data_set_temp[i]); //存进队列里
            // }
            // for (int i = serial_data_set.size() - 1; i >= 0; i--) //从后面往前推
            // {
            //     int diff_time = this_mcu_time - serial_data_set[i].MCU_Time;

            //     if (diff_time == SERIAL_COMM_DELAY) //相同直接跳出循环
            //     {
            //         _detector_thread_params.SerialPortData_ = serial_data_set[i];
            //         break;
            //     }
            //     else if (diff_time < SERIAL_COMM_DELAY) //小于直接覆盖
            //     {
            //         _detector_thread_params.SerialPortData_ = serial_data_set[i];
            //     }
            //     else //大于的话，进行插值
            //     {
            //         int high_time = diff_time - SERIAL_COMM_DELAY;
            //         int low_time = SERIAL_COMM_DELAY - (this_mcu_time - _detector_thread_params.SerialPortData_.MCU_Time); //插值
            //         float high_ratio = (float)low_time / (high_time + low_time);
            //         LOG(INFO) << "HIGH RATIO" << high_ratio;
            //         _detector_thread_params.SerialPortData_.pitch = (short)(serial_data_set[i].pitch * high_ratio + (1 - high_ratio) * _detector_thread_params.SerialPortData_.pitch);
            //         _detector_thread_params.SerialPortData_.yaw = (int)(serial_data_set[i].yaw * high_ratio + (1 - high_ratio) * _detector_thread_params.SerialPortData_.yaw);
            //         break;
            //     }
            // }
            // // LOG(INFO) << "DIFF TIME" << this_mcu_time - _detector_thread_params.SerialPortData_.MCU_Time;

            // //清除部分
            // if (serial_data_set.size() >= 100)
            // {
            //     int delete_num = serial_data_set.size() - 100;
            //     serial_data_set.erase(serial_data_set.begin(), serial_data_set.begin() + delete_num);
            // }

            if (ArmorDetectParam::debug_show)
            {
                frame.pic.copyTo(drawing);
            }

            if ((_detector_thread_params.SerialPortData_.flag & 0x08) == 8 || (_detector_thread_params.SerialPortData_.flag & 0x20) == 0x20) //大符模式
            //if(true)
            {
                if (mode != BUFF_AIM) //模式切换
                {
                    buff_detector->NewPredictor();
                    mode = BUFF_AIM;
                    last_time_point = frame.time_stamp; //设置初始时间戳
                }
                if ((_detector_thread_params.SerialPortData_.flag & 0x20) == 0x20) //小符模式
                {
                    buff_detector->setPredictMode(SMALL_BUFF);
                }
                else
                {
                    buff_detector->setPredictMode(LARGE_BUFF);
                }
                buff_detector->setColor(_detector_thread_params.SerialPortData_.flag);

                //识别之前，先传进当前的角度
                buff_detector->this_pitch = _detector_thread_params.SerialPortData_.pitch / 100.0f * M_PI / 180.0f;
                aim_buff_target = buff_detector->runBuffDetect(frame.pic);

                // 分区赛之后的大符
                if (!aim_buff_target.is_useful)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(0));

                    LOG(ERROR) << "CAN NOT GET BUFF";
                    continue;
                }
                float delta_t_by_chrono = std::chrono::duration_cast<std::chrono::microseconds>(frame.time_stamp - last_time_point).count() / 1000.0f;
                last_time_point = frame.time_stamp; //更新时间戳
                std::cout << "delta_t: " << delta_t_by_chrono << std::endl;

                //畸变矫正
                std::vector<cv::Point2f> input_points;
                std::vector<cv::Point2f> output_points;
                input_points.emplace_back(aim_buff_target.armor_rect.center);
                input_points.emplace_back(aim_buff_target.buff_center);
                solver->undistortBuffPoints(input_points, output_points);

                cv::Point2f target_2d_point;
                Eigen::Vector3d shoot_likly_target;

                float filte_angle = buff_detector->OnePointPreidct(frame.time_stamp); //获得角度
                Eigen::Vector3d actual_center = Eigen::Vector3d(0, 6.6, 1.05);
                shoot_likly_target = actual_center + cos(filte_angle) * Eigen::Vector3d::UnitX() * 0.7 + sin(filte_angle) * Eigen::Vector3d::UnitZ() * 0.7;
                float shoot_time = predict_->calShootTime(shoot_likly_target, _detector_thread_params.SerialPortData_);
                //shoot_time = 0.264;
                target_2d_point = buff_detector->OnePointPositonCalc(output_points[1], output_points[0], shoot_time + BuffParam::shoot_delay, filte_angle); //通过圆心以及装甲板点计算预测二维点

                if (BuffParam::test_mode)
                {
                    target_2d_point = output_points[0];
                }
                // std::cout<<"shoot_time"<<shoot_time<<std::endl;
                // std::cout << "target_3d: " << shoot_likly_target << std::endl;

                if (fabs(target_2d_point.x + 1) < 1e-3)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(0));
                    continue;
                }

                float yaw, pitch;

                solver->onePointCalc(_detector_thread_params.SerialPortData_, yaw, pitch, target_2d_point);
                pitch_yaw_filter->runKalman(yaw, pitch);
                float filte_yaw = pitch_yaw_filter->getYaw();
                float filte_pitch = pitch_yaw_filter->getPitch();

                float shoot_angle = buff_detector->getShootAngle(filte_angle, shoot_time + BuffParam::shoot_delay);
                // debug.sendData(Eigen::Vector3d(yaw, filte_yaw, 0), Eigen::Vector3d(pitch, filte_pitch, 0));
                sendData_.yaw = (int)(filte_yaw * 180.0f / M_PI * 100.0f) + predict_->getBuffYawDiff(shoot_angle); //转角度
                sendData_.pitch = (short)(filte_pitch * 180.0f / M_PI * 100.0f) + predict_->getBuffPitchDiff(shoot_angle);

                //记录相关信息
                float raw_yaw;
                float raw_pitch;

                solver->onePointCalc(_detector_thread_params.SerialPortData_, raw_yaw, raw_pitch, output_points[0]);
                buff_comp->recordRawComp(raw_yaw, raw_pitch, filte_angle);
                buff_comp->recordPredictComp(filte_yaw, filte_pitch, shoot_angle);

                buff_comp->saveCompToFile();

                if (delta_t_by_chrono < 50)
                {
                    static float t_raw = 0.0f;
                    static float t_predict = 0.0f;
                    t_raw += delta_t_by_chrono / 1000.0f;
                    t_predict = t_raw + shoot_time + BuffParam::shoot_delay;

                    debug.sendInfo(Eigen::Vector3d(raw_yaw * 180.0f / M_PI * 100.0f, raw_pitch * 180.0f / M_PI * 100.0f, 0), t_raw, Eigen::Vector3d(filte_yaw * 180.0f / M_PI * 100.0f, filte_pitch * 180.0f / M_PI * 100.0f, 0), t_predict);
                }
                // std::cout<<"y : "<<shoot_likly_target[2]<<std::endl;

                if (buff_detector->judgeRestart())
                {
                    buff_shoot_timer->resetCounter();
                    pitch_yaw_filter->resetKalman();
                    LOG(INFO) << "*********"
                              << "restart"
                              << "*******";
                }
                if (buff_shoot_timer->call() && ShootParam::is_shoot)
                {
                    sendData_.shootStatus = ((int)sendData_.shootStatus + 1) % 2;
                }

                SerialPort_->writeData(&sendData_);

                if (ArmorDetectParam::debug_show)
                {
                    buff_detector->drawArmor(drawing, aim_buff_target.corner);
                    // buff_detector->drawCountour(draw)
                    // buff_detector->drawAllArmor(drawing);
                    buff_detector->drawAngle(drawing, aim_buff_target.armor_rect, aim_buff_target.buff_center);
                    buff_detector->drawCircle(drawing, aim_buff_target.armor_rect.center);
                    buff_detector->drawFatherRect(drawing, aim_buff_target);
                    cv::circle(drawing, target_2d_point, 2, cv::Scalar(200, 200, 0), 2);
                    DEBUG_MODE(cv::imshow("frame", drawing));
                    DEBUG_MODE(cv::waitKey(1));
                }
            }
            else //辅瞄模式
            {
                bool is_user_change_id = false;
                mode = AUTO_AIM;

                //是否跟随敌方小陀螺
                bool is_follow_antirot = false;

                bumper->printMode(detector_mode);

                LightBarBlobs lightBarBlobs;
                //设置识别颜色
                lightBarFinder->setEnemyColor(_detector_thread_params.SerialPortData_.flag);
                //show(drawing);

                unsigned char change_symbol = (_detector_thread_params.SerialPortData_.flag & 0x10);
                if (change_symbol != last_change_id_symbol)
                {
                    last_change_id_symbol = change_symbol;
                    is_user_change_id = true;
                    LOG(ERROR) << "USER CHANGE TARGET";
                    accelerator->resetROI(); //关闭ROI功能

                    detector_mode = NOT_GET_TARGET;
                    not_wanted_id = shoot_target_id; //不要这个ID的装甲板了
                    shoot_target_id = -1;
                }

                bool isFindBlob = lightBarFinder->findLightBarBlobs(frame.pic(accelerator->getROI()), lightBarBlobs, accelerator->getRoiOffset());

                uint8_t size = lightBarBlobs.size();
                if (!isFindBlob || size < 2)
                {
                    DLOG(WARNING) << "lightbar size:" << (int)size << std::endl;
                }
                ArmorBlobs armors_candidate;
                drawLightBar(drawing, lightBarBlobs);
                for (int i = 0; i < size - 1; i++)
                {
                    //这个灯条匹配成功的话，立即跳过这个灯条继续匹配
                    int this_lightbar_fit_size = 0;        //这个灯条的匹配数
                    for (int j = 0; i + j + 1 < size; j++) //两两匹配
                    {
                        ArmorBlob armor;
                        if (armorFinder->getArmor(lightBarBlobs[i], lightBarBlobs[i + j + 1], armor) &&
                            armorFinder->matchTwoLightBar(lightBarBlobs[i], lightBarBlobs[i + j + 1]) &&
                            armorFinder->judgeArmor(armor))
                        {
                            bool is_armor = true;
                            // for (int k = i + 1; k < j; k++) //找两个灯条中间的灯条是否存在，若存在，则直接舍弃（如果爆光过高的话，数字有可能被识别为灯条）
                            // {
                            //     //判断灯条在装甲板内
                            //     if (!armorFinder->filteArmor(lightBarBlobs[k], armor))
                            //     {
                            //         is_armor = false;
                            //         break;
                            //     }
                            // }
                            if (is_armor)
                            {
                                //画出角点位置
                                drawArmorCorners(drawing, armor, Scalar(0, 0, 255));
                                armors_candidate.emplace_back(armor);
                                this_lightbar_fit_size++;
                            }
                        }
                        //匹配到一个退出
                        if (this_lightbar_fit_size > 0)
                        {
                            break;
                        }
                    }
                }
                if (ArmorDetectParam::debug_show)
                {
                    cv::imshow("drawing", drawing);
                    cv::waitKey(1);
                }

                LOG(INFO) << "armor_targets.size" << armors_candidate.size();

                //获得分类目标
                ArmorBlobs armor_targets_candidate;
                for (int i = 0; i < armors_candidate.size(); i++)
                {
                    //从候选装甲板中选取，从靠近图像中心的装甲板开始分类
                    armors_candidate[i]._class = number_classifier->predict(frame.pic, armors_candidate[i].corners);
                    if (armors_candidate[i]._class <= 0 || armors_candidate[i]._class == Engineer || armors_candidate[i]._class == Outpost) //不打工程
                    {
                        continue;
                    }

                    drawTargetStr(drawing, armors_candidate[i]);
                    number_classifier->showNumberStr(drawing, armors_candidate[i]._class, armors_candidate[i].rect);

                    armor_targets_candidate.push_back(armors_candidate[i]);
                }

                ArmorBlob armor_target;
                ArmorBlob second_armor_target;
                Sophus::SE3 armor_pose;
                Sophus::SE3 second_armor_pose;
                bool is_get_second_armor = false;

                LOG(INFO) << "armor_targets_candidate.size" << armor_targets_candidate.size();

                //如果处于未识别状态
                if (detector_mode == NOT_GET_TARGET)
                {
                    if (armor_targets_candidate.size() > 0)
                    {
                        armor_target = bumper->getAimTarget(armor_targets_candidate, -1, not_wanted_id);
                        shoot_target_id = armor_target._class;
                        detector_mode = DETECT_BUMP;
                    }
                    //选取最高优先级的作为目标
                }
                else if (detector_mode == DETECT_BUMP) //进入连续检测状态的缓冲状态
                {
                    //设定需要的目标ID,并进行一定帧数的缓冲
                    if (armor_targets_candidate.size() == 0) //在无目标的情况之下
                    {
                        detector_mode = NOT_GET_TARGET;
                        shoot_target_id = -1; //退出检测状态
                        bumper->targetLossEmpty();

                        goto LOOP_END;
                    }
                    armor_target = bumper->getAimTarget(armor_targets_candidate, shoot_target_id, not_wanted_id);
                    LOG(INFO) << "ARMOR TARGET CLASS:" << armor_target._class;

                    if (armor_target._class != shoot_target_id) //无目标ID,不能直接进入状态
                    {
                        detector_mode = NOT_GET_TARGET;
                        shoot_target_id = -1; //退出检测状态
                        bumper->targetLossEmpty();
                    }
                    else //缓冲
                    {
                        if (!bumper->lossAndCHeck(detector_mode)) //检查连续识别到的帧数，计时器
                        {
                            //缓冲状态结束，进入连续识别状态
                            detector_mode = CONTINOUS_GET_TARGET;
                            bumper->targetLossEmpty(); //清空残留的状态
                        }

                        //设置ROI信息
                        accelerator->detectFeedback(armor_target.corners);
                        accelerator->drawROI(drawing);
                    }
                }
                else if (detector_mode == LOST_BUMP)
                {
                    if (armor_targets_candidate.size() != 0)
                    {
                        armor_target = bumper->getAimTarget(armor_targets_candidate, shoot_target_id);
                    }

                    if (armor_targets_candidate.size() == 0 || armor_target._class != shoot_target_id) //无目标ID,不能直接进入状态
                    {
                        if (!bumper->lossAndCHeck(detector_mode)) //需要调节到陀螺状态不退出
                        {
                            detector_mode = NOT_GET_TARGET;
                            bumper->targetLossEmpty();
                            shoot_target_id = -1;           //退出检测状态
                            predict_->is_get_shoot = false; //陀螺状态退出
                        }
                    }
                    else //恢复连续识别状态
                    {
                        detector_mode = CONTINOUS_GET_TARGET;
                        bumper->targetLossEmpty();
                    }

                    if (predict_->is_get_shoot) //反陀螺处理
                    {
                        goto ANTIROT_PROCESS;
                    }
                }

                if (detector_mode != CONTINOUS_GET_TARGET)
                {
                    goto LOOP_END;
                }

                not_wanted_id = -1; //不需要的ID清空

                //以下为连续识别状态处理代码

                if (bumper->getIDTarget(armor_targets_candidate, shoot_target_id) && armor_targets_candidate.size() < 3)
                {
                    //说明有相应的装甲板，状态保持

                    armor_target = armor_targets_candidate[0];
                    if (armor_targets_candidate.size() == 2)
                    {
                        second_armor_target = armor_targets_candidate[1];
                        is_get_second_armor = true;
                    }
                    //时间差计算
                    float delta_t_by_chrono = std::chrono::duration_cast<std::chrono::microseconds>(frame.time_stamp - last_time_point).count() / 1000.0f;
                    LOG(INFO) << "DELTA_T: " << delta_t_by_chrono;

                    last_time_point = frame.time_stamp; //更新时间戳

                    //量测计算
                    armor_pose = solver->getPoseInCamera(armor_target._class, armor_target.corners, (_detector_thread_params.SerialPortData_));

                    if (armor_pose.translation().norm() > 8 || armor_pose.translation()[2] > 2) // 7m之外暂时不识别
                    {
                        //校验失败，进入掉帧模式
                        //无对应的装甲板，进入掉帧缓冲状态
                        detector_mode = LOST_BUMP;
                        if (predict_->is_get_shoot) //如果是小陀螺状态，则跳转
                        {
                            goto ANTIROT_PROCESS;
                        }
                        goto LOOP_END;
                    }

                    if (is_get_second_armor)
                    {
                        second_armor_pose = solver->getPoseInCamera(second_armor_target._class, second_armor_target.corners, (_detector_thread_params.SerialPortData_));
                    }

                    predict_->sortTwoArmor(armor_pose, second_armor_pose, is_get_second_armor);

                    //设置预测ID
                    predict_->target_id = armor_target._class;
                    //预测
                    shootAngleTime_pre = predict_->Predict(armor_pose.translation(), _detector_thread_params.SerialPortData_, delta_t_by_chrono);

                    predict_->runCTAntirotModel(armor_pose.translation(), frame.time_stamp, _detector_thread_params.SerialPortData_);
                    predict_->AntirotPredict(_detector_thread_params.SerialPortData_, frame.time_stamp, shootAngleTime_pre, delta_t_by_chrono);
                    // is_follow_antirot = predict_->CTModelTest(armor_pose.translation(), _detector_thread_params.SerialPortData_, delta_t_by_chrono);

                    accelerator->detectFeedback(armor_target.corners);
                    accelerator->drawROI(drawing);
                }
                else
                {
                    //无对应的装甲板，进入掉帧缓冲状态
                    detector_mode = LOST_BUMP;
                    // if (predict_->is_get_shoot) //如果是小陀螺状态，则跳转
                    // {
                    //     goto ANTIROT_PROCESS;
                    // }
                    goto LOOP_END;
                }
            ANTIROT_PROCESS:
                // predict_->checkAntirotPose(shootAngleTime_pre, _detector_thread_params.SerialPortData_);
                // if (is_follow_antirot) //小陀螺模式下的跟随
                // {
                //     shootAngleTime_pre = predict_->getAntirotShootTarget();
                // }
                // else if (predict_->is_get_shoot) //小陀螺模式但不跟随
                // {
                //     goto LOOP_END;
                // }
                // armor_pose即为目标装甲板
                //第一个数据编码
                sendDataEncode(shootAngleTime_pre, _detector_thread_params.SerialPortData_, sendData_);
                //编码发送
                sendData_.pitch = shootAngleTime_pre.pitch * 100; //求解偏移量 140四号

                LOG(INFO) << "SHOOT ID:" << shoot_target_id;
                // if (predict_->is_get_shoot)
                // {
                //     sendData_.yaw = predict_->yaw * 180 / M_PI; //转为角度
                //     double yaw = _detector_thread_params.SerialPortData_.yaw / 100.0;
                //     sendData_.yaw = (int)((sendData_.yaw + round((yaw - sendData_.yaw) / 360.0) * 360.0) * 100);
                //     sendData_.pitch = predict_->getAntirotPitch(frame.time_stamp); //求解偏移量 140四号
                // }

                if (ShootParam::is_shoot)
                {
                    if (predict_->is_get_shoot) //说明进入了反陀螺状态
                    {
                        if (predict_->getAntirotTime(frame.time_stamp))
                        {
                            sendData_.shootStatus = ((int)sendData_.shootStatus + 1) % 2;
                        }
                    }

                    else if ((shoot_timer->call() && predict_->judgeIsShoot())) //普通击打状态，给定yaw和pitch角度差
                    {
                        sendData_.shootStatus = ((int)sendData_.shootStatus + 1) % 2;
                    }
                }
                SerialPort_->writeData(&sendData_);

                //显示
                show(drawing);

                //帧率计算
                frame_freq_timer->countEnd();

            LOOP_END:
#ifdef RECORD
                _detector_thread_params.save_frame_queue->push(frame.pic);
#endif
                std::this_thread::sleep_for(std::chrono::microseconds(0));
            }
        }
        destroyAllWindows();
    }

    Detector::Detector()
    {
    }

    void Detector::drawArmorCorners(Mat &drawing, const ArmorBlob &armor, const Scalar &color)
    {
        if (ArmorDetectParam::debug_show)
        {
            for (auto i = 0; i < 4; i++)
            {
                line(drawing, armor.corners[i], armor.corners[(i + 1) % 4], color, 2, 8);
            }
        }
    }
    void Detector::drawTargetStr(Mat &drawing, const ArmorBlob &armor)
    {
        if (ArmorDetectParam::debug_show)
        {
            for (int i = 0; i < 4; i++)
            {
                string str;
                str = to_string(i);
                putText(drawing, str, armor.corners[i], FONT_HERSHEY_SCRIPT_SIMPLEX, 1, Scalar(255, 255, 0), 2);
            }
        }
    }
    void Detector::drawLightBar(Mat &drawing, const LightBarBlobs &blobs)
    {
        if (ArmorDetectParam::debug_show)
        {
            for (int i = 0; i < blobs.size(); i++)
            {
                ellipse(drawing, blobs[i], Scalar(0, 255, 0), 2, 8);
            }
        }
    }
    void Detector::show(const Mat &drawing)
    {
        // debug显示
        if (ArmorDetectParam::debug_show)
        {
            DEBUG_MODE(imshow("frame", drawing));
            DEBUG_MODE(waitKey(5));
        }
    }
    void Detector::sendDataEncode(const Angle_t &shootAngleTime_pre, const SerialPortData &data_recieve, SerialPortWriteData &sendData_)
    {
        double yaw = data_recieve.yaw / 100.0; //转角度

        sendData_.yaw = (int)((shootAngleTime_pre.yaw + round((yaw - shootAngleTime_pre.yaw) / 360.0) * 360.0) * 100) + ShootParam::shoot_up_offsetk;
    }
    void Detector::BuffsendDataEncode(const Angle_t &shootAngleTime_pre, const SerialPortData &data_recieve, SerialPortWriteData &sendData_)
    {
        sendData_.pitch = shootAngleTime_pre.pitch * 100 + BuffParam::shoot_up_offset; //求解偏移量 140四号

        double yaw = data_recieve.yaw / 100.0; //转角度

        sendData_.yaw = (int)((shootAngleTime_pre.yaw + round((yaw - shootAngleTime_pre.yaw) / 360.0) * 360.0) * 100) - BuffParam::shoot_right_offset;
    }
}
