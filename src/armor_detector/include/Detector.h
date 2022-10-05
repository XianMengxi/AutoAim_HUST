//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_DETECTOR_H
#define AUTOAIM_DETECTOR_H

#include "opencv2/opencv.hpp"
#include <algorithm>
#include <math.h>
#include <cstdio>
#include "Log.h"
#include "Params.h"
#include "LightBarFinder.h"
#include "ArmorFinder.h"
#include "SerialPort.h"
#include "VideoCapture.h"
#include "PoseSolver.h"
#include "Predictor.h"
#include "buffDetector.h"
#include "LockFreeQueue.hpp"
#include "TargetBumper.h"
#include <string>
#include "NumberClassifier.h"
#include "Timer.h"
#include "DetectAccelerator.h"
#include "SecondFilter.h"
#include "PitchYawFilter.h"
#include "buffCompensation.h"

// #define RECORD

using namespace cv;

namespace ly
{
    // static Ptr<cv::Tracker> tracker;
    //    const Scalar hsv_blue_low_boundary = Scalar (100, 43, 46);
    //    const Scalar hsv_blue_high_boundary = Scalar (124, 255, 255);

    struct Params_ToDetector
    {
        LockFreeQueue<PicStamp, 16> *frame_queue;
        vector<SerialPortData> *cache_p; //传入的串口数据池
        uint8_t *cache_idx;
        LockFreeQueue<SerialPortData, 32> *serial_queue;
        LockFreeQueue<Mat, 16> *save_frame_queue;

        std::chrono::steady_clock::time_point *time_stamp;
        Mat *armor;

        SerialPortData SerialPortData_;

        Params_ToDetector()
        {
            armor = new Mat();
            cache_idx = new uint8_t;
        }
        ~Params_ToDetector()
        {
        }
    };
    enum MODE
    {
        AUTO_AIM = 0,
        BUFF_AIM = 1,
        ANTIROT
    };

    class Detector
    {
    public:
        explicit Detector();
        ~Detector() = default;
        void setParams(const Params_ToVideo &params_to_video, const Params_ToSerialPort &params_to_serial_port);
        void startDetect(const Params_ToDetector &params, SerialPort *SerialPort_);
        void setParams(const Params_ToSave &params_to_save);

    private:
        inline void drawArmorCorners(Mat &drawing, const ArmorBlob &armor, const Scalar &color);
        inline void armorDetect();
        void drawTargetStr(Mat &drawing, const ArmorBlob &armor);
        void drawLightBar(Mat &drawing, const LightBarBlobs &blobs);
        void show(const Mat &drawing);
        void sendDataEncode(const Angle_t &shootAngleTime_pre, const SerialPortData &data_recieve, SerialPortWriteData &sendData_);
        void BuffsendDataEncode(const Angle_t &shootAngleTime_pre, const SerialPortData &data_recieve, SerialPortWriteData &sendData_);

        SerialPortWriteData sendData_;
        Params_ToDetector _detector_thread_params;
        pthread_t threadID{};
        clock_t pic_time;
        int BeginToNowTime;
        int BeginToNowTime_old = 0;
        float i_sin_debug = 0;
        int armors_buff = 0; //掉帧缓冲

        bool islast_armor = false;

        //分类种类
    };
}

#endif //AUTOAIM_DETECTOR_H
