//
// Created by zhiyu on 2021/8/20.
//

#include "VideoCapture.h"

using namespace ly;
using namespace cv;
extern std::chrono::steady_clock::time_point TX2_BeginTime;
extern int MCU_BeginTime;
ly::VideoCapture::VideoCapture()
{
    width = 1280;
    height = 1024;
    offset_x = 0;
    offset_y = 0;
}

ly::VideoCapture::~VideoCapture()
{
}

void ly::VideoCapture::chooseCameraType(ly::VideoCapture *&video)
{
    switch (CameraParam::device_type)
    {
    case DaHen:
        video = new DaHenCamera();
        break;
    case Video:
        video = new NativeVideo();
        break;
    case Picture:
        video = new NativePicture();
        break;
    default:
        video = new NativeVideo();
        break;
    }
    video->open();
}

void DaHenCamera::open()
{
    camera->initLib();
    camera->openDevice(CameraParam::sn.c_str());
    camera->setRoiParam(width, height, offset_x, offset_y);
    //1000,3000,5,16,127
    camera->setExposureGainParam(AutoExposureGain::is_use_auto_exposure,
                                 AutoExposureGain::is_use_auto_gain,
                                 CameraParam::exposure_time,
                                 AutoExposureGain::auto_exposure_min,
                                 AutoExposureGain::auto_exposure_max,
                                 CameraParam::gain,
                                 AutoExposureGain::auto_gain_min,
                                 AutoExposureGain::auto_gain_max,
                                 AutoExposureGain::target_gray_min,
                                 AutoExposureGain::target_gray_max, true);
    camera->setWhiteBalanceParam(WhiteBalanceParam::is_auto_balance, WhiteBalanceParam::white_balance_ratio_r, WhiteBalanceParam::white_balance_ratio_b, WhiteBalanceParam::white_balance_ratio_g, GX_AWB_LAMP_HOUSE_ADAPTIVE); //GX_AWB_LAMP_HOUSE_ADAPTIVE
    camera->setAAROIParam(640, 512, 320, 256);
    camera->acquisitionStart();
}

void DaHenCamera::startCapture(Params_ToVideo &params_to_video)
{

    // params out
    _video_thread_params.frame_queue = params_to_video.frame_queue;
    _video_thread_params.time_stamp = params_to_video.time_stamp;
    int empty_mat_count = 0;

    while (1)
    {

        // std::chrono::steady_clock::time_point timeStart = std::chrono::steady_clock::now();
        PicStamp temp;
        camera->ProcGetImage(&temp.pic, _video_thread_params.time_stamp);
        temp.time_stamp = *_video_thread_params.time_stamp;
        LOG_IF(ERROR, temp.pic.empty()) << "get empty picture mat!";

        //对相机无法产生图像的异常进行处理
        if (temp.pic.empty())
        {
            empty_mat_count++;
        }
        else
        {
            empty_mat_count = 0; //无异常，直接清零
        }
        if (empty_mat_count >= 10)
        {
            DLOG(FATAL) << "CAN'T GET SERIAL DATA";
        }
        _video_thread_params.frame_queue->push(temp);
        // std::chrono::steady_clock::time_point timeEnd = std::chrono::steady_clock::now();
        // int costTime = (int)std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeStart).count() / 1000.0f;
        // std::cout << "costTime: " << costTime << std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(ThreadDelay::pic_thread_delay));
    }
}

void *ly::saveFrameToNative(void *params_p)
{
    auto params = reinterpret_cast<Params_ToSave *>(params_p);
    auto save_frame_queue = params->save_frame_queue;

    cv::VideoWriter writer;
    int mode = writer.fourcc('M', 'P', '4', '2');
    bool is_writer_start = false;
    std::chrono::steady_clock::time_point writer_start_time;

    while (1)
    {
        try
        {
            cv::Mat frame;
            if (!save_frame_queue->pop(frame) || frame.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            if (!is_writer_start)
            {
                writer.open("../autoaim" + to_string(clock()) + ".avi", mode, 40.0, cv::Size(1280, 1024), true);
                if (!writer.isOpened())
                {
                    std::cerr << "ERROR WHEN OPEN WRITE MOVIE" << std::endl;
                }
                else
                {
                    is_writer_start = true;
                    writer_start_time = std::chrono::steady_clock::now();
                }
            }
            if (writer.isOpened())
            {
                auto start_write_time = std::chrono::steady_clock::now();
                writer.write(frame);
                auto end_write_time = std::chrono::steady_clock::now();
            }
            std::chrono::steady_clock::time_point writer_this_time = std::chrono::steady_clock::now();
            //两分钟之后保存视频
            if (std::chrono::duration_cast<std::chrono::seconds>(writer_this_time - writer_start_time).count() > 120)
            {
                LOG(WARNING) << "SAVE VIDEO";
                writer.release(); //保存视频
                is_writer_start = false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}

void ly::VideoCapture::startSave(Params_ToSave &params_to_save)
{
    // params in
    // _video_thread_params.writer = writer;
    _save_thread_params.save_frame_queue = params_to_save.save_frame_queue;

    int nRet = pthread_create(&threadID2, nullptr, saveFrameToNative, static_cast<void *>(&_save_thread_params));
    LOG_IF(ERROR, nRet == -1) << "error in creating video writer thread!";
}

DaHenCamera::DaHenCamera()
{
    camera = new GxCamera();
    //    string save_path = CameraParam::video_path.substr(0, CameraParam::video_path.rfind('/') + 1);
    //    FILE* fp = popen(("ls -l " + save_path + " |grep ^- | wc -l").c_str(), "r");
    //    std::fscanf(fp, "%d", &_id);
    //    pclose(fp);
    //    writer = VideoWriter(save_path + DetectorParam::color + to_string(_id) + ".avi", VideoWriter::fourcc('M','P','4','2'), 210.2, Size(1280, 1024));
    //    LOG(INFO) << "save video in: " << save_path + DetectorParam::color + to_string(_id) + ".avi";
}

DaHenCamera::~DaHenCamera()
{
    delete camera;
    writer.release();
}

void NativeVideo::open()
{
    this->video.open(CameraParam::video_path);
    LOG_IF(ERROR, !video.isOpened()) << "can't find video in " << CameraParam::video_path;
    rate = video.get(cv::CAP_PROP_FPS);
    DLOG(INFO) << "video fps: " << rate;
}

void NativeVideo::startCapture(Params_ToVideo &params)
{
    // params in
    _video_thread_params.video = this->video;
    _video_thread_params.__this = this;

    // params out
    _video_thread_params.frame_queue = params.frame_queue;

    while (1)
    {
        try
        {
            PicStamp frame;
            _video_thread_params.video >> frame.pic;
            frame.time_stamp = std::chrono::steady_clock::now();
            _video_thread_params.frame_queue->push(frame);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            break;
        }

        //        DEBUG_MODE(imshow("frame1", frame1));
        //        DEBUG_MODE(imshow("frame2", frame2));
        //        DEBUG_MODE(waitKey(1));
        std::this_thread::sleep_for(std::chrono::milliseconds(ThreadDelay::pic_thread_delay));
    }
    //    destroyAllWindows();
}

//void NativeVideo::startCapture(Params_ToVideo &params_to_video) {
//    // params in
//    _video_thread_params.video = this->video;
//    _video_thread_params.__this = this;
//
//    // params out
//    _video_thread_params.frame_p = params_to_video.frame_p;
//
//    int nRet = pthread_create(&threadID, nullptr, startCapture, static_cast<void *>(&_video_thread_params));
//    LOG_IF(ERROR, nRet == -1) << "error in creating native video thread!";
//}

NativeVideo::NativeVideo()
{
    string save_path = CameraParam::video_path.substr(0, CameraParam::video_path.rfind('/') + 1);
    FILE *fp = popen(("ls -l " + save_path + " |grep ^- | wc -l").c_str(), "r");
    std::fscanf(fp, "%d", &_id);
    pclose(fp);
    writer = VideoWriter(save_path + DetectorParam::color + to_string(_id) + ".avi", VideoWriter::fourcc('M', 'P', '4', '2'), 200, Size(1280, 1024));
    DLOG(INFO) << "save video in: " << save_path + DetectorParam::color + to_string(_id) + ".avi";
}

NativeVideo::~NativeVideo()
{
    video.release();
    writer.release();
}

void NativePicture::open()
{
}

void *ly::getFrameFromPicture(void *params_p)
{
}

void NativePicture::startCapture(Params_ToVideo &params_to_video)
{
    // 开启线程
    return;
}
