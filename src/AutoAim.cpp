#include "AutoAim.h"

using namespace ly;

int main(int argc, char **argv)
{
    // 传递给线程的参数
    Params_ToSerialPort params_to_serial_port;
    Params_ToVideo params_to_video;
    Params_ToDetector params_to_detector;
    Params_ToSave params_to_save;

    /******* init log ********/
    auto log = new Log();
    log->init(argv[0]);

    /******* init config ********/
    auto config = new Config(config_file_path);
    config->parse();

    /******* init serial port read ******/
    SerialPort *serial_port = new SerialPort();
    serial_port->startSerialPortRead(params_to_serial_port);

    /******* init camera ********/
    ly::VideoCapture *video;
    video->chooseCameraType(video);
    thread video_thread(&ly::VideoCapture::startCapture, video, ref(params_to_video));

    /******* init detector ********/
    ly::Detector detector = ly::Detector();
    detector.setParams(params_to_video, params_to_serial_port);
    detector.setParams(params_to_save);

    thread detector_thread(&ly::Detector::startDetect, &detector, ref(params_to_detector), serial_port);

#ifdef RECORD // 是否保存视频
    thread save_thread(&ly::VideoCapture::startSave, video, ref(params_to_save));

    save_thread.join();
#endif

    video_thread.join();
    detector_thread.join();
    return 0;
}
