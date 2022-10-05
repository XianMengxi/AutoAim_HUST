//动态补偿计算
#include "math.h"
#include <string>
#include <fstream>
#include <iostream>
namespace ly
{
    struct Record
    {
        float raw_yaw;
        float raw_pitch;
        int raw_num; //原来采集数据个数

        float predict_yaw;
        float predict_pitch;
        int predict_num; //预测个数
        Record()
        {
            raw_num = 0;
            predict_num = 0;
            raw_yaw = 0;
            raw_pitch = 0;
            predict_yaw = 0;
            predict_pitch = 0;
        }
    };
    struct buffComp //补偿结构体
    {
        int yaw_comp;
        int pitch_comp;
    };
    class buffCompensation
    {
    private:
        /* data */
        Record angle_record[360]; //每一度记录一个结构体
        buffComp comp_record[360];

        const std::string save_file_name = "../src/utils/tools/buff_comp.txt";

    public:
        buffCompensation(/* args */);
        ~buffCompensation();

        void recordRawComp(float raw_yaw, float raw_pitch, float raw_angle); //记录相应的角度yaw,pitch
        void recordPredictComp(float predict_yaw, float predict_pitch, float predict_angle);

        void saveCompToFile();
        void readCompFromFile();
        void getDynamicComp(int &yaw, int &pitch, float angle);
    };
}
