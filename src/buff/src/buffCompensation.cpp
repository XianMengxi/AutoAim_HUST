#include "buffCompensation.h"

namespace ly
{
    buffCompensation::buffCompensation(/* args */)
    {
    }

    buffCompensation::~buffCompensation()
    {
    }
    void buffCompensation::recordRawComp(float raw_yaw, float raw_pitch, float raw_angle) //记录相应的角度yaw,pitch
    {
        raw_angle = raw_angle * 180 / 3.1415927; //转角度
        raw_angle = raw_angle - (int)ceil(raw_angle / 360.0) * 360;
        int save_index = round(raw_angle); //四舍五入
        if (save_index == 360)
        {
            save_index = 0;
        }
        angle_record[save_index].raw_num += 1;
        angle_record[save_index].raw_pitch += raw_pitch;
        angle_record[save_index].raw_yaw += raw_yaw;
    }
    void buffCompensation::recordPredictComp(float predict_yaw, float predict_pitch, float predict_angle)
    {
        predict_angle = predict_angle * 180 / 3.1415927; //转角度
        predict_angle = predict_angle - (int)ceil(predict_angle / 360.0) * 360;
        int save_index = round(predict_angle); //四舍五入
        if (save_index == 360)
        {
            save_index = 0;
        }
        angle_record[save_index].raw_num += 1;
        angle_record[save_index].raw_pitch += predict_pitch;
        angle_record[save_index].raw_yaw += predict_yaw;
    }
    void buffCompensation::saveCompToFile()
    {
        std::fstream file_reader;
        file_reader.open(save_file_name, std::ios::in);

        if (!file_reader.is_open())
        {
            return;
        }
        for (int i = 0; i < 360; i++)
        {
            double predict_yaw, predict_pitch, raw_yaw, raw_pitch;
            if (angle_record[i].predict_num != 0)
            {
                predict_yaw = angle_record[i].predict_yaw / angle_record[i].predict_num;
                predict_pitch = angle_record[i].predict_pitch / angle_record[i].predict_num;
            }
            if (angle_record[i].raw_num != 0)
            {
                raw_yaw = angle_record[i].raw_yaw / angle_record[i].raw_num;
                raw_pitch = angle_record[i].raw_pitch / angle_record[i].raw_num;
            }

            if (angle_record[i].raw_num != 0 && angle_record[i].predict_num != 0)
            {
                file_reader << (int)((raw_yaw - predict_yaw) * 180 * 100 / M_PI) << (int)((raw_pitch - predict_pitch) * 180 * 100 / M_PI) << std::endl;
            }
            else
            {
                file_reader << 0 << 0 << std::endl;
            }
        }
    }
    void buffCompensation::readCompFromFile()
    {
        std::fstream file_reader;
        file_reader.open(save_file_name, std::ios::out);
        if (!file_reader.is_open())
        {
            return;
        }
        for (int i = 0; i < 360; i++)
        {
            file_reader >> comp_record[i].yaw_comp;
            file_reader >> comp_record[i].pitch_comp;
        }
    }
    void buffCompensation::getDynamicComp(int &yaw, int &pitch, float angle)
    {
        angle = angle * 180 / 3.1415927; //转角度
        angle = angle - (int)ceil(angle / 360.0) * 360;
        int save_index = round(angle); //四舍五入
        if (save_index == 360)
        {
            save_index = 0;
        }
        yaw = comp_record[save_index].yaw_comp;
        pitch = comp_record[save_index].pitch_comp;
    }

}
