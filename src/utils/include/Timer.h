#ifndef _TIMER_H
#define _TIMER_H
#include "Config.h"
namespace ly
{
    enum TIME_TYPE
    {
        FREQ_TIMER,      //频率计时器
        TIME_TIMER,      //时间计时器
        FRAME_FREQ_TIMER //帧率计时器
    };
    class Timer
    {
    private:
        int count;
        int timer_type;
        int max_count;
        std::chrono::_V2::steady_clock::time_point count_begin; //测试平稳帧率
        std::chrono::_V2::steady_clock::time_point count_end;

    public:
        Timer(int mode, int freq = 10); //频率计时器
        ~Timer();
        bool call();
        void countStart();
        void countEnd();
        void resetCounter();
    };

}
#endif