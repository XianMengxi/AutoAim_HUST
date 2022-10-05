#include "Timer.h"
namespace ly
{
    Timer::Timer(int mode, int freq)
    {
        timer_type = mode;
        if (mode == FREQ_TIMER)
        {
            max_count = freq;
        }
        count = 0;
    }
    bool Timer::call()
    {
        if (timer_type == FREQ_TIMER)
        {
            count++;
            count %= max_count;
            if (count % max_count == max_count - 1)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        return false;
    }
    void Timer::countStart()
    {
        count_begin = std::chrono::steady_clock::now();
    }
    void Timer::countEnd()
    {
        count++;
        count_end = std::chrono::steady_clock::now();
        DLOG(INFO) << "frame rate ：" << (std::chrono::duration_cast<std::chrono::microseconds>(count_end - count_begin)).count() / count << " us";
    }

    Timer::~Timer()
    {
    }
    void Timer::resetCounter()
    {
        count = 0;
    }
}
