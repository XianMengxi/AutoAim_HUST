#ifndef _LOCKFREE_QUEUE_
#define _LOCKFREE_QUEUE_
#include <iostream>
#include <atomic>
namespace ly
{
    template <typename T, int fixed_num>
    class LockFreeQueue
    {
        struct Element
        {
            T data_;
            std::atomic<bool> is_full; //是否有数据
        };

    public:
        LockFreeQueue();
        ~LockFreeQueue();
        bool push(T &input);
        bool pop(T &output);
        bool pop(std::vector<T> &output);
        std::atomic<short> write_index;
        Element data_queue[fixed_num];
    };
    template <typename T, int fixed_num>
    LockFreeQueue<T, fixed_num>::LockFreeQueue()
    {
        write_index = 0;
    }
    template <typename T, int fixed_num>
    LockFreeQueue<T, fixed_num>::~LockFreeQueue()
    {
    }
    template <typename T, int fixed_num>
    bool LockFreeQueue<T, fixed_num>::push(T &input) //注意由于std::move之后，原变量不能再使用，变成一个空壳
    {
        //由于后面的语句都对write_index_temp有依赖，故此处的内存模型都无所谓
        short write_index_temp = write_index.load(std::memory_order_relaxed);
        data_queue[write_index_temp].data_ = std::move(input);
        //强制前一条语句在这一条之前运行，即必须要修改完data_再将is_full置true
        data_queue[write_index_temp].is_full.store(true, std::memory_order::memory_order_release);
        write_index_temp++;
        write_index_temp %= fixed_num;
        write_index.store(write_index_temp, std::memory_order::memory_order_release);
        return true;
    }
    template <typename T, int fixed_num>
    bool LockFreeQueue<T, fixed_num>::pop(T &output) //注意由于std::move之后，原变量不能再使用，变成一个空壳
    {
        short read_index = write_index.load(std::memory_order::memory_order_relaxed);
        read_index = (read_index + fixed_num - 1) % fixed_num;
        //强制后面的语句在这一条语句之后运行,即必须要is_full为true才执行后面的代码
        if (data_queue[read_index].is_full.load(std::memory_order_acquire))
        {
            output = std::move(data_queue[read_index].data_);
            data_queue[read_index].is_full.store(false, std::memory_order::memory_order_release);
            return true;
        }
        return false;
    }
    template <typename T, int fixed_num>
    bool LockFreeQueue<T, fixed_num>::pop(std::vector<T> &output)
    {
        bool flag = false;
        short read_index = write_index.load(std::memory_order::memory_order_relaxed);
        read_index = (read_index + fixed_num - 1) % fixed_num;
        int count = 0;
        //强制后面的语句在这一条语句之后运行,即必须要is_full为true才执行后面的代码
        while ((count++) < fixed_num / 2 && data_queue[read_index].is_full.load(std::memory_order_acquire))
        {
            output.push_back(std::move(data_queue[read_index].data_));
            data_queue[read_index].is_full.store(false, std::memory_order::memory_order_release);
            read_index = (read_index + fixed_num - 1) % fixed_num;
            flag = true;
        }
        return flag;
    }

}

#endif