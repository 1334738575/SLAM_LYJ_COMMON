#ifndef SLAM_LYJ_TIMER_H
#define SLAM_LYJ_TIMER_H

#include <chrono>   // 核心头文件

namespace COMMON_LYJ
{
    /// <summary>
    /// _EXPORT_STD using nanoseconds  = duration<long long, nano>;
    /// _EXPORT_STD using microseconds = duration<long long, micro>;
    /// _EXPORT_STD using milliseconds = duration<long long, milli>;
    /// _EXPORT_STD using seconds = duration<long long>;
    /// _EXPORT_STD using minutes = duration<int, ratio<60>>;
    /// _EXPORT_STD using hours = duration<int, ratio<3600>>;
    /// </summary>
    /// <typeparam name="TimeUnit"></typeparam>
    template<typename TimeUnit = std::chrono::milliseconds>
    class Timer {
    public:
        // 构造时开始计时
        Timer() : start_time(std::chrono::high_resolution_clock::now()) {}

        // 重置计时起点
        void reset() {
            start_time = std::chrono::high_resolution_clock::now();
        }

        // 获取已耗时（返回指定单位的数值）
        double elapsed(bool reset=false) {
            if (reset)
            {
                std::chrono::high_resolution_clock::time_point tmp = start_time;
                start_time = std::chrono::high_resolution_clock::now();
                return std::chrono::duration_cast<TimeUnit>(start_time - tmp).count();
            }
            else
            {
                auto end_time = std::chrono::high_resolution_clock::now();
                return std::chrono::duration_cast<TimeUnit>(end_time - start_time).count();
            }
        }

    private:
        std::chrono::high_resolution_clock::time_point start_time;
    };
}


#endif // !SLAM_LYJ_TIMER_H
