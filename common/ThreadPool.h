#ifndef SLAM_LYJ_THREADPOOL_H
#define SLAM_LYJ_THREADPOOL_H

#include "base/Base.h"
#include <thread>
#include <functional>
#include <condition_variable>
#include <mutex>

namespace COMMON_LYJ
{

    using Task = std::function<void()>;
    using TaskFor = std::function<void(uint64_t, uint64_t)>;
    using TaskForWithId = std::function<void(uint64_t, uint64_t, uint32_t)>;

    class SLAM_LYJ_API Thread : public std::thread
    {
    private:
        /* data */
        std::queue<Task>* m_tasks = nullptr;
        std::condition_variable* m_con = nullptr;
        std::mutex* m_mtx = nullptr;
        int m_id = -1;
        bool m_toFinish = false;
        bool m_isFinish = false;
        std::mutex m_mtx2;

        std::queue<Task> m_taskFors;
        std::condition_variable m_conFor;
        std::mutex m_mtxFor;
    public:
        Thread(/* args */) = delete;

        Thread(std::queue<Task>* _tasks, std::condition_variable* _con, std::mutex* _mtx, const int _id);
        ~Thread();

        void start();
        int getIdInner();
        void finish();

        void run(TaskFor _task, uint64_t _s, uint64_t _e);
        void runWithId(TaskForWithId _task, uint64_t _s, uint64_t _e, uint32_t _id);
        void addTask(Task _task);
    };




    class SLAM_LYJ_API ThreadPool
    {
    private:
        /* data */
        std::condition_variable m_con;
        std::mutex m_mtx;
        std::queue<Task> m_tasks;
        int m_threadNum = -1;
        int m_maxThreadNum = -1;
        std::vector<Thread*> m_thds;
    public:
        ThreadPool(/* args */) = delete;
        ThreadPool(const int _threadNum);
        ~ThreadPool();

        // void start();
        void addTask(Task _task);
        void finish();
        int maxThreadNum();
        int currentThreadNum();

        void process(TaskFor _task, uint64_t _s, uint64_t _e);
        void processWithId(TaskForWithId _task, uint64_t _s, uint64_t _e);
    };




}

#endif //SLAM_LYJ_THREADPOOL_H

