#include "ThreadPool.h"

NSP_SLAM_LYJ_MATH_BEGIN




Thread::Thread(std::queue<Task> *_tasks, std::condition_variable *_con, std::mutex* _mtx, const int _id)
    :m_tasks(_tasks), m_con(_con), m_id(_id), m_mtx(_mtx), std::thread(&Thread::start, this)
{
#ifdef SYS_DEBUG
    std::cout<<"start thread: " << _id << std::endl;
#endif
}
Thread::~Thread()
{}
void Thread::start()
{
    while (true)
    {
        //take
        std::unique_lock<std::mutex> lck(m_mtxFor);
        m_conFor.wait(lck, [&] {return !m_taskFors.empty(); });
        auto func = m_taskFors.front();
        m_taskFors.pop();
        lck.unlock();
        //run
        func();
        //finish
        std::lock_guard<std::mutex> lck2(m_mtx2);
        m_isFinish = true;
        //if (m_mtx == nullptr)
        //    continue;
        ////take
        //std::unique_lock<std::mutex> lck(*m_mtx);  
        //m_con->wait(lck, [&]{return m_tasks && !m_tasks->empty();});
        //auto func = m_tasks->front();
        //m_tasks->pop();
        //lck.unlock();
        ////run
        //func();
        ////finish
        //std::lock_guard<std::mutex> lck2(m_mtx2);
        //if(m_toFinish){
        //    m_isFinish = true;
        //    break;
        //}
    }
    
}
int Thread::getIdInner(){
    return m_id;
}
void Thread::finish()
{
    {
        std::lock_guard<std::mutex> lck(m_mtx2);
        m_toFinish = true;
    }
    while(true){
        std::lock_guard<std::mutex> lck2(m_mtx2);
        if (m_isFinish)
            break;
        _sleep(1);
    }

}
void Thread::run(TaskFor _task, uint64_t _s, uint64_t _e)
{
    {
        std::lock_guard<std::mutex> lck2(m_mtx2);
        m_isFinish = false;
    }
    _task(_s, _e);
    {
        std::lock_guard<std::mutex> lck2(m_mtx2);
        m_isFinish = true;
    }
}
void Thread::runWithId(TaskForWithId _task, uint64_t _s, uint64_t _e, uint32_t _id)
{
    {
        std::lock_guard<std::mutex> lck2(m_mtx2);
        m_isFinish = false;
    }
    _task(_s, _e, _id);
    {
        std::lock_guard<std::mutex> lck2(m_mtx2);
        m_isFinish = true;
    }
}
void Thread::addTask(Task _task)
{
    std::lock_guard<std::mutex> lck(m_mtxFor);
    m_taskFors.push(_task);
}



ThreadPool::ThreadPool(const int _threadNum)
{
    m_maxThreadNum = std::thread::hardware_concurrency();
    if(_threadNum <= 0 || _threadNum > m_maxThreadNum - 3)
        m_threadNum = m_maxThreadNum-3;
    else
        m_threadNum = _threadNum;
#ifdef SYS_DEBUG
    std::cout<<"create thread pool, size: " << m_threadNum << std::endl;
#endif
    m_thds.resize(m_threadNum, nullptr);
    for(int i=0;i<m_threadNum;++i){
        m_thds[i] = new Thread(&m_tasks, &m_con, &m_mtx, i);
        m_thds[i]->detach();
    }
}
ThreadPool::~ThreadPool()
{
    for(int i=0;i<m_threadNum;++i)
    {
        delete m_thds[i];
#ifdef SYS_DEBUG
        std::cout << "delete thread:" << i << std::endl;
#endif // SYS_DEBUG

    }
}
// void ThreadPool::start()
// {
// }
void ThreadPool::addTask(Task _task)
{
    std::lock_guard<std::mutex> lck(m_mtx);
    m_tasks.push(_task);
    m_con.notify_all();
}
void ThreadPool::finish()
{
    while (true) {
        std::lock_guard<std::mutex> lck(m_mtx);
        if (m_tasks.empty()) {
            break;
        }
        _sleep(1);
    }
    for(int i=0;i<m_threadNum;++i)
    {
        m_thds[i]->finish();
#ifdef SYS_DEBUG
        std::cout<<"finishing thread: " <<i << std::endl;
#endif
    }
}
int ThreadPool::maxThreadNum()
{
    return m_maxThreadNum;
}
int ThreadPool::currentThreadNum()
{
    return m_threadNum;
}

void ThreadPool::process(TaskFor _task, uint64_t _s, uint64_t _e)
{
	if (m_threadNum <= 1)
	{
		_task(_s, _e);
		return;
	}
	uint64_t total = _e - _s;
	uint64_t block = total / m_threadNum;
	uint64_t remain = total % m_threadNum;
	uint64_t start = _s;
	uint64_t end = start + block;
	for (int i = 0; i < m_threadNum; ++i)
	{
		if (i == m_threadNum - 1)
			end += remain;
		//auto func = [_task, start, end]() {
		//	_task(start, end);
		//	};
		//addTask(func);
        m_thds[i]->run(_task, start, end);
		start = end;
		end = start + block;
	}
	finish();
}

void ThreadPool::processWithId(TaskForWithId _task, uint64_t _s, uint64_t _e)
{
    if (m_threadNum <= 1)
    {
        _task(_s, _e, 0);
        return;
    }
    uint64_t total = _e - _s;
    uint64_t block = total / m_threadNum;
    uint64_t remain = total % m_threadNum;
    uint64_t start = _s;
    uint64_t end = start + block;
    for (int i = 0; i < m_threadNum; ++i)
    {
        if (i == m_threadNum - 1)
            end += remain;
        auto func = [_task, start, end, i]() {
        	_task(start, end, i);
        	};
        //addTask(func);
        //m_thds[i]->runWithId(_task, start, end, i);
        m_thds[i]->addTask(func);
        start = end;
        end = start + block;
    }
    finish();
}

NSP_SLAM_LYJ_MATH_END