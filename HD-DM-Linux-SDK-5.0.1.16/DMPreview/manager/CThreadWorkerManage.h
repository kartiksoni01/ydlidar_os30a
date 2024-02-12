#ifndef CTHREADWORKERMANAGE_H
#define CTHREADWORKERMANAGE_H

#include <QThread>
#include <vector>
#include <deque>
#include <QMutex>

class CTaskThread;
class CTaskInfo;
class CThreadWorkerManage : public QThread
{
public:
    static CThreadWorkerManage *GetInstance()
    {
        static CThreadWorkerManage *pInstance = nullptr;
        if (nullptr == pInstance){
            pInstance = new CThreadWorkerManage();
        }
        return pInstance;
    }

    void AddTask(CTaskInfo *pInfo);
    void RemoveTask(CTaskInfo *pInfo);

protected:
    virtual void run();
private:
    CThreadWorkerManage();
    ~CThreadWorkerManage();

private:
    QMutex m_mutex;
    std::vector<CTaskThread *> m_pThreadPool;
    std::deque<CTaskInfo *> m_pTaskQueue;
    std::deque<CTaskInfo *> m_pRemoveQueue;
};

#endif // CTHREADWORKERMANAGE_H
