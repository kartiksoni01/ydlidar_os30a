#include "CTaskInfoManager.h"
#include <QThread>
#include <QMutex>
#include "CTaskThread.h"

void CTaskInfo::Pause()
{
    if (RUNNING != GetTaskState()) return;

    SetState(SUSPEND);
}

void CTaskInfo::Resume()
{
    if (SUSPEND != GetTaskState()) return;

    SetState(RUNNING);
}

void CTaskInfo::Stop()
{
    if (GetHandleThread()) {
        GetHandleThread()->wait();
        GetHandleThread()->RemoveTask();
    }
    SetHandleThread(nullptr);
    SetState(CTaskInfo::IDLE);
}

void CTaskInfo::WaitTaskFinished(unsigned int ms)
{
    const unsigned long nSleepMs = 100;
    int WaitTimes = ms / nSleepMs;
    while (IDLE != GetTaskState() && (0 == ms || WaitTimes-- > 0))
        QThread::msleep(nSleepMs);
}

CTaskInfoManager::CTaskInfoManager():
m_nCurrentIndex(0)
{
    RearrangePool();
}

CTaskInfoManager::~CTaskInfoManager()
{
    for (CTaskInfo *pInfo : m_taskPool){
        delete pInfo;
    }
}

void CTaskInfoManager::RearrangePool()
{
    for (int i = 0 ; i < 10 ; ++i){
        m_taskPool.push_back(new CTaskInfo());
    }
}

CTaskInfo *CTaskInfoManager::RequestTaskInfo(CTaskInfo::TYPE type, void *m_pParam)
{
    static QMutex mutex;
    QMutexLocker locker(&mutex);

    size_t nSize = m_taskPool.size();

    CTaskInfo *pIdleTask = nullptr;
    for (size_t i = 0 ; i < nSize ; ++i){
        if (CTaskInfo::IDLE == m_taskPool[m_nCurrentIndex % nSize]->GetTaskState()){
            pIdleTask = m_taskPool[m_nCurrentIndex % nSize];
            break;
        }
        m_nCurrentIndex = (m_nCurrentIndex + 1) % nSize;
    }

    if (!pIdleTask){
        m_nCurrentIndex = nSize;
        RearrangePool();
        pIdleTask = m_taskPool[m_nCurrentIndex];
    }

    pIdleTask->SetTaskType(type);
    pIdleTask->SetParam(m_pParam);
    pIdleTask->SetState(CTaskInfo::SUSPEND);

    return pIdleTask;
}
