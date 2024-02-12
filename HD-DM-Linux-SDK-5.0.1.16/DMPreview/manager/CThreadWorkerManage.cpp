#include "CThreadWorkerManage.h"
#include "CTaskThread.h"
#include "CTaskInfoManager.h"
#include <QApplication>

CThreadWorkerManage::CThreadWorkerManage()
{
    for (int i = 0 ; i < 30 ; ++i){
        m_pThreadPool.push_back(new CTaskThread(nullptr));
    }

    setObjectName("WorkerManager");
    start();
}

CThreadWorkerManage::~CThreadWorkerManage()
{
    for (CTaskThread *taskThread : m_pThreadPool){
        delete taskThread;
    }

    quit();
}

void CThreadWorkerManage::AddTask(CTaskInfo *pInfo)
{
    if (!pInfo) return;
    QMutexLocker locker(&m_mutex);
    m_pTaskQueue.push_back(std::move(pInfo));
}

void CThreadWorkerManage::RemoveTask(CTaskInfo *pInfo)
{
    if (!pInfo) return;
    if (CTaskInfo::IDLE == pInfo->GetTaskState()) return;

    pInfo->SetState(CTaskInfo::FINISHED);
    if (QThread::currentThread() != pInfo->GetHandleThread()){
        pInfo->Stop();
    }else{
        QMutexLocker locker(&m_mutex);
        m_pRemoveQueue.push_back(pInfo);
    }

    QThread::msleep(100);
}

void CThreadWorkerManage::run()
{
    while (true){
        {
            QMutexLocker locker(&m_mutex);
            if (!m_pTaskQueue.empty()){
                CTaskInfo *pInfo = m_pTaskQueue.front();
                for (CTaskThread *taskThread : m_pThreadPool){
                    if (taskThread->isRunning() || taskThread->HasTask()) continue;
                    if (CTaskInfo::SUSPEND == pInfo->GetTaskState()){
                        taskThread->AssignTask(pInfo);
                    }
                    m_pTaskQueue.pop_front();
                    break;
                }
            }
        }


        {
            QMutexLocker locker(&m_mutex);
            if (!m_pRemoveQueue.empty()){
                CTaskInfo *pInfo = m_pRemoveQueue.front();
                pInfo->Stop();
                m_pRemoveQueue.pop_front();
            }
        }

        QThread::msleep(100);
    }
}
