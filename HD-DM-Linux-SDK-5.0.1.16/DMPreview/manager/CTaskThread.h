#ifndef CTASKTHREAD_H
#define CTASKTHREAD_H

#include <QThread>
#include <CTaskInfoManager.h>

class CTaskThread : public QThread
{
public:
    CTaskThread(QObject *parent = Q_NULLPTR);
    virtual ~CTaskThread() = default;

    int AssignTask(CTaskInfo *pInfo);
    int RemoveTask();

    bool HasTask(){ return m_pTaskInfo != nullptr; }
protected:
    virtual void run();

    QString GetThreadName();
    QThread::Priority GetPriority();


    void DoStartStreaming();
    void DoStopStreaming();
    void DoGrabberVideoImage();
    void DoVideoSnapShot();
    void DoColdReset();
    void DoQualityRegisterSetting();
    void DoInterleaveMode();
    void DoModuleSync();
    void DoFrameSync();
    void DoRegisterPeriodicRead();
    void DoIMUCapture();
    void DoIMUCalibration();
    void DoAudioRecord();
    void DoDepthAccuracyCalculate();
    void DoDepthSpatialNoiseCalculate();
    void DoDepthTemporaNoiseCalculate();
    void DoImageDataTransform();

private:
    CTaskInfo *m_pTaskInfo;
};

#endif // CTASKTHREAD_H
