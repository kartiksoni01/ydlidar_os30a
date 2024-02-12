#ifndef CTASKINFOMANAGER_H
#define CTASKINFOMANAGER_H

#include <vector>

#define MAX_WAIT_TASK_MS (20 * 1000)
class CTaskThread;
class CTaskInfo{

public:
    enum TYPE{
        START_STREAMING,
        STOP_STREAMING,
        GRABBER_VIDEO_IMAGE_COLOR,
        GRABBER_VIDEO_IMAGE_COLOR_SLAVE,
        GRABBER_VIDEO_IMAGE_DEPTH,
        GRABBER_VIDEO_IMAGE_DEPTH_60mm,
        GRABBER_VIDEO_IMAGE_DEPTH_150mm,
        GRABBER_VIDEO_IMAGE_COLOR_WITH_DEPTH,
        GRABBER_VIDEO_IMAGE_KOLOR,
        GRABBER_VIDEO_IMAGE_KOLOR_SLAVE,
        GRABBER_VIDEO_IMAGE_TRACK,
        VIDEO_SNAP_SHOT,
        VIDEO_COLD_RESET,
        VIDEO_QUALITY_REGISTER_SETTING,
        VIDEO_INTERLEAVE_MODE,
        VIDEO_MODULE_SYNC,
        VIDEO_FRAME_SYNC,
        REGISTER_PERIODIC_READ,
        IMU_CAPTURE_DATA,
        IMU_CALIBRATION,
        AUDIO_RECORD,
        DEPTH_ACCURACY_CALCULATE,
        DEPTH_SPATIAL_NOISE_CALCULATE,
        DEPTH_TEMPORA_NOISE_CALCULATE,
        IMAGE_DATA_RAW_TO_RGB_TRANSFORM,
        //+[Thermal device]
        GRABBER_VIDEO_IMAGE_THERMAL,
        //-[Thermal device]
        NONE
    };
    enum STATE{
        RUNNING,
        SUSPEND,
        FINISHED,
        IDLE
    };

public:
   TYPE GetTaskType(){ return m_type; }
   void *GetParam(){ return m_pParam; }

   void SetState(STATE state){ m_state = state; }
   STATE GetTaskState(){ return m_state; }

   CTaskThread *GetHandleThread(){ return m_pHandleThread; }
   void SetHandleThread(CTaskThread *pThread){ m_pHandleThread = pThread; }

   void Pause();
   void Resume();
   void Stop();

   void WaitTaskFinished(unsigned int ms = MAX_WAIT_TASK_MS);

   friend class CTaskInfoManager;
private:
    void SetTaskType(TYPE type){ m_type = type; }
    void SetParam(void *pParam){ m_pParam = pParam; }

    CTaskInfo():
    m_type(NONE),
    m_state(IDLE),
    m_pParam(nullptr),
    m_pHandleThread(nullptr)
    {
    }
    ~CTaskInfo() = default;
private:
    TYPE m_type;
    STATE m_state;
    void *m_pParam;
    CTaskThread *m_pHandleThread;
};

class CTaskInfoManager
{
public:
    static CTaskInfoManager *GetInstance()
    {
        static CTaskInfoManager *pInstance = nullptr;
        if (nullptr == pInstance){
            pInstance = new CTaskInfoManager();
        }
        return pInstance;
    }

    CTaskInfo *RequestTaskInfo(CTaskInfo::TYPE type, void *m_pParam);

private:
    CTaskInfoManager();
    ~CTaskInfoManager();

    void RearrangePool();

private:
    unsigned short m_nCurrentIndex;
    std::vector<CTaskInfo *> m_taskPool;
};

#endif // CTASKINFOMANAGER_H
