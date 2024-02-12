#ifndef CFRAMESYNCMANAGER_H
#define CFRAMESYNCMANAGER_H

#include "CEYSDUIView.h"
#include "CIMUDataController.h"

#include <memory>
#include <vector>
#include <map>
#include <list>
#include <thread>
#include <set>
#include <condition_variable>

class CFrameSyncManager
{
public:
    typedef int (CEYSDUIView::*ImageCallback)(APCImageType::Value imageType, CVideoDeviceModel::STREAM_TYPE streamType,
                                               BYTE *pImageBuffer, int nImageSize,
                                               int nWidth, int nHeight, int nSerialNumber,
                                               void *pUserData);

    typedef int (CIMUDataController::*IMUCallback)(IMUData *pData, int status);

private:
    struct ImageObject{
        APCImageType::Value imageType;
        CVideoDeviceModel::STREAM_TYPE streamType;
        std::vector<BYTE> imageBuffer;
        int nImageSize;
        int nWidth;
        int nHeight;
        int nSerialNumber;
        bool bHandled;
    };

    struct IMUObject{
        IMUData imuData;
        int status;
        bool bHandled;
    };

    enum SyncCondition{
        Condition_Color = 0x1,
        Condition_Depth = 0x2,
        Condition_IMU   = 0x4
    };

    using SyncMask = unsigned int;
    using FrameCount = int;

    struct SyncObject{
        std::shared_ptr<ImageObject> imageObject[CVideoDeviceModel::STREAM_TYPE_COUNT];
        std::shared_ptr<IMUObject> imuObject;
        SyncMask syncMask;
    };

    struct SyncList{
        bool bRunning = false;

        CEYSDUIView *pControlView = nullptr;
        CIMUDataController *pIMUDataController = nullptr;

        std::mutex mutexObject;
        SyncMask syncConditionMask = 0;

        std::set<FrameCount> setHistory;
        std::map<FrameCount, SyncObject> mapSyncObject;

        QTimer timer;
        std::mutex mutexAccomplish;
        std::vector<SyncObject> vectorAccomplishFrame;
    };

public:
    static CFrameSyncManager *GetInstance(){
        static CFrameSyncManager *pInstance = nullptr;
        if (!pInstance){
            pInstance = new CFrameSyncManager();
        }

        return pInstance;
    }

public:
    int RegisterDataCallback(CVideoDeviceModel *pModel,
                             CEYSDUIView *pControlView,
                             CIMUDataController *pIMUDataController);

    int UnregisterDataCallback(CVideoDeviceModel *pModel);

    int SyncImageCallback(CVideoDeviceModel *pModel,
                          APCImageType::Value imageType, CVideoDeviceModel::STREAM_TYPE streamType,
                          BYTE *pImageBuffer, int nImageSize,
                          int nWidth, int nHeight, int nSerialNumber,
                          void *pUserData);

    int SyncIMUCallback(CVideoDeviceModel *pModel,
                        IMUData *pData, int status);

    int DoFrameSync(CVideoDeviceModel *pModel, FrameCount framCount);
    int AccomplishFrameCallback(CVideoDeviceModel *pModel);

private:
    CFrameSyncManager();
    ~CFrameSyncManager();

private:
    std::mutex m_mutex;
    std::map<CVideoDeviceModel *, std::shared_ptr<SyncList>> m_mapSyncList;
};

#endif // CFRAMESYNCMANAGER_H
