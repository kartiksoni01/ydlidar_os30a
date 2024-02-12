#include "CFrameSyncManager.h"

CFrameSyncManager::CFrameSyncManager()
{
}

CFrameSyncManager::~CFrameSyncManager()
{

}

int CFrameSyncManager::RegisterDataCallback(CVideoDeviceModel *pModel,
                                            CEYSDUIView *pControlView,
                                            CIMUDataController *pIMUDataController)
{
    std::lock_guard<std::mutex> locker(m_mutex);

    if (0 == m_mapSyncList.count(pModel)){
        m_mapSyncList[pModel] = std::make_shared<SyncList>();      
    }

    m_mapSyncList[pModel]->pControlView = pControlView;
    m_mapSyncList[pModel]->pIMUDataController = pIMUDataController;

    m_mapSyncList[pModel]->timer.moveToThread(QApplication::instance()->thread());
    QObject::connect(&m_mapSyncList[pModel]->timer, &QTimer::timeout, [=]()
    {
        AccomplishFrameCallback(pModel);
    });
    m_mapSyncList[pModel]->timer.start(5);

    return APC_OK;
}

int CFrameSyncManager::AccomplishFrameCallback(CVideoDeviceModel *pModel)
{
    std::lock_guard<std::mutex> accomplishLock(m_mapSyncList[pModel]->mutexAccomplish);

    if (m_mapSyncList[pModel]->vectorAccomplishFrame.empty()) return APC_NullPtr;

    m_mapSyncList[pModel]->pControlView->BeginFrameSync();

    for (SyncObject object : m_mapSyncList[pModel]->vectorAccomplishFrame){
        for (std::shared_ptr<ImageObject> imageObject : object.imageObject){
            if (imageObject.get()){
                m_mapSyncList[pModel]->pControlView->ImageCallback(imageObject->imageType, imageObject->streamType,
                                                                   &imageObject->imageBuffer[0], imageObject->nImageSize,
                                                                   imageObject->nWidth, imageObject->nHeight, imageObject->nSerialNumber,
                                                                   nullptr);
            }
        }

        IMUObject *pIMUObject = object.imuObject.get();
        if (pIMUObject){
            m_mapSyncList[pModel]->pIMUDataController->IMUCallback(&pIMUObject->imuData,
                                                                   pIMUObject->status);
        }
    }

    m_mapSyncList[pModel]->pControlView->EndFrameSync();

    m_mapSyncList[pModel]->vectorAccomplishFrame.clear();

    return APC_OK;
}

int CFrameSyncManager::UnregisterDataCallback(CVideoDeviceModel *pModel)
{
    if (0 == m_mapSyncList.count(pModel)) return APC_NullPtr;

    std::lock_guard<std::mutex> locker(m_mutex);
    m_mapSyncList.erase(pModel);

    return APC_OK;
}

int CFrameSyncManager::SyncImageCallback(CVideoDeviceModel *pModel,
                                         APCImageType::Value imageType, CVideoDeviceModel::STREAM_TYPE streamType,
                                         BYTE *pImageBuffer, int nImageSize,
                                         int nWidth, int nHeight, int nSerialNumber,
                                         void *pUserData)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    if (0 == m_mapSyncList.count(pModel)) return APC_NullPtr;

    if (nSerialNumber <= 0) {
        return m_mapSyncList[pModel]->pControlView->ImageCallback(imageType, streamType,
                                                                  pImageBuffer, nImageSize,
                                                                  nWidth, nHeight, nSerialNumber,
                                                                  pUserData);
    }
    std::shared_ptr<ImageObject> pNewImageObj(new ImageObject);

    pNewImageObj->imageType = imageType;
    pNewImageObj->streamType = streamType;
    pNewImageObj->imageBuffer.resize(nImageSize);
    memcpy(&pNewImageObj->imageBuffer[0], pImageBuffer, nImageSize);
    pNewImageObj->nImageSize = nImageSize;
    pNewImageObj->nWidth = nWidth;
    pNewImageObj->nHeight = nHeight;
    pNewImageObj->nSerialNumber = nSerialNumber;
    pNewImageObj->bHandled = false;
    std::lock_guard<std::mutex> objLocker(m_mapSyncList[pModel]->mutexObject);
    m_mapSyncList[pModel]->mapSyncObject[nSerialNumber].imageObject[streamType] = pNewImageObj;

    switch (streamType){
        case CVideoDeviceModel::STREAM_COLOR:
            m_mapSyncList[pModel]->syncConditionMask |= Condition_Color;
            m_mapSyncList[pModel]->mapSyncObject[nSerialNumber].syncMask |= Condition_Color;
            break;
        case CVideoDeviceModel::STREAM_DEPTH:
            m_mapSyncList[pModel]->syncConditionMask |= Condition_Depth;
            m_mapSyncList[pModel]->mapSyncObject[nSerialNumber].syncMask |= Condition_Depth;
            break;
        default: break;
    }

    DoFrameSync(pModel, nSerialNumber);

    return APC_OK;
}

int CFrameSyncManager::SyncIMUCallback(CVideoDeviceModel *pModel,
                                       IMUData *pData, int status)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    if (0 == m_mapSyncList.count(pModel)) return APC_NullPtr;
    if (pData->_frameCount <= 0) {
        return m_mapSyncList[pModel]->pIMUDataController->IMUCallback(pData, status);
    }

    std::shared_ptr<IMUObject> pNewIMUObj(new IMUObject);

    pNewIMUObj->imuData = *pData;
    pNewIMUObj->status = status;
    pNewIMUObj->bHandled = false;

    std::lock_guard<std::mutex> objLocker(m_mapSyncList[pModel]->mutexObject);
    m_mapSyncList[pModel]->mapSyncObject[pData->_frameCount].imuObject = pNewIMUObj;

    m_mapSyncList[pModel]->syncConditionMask |= Condition_IMU;
    m_mapSyncList[pModel]->mapSyncObject[pData->_frameCount].syncMask |= Condition_IMU;

    DoFrameSync(pModel, pData->_frameCount);

    return APC_OK;
}

int CFrameSyncManager::DoFrameSync(CVideoDeviceModel *pModel, FrameCount framCount)
{
    if (0 == m_mapSyncList.count(pModel)) return APC_NullPtr;
    if (framCount <= 0) return APC_NullPtr;

    if (m_mapSyncList[pModel]->syncConditionMask == m_mapSyncList[pModel]->mapSyncObject[framCount].syncMask){

        SyncObject syncObject;

        for (std::shared_ptr<ImageObject> imageObject : m_mapSyncList[pModel]->mapSyncObject[framCount].imageObject){
            if (imageObject.get() && !imageObject->bHandled){
                imageObject->bHandled = true;
                syncObject.imageObject[imageObject->streamType] = imageObject;
            }
        }

        std::shared_ptr<IMUObject> imuObject = m_mapSyncList[pModel]->mapSyncObject[framCount].imuObject;
        if (imuObject.get() && !imuObject->bHandled){
            imuObject->bHandled = true;
            syncObject.imuObject = imuObject;
        }

        for (auto it = m_mapSyncList[pModel]->setHistory.begin(); it != m_mapSyncList[pModel]->setHistory.end(); ) {
            if (framCount <= *it) break;
            m_mapSyncList[pModel]->mapSyncObject.erase(*it);
            it = m_mapSyncList[pModel]->setHistory.erase(it);
        }

        {
            std::lock_guard<std::mutex> accomplishLock(m_mapSyncList[pModel]->mutexAccomplish);
            m_mapSyncList[pModel]->vectorAccomplishFrame.push_back(std::move(syncObject));
        }
    }

    m_mapSyncList[pModel]->setHistory.emplace(framCount);

    return APC_OK;
}
