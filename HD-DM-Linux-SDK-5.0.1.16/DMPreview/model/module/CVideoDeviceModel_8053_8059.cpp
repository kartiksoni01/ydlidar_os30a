#include "CVideoDeviceModel_8053_8059.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"
#include "CThreadWorkerManage.h"
#include "RegisterSettings.h"
#include "eSPDI.h"

CVideoDeviceModel_8053_8059::CVideoDeviceModel_8053_8059(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo),
m_nLastInterLeaveColorSerial(0),
m_nLastInterLeaveDepthSerial(0)
{

}

bool CVideoDeviceModel_8053_8059::ModuleSyncSupport()
{
    if(USB_PORT_TYPE_3_0 != GetUsbType()) return false;
    if(IsInterleaveMode()) return false;
    return true;
}

int CVideoDeviceModel_8053_8059::ModuleSync()
{
    if(STREAMING != GetState()) return APC_OK;

    int ret = APC_OK;
    if(m_pVideoDeviceController->GetPreviewOptions()->IsModuleSync()){
        ret = RegisterSettings::FrameSync8053_8059_Clock(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                         m_deviceSelInfo[0]);
    }
    return ret;
}

int CVideoDeviceModel_8053_8059::ModuleSyncReset()
{
    int ret = APC_OK;
    if(m_pVideoDeviceController->GetPreviewOptions()->IsModuleSyncMaster()){
        static QMutex mutex;
        QMutexLocker locker(&mutex);
        ret = RegisterSettings::FrameSync8053_8059_Reset(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                         m_deviceSelInfo[0]);
    }
    return ret;
}

int CVideoDeviceModel_8053_8059::PrepareOpenDevice()
{
    SetSerialNumberType(m_pVideoDeviceController->GetPreviewOptions()->IsModuleSync() ? SERIAL_COUNT : FRAME_COUNT);

    return CVideoDeviceModel::PrepareOpenDevice();
}

int CVideoDeviceModel_8053_8059::StartStreamingTask()
{
    int ret = CVideoDeviceModel::StartStreamingTask();

    if(m_pVideoDeviceController->GetPreviewOptions()->IsModuleSync()){
        if(APC_OK != ModuleSync()){
            CTaskInfo *pInfo = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::VIDEO_MODULE_SYNC, this);
            CThreadWorkerManage::GetInstance()->AddTask(pInfo);
        }
    }

    return ret;
}

int CVideoDeviceModel_8053_8059::ProcessImageCallback(STREAM_TYPE streamType,
                                                      int nImageSize, int nSerialNumber)
{
    if(STREAM_COLOR == streamType && m_pVideoDeviceController->GetPreviewOptions()->IsModuleSyncMaster()){
        if(nSerialNumber > m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(streamType) * 30){
            ModuleSyncReset();
        }
    }

    return CVideoDeviceModel::ProcessImageCallback(streamType, nImageSize, nSerialNumber);
}
