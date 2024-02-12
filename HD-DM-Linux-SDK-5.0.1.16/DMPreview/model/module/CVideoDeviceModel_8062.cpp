#include "CVideoDeviceModel_8062.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

CVideoDeviceModel_8062::CVideoDeviceModel_8062(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel_8053_8059(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_8062::AdjustZDTableIndex(int &nIndex)
{
    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH)){
        return APC_OK;
    }

    std::vector<APC_STREAM_INFO> depthStreamInfo = GetStreamInfoList(STREAM_DEPTH);
    int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);

    if (USB_PORT_TYPE_2_0 == m_usbPortType &&
        360 == depthStreamInfo[nDepthIndex].nHeight){
        nIndex = 0;
    }

    return APC_OK;
}

int CVideoDeviceModel_8062::GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType)
{
    if (USB_PORT_TYPE_2_0 == GetUsbType()){
        nRectifyLogIndex = 0;
    }

    return CVideoDeviceModel::GetRectifyLogData(nDevIndex, nRectifyLogIndex, pRectifyLogData, depthType);
}

int CVideoDeviceModel_8062::ConfigIMU()
{
    if (!m_pVideoDeviceController || !m_pVideoDeviceController->GetIMUDataController()) return APC_NullPtr;

    m_pVideoDeviceController->GetIMUDataController()->SetMaxG(8.0);
    m_pVideoDeviceController->GetIMUDataController()->SetMaxDPS(2000.0f);

    return APC_OK;
}
