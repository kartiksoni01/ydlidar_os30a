#include "CVideoDeviceModel_8051.h"
#include "CVideoDeviceController.h"

CVideoDeviceModel_8051::CVideoDeviceModel_8051(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_8051::AdjustZDTableIndex(int &nIndex)
{
    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH)){
        return APC_OK;
    }

    std::vector<APC_STREAM_INFO> depthStreamInfo = GetStreamInfoList(STREAM_DEPTH);
    int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);

    if (360 == depthStreamInfo[nDepthIndex].nHeight){
        nIndex = 0;
    }

    return APC_OK;
}

int CVideoDeviceModel_8051::GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType)
{
    if (USB_PORT_TYPE_2_0 == GetUsbType()){
        nRectifyLogIndex = 0;
    }

    return CVideoDeviceModel::GetRectifyLogData(nDevIndex, nRectifyLogIndex, pRectifyLogData, depthType);
}
