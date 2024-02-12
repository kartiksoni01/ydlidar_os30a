#include "CVideoDeviceModel_8037.h"
#include "CVideoDeviceController.h"

CVideoDeviceModel_8037::CVideoDeviceModel_8037(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_8037::AdjustZDTableIndex(int &nIndex)
{
    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH)){
        return APC_OK;
    }

    std::vector<APC_STREAM_INFO> streamInfo = GetStreamInfoList(STREAM_DEPTH);
    int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);

    if (720 == streamInfo[nDepthIndex].nHeight){
        nIndex = 0;
    }else if (480 <= streamInfo[nDepthIndex].nHeight){
        nIndex = 1;
    }

    return APC_OK;
}
