#include "CVideoDeviceModel_8052.h"
#include "CVideoDeviceController.h"

CVideoDeviceModel_8052::CVideoDeviceModel_8052(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel_8036_8052(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_8052::TransformDepthDataType(int nDepthDataType, bool bRectifyData)
{
    int depthDataType = CVideoDeviceModel::TransformDepthDataType(nDepthDataType, bRectifyData);

    std::vector<APC_STREAM_INFO> depthStreamInfo = GetStreamInfoList(STREAM_DEPTH);

    if (!depthStreamInfo.empty()){
        int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);
        if (640 == depthStreamInfo[nDepthIndex].nWidth && 360 == depthStreamInfo[nDepthIndex].nHeight){
            depthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
        }
    }

    return depthDataType;

}