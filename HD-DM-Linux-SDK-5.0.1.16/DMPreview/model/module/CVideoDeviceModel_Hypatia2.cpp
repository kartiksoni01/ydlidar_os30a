#include "CVideoDeviceModel_Hypatia2.h"
#include "CVideoDeviceController.h"

CVideoDeviceModel_Hypatia2::CVideoDeviceModel_Hypatia2(DEVSELINFO *pDeviceSelfInfo) : CVideoDeviceModel(pDeviceSelfInfo)
{
}
int CVideoDeviceModel_Hypatia2::TransformDepthDataType(int nDepthDataType, bool bRectifyData)
{
    int depthDataType = CVideoDeviceModel::TransformDepthDataType(nDepthDataType, bRectifyData);

    std::vector<APC_STREAM_INFO> depthStreamInfo = GetStreamInfoList(STREAM_DEPTH);

    if (!depthStreamInfo.empty())
    {
        int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);
        if (640 == depthStreamInfo[nDepthIndex].nWidth && 460 == depthStreamInfo[nDepthIndex].nHeight)
        {
            depthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
        }
    }

    return depthDataType;
}