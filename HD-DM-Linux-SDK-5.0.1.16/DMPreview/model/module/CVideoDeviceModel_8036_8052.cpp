#include "CVideoDeviceModel_8036_8052.h"
#include "CVideoDeviceController.h"

CVideoDeviceModel_8036_8052::CVideoDeviceModel_8036_8052(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_8036_8052::AdjustZDTableIndex(int &nIndex)
{
    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH) ||
        !m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR)){
        return APC_OK;
    }

    std::vector<APC_STREAM_INFO> depthStreamInfo = GetStreamInfoList(STREAM_DEPTH);
    int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);

    std::vector<APC_STREAM_INFO> colorStreamInfo = GetStreamInfoList(STREAM_COLOR);
    int nColorIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_COLOR);

    if (colorStreamInfo[nColorIndex].nHeight / depthStreamInfo[nDepthIndex].nHeight == 2){
        nIndex = 0;
    }

    return APC_OK;
}

#if defined(_MIPI_CAMERA_)
#define BIG_RESOLUTION_BYTES (2560 * 720)
#else
#define BIG_RESOLUTION_BYTES (2000 * 720)
#endif
int CVideoDeviceModel_8036_8052::StartStreamingTask()
{
    int ret =  CVideoDeviceModel::StartStreamingTask();
    if (APC_OK != ret) return ret;

    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);

    if (bColorStream){
        if (m_imageData[STREAM_COLOR].nWidth * m_imageData[STREAM_COLOR].nHeight > BIG_RESOLUTION_BYTES){
            CreateStreamTask(STREAM_COLOR);
        }
    }

    if (bDepthStream){
        if (m_imageData[STREAM_DEPTH].nWidth * m_imageData[STREAM_DEPTH].nHeight > BIG_RESOLUTION_BYTES){
            CreateStreamTask(STREAM_DEPTH);
        }
    }

    return ret;
}
