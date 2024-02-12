#include "CVideoDeviceModel_8040.h"
#include "CVideoDeviceController.h"
#include "eSPDI.h"
#include "CEYSDDeviceManager.h"
#include "CIMUModel.h"
#include <cmath>

CVideoDeviceModel_8040::CVideoDeviceModel_8040(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo),
CVideoDeviceModel_8040_8054(pDeviceSelfInfo)
{

}

CVideoDeviceModel_8040::~CVideoDeviceModel_8040()
{

}

int CVideoDeviceModel_8040::Reset()
{
    if(m_pIMUModel){
        delete m_pIMUModel;
        m_pIMUModel = nullptr;
    }

    return CVideoDeviceModel_8040_8054::Reset();
}

int CVideoDeviceModel_8040::GetIRRange(unsigned short &nMin, unsigned short &nMax)
{
    int ret = CVideoDeviceModel_8040_8054::GetIRRange(nMin, nMax);
    if(APC_OK == ret){
        nMax = 4;
    }
    return ret;
}

int CVideoDeviceModel_8040::AdjustZDTableIndex(int &nIndex)
{
    if(m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH)){
        APC_STREAM_INFO depthStreamInfo = GetStreamInfoList(STREAM_DEPTH)[m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH)];
        if (912 == depthStreamInfo.nWidth || 912 == depthStreamInfo.nHeight ||
            456 == depthStreamInfo.nWidth || 456 == depthStreamInfo.nHeight){
            nIndex = 1;
        }
    }else{
        return CVideoDeviceModel_8040_8054::AdjustZDTableIndex(nIndex);
    }

    return APC_OK;
}

int CVideoDeviceModel_8040::GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType)
{
    if (0 == nDevIndex){
        nRectifyLogIndex = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR) ? 1 : 0;
    }else if (1 == nDevIndex &&
              m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR)){

        APC_STREAM_INFO kolorStreamInfo = GetStreamInfoList(STREAM_KOLOR)[m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_KOLOR)];
        if(2560 == kolorStreamInfo.nWidth && 1216 == kolorStreamInfo.nHeight) nRectifyLogIndex = 0;
        if(1920 == kolorStreamInfo.nWidth && 912 == kolorStreamInfo.nHeight) nRectifyLogIndex = 0;
        if(3840 == kolorStreamInfo.nWidth && 1824 == kolorStreamInfo.nHeight) nRectifyLogIndex = 0;
    }

    int ret = CVideoDeviceModel_8040_8054::GetRectifyLogData(nDevIndex, nRectifyLogIndex, pRectifyLogData, depthType);
    if(APC_OK == ret && !m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR)){
        pRectifyLogData->ReProjectMat[11] = (pRectifyLogData->OutImgHeight / 2) / tan(72/2 * 3.14159265359f / 180);
    }

    return ret;
}

int CVideoDeviceModel_8040::GetPointCloudInfo(eSPCtrl_RectLogData *pRectifyLogData, PointCloudInfo &pointCloudInfo,
                                              ImageData colorImageData, ImageData depthImageData)
{
    if (2160 == colorImageData.nWidth && 1920 == colorImageData.nHeight){
        colorImageData.nWidth = 1920;
        colorImageData.nHeight = 1080;
    }else{
        colorImageData.nWidth = 1920;
        colorImageData.nHeight = 912;
    }

    return CVideoDeviceModel_8040_8054::GetPointCloudInfo(pRectifyLogData, pointCloudInfo,
                                                          colorImageData, depthImageData);
}

std::vector<CloudPoint> CVideoDeviceModel_8040::GeneratePointCloud(std::vector<unsigned char> &depthData,
                                                                   std::vector<unsigned char> &colorData,
                                                                   unsigned short nDepthWidth,
                                                                   unsigned short nDepthHeight,
                                                                   unsigned short nColorWidth,
                                                                   unsigned short nColorHeight,
                                                                   eSPCtrl_RectLogData rectifyLogData,
                                                                   APCImageType::Value depthImageType,
                                                                   int nZNear, int nZFar,
                                                                   bool bUsePlyFilter,
                                                                   std::vector<float> imgFloatBufOut)
{
    std::vector<CloudPoint> cloudPoints;
    if (m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR)){
        eSPCtrl_RectLogData rectifyLogDataSlave;
        int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);
        if(APC_OK != GetRectifyLogData(1, nDepthIndex, &rectifyLogDataSlave)){
            return cloudPoints;
        }

        PlyWriter::EYSDFrameTo3DCylinder(nDepthWidth, nDepthHeight,
                                          depthData,
                                          nColorWidth, nColorHeight,
                                          colorData,
                                          &rectifyLogData,
                                          &rectifyLogDataSlave,
                                          depthImageType,
                                          cloudPoints,
                                          true,
                                          nZNear, nZFar,
                                          true, 1.0f);
    }else{
        PlyWriter::EYSDFrameTo3DCylinder(nDepthWidth, nDepthHeight,
                                          depthData,
                                          nColorWidth, nColorHeight,
                                          colorData,
                                          &rectifyLogData,
                                          depthImageType,
                                          cloudPoints,
                                          true,
                                          nZNear, nZFar,
                                          true, 1.0f);
    }

    return cloudPoints;
}
