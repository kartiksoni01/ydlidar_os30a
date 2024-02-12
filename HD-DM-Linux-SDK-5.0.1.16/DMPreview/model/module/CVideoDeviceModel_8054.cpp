#include "CVideoDeviceModel_8054.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"
#include "eSPDI.h"
#include "utImageProcessingUtility.h"

CVideoDeviceModel_8054::CVideoDeviceModel_8054(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo),
CVideoDeviceModel_8040_8054(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_8054::AdjustZDTableIndex(int &nIndex)
{
    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH)){
        return APC_OK;
    }

    std::vector<APC_STREAM_INFO> streamInfo = GetStreamInfoList(STREAM_DEPTH);
    int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);

    if (640 == streamInfo[nDepthIndex].nHeight){
        nIndex = 2;
    }

    return APC_OK;
}

int CVideoDeviceModel_8054::GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType)
{
    if(0 == nDevIndex){
        if (3 == nRectifyLogIndex && m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR)){
            nRectifyLogIndex = 2;
        }
    }else if (1 == nDevIndex){
        nRectifyLogIndex = (3 == nRectifyLogIndex) ? 1 : 0;
    }

    int ret = CVideoDeviceModel_8040_8054::GetRectifyLogData(nDevIndex, nRectifyLogIndex, pRectifyLogData, depthType);

    if(APC_OK == ret &&
       1 == nDevIndex){
        float fCenterX = -1.0f * pRectifyLogData->ReProjectMat[3];
        float fCenterY = -1.0f * pRectifyLogData->ReProjectMat[7];

        int nWidth = pRectifyLogData->OutImgWidth;
        int nHeight = pRectifyLogData->OutImgHeight;

        utImageProcessingUtility::Rotate2Dpoint(nWidth / 2, nHeight / 2,
                                                nHeight / 2, nWidth / 2,
                                                90.0,
                                                fCenterX, fCenterY);

        pRectifyLogData->ReProjectMat[3] = -1.0f * fCenterX;
        pRectifyLogData->ReProjectMat[7] = -1.0f * fCenterY;
        pRectifyLogData->OutImgWidth = nHeight;
        pRectifyLogData->OutImgHeight = nWidth;

        pRectifyLogData->ReProjectMat[11] = pRectifyLogData->NewCamMat2[0];
    }

    return ret;
}

std::vector<CloudPoint> CVideoDeviceModel_8054::GeneratePointCloud(std::vector<unsigned char> &depthData,
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
    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR)){
        return CVideoDeviceModel_ColorWithDepth::GeneratePointCloud(depthData, colorData,
                                                                    nDepthWidth, nDepthHeight,
                                                                    nColorWidth, nColorHeight,
                                                                    rectifyLogData, depthImageType,
                                                                    nZNear, nZFar,
                                                                    bUsePlyFilter,
                                                                    imgFloatBufOut);
    }

    eSPCtrl_RectLogData rectifyLogDataSlave;
    int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);

    if(APC_OK != GetRectifyLogData(1, nDepthIndex, &rectifyLogDataSlave)){
        return cloudPoints;
    }


    if (bUsePlyFilter && !imgFloatBufOut.empty()){
        PlyWriter::EYSDFrameTo3DMultiSensorPlyFilterFloat(nDepthWidth, nDepthHeight,
                                                           imgFloatBufOut,
                                                           nColorWidth, nColorHeight,
                                                           colorData,
                                                           &rectifyLogData,
                                                           &rectifyLogDataSlave,
                                                           depthImageType,
                                                           cloudPoints,
                                                           true,
                                                           nZNear, nZFar,
                                                           false, false,
                                                           1.0, 90);
    }else{
        PlyWriter::EYSDFrameTo3DMultiSensor(nDepthWidth, nDepthHeight,
                                             depthData,
                                             nColorWidth, nColorHeight,
                                             colorData,
                                             &rectifyLogData,
                                             &rectifyLogDataSlave,
                                             depthImageType,
                                             cloudPoints,
                                             true,
                                             nZNear, nZFar,
                                             false, false,
                                             1.0, 90);
    }

    return cloudPoints;
}
