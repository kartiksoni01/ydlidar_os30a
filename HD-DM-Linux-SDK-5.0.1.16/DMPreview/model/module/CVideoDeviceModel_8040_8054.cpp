#include "CVideoDeviceModel_8040_8054.h"
#include "CEYSDDeviceManager.h"
#include "CVideoDeviceController.h"
#include "CThreadWorkerManage.h"
#include "eSPDI.h"
#include "utImageProcessingUtility.h"
#include "RegisterSettings.h"
#include "CImageDataModel.h"

CVideoDeviceModel_8040_8054::CVideoDeviceModel_8040_8054(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo),
CVideoDeviceModel_ColorWithDepth(pDeviceSelfInfo),
CVideoDeviceModel_Kolor(pDeviceSelfInfo)
{
}

bool CVideoDeviceModel_8040_8054::IsDepthDataTypeSupport(DEPTH_DATA_TYPE type)
{
    switch (type){
        case DEPTH_DATA_11BIT:
        case DEPTH_DATA_14BIT:
            return true;
        default:
            return false;
    }
}

int CVideoDeviceModel_8040_8054::GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType)
{
    int ret = CVideoDeviceModel::GetRectifyLogData(nDevIndex, nRectifyLogIndex, pRectifyLogData, depthType);
    if (APC_OK == ret &&
        0 == nDevIndex){
        if (2176 == pRectifyLogData->OutImgWidth &&
            1920 == pRectifyLogData->OutImgHeight){
            pRectifyLogData->OutImgWidth = 1080;
            pRectifyLogData->OutImgHeight = 1920;
        }

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
    }

    return ret;
}

int CVideoDeviceModel_8040_8054::InitDeviceInformation()
{
    SetPlumAR0330(false);
    CVideoDeviceModel_Kolor::InitDeviceInformation();
    if(APC_OK == SetPlumAR0330(true)){
        m_deviceInfo.push_back(GetDeviceInformation(m_deviceSelInfo[0]));
        SetPlumAR0330(false);
    }

    return APC_OK;
}

bool CVideoDeviceModel_8040_8054::IsStreamAvailable()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);
    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);

    if(bColorStream && bKolorStream) return false;

    return bColorStream || bDepthStream || bKolorStream;
}

int CVideoDeviceModel_8040_8054::PrepareOpenDevice()
{
    CVideoDeviceModel_ColorWithDepth::PrepareOpenDevice();

    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);
    if (bDepthStream){
        unsigned short nBytePerPixel = 2;
        unsigned int nBufferSize;
        if(bColorStream){
            nBufferSize = m_imageData[STREAM_COLOR].nWidth * m_imageData[STREAM_COLOR].nHeight * nBytePerPixel;
        }else{
            nBufferSize = m_imageData[STREAM_DEPTH].nWidth * m_imageData[STREAM_DEPTH].nHeight * nBytePerPixel;
        }
        if (m_rotateBuffer.size() != nBufferSize){
            m_rotateBuffer.resize(nBufferSize);
        }
        memset(&m_rotateBuffer[0], 0, sizeof(nBufferSize));

        if (m_mirroBuffer.size() != nBufferSize){
            m_mirroBuffer.resize(nBufferSize);
        }
        memset(&m_mirroBuffer[0], 0, sizeof(nBufferSize));
    }

    m_imageData[STREAM_KOLOR].nWidth = 0;
    m_imageData[STREAM_KOLOR].nHeight = 0;
    m_imageData[STREAM_KOLOR].bMJPG = false;

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if(bKolorStream){
        std::vector<APC_STREAM_INFO> streamInfo = GetStreamInfoList(STREAM_KOLOR);
        int index = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_KOLOR);
        m_imageData[STREAM_KOLOR].nWidth = streamInfo[index].nWidth;
        m_imageData[STREAM_KOLOR].nHeight  = streamInfo[index].nHeight;
        m_imageData[STREAM_KOLOR].bMJPG = streamInfo[index].bFormatMJPG;
        m_imageData[STREAM_KOLOR].depthDataType = GetDepthDataType();
        m_imageData[STREAM_KOLOR].imageDataType = m_imageData[STREAM_KOLOR].bMJPG ?
                                                  APCImageType::COLOR_MJPG :
                                                  APCImageType::COLOR_YUY2;

        unsigned short nBytePerPixel = 2;
        unsigned int nBufferSize = m_imageData[STREAM_KOLOR].nWidth * m_imageData[STREAM_KOLOR].nHeight * nBytePerPixel;
        if (m_imageData[STREAM_KOLOR].imageBuffer.size() != nBufferSize){
            m_imageData[STREAM_KOLOR].imageBuffer.resize(nBufferSize);
        }
        memset(&m_imageData[STREAM_KOLOR].imageBuffer[0], 0, sizeof(nBufferSize));

        APC_SetDepthDataType(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                 m_deviceSelInfo[1],
                                 GetKolorDepthDataTypeValue());
    }

    return APC_OK;
}

int CVideoDeviceModel_8040_8054::OpenDevice()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);

    if(bColorStream || bDepthStream){
        int nFPS;
        int nWidth, nHeight;
        bool bIsMJEPG;

        if(bColorStream){
            nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_COLOR);
            nWidth = m_imageData[STREAM_COLOR].nWidth;
            nHeight = m_imageData[STREAM_COLOR].nHeight;
            bIsMJEPG = m_imageData[STREAM_COLOR].bMJPG;
        }else{
            nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_DEPTH);
            nWidth = m_imageData[STREAM_DEPTH].nWidth;
            nHeight = m_imageData[STREAM_DEPTH].nHeight;
            bIsMJEPG = false;
        }

        if(APC_OK != APC_OpenDevice2(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[0],
                                             nWidth, nHeight, bIsMJEPG,
                                             0, 0,
                                             DEPTH_IMG_NON_TRANSFER,
                                             true, nullptr,
                                             &nFPS,
                                             IMAGE_SN_SYNC)){
            return APC_OPEN_DEVICE_FAIL;
        }

        if(bColorStream){
            m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_COLOR, nFPS);
            if(bDepthStream) m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_DEPTH, nFPS);
        }else{
            m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_DEPTH, nFPS);
        }
    }

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if(bKolorStream)
    {
        int nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_KOLOR);
        if(APC_OK != APC_OpenDevice2(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[1],
                                             m_imageData[STREAM_KOLOR].nWidth, m_imageData[STREAM_KOLOR].nHeight, m_imageData[STREAM_KOLOR].bMJPG,
                                             0, 0,
                                             DEPTH_IMG_NON_TRANSFER,
                                             true, nullptr,
                                             &nFPS,
                                             IMAGE_SN_SYNC)){
            return APC_OPEN_DEVICE_FAIL;
        }
        m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_KOLOR, nFPS);
    }

    return APC_OK;
}

int CVideoDeviceModel_8040_8054::StartStreamingTask()
{
    CVideoDeviceModel::StartStreamingTask();

    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);

    if(bColorStream && bDepthStream){
        m_imageData[STREAM_COLOR].nWidth /= 2;
    }

    if(bDepthStream){
        m_imageData[STREAM_COLOR].nWidth ^= m_imageData[STREAM_COLOR].nHeight;
        m_imageData[STREAM_COLOR].nHeight ^= m_imageData[STREAM_COLOR].nWidth;
        m_imageData[STREAM_COLOR].nWidth ^= m_imageData[STREAM_COLOR].nHeight;

        m_imageData[STREAM_DEPTH].nWidth ^= m_imageData[STREAM_DEPTH].nHeight;
        m_imageData[STREAM_DEPTH].nHeight ^= m_imageData[STREAM_DEPTH].nWidth;
        m_imageData[STREAM_DEPTH].nWidth ^= m_imageData[STREAM_DEPTH].nHeight;
    }

    if(bColorStream || bDepthStream){
        SetColdResetThresholdMs(STREAM_COLOR, FirstOpenDeviceColdeRestThresholdMs());
        SetLastestSuccessTime(STREAM_COLOR, QTime::currentTime());
    }

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if (bKolorStream){
        CreateStreamTask(STREAM_KOLOR);
    }

    return APC_OK;
}

int CVideoDeviceModel_8040_8054::GetPointCloudInfo(eSPCtrl_RectLogData *pRectifyLogData, PointCloudInfo &pointCloudInfo,
                                                   ImageData colorImageData, ImageData depthImageData)
{
    int ret = CVideoDeviceModel::GetPointCloudInfo(pRectifyLogData, pointCloudInfo,
                                                   colorImageData, depthImageData);

    bool IsKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if(APC_OK == ret && IsKolorStream){
        eSPCtrl_RectLogData rectifyLogDataSlave;
        GetRectifyLogData(1, m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH),
                          &rectifyLogDataSlave);

        float ratio_Mat_K = (float)colorImageData.nHeight / rectifyLogDataSlave.OutImgHeight;
        pointCloudInfo.focalLength_K = rectifyLogDataSlave.ReProjectMat[11] * ratio_Mat_K;
        pointCloudInfo.baseline_K = 1.0f / rectifyLogDataSlave.ReProjectMat[14];
        pointCloudInfo.diff_K = rectifyLogDataSlave.ReProjectMat[15] * ratio_Mat_K;
    }

    return ret;
}

int CVideoDeviceModel_8040_8054::CloseDevice()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);
    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);

    int ret = APC_NoDevice;
    if(bColorStream || bDepthStream){
        if(APC_OK == APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[0])){
            ret = APC_OK;
        }
    }

    if(bKolorStream){
        if(APC_OK == APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[1])){
            ret = APC_OK;
        }
    }

    return ret;
}

int CVideoDeviceModel_8040_8054::ClosePreviewView()
{
    CVideoDeviceModel::ClosePreviewView();

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if (bKolorStream){
        m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_KOLOR);
    }

    return APC_OK;
}

int CVideoDeviceModel_8040_8054::GetImage(STREAM_TYPE type)
{
    int ret;
    switch (type){
        case STREAM_COLOR:
        case STREAM_DEPTH:
            ret = GetColorImage();
            break;
        case STREAM_KOLOR:
            ret = GetKolorImage();
            break;
        default:
            return APC_NotSupport;
    }

    return ret;
}

int CVideoDeviceModel_8040_8054::GetColorImage()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);
    int nDepthDataType = bColorStream ? m_imageData[STREAM_COLOR].depthDataType :
                                        m_imageData[STREAM_DEPTH].depthDataType;

    std::vector<BYTE> &buffer = bColorStream && bDepthStream ? m_colorWithDepthImageData.imageBuffer :
                                bDepthStream ? m_rotateBuffer :
                                               m_imageData[STREAM_COLOR].imageBuffer;

    unsigned long int nImageSize = 0;
    int nSerial = EOF;

    int ret = APC_GetColorImage(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                    m_deviceSelInfo[0],
                                    &buffer[0],
                                    &nImageSize,
                                    &nSerial,
                                    nDepthDataType);

    if (APC_OK != ret) return ret;

    if(bColorStream && bDepthStream){
        unsigned short nHalfWidth = m_colorWithDepthImageData.nWidth / 2;
        unsigned short nRowBytes = nHalfWidth * 2;
        unsigned int nBytesPerImage = nRowBytes * m_colorWithDepthImageData.nHeight;
        for (int i = 0 ; i < m_colorWithDepthImageData.nHeight ; i++) {
            memcpy(&m_rotateBuffer[i * nRowBytes], &m_colorWithDepthImageData.imageBuffer[nRowBytes * ((2 * i))], nRowBytes);
            memcpy(&m_rotateBuffer[nBytesPerImage + i * nRowBytes], &m_colorWithDepthImageData.imageBuffer[nRowBytes * ((2 * i) + 1)], nRowBytes);
        }

        m_colorMutex.lock();
        if (APC_OK == APC_RotateImg90(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                              m_deviceSelInfo[0],
                                              m_imageData[STREAM_COLOR].imageDataType,
                                              nHalfWidth, m_colorWithDepthImageData.nHeight,
                                              &m_rotateBuffer[0],
                                              &m_imageData[STREAM_COLOR].imageBuffer[0],
                                              nBytesPerImage, true)){
            ProcessImage(STREAM_COLOR, nImageSize / 2, nSerial);
        }
        m_colorMutex.unlock();

        _depthMutex.lock();
        if (APC_OK == APC_RotateImg90(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                              m_deviceSelInfo[0],
                                              m_imageData[STREAM_DEPTH].imageDataType,
                                              nHalfWidth, m_colorWithDepthImageData.nHeight,
                                              &m_rotateBuffer[nBytesPerImage],
                                              &m_imageData[STREAM_DEPTH].imageBuffer[0],
                                              nBytesPerImage, true)){
            ProcessImage(STREAM_DEPTH, nImageSize / 2, nSerial);
        }
        _depthMutex.unlock();

        return APC_OK;
    }else if (bColorStream){
        return ProcessImage(STREAM_COLOR, nImageSize, nSerial);
    }else{
        if (APC_OK == APC_RotateImg90(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                              m_deviceSelInfo[0],
                                              m_imageData[STREAM_DEPTH].imageDataType,
                                              m_imageData[STREAM_DEPTH].nHeight, m_imageData[STREAM_DEPTH].nWidth,
                                              &m_rotateBuffer[0],
                                              &m_mirroBuffer[0],
                                              nImageSize, true)){
            if (APC_OK == APC_ImgMirro(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                               m_deviceSelInfo[0],
                                               m_imageData[STREAM_DEPTH].imageDataType,
                                               m_imageData[STREAM_DEPTH].nWidth, m_imageData[STREAM_DEPTH].nHeight,
                                               &m_mirroBuffer[0],
                                               &m_imageData[STREAM_DEPTH].imageBuffer[0]
                                               )){
                return ProcessImage(STREAM_DEPTH, nImageSize, nSerial);
            }

        }


        return APC_NullPtr;
    }
}

int CVideoDeviceModel_8040_8054::GetKolorImage()
{
    unsigned long int nImageSize = 0;
    int nSerial = EOF;
    int ret = APC_GetColorImage(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                    m_deviceSelInfo[1],
                                    &m_imageData[STREAM_KOLOR].imageBuffer[0],
                                    &nImageSize,
                                    &nSerial,
                                    m_imageData[STREAM_KOLOR].depthDataType);

    if (APC_OK != ret) return ret;

    return ProcessImage(STREAM_KOLOR, nImageSize, nSerial);
}

int CVideoDeviceModel_8040_8054::FirstSuccessGetImageCallback(STREAM_TYPE type)
{
    CVideoDeviceModel::FirstSuccessGetImageCallback(type);

    if(STREAM_DEPTH == type) return APC_NotSupport;

    CTaskInfo *pInfo = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::VIDEO_FRAME_SYNC, (CVideoDeviceModel *)this);
    CThreadWorkerManage::GetInstance()->AddTask(pInfo);

    return APC_OK;
}

int CVideoDeviceModel_8040_8054::FrameSync()
{
    STREAM_TYPE colorStreamType;
    if (m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR)){
        colorStreamType = STREAM_KOLOR;
    }else{
        colorStreamType = STREAM_COLOR;
    }
    APC_STREAM_INFO colorStreamInfo = GetStreamInfoList(colorStreamType)[m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(colorStreamType)];
    APC_STREAM_INFO depthStreamInfo = GetStreamInfoList(STREAM_DEPTH)[m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH)];

    return RegisterSettings::Framesync(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                       m_deviceSelInfo[0],
                                       m_deviceSelInfo[1],
                                       depthStreamInfo.nWidth, depthStreamInfo.nHeight,
                                       colorStreamInfo.nWidth, colorStreamInfo.nHeight,
                                       colorStreamInfo.bFormatMJPG,
                                       m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(colorStreamType),
                                       m_deviceInfo[0].deviceInfomation.wPID);
}

int CVideoDeviceModel_8040_8054::UpdateFrameGrabberData(STREAM_TYPE streamType)
{
    switch(streamType){
        case STREAM_COLOR:
        case STREAM_DEPTH:
            return CVideoDeviceModel_ColorWithDepth::UpdateFrameGrabberData(streamType);
        case STREAM_KOLOR:
            return CVideoDeviceModel_Kolor::UpdateFrameGrabberData(streamType);
        default: return APC_NotSupport;
    }
}

int CVideoDeviceModel_8040_8054::ProcessFrameGrabberCallback(std::vector<unsigned char>& bufDepth, int widthDepth, int heightDepth,
                                                             std::vector<unsigned char>& bufColor, int widthColor, int heightColor,
                                                             int serialNumber)
{
    bool bIsDpethOutput = PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_DEPTH == m_pVideoDeviceController->GetPreviewOptions()->GetPointCloudViewOutputFormat();
    bool bIsKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if (bIsDpethOutput && bIsKolorStream){
        PointCloudInfo bakPointCloudInfo = m_pointCloudInfo;
        m_pointCloudInfo.focalLength_K = 0;
        int ret = CVideoDeviceModel::ProcessFrameGrabberCallback(bufDepth, widthDepth, heightDepth,
                                                                 bufColor, widthColor, heightColor,
                                                                 serialNumber);
        m_pointCloudInfo = bakPointCloudInfo;
        return ret;
    }

    return CVideoDeviceModel::ProcessFrameGrabberCallback(bufDepth, widthDepth, heightDepth,
                                                          bufColor, widthColor, heightColor,
                                                          serialNumber);
}

