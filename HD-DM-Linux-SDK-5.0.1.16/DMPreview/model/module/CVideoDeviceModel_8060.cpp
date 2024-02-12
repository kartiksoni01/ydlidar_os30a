#include "CVideoDeviceModel_8060.h"
#include "CEYSDDeviceManager.h"
#include "CVideoDeviceController.h"

CVideoDeviceModel_8060::CVideoDeviceModel_8060(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo),
CVideoDeviceModel_Kolor(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_8060::InitDeviceSelInfo()
{
    CVideoDeviceModel_Kolor::InitDeviceSelInfo();

    if(m_deviceSelInfo.empty()) return APC_NullPtr;

    DEVSELINFO *pDevSelfInfo = new DEVSELINFO;
    pDevSelfInfo->index = m_deviceSelInfo[0]->index + 2;
    m_deviceSelInfo.push_back(std::move(pDevSelfInfo));

    return APC_OK;
}

int CVideoDeviceModel_8060::InitDeviceInformation()
{
    CVideoDeviceModel_Kolor::InitDeviceInformation();

    m_deviceInfo.push_back(GetDeviceInformation(m_deviceSelInfo[2]));
    return APC_OK;
}

bool CVideoDeviceModel_8060::IsStreamSupport(STREAM_TYPE type)
{
    switch (type){
        case STREAM_TRACK:
            return true;
        default:
            return CVideoDeviceModel_Kolor::IsStreamSupport(type);
    }
}

bool CVideoDeviceModel_8060::IsStreamAvailable()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);
    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    bool bTrackStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_TRACK);

    if(bColorStream && bKolorStream) return false;

    return bColorStream || bDepthStream || bKolorStream || bTrackStream;
}

int CVideoDeviceModel_8060::GetIRRange(unsigned short &nMin, unsigned short &nMax)
{
    int ret = CVideoDeviceModel_Kolor::GetIRRange(nMin, nMax);
    nMax = 5;
    return ret;
}

int CVideoDeviceModel_8060::AdjustZDTableIndex(int &nIndex)
{
    nIndex = 0;
    return APC_OK;
}

int CVideoDeviceModel_8060::InitStreamInfoList()
{
    m_streamInfo[STREAM_TRACK].resize(MAX_STREAM_INFO_COUNT, {0, 0, false});
    int ret;
    RETRY_APC_API(ret, APC_GetDeviceResolutionList(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                         m_deviceSelInfo[2],
                                                         MAX_STREAM_INFO_COUNT, &m_streamInfo[STREAM_TRACK][0],
                                                         MAX_STREAM_INFO_COUNT, nullptr));

    if (APC_OK != ret) return ret;

    auto it = m_streamInfo[STREAM_TRACK].begin();
    for ( ; it != m_streamInfo[STREAM_TRACK].end() ; ++it){
        if (0 == (*it).nWidth){
            break;
        }
    }
    m_streamInfo[STREAM_TRACK].erase(it, m_streamInfo[STREAM_TRACK].end());
    m_streamInfo[STREAM_TRACK].shrink_to_fit();

    return CVideoDeviceModel_Kolor::InitStreamInfoList();
}

int CVideoDeviceModel_8060::AddCameraPropertyModels()
{
    CVideoDeviceModel_Kolor::AddCameraPropertyModels();
    m_cameraPropertyModel.push_back(new CCameraPropertyModel("Track", this, m_deviceSelInfo[2]));
    return APC_OK;
}

int CVideoDeviceModel_8060::Reset()
{
    if(m_pIMUModel){
        delete m_pIMUModel;
        m_pIMUModel = nullptr;
    }

    return CVideoDeviceModel_Kolor::Reset();
}

bool CVideoDeviceModel_8060::AudioSupport()
{
#if defined(UAC_X86_SUPPORTED) || defined(UAC_ARM_64_SUPPORTED)
    return true;
#else
    return false;
#endif
}

int CVideoDeviceModel_8060::PrepareOpenDevice()
{
    bool bTrackStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_TRACK);
    if(bTrackStream){
        APC_SetDepthDataType(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                 m_deviceSelInfo[2],
                                 APC_DEPTH_DATA_11_BITS);
    }

    CVideoDeviceModel_Kolor::PrepareOpenDevice();

    auto PrepareImageData = [&](STREAM_TYPE type){
        bool bStreamEnable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(type);
        if(bStreamEnable){
            std::vector<APC_STREAM_INFO> streamInfo = GetStreamInfoList(type);
            int index = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(type);
            m_imageData[type].nWidth = streamInfo[index].nWidth;
            m_imageData[type].nHeight  = streamInfo[index].nHeight;
            m_imageData[type].bMJPG = streamInfo[index].bFormatMJPG;
            m_imageData[type].depthDataType = GetDepthDataType();
            m_imageData[type].imageDataType = m_imageData[STREAM_KOLOR].bMJPG ?
                                                      APCImageType::COLOR_MJPG :
                                                      APCImageType::COLOR_YUY2;

            unsigned short nBytePerPixel = 2;
            unsigned int nBufferSize = m_imageData[type].nWidth * m_imageData[STREAM_KOLOR].nHeight * nBytePerPixel;
            if (m_imageData[type].imageBuffer.size() != nBufferSize){
                m_imageData[type].imageBuffer.resize(nBufferSize);
            }
            memset(&m_imageData[type].imageBuffer[0], 0, sizeof(nBufferSize));
        }
    };

    PrepareImageData(STREAM_KOLOR);
    PrepareImageData(STREAM_TRACK);

    return APC_OK;
}

int CVideoDeviceModel_8060::OpenDevice()
{
    if (APC_OK != CVideoDeviceModel_Kolor::OpenDevice()){
        return APC_OPEN_DEVICE_FAIL;
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

    bool bTrackStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_TRACK);
    if(bTrackStream)
    {
        int nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_TRACK);
        if(APC_OK != APC_OpenDevice2(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[2],
                                             m_imageData[STREAM_TRACK].nWidth, m_imageData[STREAM_TRACK].nHeight, m_imageData[STREAM_TRACK].bMJPG,
                                             0, 0,
                                             DEPTH_IMG_NON_TRANSFER,
                                             true, nullptr,
                                             &nFPS,
                                             IMAGE_SN_SYNC)){
            return APC_OPEN_DEVICE_FAIL;
        }

        m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_TRACK, nFPS);
    }

    return APC_OK;
}

int CVideoDeviceModel_8060::StartStreamingTask()
{
    CVideoDeviceModel_Kolor::StartStreamingTask();

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if (bKolorStream){
        CreateStreamTask(STREAM_KOLOR);
    }

    bool bTrackStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_TRACK);
    if (bTrackStream){
        CreateStreamTask(STREAM_TRACK);
    }

    return APC_OK;
}

int CVideoDeviceModel_8060::CloseDevice()
{
    int ret = CVideoDeviceModel_Kolor::CloseDevice();

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if(bKolorStream){
        if(APC_OK == APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[1])){
            ret = APC_OK;
        }
    }

    bool bTrackStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_TRACK);
    if(bTrackStream){
        if(APC_OK == APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[2])){
            ret = APC_OK;
        }
    }

    return ret;
}

int CVideoDeviceModel_8060::ClosePreviewView()
{
    CVideoDeviceModel_Kolor::ClosePreviewView();

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if (bKolorStream){
        m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_KOLOR);
    }

    bool bTrackStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_TRACK);
    if (bTrackStream){
        m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_TRACK);
    }

    return APC_OK;
}

int CVideoDeviceModel_8060::GetImage(STREAM_TYPE type)
{
    int ret;
    switch (type){
        case STREAM_KOLOR:
        case STREAM_TRACK:
            ret = GetColorImage(type);
            break;
        default:
            return CVideoDeviceModel_Kolor::GetImage(type);
    }

    return ret;
}

int CVideoDeviceModel_8060::GetColorImage(STREAM_TYPE type)
{
    DEVSELINFO *deviceSelInfo =  STREAM_KOLOR == type ? m_deviceSelInfo[1] :
                                 STREAM_TRACK == type ? m_deviceSelInfo[2] :
                                                        m_deviceSelInfo[0];

    unsigned long int nImageSize = 0;
    int nSerial = EOF;
    int ret = APC_GetColorImage(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                    deviceSelInfo,
                                    &m_imageData[type].imageBuffer[0],
                                    &nImageSize,
                                    &nSerial,
                                    0);

    if (APC_OK != ret) return ret;

    return ProcessImage(type, nImageSize, nSerial);
}
