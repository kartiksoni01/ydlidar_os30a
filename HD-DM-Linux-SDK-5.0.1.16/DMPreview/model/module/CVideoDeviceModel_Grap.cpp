#include "CVideoDeviceModel_Grap.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

//+[Thermal device]
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <sys/ioctl.h>
#include "dirent.h"
#include <iostream> 
#include <fstream> 

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include<sstream> 
//-[Thermal device]

CVideoDeviceModel_Grap::CVideoDeviceModel_Grap(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_Grap::InitDeviceSelInfo()
{
    CVideoDeviceModel::InitDeviceSelInfo();

    if(m_deviceSelInfo.empty()) return APC_NullPtr;    

    for (int i = 1 ; i <= 3 ; ++i){
        DEVSELINFO *pDevSelfInfo = new DEVSELINFO;
        pDevSelfInfo->index = m_deviceSelInfo[0]->index + i;
        m_deviceSelInfo.push_back(pDevSelfInfo);
    }

    return APC_OK;
}

int CVideoDeviceModel_Grap::InitDeviceInformation()
{
    SetPlumAR0330(false);
    CVideoDeviceModel::InitDeviceInformation();
    m_deviceInfo.push_back(GetDeviceInformation(m_deviceSelInfo[1]));
    if(APC_OK == SetPlumAR0330(true)){
        m_deviceInfo.push_back(GetDeviceInformation(m_deviceSelInfo[0]));
        SetPlumAR0330(false);
    }

    m_deviceInfo.push_back(GetDeviceInformation(m_deviceSelInfo[2]));
    m_deviceInfo.push_back(GetDeviceInformation(m_deviceSelInfo[3]));
    if(APC_OK == SetPlumAR0330(true)){
        m_deviceInfo.push_back(GetDeviceInformation(m_deviceSelInfo[2]));
        SetPlumAR0330(false);
    }

    return APC_OK;
}

bool CVideoDeviceModel_Grap::IsStreamSupport(STREAM_TYPE type)
{
    switch (type){
        case STREAM_COLOR:
        case STREAM_COLOR_SLAVE:
        case STREAM_KOLOR:
        case STREAM_KOLOR_SLAVE:
            return true;
        default:
            return false;
    }
}

int CVideoDeviceModel_Grap::InitStreamInfoList()
{
    auto AddStreamInfoList = [&](DEVSELINFO *pDevSelInfo, STREAM_TYPE type) -> int
    {
        m_streamInfo[type].resize(MAX_STREAM_INFO_COUNT, {0, 0, false});
        int ret;
        RETRY_APC_API(ret, APC_GetDeviceResolutionList(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                             pDevSelInfo,
                                                             MAX_STREAM_INFO_COUNT, &m_streamInfo[type][0],
                                                             MAX_STREAM_INFO_COUNT, nullptr));

        if (APC_OK != ret) return ret;

        auto it = m_streamInfo[type].begin();
        for ( ; it != m_streamInfo[type].end() ; ++it){
            if (0 == (*it).nWidth){
                break;
            }
        }
        m_streamInfo[type].erase(it, m_streamInfo[type].end());
        m_streamInfo[type].shrink_to_fit();

        return ret;
    };

    AddStreamInfoList(m_deviceSelInfo[0], STREAM_COLOR);    
    AddStreamInfoList(m_deviceSelInfo[2], STREAM_COLOR_SLAVE);
    AddStreamInfoList(m_deviceSelInfo[1], STREAM_KOLOR);
    AddStreamInfoList(m_deviceSelInfo[3], STREAM_KOLOR_SLAVE);

    return APC_OK;
}

bool CVideoDeviceModel_Grap::IsStreamAvailable()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);

    return bColorStream || bKolorStream;
}

int CVideoDeviceModel_Grap::PrepareOpenDevice()
{
    auto PrepareImageData = [&](STREAM_TYPE type){
        bool bStreamEnable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(type);
        if(bStreamEnable){
            std::vector<APC_STREAM_INFO> streamInfo = GetStreamInfoList(type);
            int index = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(type);
            m_imageData[type].nWidth = streamInfo[index].nWidth;
            m_imageData[type].nHeight  = streamInfo[index].nHeight;
            m_imageData[type].bMJPG = streamInfo[index].bFormatMJPG;
            m_imageData[type].depthDataType = GetDepthDataType();
            m_imageData[type].imageDataType = m_imageData[type].bMJPG ?
                                              APCImageType::COLOR_MJPG :
                                              APCImageType::COLOR_YUY2;

            unsigned short nBytePerPixel = 2;
            unsigned int nBufferSize = m_imageData[type].nWidth * m_imageData[type].nHeight * nBytePerPixel;
            if (m_imageData[type].imageBuffer.size() != nBufferSize){
                m_imageData[type].imageBuffer.resize(nBufferSize);
            }
            memset(&m_imageData[type].imageBuffer[0], 0, sizeof(nBufferSize));
        }
    };

    PrepareImageData(STREAM_COLOR);
    m_imageData[STREAM_COLOR_SLAVE] = m_imageData[STREAM_COLOR];

    PrepareImageData(STREAM_KOLOR);
    m_imageData[STREAM_KOLOR_SLAVE] = m_imageData[STREAM_KOLOR];

    //+[Thermal device]
    #if defined(THERMAL_SENSOR)
   isSupportThermal =APC_GetThermalFD(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                    &FD);
     if(isSupportThermal){
        //+set device resolution
        video_w = 360;
        video_h = 240;

        m_imageData[STREAM_THERMAL].nWidth = video_w;
        m_imageData[STREAM_THERMAL].nHeight  = video_h;
        m_imageData[STREAM_THERMAL].bMJPG = false;
        m_imageData[STREAM_THERMAL].imageDataType = APCImageType::COLOR_RGB24;

       unsigned short nBytePerPixel = 3;
       unsigned int nBufferSize = m_imageData[STREAM_THERMAL].nWidth * m_imageData[STREAM_THERMAL].nHeight * nBytePerPixel;
       if (m_imageData[STREAM_THERMAL].imageBuffer.size() != nBufferSize){
                m_imageData[STREAM_THERMAL].imageBuffer.resize(nBufferSize);
        }
        memset(&m_imageData[STREAM_THERMAL].imageBuffer[0], 0, sizeof(nBufferSize));
     }
     #endif
    //-[Thermal device]
    
    return APC_OK;
}

int CVideoDeviceModel_Grap::OpenDevice()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    if(bColorStream)
    {
        int nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_COLOR);
        if(APC_OK != APC_OpenDevice2(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[0],
                                             m_imageData[STREAM_COLOR].nWidth, m_imageData[STREAM_COLOR].nHeight, m_imageData[STREAM_COLOR].bMJPG,
                                             0, 0,
                                             DEPTH_IMG_NON_TRANSFER,
                                             true, nullptr,
                                             &nFPS,
                                             IMAGE_SN_SYNC)){
            return APC_OPEN_DEVICE_FAIL;
        }

        nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_COLOR);
        if(APC_OK != APC_OpenDevice2(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[2],
                                             m_imageData[STREAM_COLOR_SLAVE].nWidth, m_imageData[STREAM_COLOR_SLAVE].nHeight, m_imageData[STREAM_COLOR_SLAVE].bMJPG,
                                             0, 0,
                                             DEPTH_IMG_NON_TRANSFER,
                                             true, nullptr,
                                             &nFPS,
                                             IMAGE_SN_SYNC)){
            return APC_OPEN_DEVICE_FAIL;
        }

        m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_COLOR, nFPS);
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

        nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_KOLOR);
        if(APC_OK != APC_OpenDevice2(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[3],
                                             m_imageData[STREAM_KOLOR_SLAVE].nWidth, m_imageData[STREAM_KOLOR_SLAVE].nHeight, m_imageData[STREAM_KOLOR_SLAVE].bMJPG,
                                             0, 0,
                                             DEPTH_IMG_NON_TRANSFER,
                                             true, nullptr,
                                             &nFPS,
                                             IMAGE_SN_SYNC)){
            return APC_OPEN_DEVICE_FAIL;
        }
        m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_KOLOR, nFPS);
    }
    //+[Thermal device]
    //open thermal device
    #if defined(THERMAL_SENSOR)
    if(isSupportThermal) {
        CVideoDeviceModel_Grap::Open_thermal_camera();
     }
    #endif
    //-[Thermal device]    
    return APC_OK;
}

int CVideoDeviceModel_Grap::StartStreamingTask()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    if (bColorStream){
        CreateStreamTask(STREAM_COLOR);
        CreateStreamTask(STREAM_COLOR_SLAVE);
    }

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if (bKolorStream){
        CreateStreamTask(STREAM_KOLOR);
        CreateStreamTask(STREAM_KOLOR_SLAVE);
    }
    //-[Thermal device]
    #if defined(THERMAL_SENSOR)
    if(isSupportThermal) {
         CreateStreamTask(STREAM_THERMAL);
    }
    #endif
    //+[Thermal device]
    return APC_OK;
}

int CVideoDeviceModel_Grap::CloseDevice()
{
    int ret;

    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    if(bColorStream){
        if(APC_OK == APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[0])){
            ret = APC_OK;
        }

        if(APC_OK == APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[2])){
            ret = APC_OK;
        }
    }

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if(bKolorStream){
        if(APC_OK == APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[1])){
            ret = APC_OK;
        }

        if(APC_OK == APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[3])){
            ret = APC_OK;
        }
    }

    //+[Thermal device]
    #if defined(THERMAL_SENSOR)
    if(isSupportThermal) {
        free(pSurfaceTemper);
        free(pInternalTemper);

        device.StopVideo(FD);
        if(curve != NULL)
            free(curve);
        if(isFirstLoadCurve)
            guide_measure_deloadtempercurve();
     }
    #endif
    //-[Thermal device]
    return ret;
}

int CVideoDeviceModel_Grap::ClosePreviewView()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    if (bColorStream){
        m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_COLOR);
        m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_COLOR_SLAVE);
    }

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if (bKolorStream){
        m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_KOLOR);
        m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_KOLOR_SLAVE);
    }

    //+[Thermal device]
    #if defined(THERMAL_SENSOR)
    if(isSupportThermal) {
         m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_THERMAL);
     }
    #endif
    //-[Thermal device]
    return APC_OK;
}

int CVideoDeviceModel_Grap::GetImage(STREAM_TYPE type)
{
    int ret;
    switch (type){
        case STREAM_COLOR:
        case STREAM_COLOR_SLAVE:
        case STREAM_KOLOR:
        case STREAM_KOLOR_SLAVE:
            ret = GetColorImage(type);
            break;
        //+[Thermal device]
        case STREAM_THERMAL:
            #if defined(THERMAL_SENSOR)
            ret = GetThermalImage(type,video_w,device,curve,pParamExt);
            #endif
            break;
        //-[Thermal device]
        default:
            return APC_NotSupport;
    }

    return ret;
}

int CVideoDeviceModel_Grap::GetColorImage(STREAM_TYPE type)
{
    DEVSELINFO *deviceSelInfo;

    switch(type){
        case STREAM_COLOR: deviceSelInfo = m_deviceSelInfo[0]; break;
        case STREAM_COLOR_SLAVE: deviceSelInfo = m_deviceSelInfo[2]; break;
        case STREAM_KOLOR: deviceSelInfo = m_deviceSelInfo[1]; break;
        case STREAM_KOLOR_SLAVE: deviceSelInfo = m_deviceSelInfo[3]; break;
        default: return APC_NotSupport;
    }

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

//+[Thermal device]
#if defined(THERMAL_SENSOR)
int  CVideoDeviceModel_Grap::GetThermalImage(STREAM_TYPE type, int video_w,v4l2 device,short *curve,guide_measure_external_param_t *pParamExt)
{
    double centerTemp = 0.0;
    int ret =0;
    int try_count=0;
    
     while(try_count < 10)
    {
        ++try_count;
        ret = device.GetFrame(device.FD);
        if(device.queue_frame.size()>0)
        {
                device.distributeData((uchar*)device.queue_frame.dequeue().data(),video_w);
            
                if ( device.imgType == device.Y16_Param )
                {
                        device.convert_y16_to_rgb_buffer(device.Y16data, device.VIDEO_WIDTH, device.VIDEO_HEIGHT,device.rgb24);
                        int gear = guide_measure_setinternalparam((char*)device.ParamLinedata);
                        if(gear < 0){
                             usleep(1000);
                            continue;
                         }
                        if(isFirstLoadCurve)
                        {
                                ret = guide_measure_loadtempercurve(2, 1, 2);//coin417r(2) and 15mm(2)
                                qDebug()<<"loadtempercurve ret: "<<ret;
                                isFirstLoadCurve = false;

                                device.fp = fopen(paramlinedata,"ab");

                                fwrite(device.ParamLinedata, 1,device.VIDEO_WIDTH*2, device.fp);

                                usleep(10000);
                                fflush(device.fp);
                                fclose(device.fp);
                                device.fp = nullptr;
                        }
                        ret = guide_measure_convertgray2temper(2,device.Y16data,device.VIDEO_WIDTH*device.VIDEO_HEIGHT,pParamExt,pSurfaceTemper);
                        
                        centerTemp = pSurfaceTemper[device.VIDEO_WIDTH*device.VIDEO_HEIGHT/2+device.VIDEO_WIDTH/2];
                        
                        CEYSDUIView *pView = m_pVideoDeviceController->GetControlView();
                        if (!pView) return APC_OK;
                        int length = device.VIDEO_WIDTH*device.VIDEO_HEIGHT*3;
                        memcpy( &m_imageData[type].imageBuffer[0], device.rgb24, length);
       
                        pView->ImageCallback(m_imageData[type].imageDataType, type,
                                    &m_imageData[type].imageBuffer[0],
                                    length,
                                    m_imageData[type].nWidth, m_imageData[type].nHeight,
                                    centerTemp, nullptr);
                    }
                    break;
            }
            else
            {
                usleep(1000);
            }
        }
        return 0;
}

int CVideoDeviceModel_Grap::Open_thermal_camera()
{
            //set parameters for measure external
            pParamExt = (guide_measure_external_param_t*)malloc(sizeof(guide_measure_external_param_t*));
            pParamExt->emiss = 98;
            pParamExt->distance = 50;
            pParamExt->relHum = 60;
            pParamExt->atmosphericTemper = 230;
            pParamExt->reflectedTemper = 230;
            pParamExt->modifyK = 100;
            pParamExt->modifyB = 0;
            
            device.FD= FD;

            //+set device resolution
            device.setsize(video_w,video_h);
            //-set device resolution
            
            //+set image type
            device.Set_Video_Format(FD,video_w,video_h+1);
            device.imgType = device.Y16_Param;
             //-set image type

            //+start video
            device.InitDataMemory(device.VIDEO_WIDTH,device.VIDEO_HEIGHT);
            device.Get_Current_Format(FD);

            device.StartVideoPrePare(FD);
            device.StartVideo(FD);
            //-start video
            
            pSurfaceTemper = (float*)malloc(device.VIDEO_WIDTH*device.VIDEO_HEIGHT*sizeof(float*));
            pInternalTemper = (float*)malloc(device.VIDEO_WIDTH*device.VIDEO_HEIGHT*sizeof(float*));
            isSupportThermal = true;
           
            return 0;
}
#endif
//-[Thermal device]
