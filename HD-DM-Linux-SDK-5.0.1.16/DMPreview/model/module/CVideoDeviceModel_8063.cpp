#include "CVideoDeviceModel_8063.h"
#include "CEYSDDeviceManager.h"
#include "CVideoDeviceController.h"
#include "eSPDI.h"
#include <algorithm>

CVideoDeviceModel_8063::CVideoDeviceModel_8063(DEVSELINFO *pDeviceSelfInfo) :
                        CVideoDeviceModel(pDeviceSelfInfo),
                        CVideoDeviceModel_Kolor(pDeviceSelfInfo),
                        m_KolorDeviceDepthDataType(APC_DEPTH_DATA_DEFAULT) {

}
int CVideoDeviceModel_8063::GetCurrentTemperature(float& temp) {
    unsigned char device_id_reg = 0x0F;
    unsigned short device_id_reg_val = 0x00;
    unsigned char temperature_reg = 0x00;
    unsigned short temperature_reg_val = 0x00;
    unsigned char nID = 0x90;
    SENSORMODE_INFO nSensorMode = SENSOR_BOTH;
    unsigned short temperature_reg_val_tmp = 0;
    bool is_negtive = false;
    constexpr int registerFlag = FG_Address_1Byte | FG_Value_2Byte;
    int ret = APC_Init_Fail;

    RETRY_APC_API(ret, APC_GetSensorRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0], nID,
                                            (unsigned short) device_id_reg, &device_id_reg_val, registerFlag,
                                            nSensorMode));

    if (ret != APC_OK) {
        qDebug() << "Read ICM-20602 error.\n";
        return 0.0f;
    }

    unsigned short device_id_reg_val_reverse = __bswap_16(device_id_reg_val);
           // ((((unsigned char*)&device_id_reg_val)[0]) << 8) + ((unsigned char*)&device_id_reg_val)[1];
    if (device_id_reg_val_reverse != 0x7500) {
        qDebug() << "Read ICM-20602 device ID error.\n";
        return 0.0f;
    }

    RETRY_APC_API(ret, APC_GetSensorRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0], nID,
                                            (unsigned short) temperature_reg, &temperature_reg_val, registerFlag,
                                            nSensorMode));

    if (ret != APC_OK) {
        qDebug() << "Read ICM-20602 device temperature error.\n";
        return 0.0f;
    }

    unsigned short temperature_reg_val_reverse = __bswap_16(temperature_reg_val);
            //((((unsigned char*)&temperature_reg_val)[0]) << 8) + ((unsigned char*)&temperature_reg_val)[1];
    temperature_reg_val_tmp = temperature_reg_val_reverse;
    if (temperature_reg_val_reverse & 0x8000) {
        temperature_reg_val_tmp = ~temperature_reg_val_tmp + 0x0010;
        is_negtive = true;
    }

    temp = (temperature_reg_val_tmp >> 8);
    if ((temperature_reg_val_tmp & 0x00f0) & 0x80) {
        temp += 0.5;
    }

    if ((temperature_reg_val_tmp & 0x00f0) & 0x40) {
        temp += 0.25;
    }

    if ((temperature_reg_val_tmp & 0x00f0) & 0x20) {
        temp += 0.125;
    }

    if ((temperature_reg_val_tmp & 0x00f0) & 0x10) {
        temp += 0.0625;
    }

    if (is_negtive) {
        temp = 0.0 - temp;
    }

    return temp;
}
int CVideoDeviceModel_8063::StartStreamingTask(){
    int ret = CVideoDeviceModel_Kolor::StartStreamingTask();
    if (ret != APC_OK) {
        return ret;
    }

    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if (bKolorStream) {
        ret = CreateStreamTask(STREAM_KOLOR);
    }
    return ret;
}

bool CVideoDeviceModel_8063::IsStreamAvailable()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);
    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);

    return bColorStream || bDepthStream || bKolorStream;
}
int CVideoDeviceModel_8063::UpdateDepthDataType() {
    int retColorDepthDevice = CVideoDeviceModel::UpdateDepthDataType();
    if (APC_OK != retColorDepthDevice) return retColorDepthDevice;

    int retKolor;
    RETRY_APC_API(retKolor, APC_GetDepthDataType(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                 m_deviceSelInfo[1], &m_KolorDeviceDepthDataType));
    if (APC_OK != retKolor) return retKolor;

    return APC_OK;
}
int CVideoDeviceModel_8063::SetDepthDataType(int nDepthDataType){
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
    if (!pEYSDI || !m_deviceSelInfo[0] || !m_deviceSelInfo[1]) {
        fprintf(stderr,"CVideoDeviceModel_8063 APC_SetDepthDataType failed\n");
        return APC_Init_Fail;
    }

    int ret = APC_Init_Fail;
    RETRY_APC_API(ret, APC_SetDepthDataType(pEYSDI, m_deviceSelInfo[0], nDepthDataType));

    if (ret != APC_OK) {
        fprintf(stderr,"APC_SetDepthDataType m_deviceSelInfo[0] failed\n");
        return ret;
    }

    unsigned short kolorDepthDataType = APC_DEPTH_DATA_DEFAULT;

    int retKolor = APC_Init_Fail;
    RETRY_APC_API(retKolor, APC_SetDepthDataType(pEYSDI, m_deviceSelInfo[1], kolorDepthDataType));

    if (retKolor != APC_OK) {
        fprintf(stderr,"APC_SetDepthDataType m_deviceSelInfo[1] failed\n");
        return retKolor;
    }

    UpdateDepthDataType();

    return APC_OK;
}


int CVideoDeviceModel_8063::GetPointCloudInfo(eSPCtrl_RectLogData *pRectifyLogData, PointCloudInfo &pointCloudInfo,
                                              ImageData colorImageData, ImageData depthImageData)
{
    bool IsKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    memset(&pointCloudInfo, 0, sizeof(PointCloudInfo));

    pointCloudInfo.wDepthType = depthImageData.depthDataType;

    const float ratio_Mat = IsKolorStream ? 1.0f : (float) depthImageData.nHeight / pRectifyLogData->OutImgHeight;
    const float baseline  = 1.0f / pRectifyLogData->ReProjectMat[14];
    const float diff      = pRectifyLogData->ReProjectMat[15] * ratio_Mat;

    pointCloudInfo.centerX = -1.0f * pRectifyLogData->ReProjectMat[3] * ratio_Mat;
    pointCloudInfo.centerY = -1.0f * pRectifyLogData->ReProjectMat[7] * ratio_Mat;
    pointCloudInfo.focalLength = pRectifyLogData->ReProjectMat[11] * ratio_Mat;

    switch (depthImageData.imageDataType){
        case APCImageType::DEPTH_14BITS: pointCloudInfo.disparity_len = 0; break;
        case APCImageType::DEPTH_11BITS:
        {
            pointCloudInfo.disparity_len = 2048;
            for(int i = 0 ; i < pointCloudInfo.disparity_len ; ++i){
                pointCloudInfo.disparityToW[i] = ( i * ratio_Mat / 8.0f ) / baseline + diff;
            }
            break;
        }
        default:
            pointCloudInfo.disparity_len = 256;
            for(int i = 0 ; i < pointCloudInfo.disparity_len ; ++i){
                pointCloudInfo.disparityToW[i] = (i * ratio_Mat) / baseline + diff;
            }
            break;
    }

    if(IsKolorStream){
        eSPCtrl_RectLogData rectifyLogDataSlave = mSlaveDeviceRectifyLogData;

        float slaveRatio_x = (float) rectifyLogDataSlave.RECT_ScaleWidth / (rectifyLogDataSlave.OutImgWidth / 2.0f);
        float slaveRatio_y = (float) rectifyLogDataSlave.RECT_ScaleHeight / rectifyLogDataSlave.OutImgHeight;
        float slaveRECT_Crop_Row_BG = (float) rectifyLogDataSlave.RECT_Crop_Row_BG;
        float slaveRECT_Crop_Row_ED = (float) rectifyLogDataSlave.RECT_Crop_Row_ED;
        float slaveRECT_Crop_Col_BG_L = (float) rectifyLogDataSlave.RECT_Crop_Col_BG_L;

        float M_dst_scale[9] = {0.0f};

        std::copy(std::begin(rectifyLogDataSlave.CamMat2 /* M_dst */), std::end(rectifyLogDataSlave.CamMat2),
                  std::begin(M_dst_scale));

        M_dst_scale[0] = M_dst_scale[0] * slaveRatio_x;
        M_dst_scale[2] = (M_dst_scale[2] - slaveRECT_Crop_Col_BG_L) * slaveRatio_x;
        M_dst_scale[4] = M_dst_scale[4] * slaveRatio_y;
        M_dst_scale[5] = (M_dst_scale[5] - slaveRECT_Crop_Row_BG) * slaveRatio_y;

        memcpy(pointCloudInfo.slaveDeviceCamMat2, M_dst_scale, 9 * sizeof(float));
        memcpy(pointCloudInfo.slaveDeviceRotaMat, rectifyLogDataSlave.RotaMat, 9 * sizeof(float));
        memcpy(pointCloudInfo.slaveDeviceTranMat, rectifyLogDataSlave.TranMat, 3 * sizeof(float));
    }

    return APC_OK;
}

int CVideoDeviceModel_8063::CloseDevice()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);
    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);

    int ret = APC_NoDevice;
    if(bColorStream || bDepthStream){
        if(APC_OK == APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[0])) {
            ret = APC_OK;
        }
    }

    if(bKolorStream){
        if(APC_OK == APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[1])) {
            ret = APC_OK;
        }
    }

    return ret;
}

int CVideoDeviceModel_8063::ClosePreviewView()
{
    CVideoDeviceModel_Kolor::ClosePreviewView();

    // TODO refactoring to parent class
    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if (bKolorStream){
        m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_KOLOR);
    }

    return APC_OK;
}

int CVideoDeviceModel_8063::OpenDevice()
{
    bool bKolorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);
    if(bKolorStream) {
        int nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_KOLOR);
        if(APC_OK != APC_OpenDevice2(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                     m_deviceSelInfo[1],
                                     m_imageData[STREAM_KOLOR].nWidth, m_imageData[STREAM_KOLOR].nHeight, m_imageData[STREAM_KOLOR].bMJPG,
                                     0, 0,
                                     DEPTH_IMG_NON_TRANSFER,
                                     true, nullptr,
                                     &nFPS,
                                     IMAGE_SN_SYNC)) {
            return APC_OPEN_DEVICE_FAIL;
        }
        m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_KOLOR, nFPS);
    }

    if (APC_OK != CVideoDeviceModel_Kolor::OpenDevice()) {
        return APC_OPEN_DEVICE_FAIL;
    }

    return APC_OK;
}
int CVideoDeviceModel_8063::MergePointCloudInfoWithSlaveCameraMatrix(eSPCtrl_RectLogData& slaveRectifyLog,
                                                                     PointCloudInfo& pointCloudInfo)
{
    memcpy(pointCloudInfo.slaveDeviceCamMat2, slaveRectifyLog.CamMat2, 9 * sizeof(float));
    memcpy(pointCloudInfo.slaveDeviceRotaMat, slaveRectifyLog.RotaMat, 9 * sizeof(float));
    memcpy(pointCloudInfo.slaveDeviceTranMat, slaveRectifyLog.TranMat, 3 * sizeof(float));
    return APC_OK;
}
int CVideoDeviceModel_8063::PreparePointCloudInfo()
{
    constexpr unsigned short kDEPTHINDEX = 0;
    constexpr unsigned short kSLAVEINDEX = 1;
    GetRectifyLogData(kDEPTHINDEX, m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH), &m_rectifyLogData);
    GetRectifyLogData(kSLAVEINDEX, m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_KOLOR), &mSlaveDeviceRectifyLogData);
    GetPointCloudInfo(&m_rectifyLogData, m_pointCloudInfo, GetColorImageData(), GetDepthImageData());
    return APC_OK;
}
int CVideoDeviceModel_8063::GetColorImage(STREAM_TYPE type)
{
    DEVSELINFO *deviceSelInfo =  STREAM_KOLOR == type ? m_deviceSelInfo[1] : m_deviceSelInfo[0];

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

int CVideoDeviceModel_8063::GetImage(STREAM_TYPE type) {
    int ret;
    switch (type){
        case STREAM_KOLOR:
            ret = GetColorImage(type); // TODO move to parent class implementation. Verify on 8060.
            break;
        default:
            return CVideoDeviceModel_Kolor::GetImage(type);
    }
    return ret;
}

int CVideoDeviceModel_8063::PrepareOpenDevice()
{
    CVideoDeviceModel_Kolor::PrepareOpenDevice();

    auto PrepareImageData = [&](STREAM_TYPE type){
        int retFirmware = APC_NotSupport;
        bool bStreamEnable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(type);
        if(!bStreamEnable){
            RETRY_APC_API(retFirmware, APC_SetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                          0xf5, 0x1, FG_Address_1Byte | FG_Value_1Byte));
            return;
        }
        RETRY_APC_API(retFirmware, APC_SetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                      0xf5, 0x0, FG_Address_1Byte | FG_Value_1Byte));

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
        unsigned int nBufferSize = m_imageData[type].nWidth * m_imageData[type].nHeight * nBytePerPixel;
        if (m_imageData[type].imageBuffer.size() != nBufferSize){
            m_imageData[type].imageBuffer.resize(nBufferSize);
        }
        memset(&m_imageData[type].imageBuffer[0], 0, sizeof(nBufferSize));

    };

    PrepareImageData(STREAM_KOLOR);
    return APC_OK;
}

int CVideoDeviceModel_8063::InitStreamInfoList(){
    return CVideoDeviceModel_Kolor::InitStreamInfoList();
}

int CVideoDeviceModel_8063::AdjustZDTableIndex(int &nIndex) {
    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH)){
        return APC_OK;
    }

    nIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);

    return APC_OK;
}

std::vector<CloudPoint> CVideoDeviceModel_8063::GeneratePointCloud(std::vector<unsigned char> &depthData, std::vector<unsigned char> &colorData,
                                                                   unsigned short nDepthWidth, unsigned short nDepthHeight,
                                                                   unsigned short nColorWidth, unsigned short nColorHeight,
                                                                   eSPCtrl_RectLogData rectifyLogData,
                                                                   APCImageType::Value depthImageType,
                                                                   int nZNear, int nZFar,
                                                                   bool bUsePlyFilter,
                                                                   std::vector<float> imgFloatBufOut) {
    std::vector<CloudPoint> cloudPoints;
    if(bUsePlyFilter && !imgFloatBufOut.empty()) {
//        PlyWriter::EYSDFrameTo3D_PlyFilterFloat(nDepthWidth, nDepthHeight,
//                                                 imgFloatBufOut,
//                                                 nColorWidth, nColorHeight,
//                                                 colorData,
//                                                 &rectifyLogData, depthImageType,
//                                                 cloudPoints,
//                                                 true, nZNear, nZFar,
//                                                 true, false, 1.0f);
    }else{
        PlyWriter::EYSDFrameTo3D_8063(nDepthWidth, nDepthHeight, depthData,
                                  nColorWidth, nColorHeight, colorData,
                                  &rectifyLogData, &mSlaveDeviceRectifyLogData, depthImageType,
                                  cloudPoints,
                                  true, nZNear, nZFar,
                                  true, false, 1.0f);
    }

    return cloudPoints;

}
