#include "CVideoDeviceModel_8029.h"
#include "eSPDI.h"
#include "CEYSDDeviceManager.h"
#include "CVideoDeviceController.h"

CVideoDeviceModel_8029::CVideoDeviceModel_8029(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}


int CVideoDeviceModel_8029::InitUsbType()
{
    m_usbPortType = USB_PORT_TYPE_3_0;
    std::vector<APC_STREAM_INFO> pColorStreamInfo = GetStreamInfoList(STREAM_COLOR);
    for(APC_STREAM_INFO streamInfo : pColorStreamInfo){
        if(streamInfo.bFormatMJPG){
            m_usbPortType = USB_PORT_TYPE_2_0;
            break;
        }
    }

    return APC_OK;
}

int CVideoDeviceModel_8029::SetIRValue(unsigned short nValue)
{
    if(STATE::STREAMING == GetState()){
        int ret = APC_SetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                        m_deviceSelInfo[0],
                                        0x81, nValue,
                                        FG_Address_1Byte | FG_Value_1Byte);
        UpdateIR();
        return ret;
    }else{
        m_nIRValue = nValue;
        return APC_OK;
    }
}

int CVideoDeviceModel_8029::UpdateIR()
{
    m_nIRMax = 16;
    m_nIRMin = 0;
    int ret = APC_OK;
    if(STATE::STREAMING == GetState()){
        RETRY_APC_API(ret, APC_GetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                   m_deviceSelInfo[0],
                                                   0x81, &m_nIRValue,
                                                   FG_Address_1Byte | FG_Value_1Byte));
    }
    return ret;
}

int CVideoDeviceModel_8029::AdjustRegister()
{
    int ret;
    unsigned short nF05B = 0;
    nF05B = 0xF0;
    RETRY_APC_API(ret, APC_SetHWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                               m_deviceSelInfo[0],
                                               0xF05B, nF05B,
                                               FG_Address_2Byte | FG_Value_1Byte));
    nF05B = 0xF3;
    RETRY_APC_API(ret, APC_SetHWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                               m_deviceSelInfo[0],
                                               0xF05B, nF05B,
                                               FG_Address_2Byte | FG_Value_1Byte));

    return CVideoDeviceModel::AdjustRegister();
}

int CVideoDeviceModel_8029::ConfigDepthFilter()
{
    if (!m_pVideoDeviceController) return APC_OK;

    DepthFilterOptions *pDepthFilterOptions = m_pVideoDeviceController->GetDepthFilterOptions();

    pDepthFilterOptions->EnableDepthFilter(true);
    pDepthFilterOptions->SetDepthEnableLock(true);

    pDepthFilterOptions->EnableFlyingDepthCancellation(true);
    pDepthFilterOptions->SetFlyingDepthCancellationLock(true);

    return APC_OK;
}

int CVideoDeviceModel_8029::OpenDevice()
{
    int nIRValue = m_nIRValue;
    if (0 != nIRValue){
        SetIRValue(0);
    }

    int ret = CVideoDeviceModel::OpenDevice();

    if (0 != nIRValue){
        QThread::msleep(50);
        ChangeState(STREAMING);
        SetIRValue(nIRValue);
    }

    return ret;
}

int CVideoDeviceModel_8029::GetPointCloudInfo(eSPCtrl_RectLogData *pRectifyLogData, PointCloudInfo &pointCloudInfo,
                                              ImageData colorImageData, ImageData depthImageData)
{
    int ret = CVideoDeviceModel::GetPointCloudInfo(pRectifyLogData, pointCloudInfo, colorImageData, depthImageData);
    if (256 == pointCloudInfo.disparity_len){
        for(int i = 0 ; i < pointCloudInfo.disparity_len ; ++i){
            pointCloudInfo.disparityToW[i] *= 0.5f;
        }
    }
    return ret;
}

std::vector<CloudPoint> CVideoDeviceModel_8029::GeneratePointCloud(std::vector<unsigned char> &depthData,
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
    APC_FlyingDepthCancellation_D8(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                    m_deviceSelInfo[0],
                                    &depthData[0],
                                    nDepthWidth, nDepthHeight);

    if (bUsePlyFilter){
        return CVideoDeviceModel::GeneratePointCloud(depthData,
                                                     colorData,
                                                     nDepthWidth,
                                                     nDepthHeight,
                                                     nColorWidth,
                                                     nColorHeight,
                                                     rectifyLogData,
                                                     depthImageType,
                                                     nZNear, nZFar,
                                                     bUsePlyFilter,
                                                     imgFloatBufOut);
    }else{
        std::vector<CloudPoint> cloudPoints;
        PlyWriter::EYSDFrameTo3D_8029(nDepthWidth, nDepthHeight,
                                       depthData,
                                       nColorWidth, nColorHeight,
                                       colorData,
                                       &rectifyLogData,
                                       depthImageType,
                                       cloudPoints,
                                       true,
                                       nZNear, nZFar,
                                       true, false, 1.0f);
        return cloudPoints;
    }


}

int CVideoDeviceModel_8029::FrameGrabberDataTransform(std::vector<unsigned char>& bufDepth, int &widthDepth, int &heightDepth,
                                                      std::vector<unsigned char>& bufColor, int &widthColor, int &heightColor,
                                                      int &serialNumber)
{
    return APC_FlyingDepthCancellation_D8(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                              m_deviceSelInfo[0],
                                              &bufDepth[0],
                                              widthDepth, heightDepth);
}
