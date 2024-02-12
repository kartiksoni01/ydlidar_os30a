#include "CVideoDeviceModel_Kolor.h"
#include "CEYSDDeviceManager.h"
#include "CVideoDeviceController.h"
#include "eSPDI.h"
#include "CImageDataModel.h"

CVideoDeviceModel_Kolor::CVideoDeviceModel_Kolor(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_Kolor::InitDeviceSelInfo()
{
    CVideoDeviceModel::InitDeviceSelInfo();

    if(m_deviceSelInfo.empty()) return APC_NullPtr;

    DEVSELINFO *pDevSelfInfo = new DEVSELINFO;
    pDevSelfInfo->index = m_deviceSelInfo[0]->index + 1;
    m_deviceSelInfo.push_back(pDevSelfInfo);

    return APC_OK;
}

int CVideoDeviceModel_Kolor::InitDeviceInformation()
{
    CVideoDeviceModel::InitDeviceInformation();

    m_deviceInfo.push_back(GetDeviceInformation(m_deviceSelInfo[1]));
    return APC_OK;
}

int CVideoDeviceModel_Kolor::AddCameraPropertyModels()
{
    CVideoDeviceModel::AddCameraPropertyModels();
    m_cameraPropertyModel.push_back(new CCameraPropertyModel("Kolor", this, m_deviceSelInfo[1]));
    return APC_OK;
}

bool CVideoDeviceModel_Kolor::IsStreamSupport(STREAM_TYPE type)
{
    switch (type){
        case STREAM_KOLOR:
            return true;
        default:
            return CVideoDeviceModel::IsStreamSupport(type);
    }
}

int CVideoDeviceModel_Kolor::InitStreamInfoList()
{
    m_streamInfo[STREAM_KOLOR].resize(MAX_STREAM_INFO_COUNT, {0, 0, false});
    int ret;
    RETRY_APC_API(ret, APC_GetDeviceResolutionList(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                         m_deviceSelInfo[1],
                                                         MAX_STREAM_INFO_COUNT, &m_streamInfo[STREAM_KOLOR][0],
                                                         MAX_STREAM_INFO_COUNT, nullptr));

    if (APC_OK != ret) return ret;

    auto it = m_streamInfo[STREAM_KOLOR].begin();
    for ( ; it != m_streamInfo[STREAM_KOLOR].end() ; ++it){
        if (0 == (*it).nWidth){
            break;
        }
    }
    m_streamInfo[STREAM_KOLOR].erase(it, m_streamInfo[STREAM_KOLOR].end());
    m_streamInfo[STREAM_KOLOR].shrink_to_fit();

    return CVideoDeviceModel::InitStreamInfoList();
}

CVideoDeviceModel::ImageData &CVideoDeviceModel_Kolor::GetColorImageData()
{
    if (m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR))
        return m_imageData[STREAM_KOLOR];

    return CVideoDeviceModel::GetColorImageData();
}
