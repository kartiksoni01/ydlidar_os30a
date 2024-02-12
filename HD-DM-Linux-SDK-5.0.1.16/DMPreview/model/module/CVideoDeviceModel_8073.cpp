#include "CVideoDeviceModel_8073.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

CVideoDeviceModel_8073::CVideoDeviceModel_8073(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}
int CVideoDeviceModel_8073::TransformDepthDataType(int nDepthDataType, bool bRectifyData) {
    // Due to sensor type is customized, so interleave mode did not follow video mode 0.4b
    switch (nDepthDataType){
        case APC_DEPTH_DATA_8_BITS:
        case APC_DEPTH_DATA_8_BITS_RAW:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_8_BITS : APC_DEPTH_DATA_8_BITS_RAW; break;
        case APC_DEPTH_DATA_8_BITS_x80:
        case APC_DEPTH_DATA_8_BITS_x80_RAW:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_8_BITS_x80 : APC_DEPTH_DATA_8_BITS_x80_RAW; break;
        case APC_DEPTH_DATA_11_BITS:
        case APC_DEPTH_DATA_11_BITS_RAW:
        case APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_11_BITS : APC_DEPTH_DATA_11_BITS_RAW;  break;
        case APC_DEPTH_DATA_14_BITS:
        case APC_DEPTH_DATA_14_BITS_RAW:
        case APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_14_BITS : APC_DEPTH_DATA_14_BITS_RAW;  break;
        case APC_DEPTH_DATA_OFF_RAW:
        case APC_DEPTH_DATA_OFF_RECTIFY:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_OFF_RECTIFY : APC_DEPTH_DATA_OFF_RAW;  break;
        default:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_DEFAULT : APC_DEPTH_DATA_OFF_RECTIFY;  break;
    }
    return nDepthDataType;
}
int CVideoDeviceModel_8073::UpdateIR()
{
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
    int ret;
    ret = APC_SetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                            m_deviceSelInfo[0],
                            0xE2, 96,
                            FG_Address_1Byte | FG_Value_1Byte);
    if (APC_OK != ret) return ret;

    return CVideoDeviceModel::UpdateIR();
}
// ++++++ PM / FAE requested function for POC version for robotic application

void CVideoDeviceModel_8073::SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController)
{
    CVideoDeviceModel::SetVideoDeviceController(pVideoDeviceController);
    if(m_pVideoDeviceController){
        m_pVideoDeviceController->GetPreviewOptions()->SetIRLevel(60);
    }
}

int CVideoDeviceModel_8073::StartStreamingTask(){
    CVideoDeviceModel::StartStreamingTask();

    if (IsInterleaveMode()) {
        m_cameraPropertyModel[0]->SetCameraPropertyValue(CCameraPropertyModel::AUTO_EXPOSURE, 0);
        m_cameraPropertyModel[0]->SetCameraPropertySupport(CCameraPropertyModel::AUTO_EXPOSURE, true);

        m_cameraPropertyModel[0]->SetCameraPropertyValue(CCameraPropertyModel::EXPOSURE_TIME, -10);
        m_cameraPropertyModel[0]->SetCameraPropertySupport(CCameraPropertyModel::EXPOSURE_TIME, true);

        m_bPrevLowLightValue = m_cameraPropertyModel[0]->GetCameraProperty(CCameraPropertyModel::LOW_LIGHT_COMPENSATION).nValue;
        m_cameraPropertyModel[0]->SetCameraPropertyValue(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, 0);
        m_cameraPropertyModel[0]->SetCameraPropertySupport(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, true);
    }

    if(m_pVideoDeviceController){
        m_pVideoDeviceController->GetPreviewOptions()->SetIRLevel(60);
    }

    return APC_OK;
}
// ------ PM / FAE requested function for POC version for robotic application
