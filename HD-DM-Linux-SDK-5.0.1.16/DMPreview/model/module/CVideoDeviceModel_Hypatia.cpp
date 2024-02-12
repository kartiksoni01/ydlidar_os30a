#include "CVideoDeviceModel_Hypatia.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

CVideoDeviceModel_Hypatia::CVideoDeviceModel_Hypatia(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_Hypatia::UpdateIR()
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

void CVideoDeviceModel_Hypatia::SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController)
{
    CVideoDeviceModel::SetVideoDeviceController(pVideoDeviceController);
    if(m_pVideoDeviceController){
        m_pVideoDeviceController->GetPreviewOptions()->SetIRLevel(60);
    }
}
