#include "CVideoDeviceModel_Nora.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

#define DEFAULT_IR_MAX 64
#define EXTEND_IR_MAX 127
CVideoDeviceModel_Nora::CVideoDeviceModel_Nora(DEVSELINFO *pDeviceSelfInfo) : CVideoDeviceModel(pDeviceSelfInfo)
{
}

int CVideoDeviceModel_Nora::AdjustZDTableIndex(int &nIndex)
{
    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH)){
        return APC_OK;
    }

    std::vector<APC_STREAM_INFO> streamInfo = GetStreamInfoList(STREAM_DEPTH);
    int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);

    if (768 == streamInfo[nDepthIndex].nHeight){
        nIndex = 0;
    }else if (384 <= streamInfo[nDepthIndex].nHeight){
        nIndex = 1;
    }

    return APC_OK;
}

bool CVideoDeviceModel_Nora::IsIRExtended()
{
    return EXTEND_IR_MAX == m_nIRMax;
}

int CVideoDeviceModel_Nora::UpdateIR()
{
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
    int ret;
    if (IsIRExtended() == false)
    {
        ret = APC_SetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                m_deviceSelInfo[0],
                                0xE2, DEFAULT_IR_MAX,
                                FG_Address_1Byte | FG_Value_1Byte);
        if (APC_OK != ret)
            return ret;
    }

    return CVideoDeviceModel::UpdateIR();
}

int CVideoDeviceModel_Nora::ExtendIR(bool bEnable)
{
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();

    int ret;
    bool bDisableIR = (0 == m_nIRValue);
    if (bDisableIR)
    {
        RETRY_APC_API(ret, APC_SetIRMode(pEYSDI, m_deviceSelInfo[0], 0x03)); // 2 bits on for opening both 2 ir
    }

    RETRY_APC_API(ret, APC_SetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                         m_deviceSelInfo[0],
                                         0xE2, bEnable ? EXTEND_IR_MAX : DEFAULT_IR_MAX,
                                         FG_Address_1Byte | FG_Value_1Byte));

    if (bDisableIR)
    {
        RETRY_APC_API(ret, APC_SetIRMode(pEYSDI, m_deviceSelInfo[0], 0x00);); // turn off ir
    }

    if (ret != APC_OK)
        return ret;

    ret = CVideoDeviceModel::UpdateIR();

    return ret;
}

void CVideoDeviceModel_Nora::SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController)
{
    CVideoDeviceModel::SetVideoDeviceController(pVideoDeviceController);
    if(m_pVideoDeviceController){
        m_pVideoDeviceController->GetPreviewOptions()->SetIRLevel(0);
    }
}
