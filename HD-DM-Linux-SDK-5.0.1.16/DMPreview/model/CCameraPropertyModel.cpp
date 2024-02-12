#include "CCameraPropertyModel.h"
#include "CVideoDeviceModel.h"
#include "CEYSDDeviceManager.h"
#include "eSPDI.h"
#include <math.h>
#include <eSPDI_def.h>

CCameraPropertyModel::CCameraPropertyModel(QString sDeviceName,
                                           CVideoDeviceModel *pVideoDeviceModel,
                                           DEVSELINFO *pDeviceSelfInfo):
m_sDeviceName(sDeviceName),
m_pVideoDeviceModel(pVideoDeviceModel),
m_pDeviceSelfInfo(pDeviceSelfInfo)
{

}

CCameraPropertyModel::~CCameraPropertyModel()
{

}

int CCameraPropertyModel::Init()
{
    InitCameraProperty();
    return APC_OK;
}

int CCameraPropertyModel::Update()
{
    for (int i = 0 ; i < CAMERA_PROPERTY_COUNT ; ++i){
        UpdateCameraProperty((CAMERA_PROPERTY)i);
    }

    return APC_OK;
}

int CCameraPropertyModel::Reset()
{
    Init();
    return APC_OK;
}

int CCameraPropertyModel::InitCameraProperty()
{

    for (int i = 0 ; i < CAMERA_PROPERTY_COUNT ; i++){
        int nID;
        bool bIsCTProperty;
        GetCameraPropertyFlag((CAMERA_PROPERTY)i, nID, bIsCTProperty);

        CameraPropertyItem &item = m_cameraPropertyItems[i];
        if (!item.bSupport) continue;

        item.bValid = false;

        void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
        int ret;
        if (bIsCTProperty){
            RETRY_APC_API(ret, APC_GetCTRangeAndStep(pEYSDI, m_pDeviceSelfInfo, nID,
                                                           &item.nMax, &item.nMin, &item.nStep, &item.nDefault, &item.nFlags));
        }else{
            RETRY_APC_API(ret, APC_GetPURangeAndStep(pEYSDI, m_pDeviceSelfInfo, nID,
                                                           &item.nMax, &item.nMin, &item.nStep, &item.nDefault, &item.nFlags));
        }

        if (APC_OK != ret) continue;

#if !defined(_MIPI_CAMERA_)
        if(EXPOSURE_TIME == (CAMERA_PROPERTY)i){
            item.nMax = log2(item.nMax / 10000.0);
            item.nMin = log2(item.nMin  / 10000.0);
            item.nDefault = log2(item.nDefault / 10000.0);
        }
#endif
        DataToInfo((CAMERA_PROPERTY)i, item.nMax);
        DataToInfo((CAMERA_PROPERTY)i, item.nMin);
        DataToInfo((CAMERA_PROPERTY)i, item.nDefault);

        item.bValid = true;

        UpdateCameraProperty((CAMERA_PROPERTY)i);

        printf("[%s][%d][%s][nID, max, min, nsetp, default, flags] = [%d, %d, %d, %d, %d, 0x%08x]\n", \
            __func__, __LINE__, (bIsCTProperty)?"CT":"PU", nID, item.nMax, item.nMin, item.nStep, item.nDefault, item.nFlags);
    }

    for (int i = 0 ; i < CAMERA_PROPERTY_COUNT ; ++i){
        if(m_cameraPropertyItems[i].bSupport && !m_cameraPropertyItems[i].bValid) return APC_Init_Fail;
    }

    return APC_OK;

}

int CCameraPropertyModel::UpdateCameraProperty(CAMERA_PROPERTY type)
{
    int nID;
    bool bIsCTProperty;
    GetCameraPropertyFlag(type, nID, bIsCTProperty);

    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
    int ret;
    long int nValue;
    if (bIsCTProperty){
        RETRY_APC_API(ret, APC_GetCTPropVal(pEYSDI, m_pDeviceSelfInfo, nID, &nValue));
    } else {
        RETRY_APC_API(ret, APC_GetPUPropVal(pEYSDI, m_pDeviceSelfInfo, nID, &nValue));
    }

    if(APC_OK == ret){
        m_cameraPropertyItems[type].nValue = nValue;
        DataToInfo(type, m_cameraPropertyItems[type].nValue);
    }

    return ret;
}

int CCameraPropertyModel::SetDefaultCameraProperty()
{
    int ret = APC_OK;

    for (int i = 0 ; i < CAMERA_PROPERTY_COUNT ; ++i){
        bool bNeedRestoreAutoState = false;
        if(EXPOSURE_TIME == (CAMERA_PROPERTY)i){
            if(1 == m_cameraPropertyItems[AUTO_EXPOSURE].nValue){
                SetCameraPropertyValue(AUTO_EXPOSURE, 0);
                bNeedRestoreAutoState = true;
            }
        }else if(WHITE_BLANCE_TEMPERATURE == (CAMERA_PROPERTY)i){
            if(1 == m_cameraPropertyItems[AUTO_WHITE_BLANCE].nValue){
                SetCameraPropertyValue(AUTO_WHITE_BLANCE, 0);
                bNeedRestoreAutoState = true;
            }
        }

        ret = SetCameraPropertyValue((CAMERA_PROPERTY)i, m_cameraPropertyItems[i].nDefault);

        if(bNeedRestoreAutoState){
            if(EXPOSURE_TIME == (CAMERA_PROPERTY)i){
                SetCameraPropertyValue(AUTO_EXPOSURE, 1);
            }else if(WHITE_BLANCE_TEMPERATURE == (CAMERA_PROPERTY)i){
                SetCameraPropertyValue(AUTO_WHITE_BLANCE, 1);
            }
        }
    }

    return ret;
}

int CCameraPropertyModel::SetCameraPropertyValue(CAMERA_PROPERTY type, int nValue)
{
    if (!m_cameraPropertyItems[type].bSupport || !m_cameraPropertyItems[type].bValid){
        return APC_NotSupport;
    }

    int nID;
    bool bIsCTProperty;
    GetCameraPropertyFlag(type, nID, bIsCTProperty);
    long int nGetValue;
    InfoToData(type, nValue);

    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();

    int ret;
    if (bIsCTProperty){
        RETRY_APC_API(ret, APC_SetCTPropVal(pEYSDI, m_pDeviceSelfInfo, nID, nValue));
    }else{
        RETRY_APC_API(ret, APC_SetPUPropVal(pEYSDI, m_pDeviceSelfInfo, nID, nValue));
    }

    printf("[%s][%d][%s][nID, nValue] = [%d, %d(0x%08x)]\n", __func__, __LINE__, \
        (bIsCTProperty)?"CT":"PU", nID, nValue, (unsigned long)nValue);

    if (bIsCTProperty){
        RETRY_APC_API(ret, APC_GetCTPropVal(pEYSDI, m_pDeviceSelfInfo, nID, &nGetValue));
    }else{
        RETRY_APC_API(ret, APC_GetPUPropVal(pEYSDI, m_pDeviceSelfInfo, nID, &nGetValue));
    }

    printf("[%s][%d][%s][nID, nValue] = [%d, %d(0x%08x)]\n", __func__, __LINE__, \
        (bIsCTProperty)?"CT":"PU", nID, nGetValue, nGetValue);

    if (AUTO_EXPOSURE == type){
        SetCameraPropertyValue(EXPOSURE_TIME, m_cameraPropertyItems[EXPOSURE_TIME].nValue);
    }else if (AUTO_WHITE_BLANCE == type){
        SetCameraPropertyValue(WHITE_BLANCE_TEMPERATURE, m_cameraPropertyItems[WHITE_BLANCE_TEMPERATURE].nValue);
    }

    UpdateCameraProperty(type);

    return ret;
}

void CCameraPropertyModel::GetCameraPropertyFlag(CAMERA_PROPERTY type, int &nID, bool &bIsCTProperty)
{
    switch (type){
        case AUTO_EXPOSURE:
            nID = CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL;
            bIsCTProperty = true;
            break;
        case AUTO_WHITE_BLANCE:
            nID = PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL;
            bIsCTProperty = false;
            break;
        case LOW_LIGHT_COMPENSATION:
            nID = CT_PROPERTY_ID_AUTO_EXPOSURE_PRIORITY_CTRL;
            bIsCTProperty = true;
            break;
        case LIGHT_SOURCE:
            nID = PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL;
            bIsCTProperty = false;
            break;
        case EXPOSURE_TIME:
            nID = CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL;
            bIsCTProperty = true;
            break;
        case WHITE_BLANCE_TEMPERATURE:
            nID = PU_PROPERTY_ID_WHITE_BALANCE_CTRL;
            bIsCTProperty = false;
            break;
        default:
            break;
    }
}

void CCameraPropertyModel::DataToInfo(CAMERA_PROPERTY type, int &nValue)
{
    switch(type){
        case AUTO_EXPOSURE:
#if !defined(_MIPI_CAMERA_)
            nValue = (nValue == 3) ? 1 : 0;
#else
			nValue = (nValue == AE_MOD_APERTURE_PRIORITY_MODE) ? 1 : 0;
#endif
            break;
        case LIGHT_SOURCE:
            nValue = (1 == nValue) ? VALUE_50HZ : VALUE_60HZ;
            break;
        case AUTO_WHITE_BLANCE:
        case LOW_LIGHT_COMPENSATION:
        case EXPOSURE_TIME:
        case WHITE_BLANCE_TEMPERATURE:
        default:
            break;
    }
}

void CCameraPropertyModel::InfoToData(CAMERA_PROPERTY type, int &nValue)
{
    switch(type){
        case AUTO_EXPOSURE:
#if !defined(_MIPI_CAMERA_)
            nValue = (1 == nValue) ? 3 : 1;
#else
			nValue = (1 == nValue) ? AE_MOD_APERTURE_PRIORITY_MODE : AE_MOD_MANUAL_MODE;
#endif
            break;
        case LIGHT_SOURCE:
            nValue = (VALUE_50HZ == nValue) ? 1 : 2;
            break;
        case AUTO_WHITE_BLANCE:
        case LOW_LIGHT_COMPENSATION:
        case EXPOSURE_TIME:
        case WHITE_BLANCE_TEMPERATURE:
        default:
            break;
    }
}

float CCameraPropertyModel::GetManuelExposureTimeMs()
{
    float fExposureTimeMs = 0.0f;
    int ret;
    RETRY_APC_API(ret, APC_GetExposureTime(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                 m_pDeviceSelfInfo,
                                                 SENSOR_BOTH,
                                                 &fExposureTimeMs));

    return fExposureTimeMs;
}

void CCameraPropertyModel::SetManuelExposureTimeMs(float fMs)
{
    int ret;
    RETRY_APC_API(ret, APC_SetExposureTime(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                 m_pDeviceSelfInfo,
                                                 SENSOR_BOTH,
                                                 fMs));

}

float CCameraPropertyModel::GetManuelGlobalGain()
{
    float fGlobalGain = 0.0f;
    int ret;
    RETRY_APC_API(ret, APC_GetGlobalGain(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                               m_pDeviceSelfInfo,
                                               SENSOR_BOTH,
                                               &fGlobalGain));

    return fGlobalGain;
}

void CCameraPropertyModel::SetManuelGlobalGain(float fGlobalGain)
{
    int ret;
    RETRY_APC_API(ret, APC_SetGlobalGain(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                               m_pDeviceSelfInfo,
                                               SENSOR_BOTH,
                                               fGlobalGain));
}

float CCameraPropertyModel::SetAETargetIndex(int index)
{
    float EV = 0.0f;
    int ret;
    RETRY_APC_API(ret, APC_SetAETarget(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                               m_pDeviceSelfInfo,
                                               index,
                                               &EV));

    return EV;
}
