#include "CCameraPropertyController.h"

CCameraPropertyController::CCameraPropertyController(CVideoDeviceModel *pVideoDeviceModel):
m_pVideoDeviceModel(pVideoDeviceModel),
m_nCurrentIndex(0)
{

}

CCameraPropertyController::~CCameraPropertyController()
{

}

void CCameraPropertyController::SelectCurrentCameraProperty(int nIndex)
{
    if((size_t)nIndex >= m_pVideoDeviceModel->GetCameraproperty().size()) return;

    m_nCurrentIndex = nIndex;
}

unsigned short CCameraPropertyController::GetCameraPropertyDeviceCount()
{
    return m_pVideoDeviceModel->GetCameraproperty().size();
}

QString CCameraPropertyController::GetCameraPropertyDeviceName(int nIndex)
{
    if((size_t)nIndex >= m_pVideoDeviceModel->GetCameraproperty().size()) return "";

    return m_pVideoDeviceModel->GetCameraproperty()[nIndex]->GetDeviceName();
}

bool CCameraPropertyController::IsCameraPropertySupport(CCameraPropertyModel::CAMERA_PROPERTY type)
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetCameraProperty(type).bSupport;
}

bool CCameraPropertyController::IsCameraPropertyValid(CCameraPropertyModel::CAMERA_PROPERTY type)
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetCameraProperty(type).bValid;
}

int CCameraPropertyController::GetRange(CCameraPropertyModel::CAMERA_PROPERTY type, int &nMin, int &nMax)
{
    CCameraPropertyModel::CameraPropertyItem item = m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetCameraProperty(type);

    nMin = item.nMin;
    nMax = item.nMax;

    return APC_OK;
}

int CCameraPropertyController::GetValue(CCameraPropertyModel::CAMERA_PROPERTY type, int &nValue)
{
    nValue = m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetCameraProperty(type).nValue;
    return APC_OK;
}

int CCameraPropertyController::SetValue(CCameraPropertyModel::CAMERA_PROPERTY type, int nValue)
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetCameraPropertyValue(type, nValue);
}

int CCameraPropertyController::ResetCameraProperty()
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetDefaultCameraProperty();
}

bool CCameraPropertyController::IsManuelExposureTimeValid()
{
    int nValue;
    GetValue(CCameraPropertyModel::AUTO_EXPOSURE, nValue);
    return CVideoDeviceModel::STREAMING == m_pVideoDeviceModel->GetState() &&
           nValue != 1;
}

float CCameraPropertyController::GetManuelExposureTimeMs()
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetManuelExposureTimeMs();
}

void CCameraPropertyController::SetManuelExposureTimeMs(float fMs)
{
    if (!IsManuelExposureTimeValid()) return;

    m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetManuelExposureTimeMs(fMs);
}

float CCameraPropertyController::GetManuelGlobalGain()
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetManuelGlobalGain();
}

void CCameraPropertyController::SetManuelGlobalGain(float fGlobalGain)
{
    if (!IsManuelExposureTimeValid()) return;

    m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetManuelGlobalGain(fGlobalGain);
}

float CCameraPropertyController::SetAETarget(int index)
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetAETargetIndex(index);
}
