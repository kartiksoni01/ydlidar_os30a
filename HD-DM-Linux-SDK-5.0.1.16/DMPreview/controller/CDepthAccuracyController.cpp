#include "CDepthAccuracyController.h"
#include "CImageDataModel.h"
#include "CEYSDDeviceManager.h"

CDepthAccuracyController::CDepthAccuracyController(CVideoDeviceController *pVideoDeviceController):
m_pVideoDeviceController(pVideoDeviceController),
m_accuracyStreamType(CVideoDeviceModel::STREAM_DEPTH),
m_fAccuracyRegionRatio(0.0f),
m_fGroundTruthDistanceMM(0.0f)
{

}


std::vector<CVideoDeviceModel::STREAM_TYPE> CDepthAccuracyController::GetDepthAccuracyList()
{
    CVideoDeviceModel::STREAM_TYPE depthType[] = {
                                                    CVideoDeviceModel::STREAM_DEPTH,
                                                    CVideoDeviceModel::STREAM_DEPTH_30mm,
                                                    CVideoDeviceModel::STREAM_DEPTH_60mm,
                                                    CVideoDeviceModel::STREAM_DEPTH_150mm,
                                                    /*CVideoDeviceModel::STREAM_DEPTH_FUSION,*/
                                                  };

    std::vector<CVideoDeviceModel::STREAM_TYPE> accuracyDepthList;
    for (CVideoDeviceModel::STREAM_TYPE type : depthType){
        if (m_pVideoDeviceController->GetControlView()->GetPreviewImageData(type)){
            accuracyDepthList.push_back(type);
        }
    }

    return accuracyDepthList;
}

bool CDepthAccuracyController::IsValid()
{
    CVideoDeviceModel::STREAM_TYPE depthType[] = {
                                                    CVideoDeviceModel::STREAM_DEPTH,
                                                    CVideoDeviceModel::STREAM_DEPTH_30mm,
                                                    CVideoDeviceModel::STREAM_DEPTH_60mm,
                                                    CVideoDeviceModel::STREAM_DEPTH_150mm,
                                                    /*CVideoDeviceModel::STREAM_DEPTH_FUSION,*/
                                                  };
    bool bValid = false;
    for (CVideoDeviceModel::STREAM_TYPE type : depthType){
        if (m_pVideoDeviceController->GetControlView()->GetPreviewImageData(type)) {
            bValid = true;
            break;
        }
    }

    return bValid && CVideoDeviceModel::STREAMING == m_pVideoDeviceController->GetVideoDeviceModel()->GetState();
}

void CDepthAccuracyController::EnableDepthAccuracy(bool bEnable)
{
    CImageDataModel_Depth *pDepthImageData = static_cast<CImageDataModel_Depth *>(m_pVideoDeviceController->GetControlView()->GetPreviewImageData(m_accuracyStreamType));
    if (!pDepthImageData) return;

    pDepthImageData->EnableDepthAccruacy(bEnable);
    pDepthImageData->SetDepthAccuracyRegionRatio(bEnable ? m_fAccuracyRegionRatio : 0.0f);
    pDepthImageData->SetDepthAccuracyGroundTruthDistanceMM(bEnable ? m_fGroundTruthDistanceMM : 0.0f);
}

void CDepthAccuracyController::SelectDepthAccuracyStreamType(CVideoDeviceModel::STREAM_TYPE streamType)
{
    if(m_accuracyStreamType == streamType) return;

    EnableDepthAccuracy(false);
    m_accuracyStreamType = streamType;
    EnableDepthAccuracy(true);
}

void CDepthAccuracyController::SetAccuracyRegionRatio(float fRatio)
{
    m_fAccuracyRegionRatio = fRatio;

    CImageDataModel_Depth *pDepthImageData = static_cast<CImageDataModel_Depth *>(m_pVideoDeviceController->GetControlView()->GetPreviewImageData(m_accuracyStreamType));
    if (!pDepthImageData) return;

    pDepthImageData->SetDepthAccuracyRegionRatio(fRatio);
}

void CDepthAccuracyController::SetGroundTruthDistanceMM(float mm)
{
    m_fGroundTruthDistanceMM = mm;

    CImageDataModel_Depth *pDepthImageData = static_cast<CImageDataModel_Depth *>(m_pVideoDeviceController->GetControlView()->GetPreviewImageData(m_accuracyStreamType));
    if (!pDepthImageData) return;

    pDepthImageData->SetDepthAccuracyGroundTruthDistanceMM(mm);

}

CDepthAccuracyController::DepthAccuracyInfo CDepthAccuracyController::GetDepthAccuracyInfo()
{
    DepthAccuracyInfo info;
    CImageDataModel_Depth *pDepthImageData = static_cast<CImageDataModel_Depth *>(m_pVideoDeviceController->GetControlView()->GetPreviewImageData(m_accuracyStreamType));
    if(!pDepthImageData) return info;

    info.fDistance = pDepthImageData->GetDepthAccuracyDistanceMM();
    info.fFillRate = pDepthImageData->GetDepthAccuracyFillRate();
    info.fZAccuracy = pDepthImageData->GetDepthAccuracyZAccuracy();
    info.fSpatialNoise = pDepthImageData->GetDepthSpatialNoise();
    info.fAngle = pDepthImageData->GetDepthAngle();
    info.fAngleX = pDepthImageData->GetDepthAngleX();
    info.fAngleY = pDepthImageData->GetDepthAngleY();
    info.fTemporalNoise = pDepthImageData->GetDepthTemporaNoise();

    return info;
}
