#ifndef CDEPTHACCURACYCONTROLLER_H
#define CDEPTHACCURACYCONTROLLER_H
#include "CVideoDeviceController.h"

class CDepthAccuracyController
{
public:
    struct DepthAccuracyInfo{
        float fDistance = 0.0f;
        float fFillRate = 0.0f;
        float fZAccuracy = 0.0f;
        float fTemporalNoise = 0.0f;
        float fSpatialNoise = 0.0f;
        float fAngle = 0.0f;
        float fAngleX = 0.0f;
        float fAngleY = 0.0f;
    };

public:
    CDepthAccuracyController(CVideoDeviceController *pVideoDeviceController);

    std::vector<CVideoDeviceModel::STREAM_TYPE> GetDepthAccuracyList();

    bool IsValid();
    void EnableDepthAccuracy(bool bEnable);
    void SelectDepthAccuracyStreamType(CVideoDeviceModel::STREAM_TYPE streamType);
    void SetAccuracyRegionRatio(float fRatio);
    void SetGroundTruthDistanceMM(float mm);

    DepthAccuracyInfo GetDepthAccuracyInfo();
private:
    CVideoDeviceController *m_pVideoDeviceController;
    CVideoDeviceModel::STREAM_TYPE m_accuracyStreamType;

    float m_fAccuracyRegionRatio;
    float m_fGroundTruthDistanceMM;

};

#endif // CDEPTHACCURACYCONTROLLER_H
