#ifndef CVIDEODEVICEMODEL_HYPATIA2_H
#define CVIDEODEVICEMODEL_HYPATIA2_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_Hypatia2 : public CVideoDeviceModel
{
public:
    virtual int TransformDepthDataType(int nDepthDataType, bool bRectifyData);

    friend class CVideoDeviceModelFactory;

protected:
    CVideoDeviceModel_Hypatia2(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_HYPATIA2_H