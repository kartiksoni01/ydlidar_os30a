#ifndef CVIDEODEVICEMODEL_8052_H
#define CVIDEODEVICEMODEL_8052_H
#include "CVideoDeviceModel_8036_8052.h"

class CVideoDeviceModel_8052 : public CVideoDeviceModel_8036_8052
{
public:

    virtual bool InterleaveModeSupport(){ return true; }

    virtual int TransformDepthDataType(int nDepthDataType, bool bRectifyData);

    friend class CVideoDeviceModelFactory;

protected:
    CVideoDeviceModel_8052(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_8052_H