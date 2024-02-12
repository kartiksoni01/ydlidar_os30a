#ifndef CVIDEODEVICEMODEL_8036_H
#define CVIDEODEVICEMODEL_8036_H
#include "CVideoDeviceModel_8036_8052.h"

class CVideoDeviceModel_8036 : public CVideoDeviceModel_8036_8052
{
public:

    virtual int Init();

    virtual bool InterleaveModeSupport(){ return true; }

    virtual int TransformDepthDataType(int nDepthDataType, bool bRectifyData);

    virtual int GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType = STREAM_DEPTH);

    friend class CVideoDeviceModelFactory;

protected:
    CVideoDeviceModel_8036(DEVSELINFO *pDeviceSelfInfo);

private:
    bool m_bIsInterleaveSupport;
    bool m_bIsScaleDown;
};

#endif // CVIDEODEVICEMODEL_8036_H
