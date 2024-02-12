#ifndef CVIDEODEVICEMODEL_8053_8059_H
#define CVIDEODEVICEMODEL_8053_8059_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_8053_8059 : public CVideoDeviceModel
{
public:

    virtual int ModuleSync();
    virtual int ModuleSyncReset();
    virtual bool ModuleSyncSupport();

    virtual bool InterleaveModeSupport(){ return true; }

    virtual int PrepareOpenDevice();
    virtual int StartStreamingTask();

    friend class CVideoDeviceModelFactory;

protected:
    virtual int ProcessImageCallback(STREAM_TYPE streamType,
                             int nImageSize, int nSerialNumber);
protected:
    CVideoDeviceModel_8053_8059(DEVSELINFO *pDeviceSelfInfo);

private:

    int m_bPrevLowLightValue;
    int m_nLastInterLeaveColorSerial;
    int m_nLastInterLeaveDepthSerial;
};

#endif // CVIDEODEVICEMODEL_8053_8059_H
