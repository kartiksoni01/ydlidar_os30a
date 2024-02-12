#ifndef CVIDEODEVICEMODEL_8073_H
#define CVIDEODEVICEMODEL_8073_H
#include<CVideoDeviceModel.h>

class CVideoDeviceModel_8073 : public CVideoDeviceModel
{
public:
    virtual bool InterleaveModeSupport(){ return true; }
    friend class CVideoDeviceModelFactory;
    int TransformDepthDataType(int nDepthDataType, bool bRectifyData);
    virtual int UpdateIR();
    virtual bool IRExtendSupport(){ return false; }
    virtual bool IsIRExtended(){ return false; }
    virtual int ExtendIR(bool bEnable){ return APC_NotSupport; }
    virtual int StartStreamingTask();
    virtual void SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController);
    virtual int DefaultVideoMode(){ return 11; }
protected:
    CVideoDeviceModel_8073(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_8073_H
