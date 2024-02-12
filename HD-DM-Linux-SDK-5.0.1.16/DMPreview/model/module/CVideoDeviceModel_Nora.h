#ifndef CVIDEODEVICEMODEL_NORA_H
#define CVIDEODEVICEMODEL_NORA_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_Nora : public CVideoDeviceModel
{
public:
    virtual void SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController);
    virtual bool InterleaveModeSupport(){ return true; }
    virtual int UpdateIR();
    virtual int AdjustZDTableIndex(int &nIndex);
    virtual bool IsIRExtended();
    virtual int ExtendIR(bool bEnable);

    friend class CVideoDeviceModelFactory;

protected:
    CVideoDeviceModel_Nora(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_NORA_H
