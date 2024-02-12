#ifndef CVIDEODEVICEMODEL_HYPATIA_H
#define CVIDEODEVICEMODEL_HYPATIA_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_Hypatia : public CVideoDeviceModel
{
public:
    virtual void SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController);

    virtual int UpdateIR();
    virtual bool IRExtendSupport(){ return false; }
    virtual bool IsIRExtended(){ return false; }
    virtual int ExtendIR(bool bEnable){ return APC_NotSupport; }

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_Hypatia(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_HYPATIA_H
