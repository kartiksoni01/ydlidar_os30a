#ifndef CVIDEODEVICEMODEL_8037_H
#define CVIDEODEVICEMODEL_8037_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_8037 : public CVideoDeviceModel
{
public:
    virtual int AdjustZDTableIndex(int &nIndex);

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_8037(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_8037_H
