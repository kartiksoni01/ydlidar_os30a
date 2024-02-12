#ifndef CVIDEODEVICEMODEL_8053_H
#define CVIDEODEVICEMODEL_8053_H
#include "CVideoDeviceModel_8053_8059.h"

class CVideoDeviceModel_8053 : public CVideoDeviceModel_8053_8059
{
public:
    virtual int AdjustZDTableIndex(int &nIndex);
    virtual int AdjustRegister();

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_8053(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_8053_H
