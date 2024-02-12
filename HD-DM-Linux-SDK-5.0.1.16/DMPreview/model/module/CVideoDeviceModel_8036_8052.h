#ifndef CVIDEODEVICEMODEL_8036_8052_H
#define CVIDEODEVICEMODEL_8036_8052_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_8036_8052 : public CVideoDeviceModel
{
public:
    virtual int AdjustZDTableIndex(int &nIndex);
    virtual int StartStreamingTask();

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_8036_8052(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_8036_8052_H
