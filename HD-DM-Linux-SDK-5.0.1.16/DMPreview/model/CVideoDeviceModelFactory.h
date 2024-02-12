#ifndef CVIDEODEVICEMODELFACTORY_H
#define CVIDEODEVICEMODELFACTORY_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModelFactory
{
public:
    static CVideoDeviceModel *CreateVideoDeviceModel(DEVSELINFO *pDeviceSelfInfo);
    static void ReleaseModels(std::vector<CVideoDeviceModel *> models);

private:
    CVideoDeviceModelFactory(){}
    ~CVideoDeviceModelFactory(){}
};

#endif // CVIDEODEVICEMODELFACTORY_H
