#ifndef CVIDEODEVICEMODEL_KOLOR_H
#define CVIDEODEVICEMODEL_KOLOR_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_Kolor : public virtual CVideoDeviceModel
{
public:
    virtual int InitDeviceSelInfo();
    virtual int InitDeviceInformation();

    virtual bool IsStreamSupport(STREAM_TYPE type);
    virtual int InitStreamInfoList();

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_Kolor(DEVSELINFO *pDeviceSelfInfo);

    virtual int AddCameraPropertyModels();
    virtual ImageData &GetColorImageData();
};

#endif // CVIDEODEVICEMODEL_KOLOR_H
