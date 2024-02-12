#ifndef CVIDEODEVICEMODEL_8060_H
#define CVIDEODEVICEMODEL_8060_H
#include "CVideoDeviceModel_Kolor.h"

class CVideoDeviceModel_8060 : public CVideoDeviceModel_Kolor
{
public:
    virtual int Reset();

    virtual int InitDeviceSelInfo();
    virtual int InitDeviceInformation();

    virtual bool IsStreamSupport(STREAM_TYPE type);
    virtual int InitStreamInfoList();

    virtual bool IsStreamAvailable();

    virtual int GetIRRange(unsigned short &nMin, unsigned short &nMax);
    virtual bool IRExtendSupport(){ return false; }

    virtual int AdjustZDTableIndex(int &nIndex);

    virtual bool IMUSupport(){ return true; }
    virtual std::vector<CIMUModel::INFO> GetIMUInfo(){
        return {
            {0x0483, 0x5711, CIMUModel::IMU_9_AXIS}
        };
    }

    virtual bool AudioSupport();

    virtual int PrepareOpenDevice();
    virtual int OpenDevice();
    virtual int StartStreamingTask();

    virtual int CloseDevice();
    virtual int ClosePreviewView();

    virtual int GetImage(STREAM_TYPE type);

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_8060(DEVSELINFO *pDeviceSelfInfo);

    virtual int AddCameraPropertyModels();
    virtual int GetColorImage(STREAM_TYPE type);
};

#endif // CVIDEODEVICEMODEL_8060_H
