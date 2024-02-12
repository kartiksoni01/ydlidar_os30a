#ifndef CVIDEODEVICEMODEL_8062_H
#define CVIDEODEVICEMODEL_8062_H
#include "CVideoDeviceModel_8053_8059.h"

class CVideoDeviceModel_8062 : public CVideoDeviceModel_8053_8059
{
public:
    virtual int AdjustZDTableIndex(int &nIndex);
    virtual int GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType = STREAM_DEPTH);
    virtual bool IMUSupport(){ return true; }
    virtual std::vector<CIMUModel::INFO> GetIMUInfo(){
        return {
            {0x1E4E, 0x0163, CIMUModel::IMU_9_AXIS}
        };
    }

    virtual double GetCameraFOV(){ return 100.0f; }
    virtual int ConfigIMU();

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_8062(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_8062_H
