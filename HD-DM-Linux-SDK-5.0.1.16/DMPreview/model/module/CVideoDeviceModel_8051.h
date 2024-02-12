#ifndef CVIDEODEVICEMODEL_8051_H
#define CVIDEODEVICEMODEL_8051_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_8051 : public CVideoDeviceModel
{
public:
    virtual int AdjustZDTableIndex(int &nIndex);
    virtual int GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType = STREAM_DEPTH);
    virtual double GetCameraFOV(){ return 30.0f; }

    friend class CVideoDeviceModelFactory;

protected:
    CVideoDeviceModel_8051(DEVSELINFO *pDeviceSelfInfo);


};
#endif // CVIDEODEVICEMODEL_8051_H
