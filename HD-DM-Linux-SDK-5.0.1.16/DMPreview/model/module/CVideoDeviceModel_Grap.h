#ifndef CVIDEODEVICEMODEL_GRAP_H
#define CVIDEODEVICEMODEL_GRAP_H
#include "CVideoDeviceModel.h"
//+[Thermal device]
#if defined(THERMAL_SENSOR)
#include "thermal/v4l2.h"
#include "measure.h"
#include "mapping.h"
#endif
//-[Thermal device]

class CVideoDeviceModel_Grap : public CVideoDeviceModel
{
public:
    virtual int SetDepthDataType(int nDepthDataType){ return APC_NotSupport;}
    virtual bool IsDepthDataTypeSupport(DEPTH_DATA_TYPE type){ return false; }
    virtual int UpdateDepthDataType(){ return APC_NotSupport;}

    virtual int SetIRValue(unsigned short nValue){ return APC_NotSupport;}
    virtual int UpdateIR(){ return APC_NotSupport;}
    virtual int GetIRRange(unsigned short &nMin, unsigned short &nMax){ return APC_NotSupport;}
    virtual unsigned short GetIRValue(){ return 0;}

    virtual bool IRExtendSupport(){ return false; }
    virtual bool IsIRExtended(){ return false; }
    virtual int ExtendIR(bool bEnable){ return APC_NotSupport; }

    virtual bool HWPPSupprot(){ return false; }
    virtual bool IsHWPP(){ return false; }
    virtual int SetHWPP(bool bEnable){ return APC_NotSupport; }

    virtual int InitDeviceSelInfo();
    virtual int InitDeviceInformation();

    virtual bool IsStreamSupport(STREAM_TYPE type);
    virtual int InitStreamInfoList();

    virtual bool IsStreamAvailable();

    virtual int PrepareOpenDevice();
    virtual int OpenDevice();
    virtual int StartStreamingTask();

    virtual int CloseDevice();
    virtual int ClosePreviewView();

    virtual int GetImage(STREAM_TYPE type);

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_Grap(DEVSELINFO *pDeviceSelfInfo);

    virtual int GetColorImage(STREAM_TYPE type);
    //+[Thermal device]
    #if defined(THERMAL_SENSOR)
    int GetThermalImage(STREAM_TYPE type, int video_w,v4l2 device,short *curve,guide_measure_external_param_t *pParamExt);
    int Open_thermal_camera();
    
    v4l2 device;
    int video_w = 0;
    int video_h = 0;
    int FD = -1;
    short *curve;
    bool isSupportThermal = false;
    guide_measure_external_param_t *pParamExt = nullptr;

    float *pSurfaceTemper = nullptr;
    float *pInternalTemper = nullptr;
    bool isFirstLoadCurve = true;
    const char *paramlinedata = "paramlineData.raw";
    #endif
    //-[Thermal device]
};

#endif // CVIDEODEVICEMODEL_GRAP_H
