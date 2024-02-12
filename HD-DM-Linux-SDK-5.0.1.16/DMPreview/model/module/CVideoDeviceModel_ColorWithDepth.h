#ifndef CVIDEODEVICEMODEL_COLORWITHDEPTH_H
#define CVIDEODEVICEMODEL_COLORWITHDEPTH_H
#include "CVideoDeviceModel.h"
#include <unordered_map>

#define COLOR_STEAM_MASK_COLOR (0x1)
#define COLOR_STEAM_MASK_DEPTH (0x2)

class CVideoDeviceModel_ColorWithDepth : public virtual CVideoDeviceModel
{
public:
    virtual int Init();

    virtual bool IsColorWithDepthDevice(){ return true; }
    virtual int GetDepthIndexFromCombineStream(int nStreamIndex);
    virtual int GetColorIndexFromCombineStream(int nStreamIndex);
    virtual int GetCombineStreamIndexFromDepth(int nDepthIndex);

    virtual int TransformDepthDataType(int nDepthDataType, bool bRectifyData);
    virtual int PrepareOpenDevice();    

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_ColorWithDepth(DEVSELINFO *pDeviceSelfInfo);
    virtual int InitColorStreamMask();
    virtual std::vector<int> GetColorIndexListInCombineStream() = 0;
    virtual std::vector<int> GetDepthIndexListInCombineStream() = 0;

    virtual int UpdateFrameGrabberData(STREAM_TYPE streamType);

protected:
    ImageData m_colorWithDepthImageData;
    std::unordered_map<int, BYTE> m_colorStreamMask;
};

#endif // CVIDEODEVICEMODEL_COLORWITHDEPTH_H
