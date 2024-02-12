#ifndef CVIDEODEVICEMODEL_8054_H
#define CVIDEODEVICEMODEL_8054_H
#include "CVideoDeviceModel_8040_8054.h"

class CVideoDeviceModel_8054 : public CVideoDeviceModel_8040_8054
{
public:
    virtual int AdjustZDTableIndex(int &nIndex);
    virtual int GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType = STREAM_DEPTH);

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_8054(DEVSELINFO *pDeviceSelfInfo);
    virtual std::vector<int> GetColorIndexListInCombineStream()
    {
        return { 0, 1, 2, 3, 6, 7, 8, 9, 11, 12, 13 };
    }
    virtual std::vector<int> GetDepthIndexListInCombineStream()
    {
        return { 3, 4, 5, 10, 13, 14 };
    }

    virtual unsigned short GetKolorDepthDataTypeValue(){ return 0x1; }

    virtual std::vector<CloudPoint> GeneratePointCloud(std::vector<unsigned char> &depthData,
                                                       std::vector<unsigned char> &colorData,
                                                       unsigned short nDepthWidth,
                                                       unsigned short nDepthHeight,
                                                       unsigned short nColorWidth,
                                                       unsigned short nColorHeight,
                                                       eSPCtrl_RectLogData rectifyLogData,
                                                       APCImageType::Value depthImageType,
                                                       int nZNear, int nZFar,
                                                       bool bUsePlyFilter = false,
                                                       std::vector<float> imgFloatBufOut = {});
};

#endif // CVIDEODEVICEMODEL_8054_H
