#ifndef CVIDEODEVICEMODEL_8040_H
#define CVIDEODEVICEMODEL_8040_H
#include "CVideoDeviceModel_8040_8054.h"

class CVideoDeviceModel_8040 : public CVideoDeviceModel_8040_8054
{
public:
    virtual int Reset();

    virtual int GetIRRange(unsigned short &nMin, unsigned short &nMax);
    virtual bool IRExtendSupport(){ return false; }

    virtual int AdjustZDTableIndex(int &nIndex);
    virtual int GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType = STREAM_DEPTH);
    virtual int GetPointCloudInfo(eSPCtrl_RectLogData *pRectifyLogData, PointCloudInfo &pointCloudInfo,
                                  ImageData colorImageData, ImageData depthImageData);

    virtual bool IMUSupport(){ return true; }
    virtual std::vector<CIMUModel::INFO> GetIMUInfo(){
        return {
            {0x0483, 0x5710, CIMUModel::IMU_6_AXIS},
            {0x1E4E, 0x0154, CIMUModel::IMU_6_AXIS}
        };
    }

    virtual double GetCameraFOV(){ return 120.0f; }

    virtual bool DepthAccuracySupport(){ return false; }

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_8040(DEVSELINFO *pDeviceSelfInfo);
    virtual ~CVideoDeviceModel_8040();

    virtual std::vector<int> GetColorIndexListInCombineStream()
    {
        return { 0, 1, 2, 3, 6, 7, 8, 9, 11, 12, 13 };
    }
    virtual std::vector<int> GetDepthIndexListInCombineStream()
    {
        return { 3, 4, 5, 9, 10, 13, 14 };
    }

    virtual unsigned short GetKolorDepthDataTypeValue(){ return 0x0; }

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

#endif // CVIDEODEVICEMODEL_8040_H
