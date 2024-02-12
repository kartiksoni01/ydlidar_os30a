#ifndef CVIDEODEVICEMODEL_8029_H
#define CVIDEODEVICEMODEL_8029_H

#include "CVideoDeviceModel.h"

class CVideoDeviceModel_8029 : public CVideoDeviceModel
{
public:
    virtual int InitUsbType();

    virtual int SetIRValue(unsigned short nValue);
    virtual int UpdateIR();
    virtual bool IRExtendSupport(){ return false; }
    virtual bool HWPPSupprot(){ return false; }

    virtual int AdjustRegister();

    virtual int ConfigDepthFilter();

    virtual int OpenDevice();
    virtual int GetPointCloudInfo(eSPCtrl_RectLogData *pRectifyLogData, PointCloudInfo &pointCloudInfo,
                                  ImageData colorImageData, ImageData depthImageData);
    friend class CVideoDeviceModelFactory;
private:
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

    virtual int FrameGrabberDataTransform(std::vector<unsigned char>& bufDepth, int &widthDepth, int &heightDepth,
                                          std::vector<unsigned char>& bufColor, int &widthColor, int &heightColor,
                                          int &serialNumber);
protected:
    CVideoDeviceModel_8029(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_8029_H
