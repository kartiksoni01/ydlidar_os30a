#ifndef CVIDEODEVICEMODEL_8063_H
#define CVIDEODEVICEMODEL_8063_H
#include "CVideoDeviceModel_Kolor.h"

class CVideoDeviceModel_8063 : public CVideoDeviceModel_Kolor
{
public:
    virtual bool IsStreamAvailable();
    virtual int GetPointCloudInfo(eSPCtrl_RectLogData *pRectifyLogData, PointCloudInfo &pointCloudInfo,
                                  ImageData colorImageData, ImageData depthImageData);
    virtual int CloseDevice();
    virtual int GetColorImage(STREAM_TYPE type);
    virtual int GetImage(STREAM_TYPE type);
    virtual int ClosePreviewView();
    friend class CVideoDeviceModelFactory;
    virtual int OpenDevice();
    virtual int InitStreamInfoList();
    virtual int SetDepthDataType(int nDepthDataType);
    virtual int UpdateDepthDataType();
    virtual int PrepareOpenDevice();
    virtual int StartStreamingTask();
    virtual int PreparePointCloudInfo();
    virtual int AdjustZDTableIndex(int &nIndex);
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
    virtual std::vector<CIMUModel::INFO> GetIMUInfo(){
        return {
            {0x3438, 0x0166, CIMUModel::IMU_6_AXIS}
        };
    }
    virtual bool IMUSupport(){ return true; }
    virtual int GetCurrentTemperature(float& temperature);
protected:
    CVideoDeviceModel_8063(DEVSELINFO *pDeviceSelfInfo);
    unsigned short m_KolorDeviceDepthDataType;
private:
    int MergePointCloudInfoWithSlaveCameraMatrix(eSPCtrl_RectLogData& slaveRectifyLog, PointCloudInfo& pointCloudInfo);
};

#endif // CVIDEODEVICEMODEL_8063_H
