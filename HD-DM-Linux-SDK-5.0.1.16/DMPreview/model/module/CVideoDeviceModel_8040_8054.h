#ifndef CVIDEODEVICEMODEL_8040_8054_H
#define CVIDEODEVICEMODEL_8040_8054_H
#include "CVideoDeviceModel_ColorWithDepth.h"
#include "CVideoDeviceModel_Kolor.h"
#include <unordered_map>

class CVideoDeviceModel_8040_8054 : public CVideoDeviceModel_ColorWithDepth,
                                    public CVideoDeviceModel_Kolor
{
public:

    virtual int InitDeviceInformation();

    virtual bool IsStreamAvailable();
    virtual bool IsDepthDataTypeSupport(DEPTH_DATA_TYPE type);    

    virtual int GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType = STREAM_DEPTH);
    virtual bool HasSlaveRectifyLogData(){ return true; }

    virtual int PrepareOpenDevice();
    virtual int OpenDevice();
    virtual int StartStreamingTask();
    virtual int GetPointCloudInfo(eSPCtrl_RectLogData *pRectifyLogData, PointCloudInfo &pointCloudInfo,
                                  ImageData colorImageData, ImageData depthImageData);

    virtual int CloseDevice();
    virtual int ClosePreviewView();

    virtual bool FrameSyncSupport(){ return true; }
    virtual int FrameSync();

    virtual int GetImage(STREAM_TYPE type);

    virtual int ProcessFrameGrabberCallback(std::vector<unsigned char>& bufDepth, int widthDepth, int heightDepth,
                                            std::vector<unsigned char>& bufColor, int widthColor, int heightColor,
                                            int serialNumber);

    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_8040_8054(DEVSELINFO *pDeviceSelfInfo);

    virtual int GetColorImage();
    virtual int GetKolorImage();
    virtual unsigned short GetKolorDepthDataTypeValue(){ return 0; }

    virtual int OpenDeviceColdeRestThresholdMs(){ return 5 * 1000; }
    virtual int FirstSuccessGetImageCallback(STREAM_TYPE type);

    virtual int UpdateFrameGrabberData(STREAM_TYPE streamType);
private:
    std::vector<BYTE> m_rotateBuffer;
    std::vector<BYTE> m_mirroBuffer;
    QMutex m_colorMutex;
    QMutex _depthMutex;
};

#endif // CVIDEODEVICEMODEL_8040_8054_H
