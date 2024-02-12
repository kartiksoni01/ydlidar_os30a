#ifndef CVIDEODEVICEMODEL_8038_H
#define CVIDEODEVICEMODEL_8038_H
#include "CVideoDeviceModel_ColorWithDepth.h"

#define EX8038_MIN_DEPTH_RANGE (94)
class CDepthFusionHelper;
class CVideoDeviceModel_8038 : public CVideoDeviceModel_ColorWithDepth
{
public:
    virtual void SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController);

    virtual int InitDeviceSelInfo();
    virtual int InitDeviceInformation();

    virtual double GetCameraFocus(){ return 800.0; }

    virtual int GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType);
    virtual eSPCtrl_RectLogData &GetRectifyLogData(STREAM_TYPE depthType);

    virtual bool IsDepthDataTypeSupport(DEPTH_DATA_TYPE type);
    virtual int UpdateZDTable();
    virtual int AdjustRegister();

    virtual int PreparePointCloudInfo();
    virtual int PrepareOpenDevice();
    virtual int OpenDevice();
    virtual int StartStreamingTask();

    virtual int CloseDevice();
    virtual int ClosePreviewView();
    virtual int StopStreamingTask();

    virtual int GetImage(STREAM_TYPE type);

    static void DepthFusionCallbackFn(unsigned char* depthBuf, unsigned char* selectedIndex, int depthSize, int width, int height, int serialNumber, void* pParam);
    friend class CVideoDeviceModelFactory;
protected:

    CVideoDeviceModel_8038(DEVSELINFO *pDeviceSelfInfo);
    virtual ~CVideoDeviceModel_8038();

    virtual void ProcessDepthFusion(unsigned char* depthBuf, unsigned char* selectedIndex, int depthSize, int width, int height, int serialNumber);

    virtual std::vector<int> GetColorIndexListInCombineStream()
    {
        return { 0 }  ;
    }
    virtual std::vector<int> GetDepthIndexListInCombineStream()
    {
        return { 0, 1 };
    }

    virtual int GetColorImage();
    virtual int GetMultiBaselineDepthImage(STREAM_TYPE type);
    virtual int ProcessImageCallback(STREAM_TYPE streamType,
                             int nImageSize, int nSerialNumber);

    virtual CImageDataModel *GetPreivewImageDataModel(STREAM_TYPE streamType);

private:
    CDepthFusionHelper *m_pDepthFusionHelper;
    std::vector<BYTE> m_depthFusionBuffer;

    eSPCtrl_RectLogData m_rectifyLogData_150mm;
    eSPCtrl_RectLogData m_rectifyLogData_60mm;
};

#endif // CVIDEODEVICEMODEL_8038_H
