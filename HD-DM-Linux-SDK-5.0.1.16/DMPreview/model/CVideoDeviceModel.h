#ifndef CVIDEODEVICEMODEL_H
#define CVIDEODEVICEMODEL_H

#include "ColorPaletteGenerator.h"
#include "eSPDI_def.h"
#include "CCameraPropertyModel.h"
#include "PlyWriter.h"
#include "CIMUModel.h"
#include "CTaskInfoManager.h"
#include "CVideoDeviceModelFactory.h"
#include <string>
#include <QTime>
#include <QMutex>
#include <map>
#include "FrameGrabber.h"
#include "CMessageManager.h"

#define ERROR_HANDLE(ret, error_message) \
    do{ \
        if (!ret){ \
            CMessageManager::Error(error_message); \
            return APC_Init_Fail; \
        } \
    }while (0) \

#ifndef EOF
#define EOF (-1)
#endif

#ifndef RETRY_APC_API
#define RETRY_COUNT (5)
#define RETRY_APC_API(ret, func) \
    do{ \
        int retryCount = RETRY_COUNT; \
        while (retryCount > 0){ \
            ret = func; \
            if (APC_OK == ret) break; \
            --retryCount; \
        } \
    }while (false) \

#endif

#define MAX_STREAM_INFO_COUNT (64)
#define SERIAL_THRESHOLD (30)
#define MAX_DEPTH_DISTANCE (16383)

class CTaskInfo;
class CVideoDeviceController;
class CImageDataModel;
class CVideoDeviceModel
{
public:
    enum STATE{
        CLOSED,
        OPENED,
        STREAMING,
        RECONNECTING,
        RECONNECTED
    };

    enum STREAM_TYPE{
        STREAM_COLOR = 0,
        STREAM_COLOR_SLAVE,
        STREAM_KOLOR,
        STREAM_KOLOR_SLAVE,
        STREAM_TRACK,
        STREAM_DEPTH,
        STREAM_DEPTH_FUSION,
        STREAM_DEPTH_30mm,
        STREAM_DEPTH_60mm,
        STREAM_DEPTH_150mm,
        STREAM_RESERVED,
         //+[Thermal device]
        STREAM_THERMAL,
        //-[Thermal device]
        STREAM_BOTH,
        STREAM_TYPE_COUNT
    };

    enum IMAGE_TYPE{
        IMAGE_COLOR,
        IMAGE_DEPTH,
        IMAGE_DEPTH_FUSION
    };

    enum DEPTH_DATA_TYPE{
        DEPTH_DATA_8BIT,
        DEPTH_DATA_11BIT,
        DEPTH_DATA_14BIT
    };

    enum SERIAL_NUMBER_TYPE{
        FRAME_COUNT,
        SERIAL_COUNT
    };

    struct DeviceInfo{
        DEVINFORMATION deviceInfomation;
        std::string sFWVersion;
        std::string sSerialNumber;
        std::string sBusInfo;
        std::string sModelName;
    };

    struct ZDTableInfo{
        unsigned short nIndex = 0;
        unsigned short nTableSize = APC_ZD_TABLE_FILE_SIZE_11_BITS;
        unsigned short nZNear = 0;
        unsigned short nZFar  = 0;
        BYTE ZDTable[APC_ZD_TABLE_FILE_SIZE_11_BITS] = {0};
    };    

    struct ImageData{
        int nWidth = EOF;
        int nHeight = EOF;
        bool bMJPG = false;
        int depthDataType = EOF;
        APCImageType::Value imageDataType = APCImageType::IMAGE_UNKNOWN;
        std::vector<BYTE> imageBuffer;
    };

public:
    virtual int GetCurrentTemperature(float &temperature) { return APC_NotSupport; }
    virtual bool EqualModel(CVideoDeviceModel *pModel);

    virtual void SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController){
        m_pVideoDeviceController = pVideoDeviceController;
    }
    virtual CVideoDeviceController *GetVideoDeviceController(){ return m_pVideoDeviceController; }

    void AdjustDeviceSelfInfo(DEVSELINFO *pDeviceSelfInfo);

    virtual int Init();
    virtual int Reset();
    virtual int Update();

    virtual int DataVerification();

    virtual STATE GetState(){ return m_state; }
    virtual void ChangeState(STATE state);

    virtual int InitDeviceSelInfo();
    virtual std::vector<DEVSELINFO *> GetDeviceSelInfo() const;

    virtual int InitDeviceInformation();
    virtual std::vector<DeviceInfo> GetDeviceInformation() const;

    virtual int InitUsbType();
    virtual USB_PORT_TYPE GetUsbType();

    virtual bool IsColorWithDepthDevice(){ return false; }
    virtual int GetDepthIndexFromCombineStream(int nStreamIndex){ return EOF; }
    virtual int GetColorIndexFromCombineStream(int nStreamIndex){ return EOF; }
    virtual int GetCombineStreamIndexFromDepth(int nDepthIndex){ return EOF; }

    virtual bool IsStreamSupport(STREAM_TYPE type);
    virtual int InitStreamInfoList();
    virtual std::vector<APC_STREAM_INFO> GetStreamInfoList(STREAM_TYPE type);

    virtual int InitCameraproperty();
    virtual std::vector<CCameraPropertyModel *> GetCameraproperty();

    virtual int SetIRValue(unsigned short nValue);
    virtual int UpdateIR();
    virtual int GetIRRange(unsigned short &nMin, unsigned short &nMax);
    virtual unsigned short GetIRValue();

    virtual bool IRExtendSupport(){ return true; }
    virtual bool IsIRExtended();
    virtual int ExtendIR(bool bEnable);

    virtual SERIAL_NUMBER_TYPE GetSerialNumberType(){ return m_serialNumberType; }
    virtual int SetSerialNumberType(SERIAL_NUMBER_TYPE type);

    virtual bool IsDepthDataTypeSupport(DEPTH_DATA_TYPE type){ return true; }
    virtual int UpdateDepthDataType();
    virtual int NormalizeDepthDataType(int &nDepthDataType);
    virtual int TransformDepthDataType(int nDepthDataType, bool bRectifyData);
    virtual int TransformDepthDataType(int nDepthDataType);
    virtual int TransformDepthDataType(bool bRectifyData);
    virtual int SetDepthDataType(int nDepthDataType);
    virtual unsigned short GetDepthDataType();
    virtual bool IsRectifyData();
    virtual APCImageType::Value GetDepthImageType();

    virtual double GetCameraFocus(){ return 0.0; }
    virtual double GetCameraFOV(){ return 75.0f; }

    virtual bool HWPPSupprot(){ return false; }
    virtual bool IsHWPP();
    virtual int SetHWPP(bool bEnable);

    virtual int UpdateZDTable();
    virtual int AdjustZDTableIndex(int &nIndex){ return APC_OK; }
    virtual ZDTableInfo *GetZDTableInfo();

    virtual int GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType = STREAM_DEPTH);
    virtual eSPCtrl_RectLogData &GetRectifyLogData(STREAM_TYPE depthType){ return m_rectifyLogData; }

    virtual bool HasSlaveRectifyLogData(){ return false; }

    virtual int ModuleSync(){ return APC_OK; }
    virtual int ModuleSyncReset(){ return APC_OK; }
    virtual bool ModuleSyncSupport(){ return false; }

    virtual int FrameSync(){ return APC_OK; }
    virtual bool FrameSyncSupport(){ return false; }

    virtual bool InterleaveModeSupport(){ return false; }
    virtual bool IsInterleaveMode();
    virtual int AdjustInterleaveModeState();
    virtual std::vector<int> GetInterleaveModeFPS();
    virtual int SetInterleaveModeEnable(bool bEnable);

    virtual int InitIMU();
    virtual std::vector<CIMUModel::INFO> GetIMUInfo()
    {
        return {{0, 0, CIMUModel::IMU_UNKNOWN}};
    }
    virtual int ConfigIMU(){ return APC_OK; }
    virtual bool IMUSupport(){ return false; }
    virtual CIMUModel *GetIMUModel(){ return m_pIMUModel; }
    virtual void SetIMUSyncWithFrame(bool bSync);
    virtual bool IsIMUSyncWithFrame();

    virtual bool AudioSupport(){ return false; }

    virtual bool PlyFilterSupprot(){ return true; }    

    virtual bool IsStreamAvailable();

    int StartStreaming();
    virtual int PrepareOpenDevice();
    virtual int OpenDevice();
    virtual int StartStreamingTask();

    virtual int PreparePointCloudInfo();
    virtual int GetPointCloudInfo(eSPCtrl_RectLogData *pRectifyLogData, PointCloudInfo &pointCloudInfo,
                                  ImageData colorImageData, ImageData depthImageData);
    virtual int StartFrameGrabber();

    int StopStreaming(bool bRestart = false);
    virtual int StopFrameGrabber();
    virtual int StopStreamingTask();
    virtual int CloseDevice();
    virtual int ClosePreviewView();

    virtual int AdjustRegister();

    virtual int ConfigDepthFilter();

    virtual bool DepthAccuracySupport(){ return true; }

    int DoImageGrabber(CTaskInfo::TYPE type);

    virtual int GetColdResetThresholdMs(STREAM_TYPE type){ return m_nColdResetThresholdMs[type]; }
    virtual QTime GetLastestSuccessTime(STREAM_TYPE type){ return m_nLastestSuccessTime[type]; }
    virtual void SetLastestSuccessTime(STREAM_TYPE type, QTime time)
    {
        m_nLastestSuccessTime[type] = time;
    }

    virtual QTime GetColdeResetStartTime(){ return m_codeResetStartTime; }
    virtual void SetColdResetStartTime(QTime time){ m_codeResetStartTime = time; }

    virtual std::vector<CloudPoint> GeneratePointCloud(
            STREAM_TYPE depthType,
            std::vector<BYTE> &depthData, unsigned short nDepthWidth, unsigned short nDepthHeight,
            std::vector<BYTE> &colorData, unsigned short nColorWidth, unsigned short nColorHeight,
            bool bEnableFilter = true);


    static void FrameGrabberCallbackFn(std::vector<unsigned char>& bufDepth, int widthDepth, int heightDepth,
                                       std::vector<unsigned char>& bufColor, int widthColor, int heightColor,
                                       int serialNumber, void* pParam);

    friend class CVideoDeviceModelFactory;

protected:
    CVideoDeviceModel(DEVSELINFO *pDeviceSelfInfo);
    virtual ~CVideoDeviceModel();

    int SetPlumAR0330(bool bEnable);

    virtual int AddCameraPropertyModels();

    virtual int GetImage(STREAM_TYPE type);
    virtual int Get2Image(STREAM_TYPE type);
    virtual int GetColorImage();
    virtual int GetDepthImage();
    virtual int FirstSuccessGetImageCallback(STREAM_TYPE type);
    virtual int HandleGetImageResult(STREAM_TYPE type, int getImageResult);
    virtual int ProcessImage(STREAM_TYPE streamType,
                             int nImageSize, int nSerialNumber);
    virtual int ProcessImageCallback(STREAM_TYPE streamType,
                             int nImageSize, int nSerialNumber);
    virtual int UpdateFrameGrabberData(STREAM_TYPE streamType);

    virtual ImageData &GetColorImageData();

    virtual CImageDataModel *GetPreivewImageDataModel(STREAM_TYPE streamType);
public:
    virtual ImageData &GetDepthImageData();

    virtual void SerialCountToFrameCount(STREAM_TYPE streamType, int &nSerialNumber);


    int CreateStreamTask(STREAM_TYPE type);
    virtual int SetColdResetThresholdMs(STREAM_TYPE type, int ms){
        m_nColdResetThresholdMs[type] = ms;
        return APC_OK;
    }
    virtual int FirstOpenDeviceColdeRestThresholdMs(){ return 10 * 1000; }
    virtual int OpenDeviceColdeRestThresholdMs(){ return 1.5 * 1000; }

    DeviceInfo GetDeviceInformation(DEVSELINFO *pDeviceSelfInfo);

    virtual int PlyFilterTransform(std::vector<unsigned char> &depthData,
                                   std::vector<unsigned char> &colorData,
                                   unsigned short &nDepthWidth,
                                   unsigned short &nDepthHeight,
                                   unsigned short &nColorWidth,
                                   unsigned short &nColorHeight,
                                   std::vector<float> &imgFloatBufOut,
                                   eSPCtrl_RectLogData &rectifyLogData,
                                   APCImageType::Value depthImageType);

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

    virtual int ProcessFrameGrabberCallback(std::vector<unsigned char>& bufDepth, int widthDepth, int heightDepth,
                                            std::vector<unsigned char>& bufColor, int widthColor, int heightColor,
                                            int serialNumber);
    virtual int FrameGrabberDataTransform(std::vector<unsigned char>& bufDepth, int &widthDepth, int &heightDepth,
                                          std::vector<unsigned char>& bufColor, int &widthColor, int &heightColor,
                                          int &serialNumber){ return APC_OK; }
    virtual int DefaultVideoMode() { return 0; }
protected:

    STATE m_state;
    STATE m_restoreState;

    std::vector<DEVSELINFO *> m_deviceSelInfo;
    std::vector<DeviceInfo> m_deviceInfo;
    std::vector<APC_STREAM_INFO> m_streamInfo[STREAM_TYPE_COUNT];
    std::vector<CCameraPropertyModel *> m_cameraPropertyModel;
    USB_PORT_TYPE m_usbPortType;
    ZDTableInfo m_zdTableInfo;
    unsigned short m_nIRMax, m_nIRMin, m_nIRValue;
    unsigned short m_depthDataType;
    int m_nColdResetThresholdMs[STREAM_TYPE_COUNT];
    QTime m_nLastestSuccessTime[STREAM_TYPE_COUNT];

    std::vector<CTaskInfo *> m_taskInfoStorage;
    QTime m_codeResetStartTime;
    CTaskInfo * m_coldResetTask;
    ImageData m_imageData[STREAM_TYPE_COUNT];

    CVideoDeviceController *m_pVideoDeviceController;
    QMutex m_streamMutex[STREAM_TYPE_COUNT];

    QMutex m_serialCountMutex;
    std::map<STREAM_TYPE, int> m_mapSerialCountLast;
    std::map<STREAM_TYPE, int> m_mapSerialCountDiff;
    std::map<STREAM_TYPE, int> m_mapSerialCount2FrameCount;

    FrameGrabber *m_pFrameGrabber;
    eSPCtrl_RectLogData m_rectifyLogData;
    eSPCtrl_RectLogData mSlaveDeviceRectifyLogData = {{{0}}};

    PointCloudInfo m_pointCloudInfo;
    std::vector<float> m_pointCloudDepth;
    std::vector<BYTE> m_pointCloudColor;

    CIMUModel *m_pIMUModel;

    int m_bPrevLowLightValue;
    int m_nLastInterLeaveColorSerial;
    int m_nLastInterLeaveDepthSerial;

    SERIAL_NUMBER_TYPE m_serialNumberType;
};

#endif // CVIDEODEVICEMODEL_H
