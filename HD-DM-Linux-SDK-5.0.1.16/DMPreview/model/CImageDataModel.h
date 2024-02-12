#ifndef CIMAGEDATAMODEL_H
#define CIMAGEDATAMODEL_H
#include "eSPDI_def.h"
#include "CVideoDeviceModel.h"
#include <vector>
#include <list>
#include <QString>
#include <QTime>
#include <QRect>
#include <QVector3D>
#include "FPSCalculator.h"

class CImageDataModel
{
public:
    enum TYPE{
        COLOR,
        DEPTH,
        NONE
    };

public:
    CImageDataModel(CVideoDeviceModel::STREAM_TYPE streamType, TYPE modelType, CVideoDeviceController *pVideoDeviceController);
    virtual ~CImageDataModel();

    virtual bool SetImageInfo(APCImageType::Value imageType,
                              int nWidth, int nHeight);
    int GetWidth(){ return m_nWidth; }
    int GetHeight(){ return m_nHeight; }
    int GetSerialNumber(){ return m_nSerialNumber; }
    int GetRawDataBytePerPixel();
    TYPE GetModelType(){ return m_type; }

    APCImageType::Value GetImageType(){ return m_imageType; }

    std::vector<BYTE> &GetRawData(){ return m_rawData; }
    std::vector<BYTE> &GetRGBData(){ return m_rgbData; }

    virtual void SetSpecificPixel(int x, int y)
    {
        m_nSpecificX = x;
        m_nSpecificY = y;
    }

    virtual QString GetImgaeDataInfo();

    virtual int SetRawData(BYTE *pData, int nDataSize, int nSerialNumber);
    virtual int TransformRawToRGB(){ return APC_OK; }
    virtual int SetUserData(void *pUserData);

    QMutex &GetDataMutex(){ return m_dataMutex; }

protected:

    CVideoDeviceController *m_pVideoDeviceController;

    TYPE m_type;

    std::vector<BYTE> m_rawData;
    std::vector<BYTE> m_rgbData;

    CVideoDeviceModel::STREAM_TYPE m_streamType;
    APCImageType::Value m_imageType;

    std::deque<QTime> m_receiveTimeStorage;

    QString m_sDataName;

    FPSCalculator m_fpsCalculator;

    int m_nWidth;
    int m_nHeight;
    int m_nDataSize;
    int m_nSerialNumber;
    int m_nSpecificX, m_nSpecificY;

    void *m_pUserData;

    QMutex m_dataMutex;
    CTaskInfo *m_pDataTransformTask;
};

class CImageDataModel_Color : public CImageDataModel
{
public:
    CImageDataModel_Color(CVideoDeviceModel::STREAM_TYPE streamType, CVideoDeviceController *pVideoDeviceController);
    virtual ~CImageDataModel_Color();
    virtual int TransformRawToRGB();
};

#define UPDATE_DEPTH_VALUE_MS (1000 / 3)
#define TemporalNoiseCount (9)
class CVideoDeviceController;
class CImageDataModel_Depth : public CImageDataModel
{
public:
    enum COLOR_PALETTE{
        COLOR_PALETTE_RGB = 0,
        COLOR_PALETTE_GRAY,
        COLOR_PALETTE_COUNT
    };

public:
    CImageDataModel_Depth(CVideoDeviceModel::STREAM_TYPE streamType,
                          CVideoDeviceController *pVideoDeviceController,
                          double dblCamFocus = 0.0,
                          double dblBaselineDist = 0.0);
    virtual ~CImageDataModel_Depth();

    virtual bool SetImageInfo(APCImageType::Value imageType, int nWidth,
                              int nHeight);
    virtual int SetRawData(BYTE *pData, int nDataSize, int nSerialNumber);
    virtual int TransformRawToRGB();
    virtual QString GetImgaeDataInfo();
    virtual void UpdateColorPalette(int nZNear, int nZFar);

    void DepthFilter(BYTE *pData);

    void EnableDepthAccruacy(bool bEnable);
    bool IsDepthAccuracyEnable(){ return m_bEnableDepthAccuracy; }

    void SetDepthAccuracyRegionRatio(float fRatio){
        if ( fRatio > 1.0f ) return;
        m_fDepthAccuracyRegionRatio = fRatio;
    }
    float GetDepthAccuracyRegionRatio(){ return m_fDepthAccuracyRegionRatio; }

    void SetDepthAccuracyGroundTruthDistanceMM(float mm){
        m_fDepthAccuracyGroundTruthDistanceMM = mm;
        AdjustDepthInvalidBandPixel();
    }
    float GetDepthAccuracyGroundTruthDistanceMM(){ return m_fDepthAccuracyGroundTruthDistanceMM; }

    float GetDepthAccuracyFillRate(){ return m_fDepthAccuracyFillRate; }
    float GetDepthAccuracyDistanceMM(){ return m_fDepthAccuracyDistanceMM; }
    float GetDepthAccuracyZAccuracy(){ return m_fDepthAccuracyZAccuracy; }
    float GetDepthSpatialNoise(){ return m_fDepthSpatialNoise; }
    float GetDepthAngle(){ return m_fDepthAngle; }
    float GetDepthAngleX(){ return m_fDepthAngleX; }
    float GetDepthAngleY(){ return m_fDepthAngleY; }
    float GetDepthTemporaNoise(){ return m_fDepthTemporaNoise; }

    void CalculateDepthAccuracyInfo();
    void CalculateDepthSpatialNoise();
    void CalculateDepthTemporaNoise();

    QRect GetDepthAccuracyRegion();

private:
    void UpdateDepth();
    void UpdateZVaule();
    unsigned short GetDepth(int nX, int nY);
    unsigned short GetZValue(int nDepth);

    std::vector< WORD > GetDepthZOfROI(int &nWidth, int &nHeight);
    void CalculateFittedPlane(double &a, double &b, double &d,
                              std::vector< WORD > &vecDepthZ, int nWidth, int nHeight);

    double CalculateZAccuracy(std::vector< WORD > &vecDepthZ,
                              int nWidth, int nHeight,
                              double grandtrue,
                              QVector3D vecBefore, QVector3D vecAfter);

    void SortZ(std::vector< WORD > &vecDepthZ, double dblDeleteBoundaryRatio = 0.005);

    void AdjustDepthInvalidBandPixel();

private:

    int m_nDepthX, m_nDepthY;
    int m_nDepth;
    int m_nZValue;

    double m_dblCamFocus;
    double m_dblBaselineDist;

    bool m_bEnableDepthAccuracy;
    float m_fDepthAccuracyRegionRatio;
    float m_fDepthAccuracyGroundTruthDistanceMM;
    float m_fDepthAccuracyFillRate;
    float m_fDepthAccuracyDistanceMM;
    float m_fDepthAccuracyZAccuracy;
    float m_fDepthSpatialNoise;
    float m_fDepthAngle;
    float m_fDepthAngleX;
    float m_fDepthAngleY;
    float m_fDepthTemporaNoise;

    unsigned int m_nDepthInvalidBandPixel;

    QTime m_lastUpdateDepthTime;

    std::list<std::vector<short>> m_listDepth;
    CTaskInfo *m_pDepthAccuracyTask;
    CTaskInfo *m_pDepthSpatialNoiseTask;
    CTaskInfo *m_pDepthTemporaNoiseTask;
    std::vector<RGBQUAD> m_colorPalette[COLOR_PALETTE_COUNT];

    std::vector< WORD > m_vecTableZ14ToD11;
    std::vector< WORD > m_vecZ14ToD11;

    void *m_PostProcessHandle;
};

class CImageDataModel_Depth_MultiBaseline : public CImageDataModel_Depth
{
public:
    CImageDataModel_Depth_MultiBaseline(CVideoDeviceModel::STREAM_TYPE streamType,
                                        CVideoDeviceController *pVideoDeviceController,
                                        double dblCamFocus,
                                        double dblBaselineDist):
    CImageDataModel_Depth(streamType, pVideoDeviceController, dblCamFocus, dblBaselineDist)
    {}
};

class CImageDataModelFactory{

public:
    static CImageDataModel *CreateCImageDataModel(CVideoDeviceModel::STREAM_TYPE type,
                                                  CVideoDeviceController *pVideoDeviceController);
private:
    CImageDataModelFactory() = default;
};

#endif // CIMAGEDATAMODEL_H
