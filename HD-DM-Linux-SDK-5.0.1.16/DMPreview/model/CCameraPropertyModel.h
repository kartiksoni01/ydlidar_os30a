#ifndef CCAMERAPROPERTYMODEL_H
#define CCAMERAPROPERTYMODEL_H
#include "eSPDI_def.h"
#include <QString>

class CVideoDeviceModel;
class CCameraPropertyModel
{
public:
    struct CameraPropertyItem{
        bool bSupport = true;
        bool bValid = false;
        int nValue;
        int nMin;
        int nMax;
        int nDefault;
        int nFlags;
        int nStep;
    };

    enum CAMERA_PROPERTY{
        AUTO_EXPOSURE = 0,
        AUTO_WHITE_BLANCE,
        LOW_LIGHT_COMPENSATION,
        LIGHT_SOURCE,
        EXPOSURE_TIME, // unit 100 microsecond
        WHITE_BLANCE_TEMPERATURE,
        CAMERA_PROPERTY_COUNT
    };

    enum LIGHT_SOURCE_VALUE{
        VALUE_50HZ,
        VALUE_60HZ
    };

public:

    CCameraPropertyModel(QString sDeviceName,
                         CVideoDeviceModel *pVideoDeviceModel,
                         DEVSELINFO *pDeviceSelfInfo);
    ~CCameraPropertyModel();

    int Init();
    int Update();
    int Reset();

    int InitCameraProperty();
    int UpdateCameraProperty(CAMERA_PROPERTY type);
    int SetDefaultCameraProperty();
    int SetCameraPropertyValue(CAMERA_PROPERTY type, int nValue);
    int SetCameraPropertySupport(CAMERA_PROPERTY type, bool bSupport){
        m_cameraPropertyItems[type].bSupport = bSupport;
        return APC_OK;
    }

    float GetManuelExposureTimeMs();
    void SetManuelExposureTimeMs(float fMs);
    float GetManuelGlobalGain();
    void SetManuelGlobalGain(float fGlobalGain);
    float SetAETargetIndex(int index);

    QString GetDeviceName(){ return m_sDeviceName; }
    CameraPropertyItem GetCameraProperty(CAMERA_PROPERTY type){ return m_cameraPropertyItems[type]; }
private:
    void GetCameraPropertyFlag(CAMERA_PROPERTY type, int &nID, bool &bIsCTProperty);
    void DataToInfo(CAMERA_PROPERTY type, int &nValue);
    void InfoToData(CAMERA_PROPERTY type, int &nValue);

private:
    QString m_sDeviceName;
    CameraPropertyItem m_cameraPropertyItems[CAMERA_PROPERTY_COUNT];
    CVideoDeviceModel *m_pVideoDeviceModel;
    DEVSELINFO *m_pDeviceSelfInfo;
};

#endif // CAMERAPROPERTYMODEL_H
