#ifndef CIMUMODEL_H
#define CIMUMODEL_H

#include "IMUData.h"
#include <vector>
#include <QString>
#include "hidapi/hidapi.h"

class CTaskInfo;
class CVideoDeviceModel;
class CIMUModel
{
private:

    struct SetFeatureDATA_Item {
        const char* pData;
        int nDataLength;
    };

    static unsigned char GenerateIMUNumber();

public:

    static unsigned char GetIMUNumber(QString serialNumber);

    enum TYPE{
        IMU_6_AXIS,
        IMU_9_AXIS,
        IMU_UNKNOWN
    };

    enum DATA_FORMAT{
        RAW_DATA_WITHOUT_OFFSET = 1,
        RAW_DATA_WITH_OFFSET,
        OFFSET_DATA,
        DMP_DATA_WITHOT_OFFSET,
        DMP_DATA_WITH_OFFSET
    };

    struct INFO {
        unsigned short nVID;
        unsigned short nPID;
        TYPE axis;
    };

    CIMUModel(INFO info, CVideoDeviceModel *pVideoDeviceModel);
    ~CIMUModel();

    bool IsValid(){ return m_pHandle != nullptr; }
    int Init(INFO info);
    void ReadDataOutputFormat();

    void EnableDataOutout(bool bIsEnbale);
    int SelectDataFormat(DATA_FORMAT format);
    DATA_FORMAT GetDataFormat(){ return m_nCurrentIMUFormat; }

    int ReadIMUData(IMUData &imuData, bool bSync = true);
    int GetIMUDataOutputByte(DATA_FORMAT format);
    std::vector<DATA_FORMAT> GetSupportDataFormat();

    void CheckCalibratingStatus(char *pCalibratingStatus);
    void StartCalibration();
    void ReadCalibrated(char *pCalibrated);

    QString GetFWVersion();
    QString GetModuleName();
    QString GetStatus();
    TYPE GetType(){ return m_imuType; }

    QString GetSerialNumber();
    void SetSerialNumber(QString sSerialNumber);

    int GetModuleID();
    void SetModuleID(unsigned char nID);

    void SetLogFile(FILE *pLogFile){ m_pLogFile = pLogFile; }

    bool IsSyncWithCamera(){ return m_bIsSyncWithCamera; }

private:
    void GetFeatureReport(char* pData, size_t data_lenght);
    void SendFeatureReport(const char* pData, size_t data_lenght);

private:
    CVideoDeviceModel *m_pVideoDeviceModel;

    hid_device *m_pHandle;
    TYPE m_imuType;

    DATA_FORMAT m_nCurrentIMUFormat;
    FILE *m_pLogFile;

    bool m_bIsSyncWithCamera;
    unsigned short m_nSyncIndex;
};

#endif // CIMUMODEL_H
