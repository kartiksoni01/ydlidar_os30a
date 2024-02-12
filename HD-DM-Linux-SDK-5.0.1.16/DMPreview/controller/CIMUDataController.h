#ifndef CIMUDATACONTROLLER_H
#define CIMUDATACONTROLLER_H

#include "CVideoDeviceModel.h"
#include <QQuaternion>

class CIMUDataController
{
public:
    CIMUDataController(CVideoDeviceModel *pVideoDeviceModel);
    ~CIMUDataController();

    CIMUModel *GetIMUModel(){ return m_pVideoDeviceModel->GetIMUModel(); }

    void SetMaxG(float fMaxG){ m_imuData.setMaxG(fMaxG); }
    void SetMaxDPS(float fMaxDPS){ m_imuData.setMaxDPS(fMaxDPS); }

    bool IsCapturingData(){ return m_bIsCapturingData; }
    int StartCaptureData();
    int StopCaptrueData();
    int ReadIMUData();
    int IMUCallback(IMUData *pImuData, int status);

    int StartCalibration();
    int DoCalibration();
    bool IsCalibration(){ return m_bIsCalibration; }

    int StartLogData();
    int StopLogData();
    bool IsLogData(){ return m_pLogFile != nullptr; }
    char *GetLogFileName(){ return m_pLogFileName; }

    int SelectDataFormatIndex(int nIndex);
    int GetCurrentDataFormatIndex();

    int GetFWVersion();
    int GetModuleName();
    int GetStatus();

    QString GetIMUDataMessage(){ return m_sIMUData; }
    QString GetIMUInformationMessage(){
        QMutexLocker locker(&m_imuInfoMutex);
        return m_sIMUInfo;
    }

    IMUData CloneIMUData(){ return m_imuData; }

    void SetIMUSyncWithFrame(bool bIsSync){
        m_pVideoDeviceModel->SetIMUSyncWithFrame(bIsSync);
    }
    bool IsIMUSycnWithFrame(){
        return m_pVideoDeviceModel->IsIMUSyncWithFrame();
    }

    void ResetQuaternion(){ m_bNeedResetQuaternion = true; }

private:
    CVideoDeviceModel *m_pVideoDeviceModel;
    CTaskInfo *m_pCaptureDataTask;
    QString m_sIMUData;
    QString m_sIMUInfo;

    FILE *m_pLogFile;
    char m_pLogFileName[256] = {0};

    bool m_bIsCapturingData;
    bool m_bIsCalibration;

    IMUData m_imuData;

    bool m_bNeedResetQuaternion;
    QQuaternion m_quaternionBeginInverse;

    QMutex m_imuInfoMutex;
};

#endif // CIMUDATACONTROLLER_H
