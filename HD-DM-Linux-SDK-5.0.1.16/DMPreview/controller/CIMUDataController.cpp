#include "CIMUDataController.h"
#include "CThreadWorkerManage.h"
#include "CFrameSyncManager.h"
#include <math.h>
#include <QQuaternion>

CIMUDataController::CIMUDataController(CVideoDeviceModel *pVideoDeviceModel):
m_pVideoDeviceModel(pVideoDeviceModel),
m_bIsCapturingData(false),
m_bIsCalibration(false),
m_pLogFile(nullptr),
m_bNeedResetQuaternion(true)
{
    StartCaptureData();
}

CIMUDataController::~CIMUDataController()
{
    if(m_pLogFile) fclose(m_pLogFile);
}

int CIMUDataController::StartCaptureData()
{
    if (m_bIsCapturingData) return APC_OK;
    if (!m_pVideoDeviceModel->GetIMUModel()) return APC_NullPtr;

    m_pVideoDeviceModel->GetIMUModel()->ReadDataOutputFormat();
    m_pVideoDeviceModel->GetIMUModel()->EnableDataOutout(true);

    m_pCaptureDataTask = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::IMU_CAPTURE_DATA, this);
    CThreadWorkerManage::GetInstance()->AddTask(m_pCaptureDataTask);

    m_bIsCapturingData = true;

    return APC_OK;
}

int CIMUDataController::StopCaptrueData()
{
    if(!m_bIsCapturingData) return APC_OK;

    CThreadWorkerManage::GetInstance()->RemoveTask(m_pCaptureDataTask);

    m_bIsCapturingData = false;
    m_pVideoDeviceModel->GetIMUModel()->EnableDataOutout(false);
    return APC_OK;
}

int CIMUDataController::StartLogData()
{
    if(m_pLogFile){
        StopLogData();
    }

    QDate date = QDate::currentDate();
    QTime time = QTime::currentTime();
    sprintf(m_pLogFileName, "IMULog%04d%02d%02d%02d%02d%02d.txt", date.year(), date.month(), date.day(),
            time.hour(), time.minute(), time.second());

    char buf[512];
    sprintf(buf, "../out/IMULog/%s" , m_pLogFileName);
    m_pLogFile = fopen(buf, "wt");
    m_pVideoDeviceModel->GetIMUModel()->SetLogFile(m_pLogFile);

    return APC_OK;
}

int CIMUDataController::StopLogData()
{
    if(!m_pLogFile) return APC_OK;

    fclose(m_pLogFile);
    m_pLogFile = nullptr;
    m_pVideoDeviceModel->GetIMUModel()->SetLogFile(nullptr);

    return APC_OK;
}

int CIMUDataController::SelectDataFormatIndex(int nIndex)
{
    CIMUModel::DATA_FORMAT dataForamt;
    switch (nIndex){
        case 0: dataForamt = CIMUModel::RAW_DATA_WITHOUT_OFFSET; break;
        case 1: dataForamt = CIMUModel::RAW_DATA_WITH_OFFSET; break;
        case 2: dataForamt = CIMUModel::OFFSET_DATA; break;
        case 3: dataForamt = CIMUModel::DMP_DATA_WITHOT_OFFSET; break;
        case 4: dataForamt = CIMUModel::DMP_DATA_WITH_OFFSET; break;
        default: return APC_NotSupport;
    }

    return m_pVideoDeviceModel->GetIMUModel()->SelectDataFormat(dataForamt);
}

int CIMUDataController::GetCurrentDataFormatIndex()
{
    CIMUModel::DATA_FORMAT dataForamt = m_pVideoDeviceModel->GetIMUModel()->GetDataFormat();
    switch (dataForamt){
        case CIMUModel::RAW_DATA_WITHOUT_OFFSET: return 0;
        case CIMUModel::RAW_DATA_WITH_OFFSET: return 1;
        case CIMUModel::OFFSET_DATA: return 2;
        case CIMUModel::DMP_DATA_WITHOT_OFFSET: return 3;
        case CIMUModel::DMP_DATA_WITH_OFFSET: return 4;
        default: return EOF;
    }
}

int CIMUDataController::GetFWVersion()
{
    m_sIMUInfo = "FW version: " + m_pVideoDeviceModel->GetIMUModel()->GetFWVersion();
    return APC_OK;
}

int CIMUDataController::GetModuleName()
{
    m_sIMUInfo = "Module name: " + m_pVideoDeviceModel->GetIMUModel()->GetModuleName();
    return APC_OK;
}

int CIMUDataController::GetStatus()
{
    m_sIMUInfo = "Status : " + m_pVideoDeviceModel->GetIMUModel()->GetStatus();
    return APC_OK;
}

int CIMUDataController::StartCalibration()
{
    if(m_bIsCalibration) return APC_OK;

    m_bIsCalibration = true;

    CTaskInfo *pInfo = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::IMU_CALIBRATION, this);
    CThreadWorkerManage::GetInstance()->AddTask(pInfo);
    return APC_OK;
}

int CIMUDataController::DoCalibration()
{
    CIMUModel *pModel = m_pVideoDeviceModel->GetIMUModel();
    if(!pModel) return APC_NullPtr;

    char calibratingStatus = 0;
    pModel->CheckCalibratingStatus(&calibratingStatus);

    int count = 0;
    if (calibratingStatus == 0x00) {

        bool isNeedRetartImuThread = false;

        if (m_bIsCapturingData)
        {
            isNeedRetartImuThread = true;
            StopCaptrueData();
        }

        pModel->StartCalibration();

        int sleepSec = 5;
        for (int i = 0; i < sleepSec; i++) {

            m_sIMUInfo.sprintf("FW Calibration: %d%%", 100 / sleepSec * i);
            QThread::sleep(1);
        }

        m_sIMUInfo = "FW Calibration: 99%";

        do {
            pModel->CheckCalibratingStatus(&calibratingStatus);
            if (calibratingStatus == 1) {
                QThread::msleep(500);
                continue;
            }
            else if (calibratingStatus == 0) {
                break;
            }
        } while (count++ < 5);

        char calibrated = 0;
        pModel->ReadCalibrated(&calibrated);

        if (calibrated == 1) {
            m_sIMUInfo = "FW Calibration: succeed";
        }
        else
        {
            m_sIMUInfo = "FW Calibration: failed";
        }

        if (isNeedRetartImuThread) {
            StartCaptureData();
        }
    }

    m_bIsCalibration = false;

    return APC_OK;
}

int CIMUDataController::ReadIMUData()
{
    CIMUModel *pModel = m_pVideoDeviceModel->GetIMUModel();
    if (!pModel) return APC_NullPtr;
    if (CVideoDeviceModel::RECONNECTING == m_pVideoDeviceModel->GetState()) return APC_NullPtr;

    IMUData imuData;
    int ret = pModel->ReadIMUData(imuData);

    if (CIMUModel::IMU_9_AXIS == pModel->GetType()){

        QQuaternion quaternion = QQuaternion(imuData._quaternion[0],
                                             imuData._quaternion[1],
                                             imuData._quaternion[2],
                                             imuData._quaternion[3]);

        if (m_bNeedResetQuaternion){
            m_bNeedResetQuaternion = false;
            m_quaternionBeginInverse = quaternion.inverted();
        }

        quaternion = m_quaternionBeginInverse * quaternion;
        imuData._quaternion[0] = quaternion.scalar();
        imuData._quaternion[1] = quaternion.x();
        imuData._quaternion[2] = quaternion.y();
        imuData._quaternion[3] = quaternion.z();

    }

    if (IsIMUSycnWithFrame() && CVideoDeviceModel::STREAMING == m_pVideoDeviceModel->GetState()){
        return CFrameSyncManager::GetInstance()->SyncIMUCallback(m_pVideoDeviceModel, &imuData, ret);
    }else{
        return IMUCallback(&imuData, ret);
    }
}

int CIMUDataController::IMUCallback(IMUData *pImuData, int status)
{
    CIMUModel *pModel = m_pVideoDeviceModel->GetIMUModel();
    if (!pModel) return APC_NullPtr;

    if (APC_NullPtr == status){
        m_sIMUData = "unable to open device.";
        return APC_NullPtr;
    }

    m_imuData = *pImuData;

    if (0 == status){
        m_sIMUData = "waiting.";
    }else if (status < 0){
        m_sIMUData = "unable to read imu data.";
    }else{
        int nIMUDataByte = pModel->GetIMUDataOutputByte(pModel->GetDataFormat());
        const char *frameCountFormat = CVideoDeviceModel::SERIAL_COUNT == m_pVideoDeviceModel->GetSerialNumberType() ?
                                       "(Serial Count)" : "";
        CIMUModel::TYPE kIMUType = pModel->GetType();
        QMutexLocker locker(&m_imuInfoMutex);
        if (CIMUModel::IMU_9_AXIS == kIMUType){
            QVector3D eulerAngles = QQuaternion(m_imuData._quaternion[0],
                                    m_imuData._quaternion[1],
                                    m_imuData._quaternion[2],
                                    m_imuData._quaternion[3]).toEulerAngles();
            m_sIMUData.sprintf("Frame count%s:%d\nTime:%2d:%2d:%2d:%4d\nRoll[%10.3f], Pitch[%10.3f], Yaw[%10.3f]\nQuaternion:\n0:%15.8f\n1:%15.8f\n2:%15.8f\n3:%15.8f",
                                frameCountFormat, m_imuData._frameCount,
                                m_imuData._hour, m_imuData._min, m_imuData._sec, m_imuData._subSecond,
                                eulerAngles.x(), eulerAngles.y(), eulerAngles.z(),
                                m_imuData._quaternion[0], m_imuData._quaternion[1], m_imuData._quaternion[2], m_imuData._quaternion[3]);
        }else if (27 == nIMUDataByte){
            float g = sqrt((m_imuData._accelX * m_imuData._accelX) +
                           (m_imuData._accelY * m_imuData._accelY) +
                           (m_imuData._accelZ * m_imuData._accelZ));
            m_sIMUData.sprintf("Frame count%s:%d\nTime:%2d:%2d:%2d:%4d\nAccel X:%+-4.3f \t Y:%+-4.3f \t Z:%+-4.3f Total:%+-4.3f\nGyro X:%+-4.3f \t Y:%+-4.3f \t Z:%+-4.3f\n",
                               frameCountFormat, m_imuData._frameCount,
                               m_imuData._hour, m_imuData._min, m_imuData._sec, m_imuData._subSecond,
                               m_imuData._accelX, m_imuData._accelY, m_imuData._accelZ, g,
                               m_imuData._gyroScopeX, m_imuData._gyroScopeY, m_imuData._gyroScopeZ);

        }else if (58 == nIMUDataByte){
            m_sIMUData.sprintf("Frame count%s:%d\nTime:%2d:%2d:%2d:%4d\nAccel X:%4.3f Y:%4.3f Z:%4.3f\nGyro X:%4.0f Y:%4.0f Z:%4.0f\nCompass X:%.2f Y:%.2f Z:%.2f\nCompass_TBC X:%.2f Y:%.2f Z:%.2f\nAccuracy_FLAG: %d",
                                frameCountFormat, m_imuData._frameCount,
                                m_imuData._hour, m_imuData._min, m_imuData._sec, m_imuData._subSecond,
                                m_imuData._accelX, m_imuData._accelY, m_imuData._accelZ,
                                m_imuData._gyroScopeX, m_imuData._gyroScopeY, m_imuData._gyroScopeZ,
                                m_imuData._compassX, m_imuData._compassY, m_imuData._compassZ,
                                m_imuData._compassX_TBC, m_imuData._compassY_TBC, m_imuData._compassZ_TBC,
                                m_imuData._accuracy_FLAG);
        }
    }

    return APC_OK;
}
