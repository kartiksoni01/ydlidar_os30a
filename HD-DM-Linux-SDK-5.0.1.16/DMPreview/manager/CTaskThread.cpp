#include "CTaskThread.h"
#include "eSPDI_def.h"
#include "CVideoDeviceModel.h"
#include "CThreadWorkerManage.h"
#include "CVideoDeviceController.h"
#include "CRegisterReadWriteController.h"
#include "CEYSDDeviceManager.h"
#include "CMessageManager.h"
#include "CIMUDataController.h"
#include "CVideoDeviceAudoWidget.h"
#include "CImageDataModel.h"
#include <QMessageBox>
#include <QTime>
#include "CEYSDUIView.h"

CTaskThread::CTaskThread(QObject *parent):
QThread(parent),
m_pTaskInfo(nullptr)
{}

int CTaskThread::AssignTask(CTaskInfo *pInfo)
{
    if (isRunning()) return APC_NotSupport;
    if (!pInfo) return APC_NullPtr;

    m_pTaskInfo = pInfo;
    m_pTaskInfo->SetHandleThread(this);
    setObjectName(GetThreadName());
    start(GetPriority());

    return APC_OK;
}

int CTaskThread::RemoveTask()
{
    m_pTaskInfo = nullptr;
    return APC_OK;
}

QString CTaskThread::GetThreadName()
{
    switch (m_pTaskInfo->GetTaskType())
    {
        case CTaskInfo::START_STREAMING: return "StartStreaming";
        case CTaskInfo::STOP_STREAMING:  return "StopStreaming";
        case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR: return "[Color]Grabber";
        case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR_SLAVE: return "[Color_S]Grabber";
        case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH: return "[Depth]Grabber";
        case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH_60mm: return "[Depth60]Grabber";
        case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH_150mm: return "[Depth150]Grabber";
        case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR_WITH_DEPTH: return "[C+D]Grabber";
        case CTaskInfo::GRABBER_VIDEO_IMAGE_KOLOR: return "[Kolor]Grabber";
        case CTaskInfo::GRABBER_VIDEO_IMAGE_KOLOR_SLAVE: return "[Kolor_S]Grabber";
        case CTaskInfo::GRABBER_VIDEO_IMAGE_TRACK: return "[Track]Grabber";
        case CTaskInfo::VIDEO_SNAP_SHOT: return "SnapShot";
        case CTaskInfo::VIDEO_COLD_RESET: return "ColdReset";
        case CTaskInfo::VIDEO_QUALITY_REGISTER_SETTING: return "RegisterSetting";
        case CTaskInfo::REGISTER_PERIODIC_READ: return "RegisterPeriodicRead";
        case CTaskInfo::IMU_CAPTURE_DATA: return "IMUDara";
        case CTaskInfo::IMU_CALIBRATION: return "IMUCalibration";
        case CTaskInfo::AUDIO_RECORD: return "AudioRecord";
        case CTaskInfo::DEPTH_ACCURACY_CALCULATE: return "DepthAccruacy";
        case CTaskInfo::DEPTH_SPATIAL_NOISE_CALCULATE: return "DepthSpatialNoise";
        case CTaskInfo::DEPTH_TEMPORA_NOISE_CALCULATE: return "DepthTemporaNoise";
        case CTaskInfo::IMAGE_DATA_RAW_TO_RGB_TRANSFORM: return "ImgDataTransform";
        //+[Thermal device]
        case CTaskInfo::GRABBER_VIDEO_IMAGE_THERMAL: return "[Thermal]Grabber";
        //-[Thermal device]
        default: return "NoneTask";
    }
}

QThread::Priority CTaskThread::GetPriority()
{
    switch (m_pTaskInfo->GetTaskType())
    {
        case CTaskInfo::START_STREAMING:
        case CTaskInfo::STOP_STREAMING:
        case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR:
        case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR_SLAVE:
        case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH:
        case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH_60mm:
        case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH_150mm:
        case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR_WITH_DEPTH:
        case CTaskInfo::GRABBER_VIDEO_IMAGE_KOLOR:
        case CTaskInfo::GRABBER_VIDEO_IMAGE_KOLOR_SLAVE:
        case CTaskInfo::GRABBER_VIDEO_IMAGE_TRACK:
        //+[Thermal device]
        case CTaskInfo::GRABBER_VIDEO_IMAGE_THERMAL:
        //-[Thermal device]
            return TimeCriticalPriority;
        default:
            return NormalPriority;
    }
}

void CTaskThread::run()
{
    m_pTaskInfo->SetState(CTaskInfo::RUNNING);
    while (CTaskInfo::FINISHED != m_pTaskInfo->GetTaskState()){
        switch (m_pTaskInfo->GetTaskType()){
            case CTaskInfo::START_STREAMING:
                DoStartStreaming();
                break;
            case CTaskInfo::STOP_STREAMING:
                DoStopStreaming();
                break;
            case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR:
            case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR_SLAVE:
            case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH:
            case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH_60mm:
            case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH_150mm:
            case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR_WITH_DEPTH:
            case CTaskInfo::GRABBER_VIDEO_IMAGE_KOLOR:
            case CTaskInfo::GRABBER_VIDEO_IMAGE_KOLOR_SLAVE:
            case CTaskInfo::GRABBER_VIDEO_IMAGE_TRACK:
            //+[Thermal device]
            case CTaskInfo::GRABBER_VIDEO_IMAGE_THERMAL:
            //+[Thermal device]
                DoGrabberVideoImage();
                break;
            case CTaskInfo::VIDEO_SNAP_SHOT:
                DoVideoSnapShot();
                break;
            case CTaskInfo::VIDEO_COLD_RESET:
                DoColdReset();
                break;
            case CTaskInfo::VIDEO_QUALITY_REGISTER_SETTING:
                DoQualityRegisterSetting();
                break;
            case CTaskInfo::VIDEO_INTERLEAVE_MODE:
                DoInterleaveMode();
                break;
            case CTaskInfo::VIDEO_MODULE_SYNC:
                DoModuleSync();
                break;
            case CTaskInfo::VIDEO_FRAME_SYNC:
                DoFrameSync();
                break;
            case CTaskInfo::REGISTER_PERIODIC_READ:
                DoRegisterPeriodicRead();
                break;
            case CTaskInfo::IMU_CAPTURE_DATA:
                DoIMUCapture();
            break;
            case CTaskInfo::IMU_CALIBRATION:
                DoIMUCalibration();
                break;
            case CTaskInfo::AUDIO_RECORD:
                DoAudioRecord();
                break;
            case CTaskInfo::DEPTH_ACCURACY_CALCULATE:
                DoDepthAccuracyCalculate();
                break;
            case CTaskInfo::DEPTH_SPATIAL_NOISE_CALCULATE:
                DoDepthSpatialNoiseCalculate();
                break;
            case CTaskInfo::DEPTH_TEMPORA_NOISE_CALCULATE:
                DoDepthTemporaNoiseCalculate();
                break;
            case CTaskInfo::IMAGE_DATA_RAW_TO_RGB_TRANSFORM:
                DoImageDataTransform();
                break;
            default:
                break;
        }
    }
}

void CTaskThread::DoStartStreaming()
{
    CVideoDeviceController *pController = static_cast<CVideoDeviceController *>(m_pTaskInfo->GetParam());
    CMessageManager::ShowMessage("Wait device open.");

    pController->GetVideoDeviceModel()->Update();
    int ret = pController->GetVideoDeviceModel()->StartStreaming();
    pController->GetControlView()->UpdateUI();
    CMessageManager::CloseMessage();

    if(APC_OK != ret){
        CMessageManager::Error("Open device failed!");
    }
    CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
}

void CTaskThread::DoStopStreaming()
{
    CVideoDeviceController *pController = static_cast<CVideoDeviceController *>(m_pTaskInfo->GetParam());
    pController->GetVideoDeviceModel()->StopStreaming();
    CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
}

void CTaskThread::DoGrabberVideoImage()
{
    CVideoDeviceModel *pModel = static_cast<CVideoDeviceModel *>(m_pTaskInfo->GetParam());
    pModel->DoImageGrabber(m_pTaskInfo->GetTaskType());
}

void CTaskThread::DoVideoSnapShot()
{
    CVideoDeviceController *pController = static_cast<CVideoDeviceController *>(m_pTaskInfo->GetParam());
    CMessageManager::ShowMessage("Snapshot and Ply, please wait.....");
    pController->DoSnapShot(false);
    CMessageManager::CloseMessage();
    CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
}

void CTaskThread::DoColdReset()
{
    static QMutex mutex;

    if (mutex.tryLock()) {
        mutex.unlock();
    }else{
        CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
        return;
    }

    QMutexLocker locker(&mutex);
    CVideoDeviceModel *pModel = static_cast<CVideoDeviceModel *>(m_pTaskInfo->GetParam());
    QTime currentTime = QTime::currentTime();

    if (pModel->GetColdeResetStartTime().secsTo(currentTime) > 10 ||
        pModel->GetColdeResetStartTime().secsTo(currentTime) < 0){
        CMessageManager::CloseMessage();
        QMessageBox::StandardButton result;
        RUN_ON_UI_THREAD(
           result = QMessageBox::critical(nullptr , "Error", "No connected any device. please plug-in device again.", QMessageBox::Ok | QMessageBox::Cancel);
        );

        if (QMessageBox::Ok == result) {
            pModel->SetColdResetStartTime(QTime::currentTime());
        }else{
            pModel->StopStreaming();
            if (pModel->GetVideoDeviceController() && pModel->GetVideoDeviceController()->GetControlView()){
                pModel->GetVideoDeviceController()->GetControlView()->UpdateUI();
            }
            CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
            return;
        }
    }

    CMessageManager::ShowMessage("Reconnecting Device.");
    if (APC_OK == CEYSDDeviceManager::GetInstance()->Reconnect()){
        CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
        CMessageManager::CloseMessage();
    }

    QThread::sleep(1);
}

void CTaskThread::DoQualityRegisterSetting()
{
    CVideoDeviceModel *pModel = static_cast<CVideoDeviceModel *>(m_pTaskInfo->GetParam());
    pModel->AdjustRegister();
    CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
}

void CTaskThread::DoInterleaveMode()
{
    CVideoDeviceModel *pModel = static_cast<CVideoDeviceModel *>(m_pTaskInfo->GetParam());
    if (APC_OK == pModel->AdjustInterleaveModeState()){
        CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
    }
}

void CTaskThread::DoModuleSync()
{
    CVideoDeviceModel *pModel = static_cast<CVideoDeviceModel *>(m_pTaskInfo->GetParam());
    if(APC_OK == pModel->ModuleSync()){
        CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
    }
    QThread::sleep(1);
}

void CTaskThread::DoFrameSync()
{
    CVideoDeviceModel *pModel = static_cast<CVideoDeviceModel *>(m_pTaskInfo->GetParam());
    QThread::sleep(1);
    if(APC_OK == pModel->FrameSync()){
        CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
    }
}

void CTaskThread::DoRegisterPeriodicRead()
{
    CRegisterReadWriteController *pController = static_cast<CRegisterReadWriteController *>(m_pTaskInfo->GetParam());
    QTime currentTime = QTime::currentTime();
    int nPeriodTime = pController->GetLastestReadTime().msecsTo(currentTime);
    if (nPeriodTime > pController->GetRegisterReadWriteOptions()-> GetPeriodTimeMs()){
        pController->ReadRegister();
        pController->SetRealPeroidTimeMs(nPeriodTime);
    }
}

void CTaskThread::DoIMUCapture()
{
    CIMUDataController *pIMUController = static_cast<CIMUDataController *>(m_pTaskInfo->GetParam());
    pIMUController->ReadIMUData();
    QThread::msleep(5); // 200HZ
}

void CTaskThread::DoIMUCalibration()
{
    CIMUDataController *pIMUController = static_cast<CIMUDataController *>(m_pTaskInfo->GetParam());
    pIMUController->DoCalibration();
    CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
}

void CTaskThread::DoAudioRecord()
{
    CVideoDeviceAudoWidget *pAudioWidget = static_cast<CVideoDeviceAudoWidget *>(m_pTaskInfo->GetParam());
    CMessageManager::ShowMessage("Recording audio...");
    pAudioWidget->DoAudioRecord();
    CMessageManager::CloseMessage();
    CThreadWorkerManage::GetInstance()->RemoveTask(m_pTaskInfo);
}

void CTaskThread::DoDepthAccuracyCalculate()
{
    CImageDataModel_Depth *pDepthImageDataModel = static_cast<CImageDataModel_Depth *>(m_pTaskInfo->GetParam());
    pDepthImageDataModel->CalculateDepthAccuracyInfo();
    QThread::msleep(10);
}

void CTaskThread::DoDepthSpatialNoiseCalculate()
{
    CImageDataModel_Depth *pDepthImageDataModel = static_cast<CImageDataModel_Depth *>(m_pTaskInfo->GetParam());
    pDepthImageDataModel->CalculateDepthSpatialNoise();
    QThread::msleep(10);
}

void CTaskThread::DoDepthTemporaNoiseCalculate()
{
    CImageDataModel_Depth *pDepthImageDataModel = static_cast<CImageDataModel_Depth *>(m_pTaskInfo->GetParam());
    pDepthImageDataModel->CalculateDepthTemporaNoise();
    QThread::msleep(10);
}

void CTaskThread::DoImageDataTransform()
{
    CImageDataModel *pImageDataModel = static_cast<CImageDataModel *>(m_pTaskInfo->GetParam());
    pImageDataModel->TransformRawToRGB();
    QThread::msleep(10);
}
