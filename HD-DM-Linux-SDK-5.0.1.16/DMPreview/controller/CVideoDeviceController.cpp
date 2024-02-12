#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"
#include "CImageDataModel.h"
#include "CThreadWorkerManage.h"
#include "CRegisterReadWriteController.h"
#include "eSPDI.h"
#include <unistd.h>
#include <QMessageBox>
#include <ModeConfig.h>

#include "CFrameSyncManager.h"

CVideoDeviceController::CVideoDeviceController(CVideoDeviceModel *pVideoDeviceModel,
                                               CEYSDUIView *pView):
m_pVideoDeviceModel(pVideoDeviceModel),
m_pControlView(pView),
m_pIMUDataController(nullptr)
{
    m_pPreviewOptions = new PreviewOptions();

    m_pVideoDeviceModel->SetVideoDeviceController(this);
    m_pVideoDeviceModel->ChangeState(CVideoDeviceModel::OPENED);

    m_pModeConfigOptions = new ModeConfigOptions(m_pVideoDeviceModel->GetUsbType(),
                                                 m_pVideoDeviceModel->GetDeviceInformation()[0].deviceInfomation.wPID);
    m_pRegisterReadWriteController = new CRegisterReadWriteController(pVideoDeviceModel);
    m_pCameraPropertyController = new CCameraPropertyController(pVideoDeviceModel);
    if(m_pVideoDeviceModel->IMUSupport()){
        m_pIMUDataController = new CIMUDataController(m_pVideoDeviceModel);
        m_pVideoDeviceModel->ConfigIMU();
    }
    m_pDepthAccuracyController = new CDepthAccuracyController(this);
    m_pDepthFilterOptions = new DepthFilterOptions(this);
    m_pDepthFilterOptions->SetDefaultValue();
    m_pVideoDeviceModel->ConfigDepthFilter();

    Init();

    if(m_pModeConfigOptions->GetModeCount() != 0){
        GetPreviewOptions()->EnableModeConfig(true);
        const int videoModePredefined = m_pVideoDeviceModel->DefaultVideoMode();
        bool isVideoModePredefined = videoModePredefined > 0;
        const int listIndex = isVideoModePredefined ?
                    GetModeConfigOptions()->FindArrayIndexWithVideoMode(videoModePredefined):
                    GetModeConfigOptions()->GetCurrentIndex();

        SelectModeConfigIndex(listIndex);

        if (GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_COLOR)){
            UpdateStreamOptionForCombineMode(GetPreviewOptions()->GetStreamIndex(CVideoDeviceModel::STREAM_COLOR));
        }else if (GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_DEPTH)){
             UpdateStreamOptionForCombineMode(GetVideoDeviceModel()->GetCombineStreamIndexFromDepth(GetPreviewOptions()->GetStreamIndex(CVideoDeviceModel::STREAM_DEPTH)));
        }
    }
}

CVideoDeviceController::~CVideoDeviceController()
{
    if (m_pPreviewOptions) delete m_pPreviewOptions;
    if (m_pRegisterReadWriteController) delete m_pRegisterReadWriteController;
    if (m_pCameraPropertyController) delete m_pCameraPropertyController;
    if (m_pIMUDataController) delete m_pIMUDataController;
    if (m_pDepthAccuracyController) delete m_pDepthAccuracyController;
    if (m_pDepthFilterOptions) delete m_pDepthFilterOptions;
}

void CVideoDeviceController::Init()
{
    SetDepthDataType(m_pPreviewOptions->GetDepthDataType());
    EnableRectifyData(m_pPreviewOptions->IsRectify());
    EnableIRExtend(m_pPreviewOptions->IsIRExtend());
    SetIRLevel(m_pPreviewOptions->GetIRLevel());
    for (int i = 0 ; i < CVideoDeviceModel::STREAM_TYPE_COUNT ; ++i){
        CVideoDeviceModel::STREAM_TYPE type = (CVideoDeviceModel::STREAM_TYPE)i;
        GetPreviewOptions()->EnableStream(type, m_pVideoDeviceModel->IsStreamSupport(type));
    }
}

int CVideoDeviceController::SetIRLevel(unsigned short nLevel)
{
    m_pPreviewOptions->SetIRLevel(nLevel);
    return m_pVideoDeviceModel->SetIRValue(nLevel);
}

void CVideoDeviceController::EnableRectifyData(bool bEnable)
{
    if (bEnable == m_pVideoDeviceModel->IsRectifyData()) return;

    m_pPreviewOptions->SetRectify(bEnable);
    SetDepthDataType(m_pVideoDeviceModel->TransformDepthDataType(bEnable));
}

int CVideoDeviceController::SetDepthDataBits(int nDepthDataBits)
{
    int depthDataType = APC_DEPTH_DATA_DEFAULT;
    switch (nDepthDataBits){
        case 8: depthDataType = APC_DEPTH_DATA_8_BITS; break;
        case 11: depthDataType = APC_DEPTH_DATA_11_BITS; break;
        case 14: depthDataType = APC_DEPTH_DATA_14_BITS; break;
        case 0 : depthDataType =  APC_DEPTH_DATA_OFF_RAW; break;
        default: return APC_NotSupport;
    }

    return SetDepthDataType(m_pVideoDeviceModel->TransformDepthDataType(depthDataType));
}

int CVideoDeviceController::SetDepthDataBits(int nDepthDataBits, bool bRectify)
{
    int depthDataType = APC_DEPTH_DATA_DEFAULT;
    switch (nDepthDataBits){
        case 8: depthDataType = APC_DEPTH_DATA_8_BITS; break;
        case 11: depthDataType = APC_DEPTH_DATA_11_BITS; break;
        case 14: depthDataType = APC_DEPTH_DATA_14_BITS; break;
        case 0 : depthDataType =  APC_DEPTH_DATA_OFF_RAW; break;
        default: return APC_NotSupport;
    }

    return SetDepthDataType(m_pVideoDeviceModel->TransformDepthDataType(depthDataType, bRectify));
}

int CVideoDeviceController::SetDepthDataType(int depthDataType)
{
    m_pPreviewOptions->SetDepthDataType(depthDataType);
    return m_pVideoDeviceModel->SetDepthDataType(depthDataType);
}

int CVideoDeviceController::EnableIRExtend(bool bEnable)
{
    m_pPreviewOptions->SetIRExtend(bEnable);
    return m_pVideoDeviceModel->ExtendIR(bEnable);
}

bool CVideoDeviceController::IsIRExtend()
{
    return m_pVideoDeviceModel->IsIRExtended();
}

int CVideoDeviceController::EnableHWPP(bool bEnable)
{
    return m_pVideoDeviceModel->SetHWPP(bEnable);
}

int CVideoDeviceController::StartStreaming()
{
    CTaskInfo *pTask = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::START_STREAMING, this);
    CThreadWorkerManage::GetInstance()->AddTask(pTask);
    return APC_OK;
}

int CVideoDeviceController::StopStreaming()
{
#if 1
    GetVideoDeviceModel()->StopStreaming();
#else
    CTaskInfo *pTask = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::STOP_STREAMING, this);
    CThreadWorkerManage::GetInstance()->AddTask(pTask);
    pTask->WaitTaskFinished();
#endif
    GetControlView()->UpdateUI();
    return APC_OK;
}

int CVideoDeviceController::DoSnapShot(bool bAsync)
{
    if (bAsync){
        CTaskInfo *pInfo = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::VIDEO_SNAP_SHOT, this);
        CThreadWorkerManage::GetInstance()->AddTask(pInfo);
        return APC_OK;
    }

    unsigned short nIRValue =  GetVideoDeviceModel()->GetIRValue();
    bool bNeedResetIR = 0 != nIRValue &&
                        !m_pVideoDeviceModel->IsInterleaveMode() &&
                        !m_pPreviewOptions->IsStreamEnable(CVideoDeviceModel::STREAM_KOLOR);
    if (APC_PID_HYPATIA == m_pVideoDeviceModel->GetDeviceInformation()[0].deviceInfomation.wPID)
    {
        bNeedResetIR = false;
    }
    if (bNeedResetIR){
        SetIRLevel(0);
        usleep(1000 * 1000);
    }

    unsigned short nColorWidth, nColorHeight;
    int nColorSerialNumber;
    std::vector<BYTE> colorRawBuffer;
    std::vector<BYTE> colorRGBBuffer;
    unsigned short nColorRawBytePerPixel = EOF;
    unsigned short nColorRGBBytePerPixel = 3;

    CImageDataModel *pColorData = nullptr;

    if(m_pPreviewOptions->IsStreamEnable(CVideoDeviceModel::STREAM_COLOR)){
        pColorData = GetControlView()->GetPreviewImageData(CVideoDeviceModel::STREAM_COLOR);;
    }else if(m_pPreviewOptions->IsStreamEnable(CVideoDeviceModel::STREAM_KOLOR)){
        pColorData = GetControlView()->GetPreviewImageData(CVideoDeviceModel::STREAM_KOLOR);;
    }

    if (pColorData){
        nColorWidth = pColorData->GetWidth();
        nColorHeight = pColorData->GetHeight();
        nColorSerialNumber = pColorData->GetSerialNumber();

        colorRawBuffer = pColorData->GetRawData();
        colorRGBBuffer = pColorData->GetRGBData();
        nColorRawBytePerPixel = pColorData->GetRawDataBytePerPixel();
    }

    if (bNeedResetIR){
        SetIRLevel(nIRValue);
        usleep(1000 * 1000);
    }

    CVideoDeviceModel::STREAM_TYPE depthTypes[] = {
                                                    CVideoDeviceModel::STREAM_DEPTH,
                                                    CVideoDeviceModel::STREAM_DEPTH_30mm,
                                                    CVideoDeviceModel::STREAM_DEPTH_60mm,
                                                    CVideoDeviceModel::STREAM_DEPTH_150mm,
                                                    CVideoDeviceModel::STREAM_DEPTH_FUSION
                                                  };

    struct depth_data{
        CVideoDeviceModel::STREAM_TYPE depthType;
        int nDepthWidth, nDepthHeight;
        int nDepthSerialNumber;
        std::vector<BYTE> depthRawBuffer;
        std::vector<BYTE> depthRGBBuffer;
        int nDepthRawBytePerPixel;
        int nDepthRGBBytePerPixel;
    };
    std::vector<depth_data> depthDatas;

    for(CVideoDeviceModel::STREAM_TYPE depthType : depthTypes){
        CImageDataModel *pDepthData = GetControlView()->GetPreviewImageData(depthType);
        if(!pDepthData) continue;
        depth_data depthData = {
                                  depthType,
                                  pDepthData->GetWidth(), pDepthData->GetHeight(),
                                  pDepthData->GetSerialNumber(),
                                  pDepthData->GetRawData(),
                                  pDepthData->GetRGBData(),
                                  pDepthData->GetRawDataBytePerPixel(),
                                  3
                                };
        depthDatas.push_back(std::move(depthData));
    }

    QDate currentDate = QDate::currentDate();
    QTime currentTime = QTime::currentTime();

    char timeStamp[64];
    sprintf(timeStamp, "%04d%02d%02d_%02d%02d",
            currentDate.year(), currentDate.month(), currentDate.day(),
            currentTime.hour(), currentTime.minute(), currentTime.second());

    char pFilePath[256] = {0};

    if (pColorData){
        sprintf(pFilePath, "../out/image/color_%d_%s.bmp", nColorSerialNumber, timeStamp);
        SaveBitmap(pFilePath,
                   &colorRGBBuffer[0], nColorWidth, nColorHeight,
                   nColorRGBBytePerPixel);

        sprintf(pFilePath, "../out/image/color_%d_%s.yuv", nColorSerialNumber, timeStamp);
        SaveYUV(pFilePath,
                &colorRawBuffer[0], nColorWidth, nColorHeight,
                nColorRawBytePerPixel);
    }

    for(depth_data depthData : depthDatas){

        QString depthName;
        switch (depthData.depthType){
            case CVideoDeviceModel::STREAM_DEPTH: depthName = ""; break;
            case CVideoDeviceModel::STREAM_DEPTH_30mm: depthName = "30mm"; break;
            case CVideoDeviceModel::STREAM_DEPTH_60mm: depthName = "60mm"; break;
            case CVideoDeviceModel::STREAM_DEPTH_150mm: depthName = "150mm"; break;
            case CVideoDeviceModel::STREAM_DEPTH_FUSION: depthName = "fusion"; break;
            default: depthName = ""; break;
        }

        sprintf(pFilePath, "../out/image/depth_%s_%d_%s.bmp", depthName.toLocal8Bit().data(),
                                                        depthData.nDepthSerialNumber,
                                                        timeStamp);
        SaveBitmap(pFilePath,
                   &depthData.depthRGBBuffer[0],
                   depthData.nDepthWidth, depthData.nDepthHeight,
                   depthData.nDepthRGBBytePerPixel);

        sprintf(pFilePath, "../out/image/depth_%s_%d_%s.yuv", depthName.toLocal8Bit().data(),
                                                        depthData.nDepthSerialNumber, timeStamp);
        SaveYUV(pFilePath,
                &depthData.depthRawBuffer[0],
                depthData.nDepthWidth, depthData.nDepthHeight,
                depthData.nDepthRawBytePerPixel);

        if (!pColorData) continue;

        // Trim SN data
        constexpr size_t kSerialBytes = 16;
        for (int i = 0; i < kSerialBytes; ++i) {
            depthData.depthRawBuffer[i] = 0x0;
        }

        std::vector<CloudPoint> cloudPoints = m_pVideoDeviceModel->GeneratePointCloud(
                                              depthData.depthType,
                                              depthData.depthRawBuffer,
                                              depthData.nDepthWidth, depthData.nDepthHeight,
                                              colorRGBBuffer, nColorWidth, nColorHeight);

        sprintf(pFilePath, "../out/image/cloud_%s_%d_%s.ply", depthName.toLocal8Bit().data(),
                                                        depthData.nDepthSerialNumber, timeStamp);
        SavePly(pFilePath, cloudPoints);
    }

    return APC_OK;

}

int CVideoDeviceController::SaveBitmap(char *pFilePath,
               BYTE *pBuffer,
               unsigned short nWidth, unsigned short nHeight,
               unsigned short nBytePerPixel)
{
    if (!pFilePath || !pBuffer) return APC_NullPtr;
    APC_RGB2BMP(pFilePath, nWidth, nHeight, pBuffer);
    return APC_OK;
}

int CVideoDeviceController::SaveYUV(char *pFilePath,
            BYTE *pBuffer,
            unsigned short nWidth, unsigned short nHeight,
            unsigned short nBytePerPixel)
{
    if (!pFilePath || !pBuffer) return APC_NullPtr;

    FILE *pFile = fopen(pFilePath, "wb");
    if (!pFile) return APC_NullPtr;

    fseek(pFile, 0, SEEK_SET);
    fwrite(&pBuffer[0], sizeof(unsigned char), nWidth * nHeight * nBytePerPixel, pFile);
    fclose(pFile);
    return APC_OK;
}

int CVideoDeviceController::SavePly(char *pFilePath, std::vector<CloudPoint> cloudPoints)
{
    if (!pFilePath) return APC_NullPtr;
    if (cloudPoints.empty()) return APC_NullPtr;

    PlyWriter::writePly(cloudPoints, pFilePath);

    char path[256];
    char buffer[256];
    chdir("/path/to/change/directory/to");
    getcwd(path, sizeof(path));
    sprintf(buffer, "meshlab \"%s/%s\" &", path, pFilePath);
    system(buffer);

    return APC_OK;
}

int CVideoDeviceController::GetRectifyLogData(int nIndex, eSPCtrl_RectLogData *pRectifyLogData)
{
    std::vector<DEVSELINFO *> deviceSelInfo = m_pVideoDeviceModel->GetDeviceSelInfo();
    return APC_GetRectifyMatLogData(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                        deviceSelInfo[0],
                                        pRectifyLogData,
                                        nIndex);
}

int CVideoDeviceController::GetSlaveRectifyLogData(int nIndex, eSPCtrl_RectLogData *pRectifyLogData)
{
    std::vector<DEVSELINFO *> deviceSelInfo = m_pVideoDeviceModel->GetDeviceSelInfo();
    if(deviceSelInfo.size() < 2) return APC_NotSupport;

    return APC_GetRectifyMatLogData(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                        deviceSelInfo[1],
                                        pRectifyLogData,
                                        nIndex);
}

int CVideoDeviceController::SelectModeConfigIndex(int nIndex)
{
    int ret = GetModeConfigOptions()->SelectCurrentIndex(nIndex);
    if(APC_OK != ret) return ret;

    auto SetStreamInfo = [&](CVideoDeviceModel::STREAM_TYPE streamType,
                               int nWidth,
                               int nHeight,
                               bool bIsMJPEG,
                               int nFPS)
    {
        GetPreviewOptions()->EnableStream(streamType, false);
        if(0 == nWidth && 0 == nHeight) {
            return;
        }

        std::vector<APC_STREAM_INFO> streamInfoList = m_pVideoDeviceModel->GetStreamInfoList(streamType);

        int nIndex = EOF;
        for(size_t i = 0 ; i < streamInfoList.size() ; ++i){
            if(nWidth == streamInfoList[i].nWidth &&
               nHeight == streamInfoList[i].nHeight &&
               bIsMJPEG == streamInfoList[i].bFormatMJPG){
               nIndex = i;
               break;
            }
        }

        if(EOF == nIndex) return;

        GetPreviewOptions()->EnableStream(streamType, true);
        GetPreviewOptions()->SelectStreamIndex(streamType, nIndex);
        if(0 != nFPS)GetPreviewOptions()->SetStreamFPS(streamType, nFPS);
    };

    ModeConfig::MODE_CONFIG modeConfig = GetModeConfigOptions()->GetCurrentModeInfo();
    SetStreamInfo(CVideoDeviceModel::STREAM_COLOR,
                  modeConfig.L_Resolution.Width,
                  modeConfig.L_Resolution.Height,
                  modeConfig.eDecodeType_L == ModeConfig::MODE_CONFIG::MJPEG,
                  modeConfig.vecColorFps.empty() ? 0 : modeConfig.vecColorFps[0]);

    SetStreamInfo(CVideoDeviceModel::STREAM_DEPTH,
                  modeConfig.D_Resolution.Width,
                  modeConfig.D_Resolution.Height,
                  false,
                  modeConfig.vecDepthFps.empty() ? 0 : modeConfig.vecDepthFps[0]);

    SetStreamInfo(CVideoDeviceModel::STREAM_KOLOR,
                  modeConfig.K_Resolution.Width,
                  modeConfig.K_Resolution.Height,
                  modeConfig.eDecodeType_K == ModeConfig::MODE_CONFIG::MJPEG,
                  modeConfig.vecColorFps.empty() ? 0 : modeConfig.vecColorFps[0]);

    SetStreamInfo(CVideoDeviceModel::STREAM_TRACK,
                  modeConfig.T_Resolution.Width,
                  modeConfig.T_Resolution.Height,
                  modeConfig.eDecodeType_T == ModeConfig::MODE_CONFIG::MJPEG,
                  modeConfig.vecColorFps.empty() ? 0 : modeConfig.vecColorFps[0]);

    if(GetVideoDeviceModel()->IsColorWithDepthDevice()){
        if (GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_COLOR)){
            UpdateStreamOptionForCombineMode(GetPreviewOptions()->GetStreamIndex(CVideoDeviceModel::STREAM_COLOR));
        }else if (!GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_DEPTH)){
            int nRotateWidth = modeConfig.D_Resolution.Height;
            int nRotateHeight = modeConfig.D_Resolution.Width;
            SetStreamInfo(CVideoDeviceModel::STREAM_DEPTH,
                          nRotateWidth,
                          nRotateHeight,
                          false,
                          modeConfig.vecDepthFps.empty() ? 0 : modeConfig.vecDepthFps[0]);
        }
    }

    if(!modeConfig.vecDepthType.empty() && GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_DEPTH)){
        SetDepthDataBits(modeConfig.vecDepthType[0], modeConfig.bRectifyMode);
    }else{
        SetDepthDataBits(0, modeConfig.bRectifyMode);
    }

    return APC_OK;
}

int CVideoDeviceController::SetModuleSync(bool bModuleSync)
{
    GetPreviewOptions()->EnableModuleSync(bModuleSync);
    if (bModuleSync) GetVideoDeviceModel()->ModuleSync();

    return APC_OK;
}

int CVideoDeviceController::SetModuleSyncMaster(bool bMaster)
{
    GetPreviewOptions()->SetModuleSyncMaster(bMaster);
    if (bMaster) GetVideoDeviceModel()->ModuleSyncReset();

    return APC_OK;
}

int CVideoDeviceController::SetZRange(int nZNear, int nZFar)
{
    nZNear = std::max(nZNear, (int)m_pVideoDeviceModel->GetZDTableInfo()->nZNear);
    GetPreviewOptions()->SetZRange(nZNear, nZFar);
    GetControlView()->UpdateColorPalette();
    return APC_OK;
}

int CVideoDeviceController::AdjustZRange()
{
    int nZNear, nZFar;
    GetPreviewOptions()->GetZRange(nZNear, nZFar);
    nZNear = std::max(nZNear, (int)m_pVideoDeviceModel->GetZDTableInfo()->nZNear);
    GetPreviewOptions()->SetZRange(nZNear, nZFar);

    int nDefaultZNear, nDefaultZFar;
    GetPreviewOptions()->GetDefaultZRange(nDefaultZNear, nDefaultZFar);
    nDefaultZNear = std::max(nDefaultZNear, (int)m_pVideoDeviceModel->GetZDTableInfo()->nZNear);
    GetPreviewOptions()->SetDefaultZRange(nDefaultZNear, nDefaultZFar);

    GetControlView()->UpdateColorPalette();

    return APC_OK;
}

int CVideoDeviceController::UpdateStreamOptionForCombineMode(int nIndex)
{
    if(!GetVideoDeviceModel()->IsColorWithDepthDevice()) return APC_NotSupport;

    int nDepthStreamIndex = GetVideoDeviceModel()->GetDepthIndexFromCombineStream(nIndex);
    GetPreviewOptions()->EnableStream(CVideoDeviceModel::STREAM_DEPTH, nDepthStreamIndex != EOF);
    if(EOF != nDepthStreamIndex){
        GetPreviewOptions()->SelectStreamIndex(CVideoDeviceModel::STREAM_DEPTH, nDepthStreamIndex);
    }

    int nColorStreamIndex = GetVideoDeviceModel()->GetColorIndexFromCombineStream(nIndex);
    GetPreviewOptions()->EnableStream(CVideoDeviceModel::STREAM_COLOR, nColorStreamIndex != EOF);
    GetPreviewOptions()->SelectStreamIndex(CVideoDeviceModel::STREAM_COLOR, nIndex);

    return APC_OK;
}

int CVideoDeviceController::UpdateSpecificDepthPosition(int x, int y)
{
    CVideoDeviceModel::STREAM_TYPE depthStreams[] = {
                                                      CVideoDeviceModel::STREAM_DEPTH,
                                                      CVideoDeviceModel::STREAM_DEPTH_30mm,
                                                      CVideoDeviceModel::STREAM_DEPTH_60mm,
                                                      CVideoDeviceModel::STREAM_DEPTH_150mm,
                                                      CVideoDeviceModel::STREAM_DEPTH_FUSION
                                                    };

    for (CVideoDeviceModel::STREAM_TYPE depthStream : depthStreams){
        CImageDataModel *pDepthImageDataModel = GetControlView()->GetPreviewImageData(depthStream);
        if (!pDepthImageDataModel) continue;

        pDepthImageDataModel->SetSpecificPixel(x, y);
    }

    return APC_OK;
}

int CVideoDeviceController::StartIMUSyncWithFrame()
{
    return CFrameSyncManager::GetInstance()->RegisterDataCallback(m_pVideoDeviceModel,
                                                                  m_pControlView,
                                                                  m_pIMUDataController);
}

int CVideoDeviceController::StopIMUSyncWithFrame()
{
    return CFrameSyncManager::GetInstance()->UnregisterDataCallback(m_pVideoDeviceModel);
}
