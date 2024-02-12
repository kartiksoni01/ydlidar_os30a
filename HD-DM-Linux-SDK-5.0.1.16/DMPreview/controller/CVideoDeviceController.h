#ifndef CVIDEODEVICECONTROLLER_H
#define CVIDEODEVICECONTROLLER_H
#include <QWidget>
#include "CVideoDeviceModel.h"
#include "CEYSDUIView.h"
#include "PreviewOptions.h"
#include "CRegisterReadWriteController.h"
#include "CCameraPropertyController.h"
#include "CIMUDataController.h"
#include "CDepthAccuracyController.h"
#include "ModeConfigOptions.h"
#include "DepthFilterOptions.h"

class CDepthAccuracyController;
class CVideoDeviceController
{
public:
    CVideoDeviceController(CVideoDeviceModel *pVideoDeviceModel,
                           CEYSDUIView *pView);
    ~CVideoDeviceController();

    void Init();

    CEYSDUIView *GetControlView(){ return m_pControlView; }
    CVideoDeviceModel *GetVideoDeviceModel(){ return m_pVideoDeviceModel; }
    PreviewOptions *GetPreviewOptions(){ return m_pPreviewOptions; }
    ModeConfigOptions *GetModeConfigOptions(){ return m_pModeConfigOptions; }
    CRegisterReadWriteController *GetRegisterReadWriteController(){ return m_pRegisterReadWriteController; }
    CCameraPropertyController *GetCameraPropertyController(){ return m_pCameraPropertyController; }
    CIMUDataController *GetIMUDataController(){ return m_pIMUDataController; }
    CDepthAccuracyController *GetDepthAccuracyController(){ return m_pDepthAccuracyController; }
    DepthFilterOptions *GetDepthFilterOptions(){ return m_pDepthFilterOptions; }

    void EnableRectifyData(bool bEnable);
    int SetDepthDataBits(int nDepthDataBits, bool bRectify);
    int SetDepthDataBits(int nDepthDataBits);
    int SetIRLevel(unsigned short nLevel);
    int EnableIRExtend(bool bEnable);
    bool IsIRExtend();
    int EnableHWPP(bool bEnable);

    int StartStreaming();
    int StopStreaming();

    int GetRectifyLogData(int nIndex, eSPCtrl_RectLogData *pRectifyLogData);
    int GetSlaveRectifyLogData(int nIndex, eSPCtrl_RectLogData *pRectifyLogData);

    int SelectModeConfigIndex(int nIndex);

    int SetModuleSync(bool bModuleSync);
    int SetModuleSyncMaster(bool bMaster);

    int SetZRange(int nZNear, int nZFar);
    int AdjustZRange();

    int UpdateStreamOptionForCombineMode(int nIndex);

    int DoSnapShot(bool bAsync = true);

    int UpdateSpecificDepthPosition(int x, int y);

    int StartIMUSyncWithFrame();
    int StopIMUSyncWithFrame();

private:
    int SetDepthDataType(int depthDataType);

    int SaveBitmap(char *pFilePath,
                   BYTE *pBuffer,
                   unsigned short nWidth, unsigned short nHeight,
                   unsigned short nBytePerPixel);
    int SaveYUV(char *pFilePath,
                BYTE *pBuffer,
                unsigned short nWidth, unsigned short nHeight,
                unsigned short nBytePerPixel);
    int SavePly(char *pFilePath, std::vector<CloudPoint> cloudPoints);

private:
    CVideoDeviceModel *m_pVideoDeviceModel;
    CEYSDUIView *m_pControlView;
    PreviewOptions    *m_pPreviewOptions;
    ModeConfigOptions *m_pModeConfigOptions;
    CRegisterReadWriteController *m_pRegisterReadWriteController;
    CCameraPropertyController *m_pCameraPropertyController;
    CIMUDataController *m_pIMUDataController;
    CDepthAccuracyController *m_pDepthAccuracyController;
    DepthFilterOptions *m_pDepthFilterOptions;
};

#endif // CVIDEODEVICECONTROLLER_H
