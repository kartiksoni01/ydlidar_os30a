#ifndef PREVIEWOPTIONS_H
#define PREVIEWOPTIONS_H
#include "CVideoDeviceModel.h"

class CVideoDeviceController;
class PreviewOptions
{
public:
    struct StreamOption{
        bool bEnable = true;
        short nIndex = 0;
        int nFPS = 30;
    };

    enum POINT_CLOUDE_VIEWER_OUTPUT_FORMAT{
        POINT_CLOUDE_VIEWER_OUTPUT_COLOR,
        POINT_CLOUDE_VIEWER_OUTPUT_DEPTH,
        POINT_CLOUDE_VIEWER_OUTPUT_SINGLE_COLOR
    };

    struct PointCloudViewerOption{
        bool bEnable = false;
        unsigned short nPointSize = 1;
        POINT_CLOUDE_VIEWER_OUTPUT_FORMAT outputFormat = POINT_CLOUDE_VIEWER_OUTPUT_COLOR;
    };

public:
    PreviewOptions() = default;
    ~PreviewOptions() = default;

    void EnableModeConfig(bool bEnable){ bModeConfig = bEnable; }
    bool IsModeConfig(){ return bModeConfig; }

    void EnableStream(CVideoDeviceModel::STREAM_TYPE type, bool bEnable)
    { streamOption[type].bEnable = bEnable; }
    bool IsStreamEnable(CVideoDeviceModel::STREAM_TYPE type){ return streamOption[type].bEnable; }

    void SelectStreamIndex(CVideoDeviceModel::STREAM_TYPE type, short nIndex)
    { streamOption[type].nIndex = nIndex; }
    short GetStreamIndex(CVideoDeviceModel::STREAM_TYPE type){ return streamOption[type].nIndex; }

    void SetStreamFPS(CVideoDeviceModel::STREAM_TYPE type, int nFPS)
    { streamOption[type].nFPS = nFPS; }
    int GetStreamFPS(CVideoDeviceModel::STREAM_TYPE type){ return streamOption[type].nFPS; }

    void SetDepthROI(unsigned short nValue){ nDepthROI = nValue; }
    short GetDepthROI(){ return nDepthROI; }

    void EnablePlyFilter(bool bEnable){ bPlyFilter = bEnable; }
    bool IsPlyFilter(){ return bPlyFilter; }

    void SetDepthDataTransferControl(DEPTH_TRANSFER_CTRL dtc){ depthTransferControl = dtc; }
    DEPTH_TRANSFER_CTRL GetDepthDataTransferControl(){ return depthTransferControl; }

    void EnablePointCloudViewer(bool bEnable){ pointCloudViewerOption.bEnable = bEnable; }
    bool IsPointCloudViewer(){ return pointCloudViewerOption.bEnable; }

    void SetPointCloudViewerPointSize(unsigned short nSize){ pointCloudViewerOption.nPointSize = nSize; }
    unsigned short GetPointCloudSize(){ return pointCloudViewerOption.nPointSize; }

    void SetPointCloudViewOutputFormat(PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_FORMAT output){
        pointCloudViewerOption.outputFormat = output;
    }
    PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_FORMAT GetPointCloudViewOutputFormat(){
        return pointCloudViewerOption.outputFormat;
    }

    void SetZRange(int nNear, int nFar)
    {
        nZNear = nNear;
        nZFar  = nFar > nNear ? nFar : nNear;
    }
    void GetZRange(int &nNear, int &nFar)
    {
        nNear = nZNear;
        nFar  = nZFar;
    }

    void SetDefaultZRange(int nNear, int nFar)
    {
        nDefaultZNear = nNear;
        nDefaultZFar  = nFar > nNear ? nFar : nNear;
    }
    void GetDefaultZRange(int &nNear, int &nFar)
    {
        nNear = nDefaultZNear;
        nFar  = nDefaultZFar;
    }

    void SetModuleSyncMaster(bool bMaster){ bModuleSyncMaster = bMaster; }
    bool IsModuleSyncMaster(){ return bModuleSyncMaster; }

    void EnableModuleSync(bool bSync){ bModuleSync = bSync; }
    bool IsModuleSync(){ return bModuleSync; }

    void SetIRLevel(int nLevel){ nIRLevel = nLevel; }
    int GetIRLevel(){ return nIRLevel; }

    void SetIRExtend(bool bExtend){ bIRExtend = bExtend; }
    bool IsIRExtend(){ return bIRExtend; }

    void SetDepthDataType(int nType){ nDepthDataType = nType; }
    int GetDepthDataType(){ return nDepthDataType; }

    void SetRectify(bool IsRectify){ bRectify = IsRectify; }
    bool IsRectify(){ return bRectify; }

    void SetIMUSyncWithFrame(bool bIsSync){ bIMUSyncWithFrame = bIsSync; }
    bool IsIMUSyncWithFrame(){ return bIMUSyncWithFrame; }

private:
    int nIRLevel = 3;
    bool bIRExtend = false;

    int nDepthDataType = APC_DEPTH_DATA_11_BITS;
    bool bRectify = true;

    bool bModeConfig = false;

    StreamOption streamOption[CVideoDeviceModel::STREAM_TYPE_COUNT];
    DEPTH_TRANSFER_CTRL depthTransferControl = DEPTH_IMG_COLORFUL_TRANSFER;
    short nDepthROI = 20; // region of interest.
    bool bPlyFilter = false;
    int nDefaultZNear = 0;
    int nDefaultZFar = 1000;
    int nZNear = 0;
    int nZFar = 1000;

    bool bModuleSyncMaster;
    bool bModuleSync;
    PointCloudViewerOption pointCloudViewerOption;

    bool bIMUSyncWithFrame = false;
};

#endif // PREVIEWOPTIONS_H
