#ifndef CVIDEODEVICEDIALOG_H
#define CVIDEODEVICEDIALOG_H

#include <QDialog>
#include "CEYSDUIView.h"
#include "CPointCloudViewerDialog.h"

namespace Ui {
class VideoDeviceDialog;
}

class CVideoDeviceDepthFilterWidget;
class CVideoDevicePreviewWidget;
class CVideoDeviceRegisterWidget;
class CVideoDeviceCameraPropertyWidget;
class CVideoDeviceDepthAccuracyWidget;
class CVideoDeviceIMUWidget;
class CVideoDeviceAudoWidget;
class CPreviewDialog;
class CVideoDeviceController;
class CVideoDeviceDialog : public QDialog,
                           public CEYSDUIView
{
    Q_OBJECT
public:
    explicit CVideoDeviceDialog(CVideoDeviceModel *pVideoDeviceModel, QWidget *parent);
    ~CVideoDeviceDialog();

    void AutoPreview();

    virtual void showEvent(QShowEvent *event);
    virtual void closeEvent(QCloseEvent *event);

    virtual void BeginFrameSync();
    virtual void EndFrameSync();

    virtual void UpdateSelf();
    virtual void OpenPreviewView(CVideoDeviceModel::STREAM_TYPE type);
    virtual void ClosePreviewView(CVideoDeviceModel::STREAM_TYPE type);
    virtual CImageDataModel *GetPreviewImageData(CVideoDeviceModel::STREAM_TYPE type);
    virtual int ImageCallback(APCImageType::Value imageType,
                              CVideoDeviceModel::STREAM_TYPE streamType,
                              BYTE *pImageBuffer, int nImageSize,
                              int nWidth, int nHeight, int nSerialNumber,
                              void *pUserData);

    virtual void ClosePointCloud();
    virtual int PointCloudCallback(std::vector<float> &cloudPoints, std::vector<BYTE> &colors);
    virtual void UpdateColorPalette();

private slots:
    void on_pushButton_rectify_read_clicked();
    void on_pushButton_temperature_clicked();

private:

    void UpdateTabView();
    void UpdatePreview();
    void UpdateRegister();
    void UpdateCameraProperty();
    void UpdateDepthAccuracy();
    void UpdateIMU();
    void UpdateAudio();
    void UpdateDepthFilter();
    void UpdateModuleInformation();
    void UpdateRectifyLog();
    void RelocateDialogPosition();
    void UpdateThermalUI();

private:
    CVideoDeviceController *m_pVideoDeviceController = nullptr;
    CVideoDeviceModel *m_pVideoDeviceModel = nullptr;
    Ui::VideoDeviceDialog *ui = nullptr;
    CPreviewDialog *m_pPreviewDialog[CVideoDeviceModel::STREAM_TYPE_COUNT] = {0};
    CVideoDevicePreviewWidget *m_pPreviewWidget;
    CVideoDeviceRegisterWidget *m_pRegisterWidget;
    CVideoDeviceCameraPropertyWidget *m_pCameraPropertyWidget;
    CVideoDeviceDepthAccuracyWidget *m_pDepthAccuracyWidget;
    CVideoDeviceIMUWidget *m_pIMUWidget;
    CVideoDeviceAudoWidget *m_pAudioWidget;
    CVideoDeviceDepthFilterWidget *m_pDepthFilterWidget;

    CPointCloudViewerDialog *m_pPointCloudViewerDialog;
    QMutex m_previewDialogMutex;

};

#endif // CVIDEODEVICEDIALOG_H
