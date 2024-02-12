#ifndef CVIDEODEVICEIMUWIDGET_H
#define CVIDEODEVICEIMUWIDGET_H

#include <QWidget>
#include "CEYSDUIView.h"
#include "CIMUDataViewerWidget.h"

namespace Ui {
class CVideoDeviceIMUWidget;
}

class CIMUDataController;
class CVideoDeviceIMUWidget : public QWidget,
                              public CEYSDUIView
{
    Q_OBJECT
public:    
    explicit CVideoDeviceIMUWidget(CIMUDataController *pIMUDataController,
                                   QWidget *parent = nullptr);
    ~CVideoDeviceIMUWidget();

    virtual void UpdateSelf();

    virtual void showEvent(QShowEvent *event);
    virtual void paintEvent(QPaintEvent *event);
private slots:
    void on_comboBox_imu_data_option_currentIndexChanged(int nIndex);

    void on_pushButton_enable_imu_clicked();

    void on_pushButton_disable_imu_clicked();

    void on_pushButton_get_status_clicked();

    void on_pushButton_get_module_name_clicked();

    void on_pushButton_get_fw_version_clicked();

    void on_pushButton_save_imu_raw_data_clicked();

    void on_pushButton_stop_save_imu_raw_data_clicked();

    void on_pushButton_start_fw_calibration_clicked();

    void on_checkBox_frame_sync_stateChanged(int state);

    void on_pushButton_imu_data_3d_reset_clicked();

private:
    void Update3DModuleWidget();
    void UpdateFrameSync();
    void UpdateInformation();
    void UpdateIMUData();
    void UpdateIMUDataOption();
    void UpdateIMUEnable();
    void UpdateFWCalibrationInfo();
    void UpdateSaveIMURAWDataInfo();

private:
    Ui::CVideoDeviceIMUWidget *ui;
    CIMUDataController *m_pIMUDataController;
    CIMUDataViewerWidget m_imuDataViewer;
};

#endif // CVIDEODEVICEIMUWIDGET_H
