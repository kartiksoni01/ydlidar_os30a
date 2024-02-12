#ifndef CVIDEODEVICECAMERAPROPERTYWIDGET_H
#define CVIDEODEVICECAMERAPROPERTYWIDGET_H

#include <QWidget>
#include "CEYSDUIView.h"

namespace Ui {
class CVideoDeviceCameraPropertyWidget;
}

class CCameraPropertyController;
class CVideoDeviceCameraPropertyWidget : public QWidget,
                                         public CEYSDUIView
{
    Q_OBJECT
public:
    explicit CVideoDeviceCameraPropertyWidget(CCameraPropertyController *pCameraPropertyController,
                                              QWidget *parent = nullptr);
    ~CVideoDeviceCameraPropertyWidget();

    virtual void UpdateSelf();
    virtual void showEvent(QShowEvent *event);

private slots:
    void on_checkBox_auto_exposure_stateChanged(int state);

    void on_horizontalSlider_exposure_time_value_valueChanged(int nValue);

    void on_horizontalSlider_ae_target_valueChanged(int nValue);

    void on_checkBox_auto_white_balance_stateChanged(int state);

    void on_horizontalSlider_white_balance_temperature_value_valueChanged(int nValue);

    void on_radioButton_light_source_50hz_clicked();

    void on_radioButton_light_source_60hz_clicked();

    void on_radioButton_low_light_compensation_on_clicked();

    void on_radioButton_low_light_compensation_off_clicked();

    void on_comboBox_device_currentIndexChanged(int nIndex);

    void on_pushButton_reset_clicked();

    void on_pushButton_read_manuel_exposure_time_value_clicked();

    void on_pushButton_read_manuel_global_gain_value_clicked();

    void on_pushButton_write_manuel_exposure_time_value_clicked();

    void on_pushButton_write_manuel_global_gain_value_clicked();

private:
    void UpdateDevice();
    void UpdateExposureTime();
    void UpdateWhiteBalanceTemperature();
    void UpdateLightSource();
    void UpdateLowLightCompensation();
    void UpdateManuelExposureTime();
    void UpdateAETarget();
    float EV_value = 0;
private:
    CCameraPropertyController *m_pCameraPropertyController;
    Ui::CVideoDeviceCameraPropertyWidget *ui;
};

#endif // CVIDEODEVICECAMERAPROPERTYWIDGET_H
