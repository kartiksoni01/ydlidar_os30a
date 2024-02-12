#include "CVideoDeviceCameraPropertyWidget.h"
#include "ui_CVideoDeviceCameraPropertyWidget.h"
#include "CCameraPropertyController.h"

CVideoDeviceCameraPropertyWidget::CVideoDeviceCameraPropertyWidget(CCameraPropertyController *pCameraPropertyController,
                                                                   QWidget *parent):
QWidget(parent),
ui(new Ui::CVideoDeviceCameraPropertyWidget),
m_pCameraPropertyController(pCameraPropertyController)
{
    ui->setupUi(this);
}

CVideoDeviceCameraPropertyWidget::~CVideoDeviceCameraPropertyWidget()
{
    delete ui;
}

void CVideoDeviceCameraPropertyWidget::UpdateSelf()
{
    UpdateDevice();
    UpdateExposureTime();
    UpdateWhiteBalanceTemperature();
    UpdateLightSource();
    UpdateLowLightCompensation();
    UpdateManuelExposureTime();
    UpdateAETarget();
}

void CVideoDeviceCameraPropertyWidget::showEvent(QShowEvent *event)
{
    UpdateUI();
}

void CVideoDeviceCameraPropertyWidget::UpdateDevice()
{
    ui->comboBox_device->blockSignals(true);
    ui->comboBox_device->clear();
    for(int i = 0 ; i < m_pCameraPropertyController->GetCameraPropertyDeviceCount() ; ++i){
        ui->comboBox_device->addItem(m_pCameraPropertyController->GetCameraPropertyDeviceName(i));
    }
    ui->comboBox_device->blockSignals(false);
    ui->comboBox_device->setCurrentIndex(m_pCameraPropertyController->GetCurrentCameraPropertyDeviceIndex());
}

void CVideoDeviceCameraPropertyWidget::UpdateExposureTime()
{
    bool bIsAutoExposureSupport = m_pCameraPropertyController->IsCameraPropertySupport(CCameraPropertyModel::AUTO_EXPOSURE);
    bool bIsExposureTimeSupport = m_pCameraPropertyController->IsCameraPropertySupport(CCameraPropertyModel::EXPOSURE_TIME);

    bool bIsAutoExposureValid = m_pCameraPropertyController->IsCameraPropertyValid(CCameraPropertyModel::AUTO_EXPOSURE);
    bool bIsExposureTimeValid = m_pCameraPropertyController->IsCameraPropertyValid(CCameraPropertyModel::EXPOSURE_TIME);

    ui->widget_auto_exposure->setVisible(bIsAutoExposureSupport);
    ui->widget_auto_exposure->setEnabled(bIsAutoExposureValid);

    int nValue = 0;
    if (bIsAutoExposureValid){
        m_pCameraPropertyController->GetValue(CCameraPropertyModel::AUTO_EXPOSURE, nValue);
    }
    bool IsAutoExposure = ((1 == nValue) && bIsAutoExposureSupport);
    ui->checkBox_auto_exposure->blockSignals(true);
    ui->checkBox_auto_exposure->setChecked(IsAutoExposure);
    ui->checkBox_auto_exposure->blockSignals(false);

    ui->widget_exposure_time->setVisible(!IsAutoExposure && bIsExposureTimeSupport);
    ui->widget_exposure_time->setEnabled(bIsExposureTimeValid);

    if(ui->widget_exposure_time->isVisible() && bIsExposureTimeValid){
        int nMin, nMax;
        m_pCameraPropertyController->GetRange(CCameraPropertyModel::EXPOSURE_TIME, nMin, nMax);

        ui->horizontalSlider_exposure_time_value->blockSignals(true);
        ui->horizontalSlider_exposure_time_value->setMinimum(nMin);
        ui->horizontalSlider_exposure_time_value->setMaximum(nMax);
        ui->label_exposure_time_min_value->setText(QString::number(nMin));
        ui->label_exposure_time_max_value->setText(QString::number(nMax));

        m_pCameraPropertyController->GetValue(CCameraPropertyModel::EXPOSURE_TIME, nValue);        
        ui->horizontalSlider_exposure_time_value->setValue(nValue);
        ui->horizontalSlider_exposure_time_value->blockSignals(false);
        ui->label_exposure_time_value->setText(QString::number(nValue));
    }
}

void CVideoDeviceCameraPropertyWidget::UpdateWhiteBalanceTemperature()
{
    bool bIsAutoWhiteBalanceSupport = m_pCameraPropertyController->IsCameraPropertySupport(CCameraPropertyModel::AUTO_WHITE_BLANCE);
    bool bIsWhiteBalanceSupport = m_pCameraPropertyController->IsCameraPropertySupport(CCameraPropertyModel::WHITE_BLANCE_TEMPERATURE);

    bool bIsAutoWhiteBalanceValid = m_pCameraPropertyController->IsCameraPropertyValid(CCameraPropertyModel::AUTO_WHITE_BLANCE);
    bool bIsWhiteBalanceTemperatureValid = m_pCameraPropertyController->IsCameraPropertyValid(CCameraPropertyModel::WHITE_BLANCE_TEMPERATURE);

    ui->widget_auto_white_balance->setVisible(bIsAutoWhiteBalanceSupport);
    ui->widget_auto_white_balance->setEnabled(bIsAutoWhiteBalanceValid);

    int nValue = 0;
    if (bIsAutoWhiteBalanceValid){
        m_pCameraPropertyController->GetValue(CCameraPropertyModel::AUTO_WHITE_BLANCE, nValue);
    }
    bool IsAutoWhiteBalance = ((1 == nValue) && bIsAutoWhiteBalanceSupport);
    ui->checkBox_auto_white_balance->blockSignals(true);
    ui->checkBox_auto_white_balance->setChecked(IsAutoWhiteBalance);
    ui->checkBox_auto_white_balance->blockSignals(false);

    ui->widget_white_balance_temperature->setVisible(!IsAutoWhiteBalance && bIsWhiteBalanceSupport);
    ui->widget_white_balance_temperature->setEnabled(bIsWhiteBalanceTemperatureValid);

    if(ui->widget_white_balance_temperature->isVisible() && bIsWhiteBalanceTemperatureValid){
        int nMin, nMax;
        m_pCameraPropertyController->GetRange(CCameraPropertyModel::WHITE_BLANCE_TEMPERATURE, nMin, nMax);

        ui->horizontalSlider_white_balance_temperature_value->blockSignals(true);
        ui->horizontalSlider_white_balance_temperature_value->setMinimum(nMin);
        ui->horizontalSlider_white_balance_temperature_value->setMaximum(nMax);
        ui->label_white_balance_temperature_min_value->setText(QString::number(nMin));
        ui->label_white_balance_temperature_max_value->setText(QString::number(nMax));

        m_pCameraPropertyController->GetValue(CCameraPropertyModel::WHITE_BLANCE_TEMPERATURE, nValue);
        ui->horizontalSlider_white_balance_temperature_value->setValue(nValue);
        ui->horizontalSlider_white_balance_temperature_value->blockSignals(false);
        ui->label_white_balance_temperature_value->setText(QString::number(nValue));
    }
}

void CVideoDeviceCameraPropertyWidget::UpdateLightSource()
{
    bool bIsLightSourceSupport = m_pCameraPropertyController->IsCameraPropertySupport(CCameraPropertyModel::LIGHT_SOURCE);
    ui->widget_light_source->setVisible(bIsLightSourceSupport);
    if(!bIsLightSourceSupport) return;

    bool bIsLightSourceValid = m_pCameraPropertyController->IsCameraPropertyValid(CCameraPropertyModel::LIGHT_SOURCE);
    ui->widget_light_source->setEnabled(bIsLightSourceValid);

    int nValue = CCameraPropertyModel::VALUE_50HZ;
    if (bIsLightSourceValid){
        m_pCameraPropertyController->GetValue(CCameraPropertyModel::LIGHT_SOURCE, nValue);
    }
    ui->radioButton_light_source_50hz->blockSignals(true);
    ui->radioButton_light_source_60hz->blockSignals(true);
    switch (nValue){
        case CCameraPropertyModel::VALUE_50HZ:
            ui->radioButton_light_source_50hz->setChecked(true);
            break;
        case CCameraPropertyModel::VALUE_60HZ:
            ui->radioButton_light_source_60hz->setChecked(true);
            break;
        default: break;
    }
    ui->radioButton_light_source_50hz->blockSignals(false);
    ui->radioButton_light_source_60hz->blockSignals(false);
}

void CVideoDeviceCameraPropertyWidget::UpdateLowLightCompensation()
{
    bool bIsLowLightCompensationSupport = m_pCameraPropertyController->IsCameraPropertySupport(CCameraPropertyModel::LOW_LIGHT_COMPENSATION);
    ui->widget_low_light_compenstation->setVisible(bIsLowLightCompensationSupport);
    if(!bIsLowLightCompensationSupport) return;

    bool bIsLowLightCompensationValid = m_pCameraPropertyController->IsCameraPropertyValid(CCameraPropertyModel::LOW_LIGHT_COMPENSATION);
    ui->widget_low_light_compenstation->setEnabled(bIsLowLightCompensationValid);

    int nValue = 0;
    if (bIsLowLightCompensationValid){
        m_pCameraPropertyController->GetValue(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, nValue);
    }

    ui->radioButton_low_light_compensation_on->blockSignals(true);
    ui->radioButton_low_light_compensation_off->blockSignals(true);
    if(1 == nValue){
        ui->radioButton_low_light_compensation_on->setChecked(true);
    }else{
        ui->radioButton_low_light_compensation_off->setChecked(true);
    }
    ui->radioButton_low_light_compensation_on->blockSignals(false);
    ui->radioButton_low_light_compensation_off->blockSignals(false);
}

void CVideoDeviceCameraPropertyWidget::UpdateManuelExposureTime()
{
    bool bValid = m_pCameraPropertyController->IsManuelExposureTimeValid();

    ui->doubleSpinBox_manuel_exposure_time_value->setEnabled(bValid);
    ui->pushButton_write_manuel_exposure_time_value->setEnabled(bValid);

    ui->doubleSpinBox_manuel_global_gain_value->setEnabled(bValid);
    ui->pushButton_write_manuel_global_gain_value->setEnabled(bValid);
}

void CVideoDeviceCameraPropertyWidget::UpdateAETarget()
{
    bool bIsAutoExposureSupport = m_pCameraPropertyController->IsCameraPropertySupport(CCameraPropertyModel::AUTO_EXPOSURE);
    bool bIsAutoExposureValid = m_pCameraPropertyController->IsCameraPropertyValid(CCameraPropertyModel::AUTO_EXPOSURE);

    int nValue = 0;
    if (bIsAutoExposureValid){
        m_pCameraPropertyController->GetValue(CCameraPropertyModel::AUTO_EXPOSURE, nValue);
    }
    bool IsAutoExposure = ((1 == nValue) && bIsAutoExposureSupport);
    ui->checkBox_auto_exposure->blockSignals(true);
    ui->checkBox_auto_exposure->setChecked(IsAutoExposure);
    ui->checkBox_auto_exposure->blockSignals(false);

    ui->widget_ae_target->setVisible(IsAutoExposure);
    ui->widget_ae_target->setEnabled(IsAutoExposure);

    if(ui->widget_ae_target->isVisible()){
        //AE target index range from -6 to 9
        ui->horizontalSlider_ae_target->setMinimum(-6);
        ui->horizontalSlider_ae_target->setMaximum(9);
        ui->label_ae_value->setText(QString::number(EV_value));
    }
}

void CVideoDeviceCameraPropertyWidget::on_checkBox_auto_exposure_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pCameraPropertyController->SetValue(CCameraPropertyModel::AUTO_EXPOSURE, bIsChecked ? 1 : 0);
    UpdateExposureTime();
    UpdateManuelExposureTime();
    UpdateAETarget();
}

void CVideoDeviceCameraPropertyWidget::on_horizontalSlider_exposure_time_value_valueChanged(int nValue)
{
    m_pCameraPropertyController->SetValue(CCameraPropertyModel::EXPOSURE_TIME, nValue);
    UpdateExposureTime();
}

void CVideoDeviceCameraPropertyWidget::on_horizontalSlider_ae_target_valueChanged(int nValue)
{
    EV_value = m_pCameraPropertyController->SetAETarget(nValue);
    ui->label_ae_value->setText(QString::number(EV_value));
    UpdateAETarget();
}


void CVideoDeviceCameraPropertyWidget::on_checkBox_auto_white_balance_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pCameraPropertyController->SetValue(CCameraPropertyModel::AUTO_WHITE_BLANCE, bIsChecked ? 1 : 0);
    UpdateWhiteBalanceTemperature();
}

void CVideoDeviceCameraPropertyWidget::on_horizontalSlider_white_balance_temperature_value_valueChanged(int nValue)
{
    m_pCameraPropertyController->SetValue(CCameraPropertyModel::WHITE_BLANCE_TEMPERATURE, nValue);
    UpdateWhiteBalanceTemperature();
}

void CVideoDeviceCameraPropertyWidget::on_radioButton_light_source_50hz_clicked()
{
    int nValue;
    m_pCameraPropertyController->GetValue(CCameraPropertyModel::LIGHT_SOURCE, nValue);
    if (CCameraPropertyModel::VALUE_50HZ == nValue) return;
    m_pCameraPropertyController->SetValue(CCameraPropertyModel::LIGHT_SOURCE, CCameraPropertyModel::VALUE_50HZ);
    UpdateLightSource();
}

void CVideoDeviceCameraPropertyWidget::on_radioButton_light_source_60hz_clicked()
{
    int nValue;
    m_pCameraPropertyController->GetValue(CCameraPropertyModel::LIGHT_SOURCE, nValue);
    if (CCameraPropertyModel::VALUE_60HZ == nValue) return;
    m_pCameraPropertyController->SetValue(CCameraPropertyModel::LIGHT_SOURCE, CCameraPropertyModel::VALUE_60HZ);
    UpdateLightSource();
}

void CVideoDeviceCameraPropertyWidget::on_radioButton_low_light_compensation_on_clicked()
{
    int nValue;
    m_pCameraPropertyController->GetValue(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, nValue);
    if (1 == nValue) return;
    m_pCameraPropertyController->SetValue(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, 1);
    UpdateLowLightCompensation();
}

void CVideoDeviceCameraPropertyWidget::on_radioButton_low_light_compensation_off_clicked()
{
    int nValue;
    m_pCameraPropertyController->GetValue(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, nValue);
    if (0 == nValue) return;
    m_pCameraPropertyController->SetValue(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, 0);
    UpdateLowLightCompensation();
}

void CVideoDeviceCameraPropertyWidget::on_comboBox_device_currentIndexChanged(int nIndex)
{
    if(nIndex == m_pCameraPropertyController->GetCurrentCameraPropertyDeviceIndex()) return;

    m_pCameraPropertyController->SelectCurrentCameraProperty(nIndex);
    UpdateUI();
}

void CVideoDeviceCameraPropertyWidget::on_pushButton_reset_clicked()
{
    m_pCameraPropertyController->ResetCameraProperty();
    UpdateUI();
}

void CVideoDeviceCameraPropertyWidget::on_pushButton_read_manuel_exposure_time_value_clicked()
{
    ui->doubleSpinBox_manuel_exposure_time_value->setValue(m_pCameraPropertyController->GetManuelExposureTimeMs());
}

void CVideoDeviceCameraPropertyWidget::on_pushButton_read_manuel_global_gain_value_clicked()
{
    ui->doubleSpinBox_manuel_global_gain_value->setValue(m_pCameraPropertyController->GetManuelGlobalGain());
}

void CVideoDeviceCameraPropertyWidget::on_pushButton_write_manuel_exposure_time_value_clicked()
{
    m_pCameraPropertyController->SetManuelExposureTimeMs(ui->doubleSpinBox_manuel_exposure_time_value->value());
}

void CVideoDeviceCameraPropertyWidget::on_pushButton_write_manuel_global_gain_value_clicked()
{
    m_pCameraPropertyController->SetManuelGlobalGain(ui->doubleSpinBox_manuel_global_gain_value->value());
}
