#include "CVideoDeviceIMUWidget.h"
#include "ui_CVideoDeviceIMUWidget.h"
#include "CIMUDataController.h"


CVideoDeviceIMUWidget::CVideoDeviceIMUWidget(CIMUDataController *pIMUDataController,
                                             QWidget *parent):
QWidget(parent),
ui(new Ui::CVideoDeviceIMUWidget),
m_pIMUDataController(pIMUDataController),
m_imuDataViewer(this)
{
    ui->setupUi(this);

    m_imuDataViewer.setVisible(false);
    ui->widget_imu_data_3d->layout()->addWidget(&m_imuDataViewer);

    AddUpdateTimer(10);
}

CVideoDeviceIMUWidget::~CVideoDeviceIMUWidget()
{
    delete ui;
}

void CVideoDeviceIMUWidget::Update3DModuleWidget()
{
    bool isEnableImu3dView = m_pIMUDataController->GetIMUModel()->GetType() == CIMUModel::IMU_9_AXIS;
    m_imuDataViewer.setVisible(isEnableImu3dView);
    ui->pushButton_imu_data_3d_reset->setVisible(isEnableImu3dView);
}

void CVideoDeviceIMUWidget::UpdateSelf()
{
    Update3DModuleWidget();
    UpdateFrameSync();
    UpdateInformation();
    UpdateIMUData();
    UpdateIMUDataOption();
    UpdateIMUEnable();
    UpdateFWCalibrationInfo();
    UpdateSaveIMURAWDataInfo();
}

void CVideoDeviceIMUWidget::showEvent(QShowEvent *event)
{
    UpdateUI();
    QWidget::showEvent(event);
}

void CVideoDeviceIMUWidget::paintEvent(QPaintEvent *event)
{
    UpdateInformation();
    UpdateIMUData();
    UpdateFWCalibrationInfo();
    if (m_imuDataViewer.isVisible()){
        IMUData imuData = m_pIMUDataController->CloneIMUData();
        m_imuDataViewer.updateData(&imuData, m_pIMUDataController->GetIMUModel()->GetType());
    }
    QWidget::paintEvent(event);
}

void CVideoDeviceIMUWidget::UpdateFrameSync()
{
    ui->checkBox_frame_sync->setChecked(m_pIMUDataController->IsIMUSycnWithFrame());
}

void CVideoDeviceIMUWidget::UpdateInformation()
{
    ui->label_content_imu_information->setText(m_pIMUDataController->GetIMUInformationMessage());
}

void CVideoDeviceIMUWidget::UpdateIMUData()
{
    ui->label_content_imu_data->setText(m_pIMUDataController->GetIMUDataMessage());
}

void CVideoDeviceIMUWidget::UpdateIMUDataOption()
{
    if (!m_pIMUDataController->GetIMUModel()) return;

    std::vector<CIMUModel::DATA_FORMAT> imuDataFormat = m_pIMUDataController->GetIMUModel()->GetSupportDataFormat();
    if (imuDataFormat.empty()){
        ui->comboBox_imu_data_option->setVisible(false);
        return;
    }

    ui->comboBox_imu_data_option->blockSignals(true);
    ui->comboBox_imu_data_option->clear();    
    for(size_t i = 0 ; i < imuDataFormat.size() ; ++i){
        QString sFormat = QString::number(i + 1) + ". ";
        switch(imuDataFormat[i]){
            case CIMUModel::RAW_DATA_WITHOUT_OFFSET: sFormat += "Raw data without format."; break;
            case CIMUModel::RAW_DATA_WITH_OFFSET: sFormat += "Raw data with format."; break;
            case CIMUModel::OFFSET_DATA: sFormat += "Offset data."; break;
            case CIMUModel::DMP_DATA_WITHOT_OFFSET: sFormat += "Dmp Data without offset."; break;
            case CIMUModel::DMP_DATA_WITH_OFFSET: sFormat += "Dmp data with offset."; break;
            default: continue;
        }

        ui->comboBox_imu_data_option->addItem(sFormat);
    }
    ui->comboBox_imu_data_option->setCurrentIndex(m_pIMUDataController->GetCurrentDataFormatIndex());
    ui->comboBox_imu_data_option->blockSignals(false);
}

void CVideoDeviceIMUWidget::UpdateIMUEnable()
{
    bool bIsCapturingIMU = m_pIMUDataController->IsCapturingData();
    ui->pushButton_disable_imu->setVisible(bIsCapturingIMU);
    ui->pushButton_enable_imu->setVisible(!bIsCapturingIMU);
}

void CVideoDeviceIMUWidget::UpdateFWCalibrationInfo()
{
    if (!m_pIMUDataController->GetIMUModel()) return;

    if (CIMUModel::IMU_9_AXIS == m_pIMUDataController->GetIMUModel()->GetType())
    {
        ui->pushButton_start_fw_calibration->setVisible(false);
        return;
    }

    ui->pushButton_start_fw_calibration->setEnabled(!m_pIMUDataController->IsCalibration());
}

void CVideoDeviceIMUWidget::UpdateSaveIMURAWDataInfo()
{
    bool bIsLogData = m_pIMUDataController->IsLogData();
    ui->pushButton_save_imu_raw_data->setVisible(!bIsLogData);
    ui->pushButton_stop_save_imu_raw_data->setVisible(bIsLogData);
    ui->label_save_imu_raw_data_info->setVisible(bIsLogData);
    if(ui->label_save_imu_raw_data_info->isVisible()){
        QString sLogPath;
        sLogPath.sprintf("Logs are saved to \"../out/IMULog/%s.\"", m_pIMUDataController->GetLogFileName());
        ui->label_save_imu_raw_data_info->setText(sLogPath);
    }
}

void CVideoDeviceIMUWidget::on_comboBox_imu_data_option_currentIndexChanged(int nIndex)
{
    m_pIMUDataController->SelectDataFormatIndex(nIndex);
    UpdateIMUDataOption();
}

void CVideoDeviceIMUWidget::on_pushButton_enable_imu_clicked()
{
    m_pIMUDataController->StartCaptureData();
    UpdateIMUEnable();
}

void CVideoDeviceIMUWidget::on_pushButton_disable_imu_clicked()
{
    m_pIMUDataController->StopCaptrueData();
    UpdateIMUEnable();
}

void CVideoDeviceIMUWidget::on_pushButton_get_status_clicked()
{
    m_pIMUDataController->GetStatus();
    UpdateInformation();
}

void CVideoDeviceIMUWidget::on_pushButton_get_module_name_clicked()
{
    m_pIMUDataController->GetModuleName();
    UpdateInformation();
}

void CVideoDeviceIMUWidget::on_pushButton_get_fw_version_clicked()
{
    m_pIMUDataController->GetFWVersion();
    UpdateInformation();
}

void CVideoDeviceIMUWidget::on_pushButton_save_imu_raw_data_clicked()
{
    m_pIMUDataController->StartLogData();
    UpdateSaveIMURAWDataInfo();
}

void CVideoDeviceIMUWidget::on_pushButton_stop_save_imu_raw_data_clicked()
{
    m_pIMUDataController->StopLogData();
    UpdateSaveIMURAWDataInfo();
}

void CVideoDeviceIMUWidget::on_pushButton_start_fw_calibration_clicked()
{
    m_pIMUDataController->StartCalibration();
    UpdateFWCalibrationInfo();
}

void CVideoDeviceIMUWidget::on_checkBox_frame_sync_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pIMUDataController->SetIMUSyncWithFrame(bIsChecked);
}

void CVideoDeviceIMUWidget::on_pushButton_imu_data_3d_reset_clicked()
{
    m_pIMUDataController->ResetQuaternion();
}
