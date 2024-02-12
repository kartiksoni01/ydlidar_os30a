#include "CVideoDeviceRegisterWidget.h"
#include "ui_CVideoDeviceRegisterWidget.h"
#include "CVideoDeviceController.h"

CVideoDeviceRegisterWidget::CVideoDeviceRegisterWidget(CRegisterReadWriteController *pRegisterReadWriteController,
                                                       QWidget *parent) :
QWidget(parent),
ui(new Ui::CVideoDeviceRegisterWidget),
m_pRegisterReadWriteController(pRegisterReadWriteController)
{
    setAttribute(Qt::WA_DeleteOnClose);

    ui->setupUi(this);

    m_requestAddressEdit.push_back(ui->lineEdit_register_address_1);
    m_requestAddressEdit.push_back(ui->lineEdit_register_address_2);
    m_requestAddressEdit.push_back(ui->lineEdit_register_address_3);
    m_requestAddressEdit.push_back(ui->lineEdit_register_address_4);
    m_requestAddressEdit.push_back(ui->lineEdit_register_address_5);
    m_requestAddressEdit.push_back(ui->lineEdit_register_address_6);
    m_requestAddressEdit.push_back(ui->lineEdit_register_address_7);
    m_requestAddressEdit.push_back(ui->lineEdit_register_address_8);

    m_requestValueEdit.push_back(ui->lineEdit_register_value_1);
    m_requestValueEdit.push_back(ui->lineEdit_register_value_2);
    m_requestValueEdit.push_back(ui->lineEdit_register_value_3);
    m_requestValueEdit.push_back(ui->lineEdit_register_value_4);
    m_requestValueEdit.push_back(ui->lineEdit_register_value_5);
    m_requestValueEdit.push_back(ui->lineEdit_register_value_6);
    m_requestValueEdit.push_back(ui->lineEdit_register_value_7);
    m_requestValueEdit.push_back(ui->lineEdit_register_value_8);

    for (size_t i = 0 ; i < m_requestAddressEdit.size() ; ++i){
        connect(m_requestAddressEdit[i], &QLineEdit::textChanged, this, [=](const QString &text){
            RegisterReadWriteOptions *pRegisterReadWriteOptions = m_pRegisterReadWriteController->GetRegisterReadWriteOptions();
            if (!text.compare("")) {
                pRegisterReadWriteOptions->SetRequestAddress(i, EOF);
            }else{
                int nAddress = text.toInt(nullptr, 16);
                pRegisterReadWriteOptions->SetRequestAddress(i, nAddress);
            }
        });
    }

    for (size_t i = 0 ; i < m_requestValueEdit.size() ; ++i){
        connect(m_requestValueEdit[i], &QLineEdit::textChanged, this, [=](const QString &text){
            RegisterReadWriteOptions *pRegisterReadWriteOptions = m_pRegisterReadWriteController->GetRegisterReadWriteOptions();
            if (!text.compare("")) {
                pRegisterReadWriteOptions->SetRequestValue(i, EOF);
            }else{
                int nValue = text.toInt(nullptr, 16);
                pRegisterReadWriteOptions->SetRequestValue(i, nValue);
            }
        });
    }

    UpdateUI();
}

CVideoDeviceRegisterWidget::~CVideoDeviceRegisterWidget()
{
    delete ui;
}

void CVideoDeviceRegisterWidget::UpdateSelf()
{
    setUpdatesEnabled(false);
    UpdateType();
    UpdateSlaveID();
    UpdateAddressSize();
    UpdateValueSize();
    UpdateSensorMode();
    UpdateAddressAndValue();
    UpdateOptions();
    UpdateButtonState();
    setUpdatesEnabled(true);
}

void CVideoDeviceRegisterWidget::showEvent(QShowEvent *event)
{
    UpdateUI();
}

void CVideoDeviceRegisterWidget::paintEvent(QPaintEvent *event)
{
    UpdateUI();
}

void CVideoDeviceRegisterWidget::UpdateType()
{
    switch (m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->GetType())
    {
        case RegisterReadWriteOptions::IC2:
            ui->radioButton_ic2->setChecked(true);
            break;
        case RegisterReadWriteOptions::ASIC:
            ui->radioButton_asic->setChecked(true);
            break;
        case RegisterReadWriteOptions::FW:
            ui->radioButton_fw->setChecked(true);
            break;
        default: break;
    }
}

void CVideoDeviceRegisterWidget::UpdateSlaveID()
{
    int nSlaveID = m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->GetSlaveID();
    if (EOF == nSlaveID) return;

    ui->lineEdit_slave_id->setText(QString::number(nSlaveID, 16));
}

void CVideoDeviceRegisterWidget::UpdateAddressSize()
{
    bool Is2Byte = FG_Address_2Byte == m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->GetAddressSize();
    ui->checkBox_address_2_byte->setChecked(Is2Byte);
}

void CVideoDeviceRegisterWidget::UpdateValueSize()
{
    bool Is2Byte = FG_Value_2Byte == m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->GetValueSize();
    ui->checkBox_value_2_byte->setChecked(Is2Byte);
}

void CVideoDeviceRegisterWidget::UpdateSensorMode()
{
    SENSORMODE_INFO sensorMode = m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->GetSensorMode();
    switch (sensorMode)
    {
        case SENSOR_A:
            ui->radioButton_sensor_1->setChecked(true);
            break;
        case SENSOR_B:
            ui->radioButton_sensor_2->setChecked(true);
            break;
        case SENSOR_C:
            ui->radioButton_sensor_3->setChecked(true);
            break;
        case SENSOR_D:
            ui->radioButton_sensor_4->setChecked(true);
            break;
        case SENSOR_BOTH:
            ui->radioButton_sensor_all->setChecked(true);
            break;
        default: break;
    }
}

void CVideoDeviceRegisterWidget::UpdateAddressAndValue()
{
    for (int i = 0 ; i < REGISTER_REQUEST_MAX_COUNT ; ++i){
        int address = m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->GetRequestAddress(i);
        if (EOF != address){
            m_requestAddressEdit[i]->setText(QString::number(address, 16));
        }

        int value = m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->GetRequestValue(i);
        if (EOF != value){
            m_requestValueEdit[i]->setText(QString::number(value, 16));
        }
    }
}

void CVideoDeviceRegisterWidget::UpdateOptions()
{
    bool IsRegisterPerodicRead = m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->IsPerodicRead();
    ui->checkBox_peroiodic_read->setChecked(IsRegisterPerodicRead);
    ui->spinBox_periodic_read_time->setEnabled(IsRegisterPerodicRead);
    ui->checkBox_save_log->setEnabled(IsRegisterPerodicRead);

    ui->spinBox_periodic_read_time->setValue(m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->GetPeriodTimeMs());
    ui->checkBox_save_log->setChecked(m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->IsSaveLog());
}

void CVideoDeviceRegisterWidget::UpdateButtonState()
{
    bool IsRegisterPerodicReadRunning = m_pRegisterReadWriteController->IsPeriodicReadRunning();
    ui->pushButton_read->setVisible(!IsRegisterPerodicReadRunning);
    ui->pushButton_stop_reading->setVisible(IsRegisterPerodicReadRunning);
}

void CVideoDeviceRegisterWidget::on_radioButton_ic2_clicked()
{
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetType(RegisterReadWriteOptions::IC2);
    UpdateType();
}

void CVideoDeviceRegisterWidget::on_radioButton_asic_clicked()
{
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetType(RegisterReadWriteOptions::ASIC);
    UpdateType();
}

void CVideoDeviceRegisterWidget::on_radioButton_fw_clicked()
{
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetType(RegisterReadWriteOptions::FW);
    UpdateType();
}

void CVideoDeviceRegisterWidget::on_lineEdit_slave_id_textChanged(const QString &text)
{
    if (!text.compare("")){
        m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetSlaveID(EOF);
    }else{
        m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetSlaveID(text.toInt(nullptr, 16));
    }
    UpdateSlaveID();
}

void CVideoDeviceRegisterWidget::on_checkBox_address_2_byte_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetAddressSize(bIsChecked ? FG_Address_2Byte :
                                                                                               FG_Address_1Byte);
    UpdateAddressSize();
}

void CVideoDeviceRegisterWidget::on_checkBox_value_2_byte_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetValueSize(bIsChecked ? FG_Value_2Byte :
                                                                                             FG_Value_1Byte);
    UpdateValueSize();
}

void CVideoDeviceRegisterWidget::on_radioButton_sensor_1_clicked()
{
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetSensorMode(SENSOR_A);
    UpdateSensorMode();
}

void CVideoDeviceRegisterWidget::on_radioButton_sensor_2_clicked()
{
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetSensorMode(SENSOR_B);
    UpdateSensorMode();
}

void CVideoDeviceRegisterWidget::on_radioButton_sensor_3_clicked()
{
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetSensorMode(SENSOR_C);
    UpdateSensorMode();
}

void CVideoDeviceRegisterWidget::on_radioButton_sensor_4_clicked()
{
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetSensorMode(SENSOR_D);
    UpdateSensorMode();
}

void CVideoDeviceRegisterWidget::on_radioButton_sensor_all_clicked()
{
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetSensorMode(SENSOR_BOTH);
    UpdateSensorMode();
}

void CVideoDeviceRegisterWidget::on_checkBox_peroiodic_read_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->EnablePerodicRead(bIsChecked);
    UpdateOptions();
}

void CVideoDeviceRegisterWidget::on_spinBox_periodic_read_time_valueChanged(int value)
{
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->SetPeriodTimeMs(value);
    UpdateOptions();
}

void CVideoDeviceRegisterWidget::on_checkBox_save_log_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pRegisterReadWriteController->GetRegisterReadWriteOptions()->EnableSaveLog(bIsChecked);
    UpdateOptions();
}

void CVideoDeviceRegisterWidget::on_pushButton_read_clicked()
{
    m_pRegisterReadWriteController->StartReadRegister();
    UpdateUI();
}

void CVideoDeviceRegisterWidget::on_pushButton_stop_reading_clicked()
{
    m_pRegisterReadWriteController->StopPeriodicRead();
    UpdateUI();
}

void CVideoDeviceRegisterWidget::on_pushButton_write_clicked()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    m_pRegisterReadWriteController->WriteRegister();
    QApplication::restoreOverrideCursor();
    UpdateUI();

}
