#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "CEYSDDeviceManager.h"
#include "CVideoDeviceModel.h"
#include "CVideoDeviceDialog.h"
#include "utDisplayMetrics.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_bIsLogEnabled(false),
    m_bAutoPreview(false)
{
    ui->setupUi(this);
    if (APC_OK != CEYSDDeviceManager::GetInstance()->UpdateDevice() ||
        CEYSDDeviceManager::GetInstance()->GetDeviceModels().empty()){
        QMessageBox::critical(NULL, "Error", "EYSD Init Failed.", QMessageBox::Yes , QMessageBox::Yes);
        exit(0);
    }

    UpdateListState();

    if (1 == CEYSDDeviceManager::GetInstance()->GetDeviceModels().size()){
        m_bAutoPreview = true;
        ui->open_dialog_button->click();
        m_bAutoPreview = false;
    }

    int nScreenWidth, nScreenHeight;
    utDisplayMetrics::GetDisplayResolution(nScreenWidth, nScreenHeight);
    move((nScreenWidth - size().width()) / 2, (nScreenHeight - size().height()) / 2);

    AddUpdateTimer(2000);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    UpdateLogEnable();
    UpdateListState();
    return QMainWindow::paintEvent(event);
}

void MainWindow::UpdateListState()
{
    std::vector<CVideoDeviceModel *> models = CEYSDDeviceManager::GetInstance()->GetDeviceModels();
    if(models.size() != (size_t)ui->device_comboBox->count()) {
        ui->device_comboBox->clear();
        for (size_t index = 0 ; index < models.size() ; ++index){
            ui->device_comboBox->addItem("EYSD Video Device");
        }
        ui->device_comboBox->setCurrentIndex(0);
    }

    for(size_t i = 0 ; i < models.size() ; ++i){        
        QString sState;
        switch(models[i]->GetState()){
            case CVideoDeviceModel::OPENED: sState = "(Opened)"; break;
            case CVideoDeviceModel::STREAMING: sState = "(Streaming)"; break;
            case CVideoDeviceModel::RECONNECTING: continue;
            default: break;
        }

        CVideoDeviceModel::DeviceInfo deviceInfo = models[i]->GetDeviceInformation()[0];
        QString sDevicedescription;
        sDevicedescription.sprintf("%s %s %s",
                                  sState.toLocal8Bit().data(),
                                  deviceInfo.sModelName.find("80") != std::string::npos ?
                                  deviceInfo.sModelName.c_str() :
                                  deviceInfo.sFWVersion.c_str(),
                                  deviceInfo.deviceInfomation.strDevName
                                  );
        ui->device_comboBox->setItemText(i, sDevicedescription);
    }
}

void MainWindow::UpdateLogEnable()
{
    ui->enable_sdk_log_checkBox->blockSignals(true);
    ui->enable_sdk_log_checkBox->setChecked(m_bIsLogEnabled);
    ui->enable_sdk_log_checkBox->blockSignals(false);
}

void MainWindow::on_open_dialog_button_clicked()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    CEYSDDeviceManager::GetInstance()->EnableSDKLog(m_bIsLogEnabled);
    QApplication::restoreOverrideCursor();

    CVideoDeviceModel *pModel = CEYSDDeviceManager::GetInstance()->GetDeviceModels()[ui->device_comboBox->currentIndex()];
    if(CVideoDeviceModel::CLOSED == pModel->GetState()){
        m_dialogMap[pModel] = new CVideoDeviceDialog(pModel, this);
    }
    QString sDevName;
    sDevName.sprintf("%s", pModel->GetDeviceInformation()[0].deviceInfomation.strDevName);
    QString title = "DMPreview ";
    title += APC_VERSION;
    m_dialogMap[pModel]->setWindowTitle(title + " (" + sDevName + ") ");
    m_dialogMap[pModel]->show();
    m_dialogMap[pModel]->activateWindow();
    if (m_bAutoPreview) m_dialogMap[pModel]->AutoPreview();
    update();
}

void MainWindow::on_enable_sdk_log_checkBox_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_bIsLogEnabled = bIsChecked;
}
