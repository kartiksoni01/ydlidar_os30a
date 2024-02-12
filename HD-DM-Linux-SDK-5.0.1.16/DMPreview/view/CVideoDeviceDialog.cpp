#include "CVideoDeviceDialog.h"
#include "ui_CVideoDeviceDialog.h"
#include <QFile>
#include <QMessageBox>
#include "CVideoDeviceController.h"
#include "CVideoDevicePreviewWidget.h"
#include "CVideoDeviceRegisterWidget.h"
#include "CVideoDeviceCameraPropertyWidget.h"
#include "CVideoDeviceIMUWidget.h"
#include "CVideoDeviceAudoWidget.h"
#include "CVideoDeviceDepthAccuracyWidget.h"
#include "CPreviewDialog.h"
#include "CImageDataModel.h"
#include "utDisplayMetrics.h"
#include "CEYSDDeviceManager.h"
#include "CVideoDeviceDepthFilterWidget.h"
#include "CFrameSyncManager.h"

CVideoDeviceDialog::CVideoDeviceDialog(CVideoDeviceModel *pVideoDeviceModel, QWidget *parent):
    QDialog(parent),
    ui(new Ui::VideoDeviceDialog),
    m_pVideoDeviceModel(pVideoDeviceModel),
    m_pPreviewWidget(nullptr),
    m_pRegisterWidget(nullptr),
    m_pCameraPropertyWidget(nullptr),
    m_pDepthAccuracyWidget(nullptr),
    m_pIMUWidget(nullptr),
    m_pAudioWidget(nullptr),
    m_pDepthFilterWidget(nullptr),
    m_pPointCloudViewerDialog(nullptr)
{
    setAttribute(Qt::WA_DeleteOnClose);

    ui->setupUi(this);

    m_pVideoDeviceController = new CVideoDeviceController(pVideoDeviceModel, this);

    QRect availableRect = utDisplayMetrics::GetAvailableGeometry();
    move(availableRect.x() + (availableRect.width() - size().width()) / 2,
         availableRect.y() + (availableRect.height() - size().height()) / 2);

    ui->pushButton_rectify_read->setAutoDefault(false);

    UpdateUI();
}

CVideoDeviceDialog::~CVideoDeviceDialog()
{
    delete ui;
}

void CVideoDeviceDialog::AutoPreview()
{
    if (m_pPreviewWidget) m_pPreviewWidget->Preview();
}

void CVideoDeviceDialog::showEvent(QShowEvent *event)
{
    UpdateUI();

    QDialog::showEvent(event);
}

void CVideoDeviceDialog::closeEvent(QCloseEvent *event)
{
    for (CPreviewDialog *pPreviewDialog : m_pPreviewDialog){
        if (pPreviewDialog){
            pPreviewDialog->close();
        }
    }

    if (m_pPointCloudViewerDialog){
        m_pPointCloudViewerDialog->close();
    }

    m_pVideoDeviceModel->ChangeState(CVideoDeviceModel::CLOSED);

    QDialog::closeEvent(event);
}

void CVideoDeviceDialog::UpdateSelf()
{
    UpdateTabView();
    UpdateModuleInformation();
    UpdateRectifyLog();
}

void CVideoDeviceDialog::BeginFrameSync()
{
    for (CPreviewDialog *pPreviewDialog : m_pPreviewDialog){
        if (pPreviewDialog){
            pPreviewDialog->setUpdatesEnabled(false);
        }
    }
    setUpdatesEnabled(false);
}

void CVideoDeviceDialog::EndFrameSync()
{
    for (CPreviewDialog *pPreviewDialog : m_pPreviewDialog){
        if (pPreviewDialog){
            pPreviewDialog->setUpdatesEnabled(true);
        }
    }
    setUpdatesEnabled(true);

    RUN_ON_UI_THREAD(
    m_pIMUWidget->UpdateSelf();
    m_pIMUWidget->repaint();
    for (CPreviewDialog *pPreviewDialog : m_pPreviewDialog){
        if (pPreviewDialog){
            pPreviewDialog->repaint();
        }
    }
    );

}

void CVideoDeviceDialog::OpenPreviewView(CVideoDeviceModel::STREAM_TYPE type)
{
    RUN_ON_UI_THREAD(
    if(!m_pPreviewDialog[type]){
        CImageDataModel *pImageDataModel = CImageDataModelFactory::CreateCImageDataModel(type, m_pVideoDeviceController);
        m_pPreviewDialog[type] = new CPreviewDialog(m_pVideoDeviceController, pImageDataModel, this);
        if(!m_pVideoDeviceController->GetPreviewOptions()->IsPointCloudViewer()){
			#ifdef TI_EVM
            m_pPreviewDialog[type]->adjustSize();
			#endif
            m_pPreviewDialog[type]->show();
            RelocateDialogPosition();
        }
    }
    );
}

void CVideoDeviceDialog::ClosePreviewView(CVideoDeviceModel::STREAM_TYPE type)
{
    RUN_ON_UI_THREAD(
    if (m_pPreviewDialog[type]){
        m_pPreviewDialog[type]->close();
        m_pPreviewDialog[type] = nullptr;
    }
    );
}

void CVideoDeviceDialog::ClosePointCloud()
{
    RUN_ON_UI_THREAD(
    if (m_pPointCloudViewerDialog){
        m_pPointCloudViewerDialog->close();
        m_pPointCloudViewerDialog = nullptr;
    }
    );
}

void CVideoDeviceDialog::UpdateColorPalette()
{
    for (CPreviewDialog *pPreviewDlog : m_pPreviewDialog){
        if (!pPreviewDlog) continue;

        CImageDataModel *pImageDataModel = pPreviewDlog->GetImageDataModel();
        if (!pImageDataModel) continue;

        if (CImageDataModel::DEPTH == pImageDataModel->GetModelType()){
            int nZNear, nZFar;
            m_pVideoDeviceController->GetPreviewOptions()->GetZRange(nZNear, nZFar);
            ((CImageDataModel_Depth *)pImageDataModel)->UpdateColorPalette(nZNear, nZFar);
        }
    }
}

void CVideoDeviceDialog::RelocateDialogPosition()
{
    std::vector<CPreviewDialog *> visiblePreviewDialog;
    for (CPreviewDialog *pPreviewDlog : m_pPreviewDialog){
        if (pPreviewDlog && pPreviewDlog->isVisible()) {
            visiblePreviewDialog.push_back(pPreviewDlog);
        }
    }

    const int nMargin = 5;
    QRect availableRect = utDisplayMetrics::GetAvailableGeometry();
    int nLeft = availableRect.x();
    int nTop = availableRect.y();
    int nWidth = availableRect.width();
    int nHeight = availableRect.height();
    int nTitleHeight = QApplication::style()->pixelMetric(QStyle::PM_TitleBarHeight);

    if (3 == visiblePreviewDialog.size()){
        int nWidthPerPreview = (nWidth - nMargin * 2) / 3;
        int nHeightPerPreview = (nHeight - nTitleHeight) / 2;

        for (size_t i = 0 ; i < visiblePreviewDialog.size() ; ++i){
            visiblePreviewDialog[i]->move(nLeft + (i * (nWidthPerPreview + nMargin)),
                                          nTop);
            visiblePreviewDialog[i]->SetPreferSize(nWidthPerPreview,
                                                   nHeightPerPreview);
        }
    }else{
        int nWidthPerPreviewTop = (nWidth - nMargin) / 2;
        int nHeightPerPreviewTop = (nHeight - (nTitleHeight * 2) - nMargin) / 2;
        int nWidthPerPreviewBottom = (nWidth - nMargin * 2 ) / 3;
        int nHeightPerPreviewBottom = (nHeight - (nTitleHeight * 2) - nMargin) / 2;

        for (size_t i = 0 ; i < visiblePreviewDialog.size() ; ++i){

            int nRowIndex = (i > 1) ? 1 : 0;

            switch(nRowIndex){
                case 0:
                    visiblePreviewDialog[i]->move(nLeft + (i * (nWidthPerPreviewTop + nMargin)),
                                                  nTop);
                    visiblePreviewDialog[i]->SetPreferSize(nWidthPerPreviewTop,
                                                           nHeightPerPreviewTop);
                    break;
                case 1:
                    if (5 == visiblePreviewDialog.size()){
                        visiblePreviewDialog[i]->move(nLeft + ((i - 2) * (nWidthPerPreviewBottom + nMargin)),
                                                      nTop + nHeightPerPreviewTop + nTitleHeight + nMargin);
                        visiblePreviewDialog[i]->SetPreferSize(nWidthPerPreviewBottom,
                                                               nHeightPerPreviewTop);
                    }else{
                        visiblePreviewDialog[i]->move(nLeft + ((i - 2) * (nWidthPerPreviewTop + nMargin)),
                                                      nTop + nHeightPerPreviewTop + nTitleHeight + nMargin);
                        visiblePreviewDialog[i]->SetPreferSize(nWidthPerPreviewTop,
                                                               nHeightPerPreviewTop);
                    }

                    break;
                default: continue;
            }
			#ifndef TI_EVM
            visiblePreviewDialog[i]->ResizePreviewDialog();
			#endif


        }
    }

    move(nLeft, nTop + nHeight / 2 + nMargin);

}

int CVideoDeviceDialog::ImageCallback(APCImageType::Value imageType,
                                      CVideoDeviceModel::STREAM_TYPE streamType,
                                      BYTE *pImageBuffer, int nImageSize,
                                      int nWidth, int nHeight, int nSerialNumber,
                                      void *pUserData)
{
    if (!m_pPreviewDialog[streamType]) {
        QMutexLocker locker(&m_previewDialogMutex);
        OpenPreviewView(streamType);
        QThread::msleep(100);
    }

    if (m_pPreviewDialog[streamType]->GetImageDataModel()){
        m_pPreviewDialog[streamType]->GetImageDataModel()->SetImageInfo(imageType, nWidth, nHeight);
        if (m_pPreviewDialog[streamType]->isVisible()){
            m_pPreviewDialog[streamType]->ResizePreviewDialog();
        }
        m_pPreviewDialog[streamType]->GetImageDataModel()->SetUserData(pUserData);
        m_pPreviewDialog[streamType]->GetImageDataModel()->SetRawData(pImageBuffer, nImageSize, nSerialNumber);
    }

    return APC_OK;
}

int CVideoDeviceDialog::PointCloudCallback(std::vector<float> &cloudPoints, std::vector<BYTE> &colors)
{
    if(!m_pPointCloudViewerDialog){
        RUN_ON_UI_THREAD(
        if(!m_pPointCloudViewerDialog){
            m_pPointCloudViewerDialog = new CPointCloudViewerDialog(m_pVideoDeviceController, this);
            m_pPointCloudViewerDialog->setGeometry({0, 0, 800, 480});
            m_pPointCloudViewerDialog->show();
            RelocateDialogPosition();
        }
        );
    }

    m_pPointCloudViewerDialog->GetPointCloudViewer().updateData(cloudPoints, colors);

    return APC_OK;
}

CImageDataModel *CVideoDeviceDialog::GetPreviewImageData(CVideoDeviceModel::STREAM_TYPE type)
{
    if (!m_pPreviewDialog[type]) return nullptr;

    return m_pPreviewDialog[type]->GetImageDataModel();
}

void CVideoDeviceDialog::UpdateModuleInformation()
{
    std::vector<CVideoDeviceModel::DeviceInfo> deviceInfo = m_pVideoDeviceModel->GetDeviceInformation();
    if (deviceInfo.empty()) return;

    QString sVID = "";
    QString sPID = "";
    QString sSerial = "";
    QString sFWVer = "";

    sVID.sprintf("0x%x", deviceInfo[0].deviceInfomation.wVID);
    sSerial = QString::fromStdString(deviceInfo[0].sSerialNumber);
    for (CVideoDeviceModel::DeviceInfo info : deviceInfo){
        QString sPIDHex;
        sPIDHex.sprintf("0x%x", info.deviceInfomation.wPID);
        sPID += sPIDHex + ", ";
        sFWVer += QString::fromStdString(info.sFWVersion) + "\n";
    }

    sPID[sPID.length() - 2] = '\0';
    sFWVer[sFWVer.length() - 1] = '\0';

    ui->label_content_verder_id->setText(sVID);
    ui->label_content_product_id->setText(sPID);
    ui->label_content_serial_number->setText(sSerial);
    ui->label_content_fw_ver->setText(sFWVer);
}

void CVideoDeviceDialog::UpdateRectifyLog()
{
    ui->checkBox_slave_dev->setEnabled(m_pVideoDeviceModel->HasSlaveRectifyLogData());
}

void CVideoDeviceDialog::UpdateTabView()
{
    ui->tabWidget->clear();
    UpdatePreview();
    UpdateDepthFilter();
    UpdateCameraProperty();
    UpdateDepthAccuracy();
    UpdateIMU();
    UpdateAudio();
    UpdateRegister();
    UpdateThermalUI();
}

void CVideoDeviceDialog::UpdatePreview()
{
    if (m_pPreviewWidget) delete m_pPreviewWidget;
    m_pPreviewWidget = new CVideoDevicePreviewWidget(m_pVideoDeviceController,
                                                   this);
    ui->tabWidget->addTab(m_pPreviewWidget, "Preview");

}

void CVideoDeviceDialog::UpdateRegister()
{
    if (m_pRegisterWidget) delete m_pRegisterWidget;
    m_pRegisterWidget = new CVideoDeviceRegisterWidget(m_pVideoDeviceController->GetRegisterReadWriteController(),
                                                     this);
    ui->tabWidget->addTab(m_pRegisterWidget, "Register");
}

void CVideoDeviceDialog::UpdateCameraProperty()
{
    if (m_pCameraPropertyWidget) delete m_pCameraPropertyWidget;
    m_pCameraPropertyWidget = new CVideoDeviceCameraPropertyWidget(m_pVideoDeviceController->GetCameraPropertyController(),
                                                                 this);
    ui->tabWidget->addTab(m_pCameraPropertyWidget, "Property");
}

void CVideoDeviceDialog::UpdateDepthAccuracy()
{
    if(!m_pVideoDeviceController->GetVideoDeviceModel()->DepthAccuracySupport()) return;

    if (m_pDepthAccuracyWidget) delete m_pDepthAccuracyWidget;
    m_pDepthAccuracyWidget = new CVideoDeviceDepthAccuracyWidget(m_pVideoDeviceController->GetDepthAccuracyController(),
                                                               this);
    ui->tabWidget->addTab(m_pDepthAccuracyWidget, "Accuracy");
}

void CVideoDeviceDialog::UpdateIMU()
{
    if(!m_pVideoDeviceController->GetIMUDataController()) return;

    if (m_pIMUWidget) delete m_pIMUWidget;
    m_pIMUWidget = new CVideoDeviceIMUWidget(m_pVideoDeviceController->GetIMUDataController(),
                                           this);
    ui->tabWidget->addTab(m_pIMUWidget, "IMU");
}

void CVideoDeviceDialog::UpdateAudio()
{
    if(!m_pVideoDeviceController->GetVideoDeviceModel()->AudioSupport()) return;

    if (m_pAudioWidget) delete m_pAudioWidget;
    m_pAudioWidget = new CVideoDeviceAudoWidget(this);
    ui->tabWidget->addTab(m_pAudioWidget, "Audio");
}

void CVideoDeviceDialog::UpdateDepthFilter()
{
    if (m_pDepthFilterWidget) delete m_pDepthFilterWidget;
    m_pDepthFilterWidget = new CVideoDeviceDepthFilterWidget(m_pVideoDeviceController,
                                                             this);
    ui->tabWidget->addTab(m_pDepthFilterWidget, "Depth Filter");
}

void CVideoDeviceDialog::UpdateThermalUI() {
    float f = 0.0f;
    if (APC_NotSupport == m_pVideoDeviceController->GetVideoDeviceModel()->GetCurrentTemperature(f)) {
        ui->groupBox_temperature->hide();
    } else {
        qDebug() << "This module support read thermal sensor";
    }
}

void CVideoDeviceDialog::on_pushButton_temperature_clicked() {
    QString temperatureString;
    float deviceTemperature = 0.0f;

    m_pVideoDeviceController->GetVideoDeviceModel()->GetCurrentTemperature(deviceTemperature);
    temperatureString = QString(std::to_string(deviceTemperature).c_str());
    temperatureString.append("°C");

    ui->label_temperature->setText(temperatureString);
}

void CVideoDeviceDialog::on_pushButton_rectify_read_clicked()
{
    int nIndex = ui->comboBox_log_index->currentText().toInt();
    eSPCtrl_RectLogData rectLogData;
    int ret;
    if (ui->checkBox_slave_dev->isChecked()){
        ret = m_pVideoDeviceController->GetSlaveRectifyLogData(nIndex, &rectLogData);
    }else{
        ret = m_pVideoDeviceController->GetRectifyLogData(nIndex, &rectLogData);
    }

    if (APC_OK != ret) {
        QMessageBox::information(NULL, "Error", "Read Rectify Log Fail !!", QMessageBox::Yes, QMessageBox::Yes);
        return;
    }

    FILE *pFile;
    char buf[256];

    unsigned short nDevType = m_pVideoDeviceModel->GetDeviceInformation()[0].deviceInfomation.nDevType;
    if (PUMA == nDevType)
        sprintf(buf, "../out/RectifyLog/RectfyLog_PUMA_%d.bin", nIndex);
    else
        sprintf(buf, "../out/RectifyLog/RectfyLog_AXES1_%d.bin", nIndex);

    pFile = fopen(buf, "wb");
    if (nullptr != pFile) {
        fwrite(&rectLogData, 1 , 1024, pFile);  // union asssigned 1024 bytes.
        fclose(pFile);
    }

    if (PUMA == nDevType)
        sprintf(buf, "../out/RectifyLog/RectfyLog_PUMA_%d.txt", nIndex);
    else
        sprintf(buf, "../out/RectifyLog/RectfyLog_AXES1_%d.txt", nIndex);

    pFile = fopen(buf, "wt");
    if (nullptr != pFile) {
        int i;
        //
        fprintf(pFile, "InImgWidth = %d\n",        rectLogData.InImgWidth);
        fprintf(pFile, "InImgHeight = %d\n",       rectLogData.InImgHeight);
        fprintf(pFile, "OutImgWidth = %d\n",       rectLogData.OutImgWidth);
        fprintf(pFile, "OutImgHeight = %d\n",      rectLogData.OutImgHeight);
        //
        fprintf(pFile, "RECT_ScaleWidth = %d\n",   rectLogData.RECT_ScaleWidth);
        fprintf(pFile, "RECT_ScaleHeight = %d\n",  rectLogData.RECT_ScaleHeight);
        //
        fprintf(pFile, "CamMat1 = ");
        for (i=0; i<9; i++) {
            fprintf(pFile, "%.8f, ",  rectLogData.CamMat1[i]);
        }
        fprintf(pFile, "\n");
        //
        fprintf(pFile, "CamDist1 = ");
        for (i=0; i<8; i++) {
            fprintf(pFile, "%.8f, ",  rectLogData.CamDist1[i]);
        }
        fprintf(pFile, "\n");
        //
        fprintf(pFile, "CamMat2 = ");
        for (i=0; i<9; i++) {
            fprintf(pFile, "%.8f, ",  rectLogData.CamMat2[i]);
        }
        fprintf(pFile, "\n");
        //
        fprintf(pFile, "CamDist2 = ");
        for (i=0; i<8; i++) {
            fprintf(pFile, "%.8f, ",  rectLogData.CamDist2[i]);
        }
        fprintf(pFile, "\n");
        //
        fprintf(pFile, "RotaMat = ");
        for (i=0; i<9; i++) {
            fprintf(pFile, "%.8f, ",  rectLogData.RotaMat[i]);
        }
        fprintf(pFile, "\n");
        //
        fprintf(pFile, "TranMat = ");
        for (i=0; i<3; i++) {
            fprintf(pFile, "%.8f, ",  rectLogData.TranMat[i]);
        }
        fprintf(pFile, "\n");
        //
        fprintf(pFile, "LRotaMat = ");
        for (i=0; i<9; i++) {
            fprintf(pFile, "%.8f, ",  rectLogData.LRotaMat[i]);
        }
        fprintf(pFile, "\n");
        //
        fprintf(pFile, "RRotaMat = ");
        for (i=0; i<9; i++) {
            fprintf(pFile, "%.8f, ",  rectLogData.RRotaMat[i]);
        }
        fprintf(pFile, "\n");
        //
        fprintf(pFile, "NewCamMat1 = ");
        for (i=0; i<12; i++) {
            fprintf(pFile, "%.8f, ",  rectLogData.NewCamMat1[i]);
        }
        fprintf(pFile, "\n");
        //
        fprintf(pFile, "NewCamMat2 = ");
        for (i=0; i<12; i++) {
            fprintf(pFile, "%.8f, ",  rectLogData.NewCamMat2[i]);
        }
        fprintf(pFile, "\n");
        //
        fprintf(pFile, "RECT_Crop_Row_BG = %d\n",   rectLogData.RECT_Crop_Row_BG);
        fprintf(pFile, "RECT_Crop_Row_ED = %d\n",   rectLogData.RECT_Crop_Row_ED);
        fprintf(pFile, "RECT_Crop_Col_BG_L = %d\n", rectLogData.RECT_Crop_Col_BG_L);
        fprintf(pFile, "RECT_Crop_Col_ED_L = %d\n", rectLogData.RECT_Crop_Col_ED_L);
        fprintf(pFile, "RECT_Scale_Col_M = %d\n",   rectLogData.RECT_Scale_Col_M);
        fprintf(pFile, "RECT_Scale_Col_N = %d\n",   rectLogData.RECT_Scale_Col_N);
        fprintf(pFile, "RECT_Scale_Row_M = %d\n",   rectLogData.RECT_Scale_Row_M);
        fprintf(pFile, "RECT_Scale_Row_N = %d\n",   rectLogData.RECT_Scale_Row_N);
        //
        fprintf(pFile, "RECT_AvgErr = %.8f\n", rectLogData.RECT_AvgErr);
        //
        fprintf(pFile, "nLineBuffers = %d\n",  rectLogData.nLineBuffers);
        //
        if (PUMA == nDevType) {
            fprintf(pFile, "ReProjectMat = ");
            for (i = 0; i < 16; i++) {
                fprintf(pFile, "%.8f, ", rectLogData.ReProjectMat[i]);
            }
            fprintf(pFile, "\n");
        }
        fclose(pFile);
    }

    if (PUMA == nDevType){
        QString sRectifyLogInfo;
        sRectifyLogInfo.sprintf("index:%d\n", nIndex);
        sRectifyLogInfo += "ReProjectMat=\n";
        for (int i = 0 ; i < 16 ; ++i){
            if (rectLogData.ReProjectMat[i] > 0) sRectifyLogInfo += " ";
            sRectifyLogInfo += QString::number(rectLogData.ReProjectMat[i], 'f', 6);
            if (i % 4 == 3) sRectifyLogInfo += "\n";
            else           sRectifyLogInfo += ",";
        }
        QMessageBox::information(NULL, "Rectify Log Info", sRectifyLogInfo, QMessageBox::Yes, QMessageBox::Yes);
    }else{
        QMessageBox::information(NULL, "Rectify Log Info", "Read Rectify Log Complete !!", QMessageBox::Yes, QMessageBox::Yes);
    }
}
