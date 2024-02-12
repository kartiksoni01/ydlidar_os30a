#include "CVideoDeviceDepthAccuracyWidget.h"
#include "ui_CVideoDeviceDepthAccuracyWidget.h"
#include "CDepthAccuracyController.h"

CVideoDeviceDepthAccuracyWidget::CVideoDeviceDepthAccuracyWidget(CDepthAccuracyController *pDepthAccuracyController,
                                                                 QWidget *parent):
QWidget(parent),
ui(new Ui::CVideoDeviceDepthAccuracyWidget),
m_pDepthAccuracyController(pDepthAccuracyController)
{
    ui->setupUi(this);

    AddUpdateTimer(10);
}

CVideoDeviceDepthAccuracyWidget::~CVideoDeviceDepthAccuracyWidget()
{
    delete ui;
}

void CVideoDeviceDepthAccuracyWidget::paintEvent(QPaintEvent *event)
{
    if (!m_pDepthAccuracyController->IsValid()) return;

    UpdateValue();
}

void CVideoDeviceDepthAccuracyWidget::showEvent(QShowEvent *event)
{
    m_pDepthAccuracyController->EnableDepthAccuracy(isActiveWindow());
    UpdateUI();
}

void CVideoDeviceDepthAccuracyWidget::hideEvent(QHideEvent *event)
{
    m_pDepthAccuracyController->EnableDepthAccuracy(false);
}

void CVideoDeviceDepthAccuracyWidget::UpdateSelf()
{
    bool bEnable = m_pDepthAccuracyController->IsValid();

    ui->groupBox_region_accuracy->setEnabled(bEnable);

    if (!bEnable) return;

    UpdateAccuracyList();
    UpdateAccuracyRegion();
    UpdateGroundTruth();
    UpdateValue();
}

void CVideoDeviceDepthAccuracyWidget::UpdateAccuracyList()
{
    std::vector<CVideoDeviceModel::STREAM_TYPE> typeList = m_pDepthAccuracyController->GetDepthAccuracyList();
    ui->widget_depth_accuracy_stream->setVisible(typeList.size() > 1);

    if (!ui->widget_depth_accuracy_stream->isVisible()) {
        m_pDepthAccuracyController->SelectDepthAccuracyStreamType(CVideoDeviceModel::STREAM_DEPTH);
        return;
    }

    if (0 == ui->comboBox_depth_accuracy_stream->count()){
        ui->comboBox_depth_accuracy_stream->blockSignals(true);
        ui->comboBox_depth_accuracy_stream->clear();
        for (CVideoDeviceModel::STREAM_TYPE streamType : typeList){
            QString sStreamName = "";
            switch(streamType){
                case CVideoDeviceModel::STREAM_DEPTH: sStreamName = "Depth"; break;
                case CVideoDeviceModel::STREAM_DEPTH_30mm: sStreamName = "Depth_30mm"; break;
                case CVideoDeviceModel::STREAM_DEPTH_60mm: sStreamName = "Depth_60mm"; break;
                case CVideoDeviceModel::STREAM_DEPTH_150mm: sStreamName = "Depth_150mm"; break;
                case CVideoDeviceModel::STREAM_DEPTH_FUSION: sStreamName = "Depth_Fusion"; break;
                default: break;
            }
            ui->comboBox_depth_accuracy_stream->addItem(sStreamName);
        }
        ui->comboBox_depth_accuracy_stream->blockSignals(false);
    }

    ui->comboBox_depth_accuracy_stream->currentTextChanged(ui->comboBox_depth_accuracy_stream->currentText());
}

void CVideoDeviceDepthAccuracyWidget::UpdateAccuracyRegion()
{
    ui->comboBox_depth_accuracy_roi->currentTextChanged(ui->comboBox_depth_accuracy_roi->currentText());
}

void CVideoDeviceDepthAccuracyWidget::UpdateGroundTruth()
{
    bool bGroundTruthEnable = ui->checkBox_depth_accuracy_ground_truth->isChecked();
    ui->doubleSpinBox_depth_accuracy_ground_truth_vaule->setEnabled(bGroundTruthEnable);
    if (!bGroundTruthEnable){
        m_pDepthAccuracyController->SetGroundTruthDistanceMM(0.0f);
        ui->doubleSpinBox_depth_accuracy_ground_truth_vaule->setValue(0.0f);
    }

}

void CVideoDeviceDepthAccuracyWidget::UpdateValue()
{
    CDepthAccuracyController::DepthAccuracyInfo accuracyInfo = m_pDepthAccuracyController->GetDepthAccuracyInfo();
    QString sDistance;
    sDistance.sprintf("%.2f mm", accuracyInfo.fDistance);
    ui->lineEdit_depth_accuracy_distance_value->setText(sDistance);

    QString sFillRate;
    sFillRate.sprintf("%.2f %%", accuracyInfo.fFillRate * 100.0);
    ui->lineEdit_depth_accuracy_fill_rate_value->setText(sFillRate);

    QString sZAccuracy;
    sZAccuracy.sprintf("%.2f %%", accuracyInfo.fZAccuracy * 100.0);
    ui->lineEdit_depth_accuracy_z_acuracy_value->setText(sZAccuracy);

    QString sTemporaNoise;
    sTemporaNoise.sprintf("%.2f %%", accuracyInfo.fTemporalNoise * 100.0f);
    ui->lineEdit_depth_tempora_noise->setText(sTemporaNoise);

    QString sSpatialNoise;
    sSpatialNoise.sprintf("%.2f %%", accuracyInfo.fSpatialNoise * 100.0f);
    ui->lineEdit_depth_spatial_noise->setText(sSpatialNoise);

    QString sAngle;
    sAngle.sprintf("%.2f deg", accuracyInfo.fAngle);
    ui->lineEdit_depth_angle->setText(sAngle);

    QString sAngleX;
    sAngleX.sprintf("%.2f deg", accuracyInfo.fAngleX);
    ui->lineEdit_depth_angle_x->setText(sAngleX);

    QString sAngleY;
    sAngleY.sprintf("%.2f deg", accuracyInfo.fAngleY);
    ui->lineEdit_depth_angle_y->setText(sAngleY);
}

void CVideoDeviceDepthAccuracyWidget::on_comboBox_depth_accuracy_stream_currentTextChanged(const QString &text)
{
    if (!text.compare("Depth")){
        m_pDepthAccuracyController->SelectDepthAccuracyStreamType(CVideoDeviceModel::STREAM_DEPTH);
    }else if (!text.compare("Depth_30mm")){
        m_pDepthAccuracyController->SelectDepthAccuracyStreamType(CVideoDeviceModel::STREAM_DEPTH_30mm);
    }else if (!text.compare("Depth_60mm")){
        m_pDepthAccuracyController->SelectDepthAccuracyStreamType(CVideoDeviceModel::STREAM_DEPTH_60mm);
    }else if (!text.compare("Depth_150mm")){
        m_pDepthAccuracyController->SelectDepthAccuracyStreamType(CVideoDeviceModel::STREAM_DEPTH_150mm);
    }else if (!text.compare("Depth_Fusion")){
        m_pDepthAccuracyController->SelectDepthAccuracyStreamType(CVideoDeviceModel::STREAM_DEPTH_FUSION);
    }
}

void CVideoDeviceDepthAccuracyWidget::on_comboBox_depth_accuracy_roi_currentTextChanged(const QString &text)
{
    QStringRef sNumber(&text, 0, text.length() - 1);
    int ratio = sNumber.toInt();
    m_pDepthAccuracyController->SetAccuracyRegionRatio(ratio / 100.0f);
}

void CVideoDeviceDepthAccuracyWidget::on_checkBox_depth_accuracy_ground_truth_stateChanged(int state)
{
    UpdateGroundTruth();
}

void CVideoDeviceDepthAccuracyWidget::on_doubleSpinBox_depth_accuracy_ground_truth_vaule_valueChanged(double dblValue)
{
    m_pDepthAccuracyController->SetGroundTruthDistanceMM(dblValue);
}
