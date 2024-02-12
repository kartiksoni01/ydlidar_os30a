#include "CVideoDevicePreviewWidget.h"
#include "ui_CVideoDevicePreviewWidget.h"
#include "CVideoDeviceModel.h"
#include "CVideoDeviceController.h"

CVideoDevicePreviewWidget::CVideoDevicePreviewWidget(CVideoDeviceController *pVideoDeviceController,
                                                     QWidget *parent) :
QWidget(parent),
ui(new Ui::CVideoDevicePreviewWidget),
m_pVideoDeviceController(pVideoDeviceController)
{
    setAttribute(Qt::WA_DeleteOnClose);

    ui->setupUi(this);
    UpdateUI();
}

CVideoDevicePreviewWidget::~CVideoDevicePreviewWidget()
{
    delete ui;
}

void CVideoDevicePreviewWidget::showEvent(QShowEvent *event)
{
    UpdateUI();
}

void CVideoDevicePreviewWidget::UpdateSelf()
{    
    setUpdatesEnabled(false);
    UpdateUSBPort();
    UpdateStreamimgComponetState(CVideoDeviceModel::STREAMING ==
                                 m_pVideoDeviceController->GetVideoDeviceModel()->GetState());
    UpdateModeConfig();
    UpdateStream();
    UpdateDepthOutput();
    UpdateDepthmapBits();
    UpdateZValue();
    UpdateIRLevel();
    UpdateDepthROI();
    UpdateRectify();
    UpdateHWPP();
    UpdateModuleSync();
    UpdateInterleaveMode();
    UpdatePlyFilter();
    UpdatePointCloudViewer();
    setUpdatesEnabled(true);
}

void CVideoDevicePreviewWidget::Preview()
{
    ui->pushButton_preview->click();
}

void CVideoDevicePreviewWidget::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);
}

void CVideoDevicePreviewWidget::UpdateUSBPort()
{
    switch(m_pVideoDeviceController->GetVideoDeviceModel()->GetUsbType()){
        case USB_PORT_TYPE_2_0:
            ui->label_usb_type->setText("USB 2.0");
            break;
        case USB_PORT_TYPE_3_0:
            ui->label_usb_type->setText("USB 3.0 Gen 1");
            break;
        default:
            ui->label_usb_type->setText("unknown USB port");
            break;
    }
}

void CVideoDevicePreviewWidget::UpdateStreamimgComponetState(bool bIsStreaming)
{
    ui->groupBox_z_value->setEnabled(bIsStreaming);
    ui->widget_snapshot->setEnabled(bIsStreaming);

    ui->checkBox_point_cloud_viewer->setEnabled(!bIsStreaming);
    ui->widget_stream_info->setEnabled(!bIsStreaming);
    ui->checkBox_rectify_data->setEnabled(!bIsStreaming);
    ui->pushButton_preview->setEnabled(!bIsStreaming);
    ui->widget_mode_config_option->setEnabled(!bIsStreaming);
    ui->checkBox_hw_pp->setEnabled(bIsStreaming);
    ui->checkBox_master->setEnabled(bIsStreaming);
}

void CVideoDevicePreviewWidget::UpdateStream()
{
    for(int i = 0 ; i < CVideoDeviceModel::STREAM_TYPE_COUNT ; ++i){
        UpdateStream((CVideoDeviceModel::STREAM_TYPE)i);
    }
    UpdateColorWithDepthStream();
}

void CVideoDevicePreviewWidget::UpdateColorWithDepthStream()
{
    bool bIsModeConfig = m_pVideoDeviceController->GetPreviewOptions()->IsModeConfig();
    bool bColorWithDepthDevice = m_pVideoDeviceController->GetVideoDeviceModel()->IsColorWithDepthDevice();
    ui->widget_color_and_depth_stream->setVisible(bColorWithDepthDevice && !bIsModeConfig);
    if(!ui->widget_color_and_depth_stream->isVisible()) return;

    bool bEnable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_COLOR) ||
                   m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_DEPTH);
    ui->checkBox_color_and_depth_stream->blockSignals(true);
    ui->checkBox_color_and_depth_stream->setChecked(bEnable);
    ui->checkBox_color_and_depth_stream->blockSignals(false);

    ui->comboBox_color_and_depth_stream->setEnabled(ui->checkBox_color_and_depth_stream->isChecked());
    ui->spinBox_color_and_depth_fps->setEnabled(ui->checkBox_color_and_depth_stream->isChecked());

    ui->comboBox_color_and_depth_stream->blockSignals(true);
    ui->comboBox_color_and_depth_stream->clear();
    std::vector<APC_STREAM_INFO> infoList = m_pVideoDeviceController->GetVideoDeviceModel()->GetStreamInfoList(CVideoDeviceModel::STREAM_COLOR);
    for (APC_STREAM_INFO info : infoList){
        QString resolution;
        resolution.sprintf("[%d x %d] %s", info.nWidth, info.nHeight, info.bFormatMJPG ? "MJPG" : "YUV");
        ui->comboBox_color_and_depth_stream->addItem(resolution);
    }

    ui->comboBox_color_and_depth_stream->setCurrentIndex(m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(CVideoDeviceModel::STREAM_COLOR));
    if(ui->checkBox_color_and_depth_stream->isChecked()){
        m_pVideoDeviceController->UpdateStreamOptionForCombineMode(ui->comboBox_color_and_depth_stream->currentIndex());
    }
    ui->comboBox_color_and_depth_stream->blockSignals(false);

    int nFPS = 0;
    if (m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_COLOR)){
        nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(CVideoDeviceModel::STREAM_COLOR);
    }else{
        nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(CVideoDeviceModel::STREAM_DEPTH);
    }

    ui->spinBox_color_and_depth_fps->setValue(nFPS);

}

void CVideoDevicePreviewWidget::UpdateStream(CVideoDeviceModel::STREAM_TYPE type)
{
    bool bIsSupprot = m_pVideoDeviceController->GetVideoDeviceModel()->IsStreamSupport(type);
    QCheckBox *pEnableStream = nullptr;
    QWidget *pWidget = nullptr;
    QComboBox *pStreamInfo = nullptr;

    switch (type){
        case CVideoDeviceModel::STREAM_COLOR:            
            pWidget = ui->widget_color_stream;
            pEnableStream = ui->checkBox_color_stream;
            pStreamInfo = ui->comboBox_color_stream;
            break;
        case CVideoDeviceModel::STREAM_DEPTH:            
            pWidget = ui->widget_depth_stream;
            pEnableStream = ui->checkBox_depth_stream;
            pStreamInfo = ui->comboBox_depth_stream;
            break;
        case CVideoDeviceModel::STREAM_KOLOR:            
            pWidget = ui->widget_kolor_stream;
            pEnableStream = ui->checkBox_kolor_stream;
            pStreamInfo = ui->comboBox_kolor_stream;
            break;
        case CVideoDeviceModel::STREAM_TRACK:
            pWidget = ui->widget_track_stream;
            pEnableStream = ui->checkBox_track_stream;
            pStreamInfo = ui->comboBox_track_stream;
            break;
        default:
            return;
    }

    pWidget->setVisible(bIsSupprot);
    if (!bIsSupprot) return;

    if (CVideoDeviceModel::STREAM_COLOR == type ||
        CVideoDeviceModel::STREAM_DEPTH == type  ){
        pWidget->setEnabled(!ui->widget_color_and_depth_stream->isVisible());
    }

    bool bIsModeConfig = m_pVideoDeviceController->GetPreviewOptions()->IsModeConfig();
    bool bEnable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(type);
    pEnableStream->setChecked(bEnable);
    pEnableStream->setEnabled(!bIsModeConfig);
    pStreamInfo->setEnabled(bEnable && !bIsModeConfig);

    pStreamInfo->blockSignals(true);
    pStreamInfo->clear();
    std::vector<APC_STREAM_INFO> infoList = m_pVideoDeviceController->GetVideoDeviceModel()->GetStreamInfoList(type);
    for (APC_STREAM_INFO info : infoList){
        QString resolution;
        resolution.sprintf("[%d x %d] %s", info.nWidth, info.nHeight, info.bFormatMJPG ? "MJPG" : "YUV");
        pStreamInfo->addItem(resolution);
    }

    pStreamInfo->setCurrentIndex(m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(type));
    pStreamInfo->blockSignals(false);

    UpdateFPS(type);
}

void CVideoDevicePreviewWidget::UpdateFPS(CVideoDeviceModel::STREAM_TYPE type)
{
    bool bIsSupprot = m_pVideoDeviceController->GetVideoDeviceModel()->IsStreamSupport(type);
    if (!bIsSupprot) return;

    int nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(type);

    if (CVideoDeviceModel::STREAM_DEPTH == type){
        ui->spinBox_depth_fps->setEnabled(!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_COLOR));
        ui->comboBox_depth_fps->setEnabled(!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_COLOR));
        if (m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_COLOR)){
            nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(CVideoDeviceModel::STREAM_COLOR);
        }
    }

    QLabel *pFpsLabel = nullptr;
    QSpinBox *pFpsSpinBox = nullptr;
    QComboBox *pFpsComboBox = nullptr;
    switch (type){
        case CVideoDeviceModel::STREAM_COLOR:
            pFpsLabel = ui->label_title_color_fps;
            pFpsSpinBox = ui->spinBox_color_fps;
            pFpsComboBox = ui->comboBox_color_fps;
            break;
        case CVideoDeviceModel::STREAM_DEPTH:
            pFpsLabel = ui->label_title_depth_fps;
            pFpsSpinBox = ui->spinBox_depth_fps;
            pFpsComboBox = ui->comboBox_depth_fps;
            break;
        case CVideoDeviceModel::STREAM_KOLOR:
            pFpsLabel = ui->label_title_kolor_fps;
            pFpsSpinBox = ui->spinBox_kolor_fps;
            pFpsComboBox = ui->comboBox_kolor_fps;
            break;
        case CVideoDeviceModel::STREAM_TRACK:
            pFpsLabel = ui->label_title_track_fps;
            pFpsSpinBox = ui->spinBox_track_fps;
            pFpsComboBox = ui->comboBox_track_fps;
            break;
        default:
            return;
    }
    bool bEnable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(type);
    pFpsLabel->setEnabled(bEnable);
    pFpsSpinBox->setEnabled(bEnable);
    pFpsComboBox->setEnabled(bEnable);

    bool bIsModeConfig = m_pVideoDeviceController->GetPreviewOptions()->IsModeConfig();
    pFpsSpinBox->setVisible(!bIsModeConfig);
    pFpsComboBox->setVisible(bIsModeConfig);

    if(bIsModeConfig){
        pFpsComboBox->blockSignals(true);
        pFpsComboBox->clear();
        std::vector< int > fpsList;
        if(CVideoDeviceModel::STREAM_DEPTH == type){
            fpsList = m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentModeInfo().vecDepthFps;
            if (0 == fpsList.size()){
                fpsList = m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentModeInfo().vecColorFps;
                pFpsComboBox->setEnabled(false);
            }
        }else{
            fpsList = m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentModeInfo().vecColorFps;
        }
        for(int fps : fpsList){
            pFpsComboBox->addItem(QString::number(fps));
        }
        pFpsComboBox->setCurrentText(QString::number(nFPS));
        pFpsComboBox->blockSignals(false);

        pFpsComboBox->setEnabled(pFpsComboBox->isEnabled() && pFpsComboBox->count() > 1);
        pFpsLabel->setEnabled(pFpsComboBox->isEnabled());


    }else{
        pFpsSpinBox->setValue(nFPS);
    }
}

void CVideoDevicePreviewWidget::UpdateModeConfig()
{
    bool bIsModeConfig = m_pVideoDeviceController->GetPreviewOptions()->IsModeConfig();
    std::vector<ModeConfig::MODE_CONFIG> modeConfigs = m_pVideoDeviceController->GetModeConfigOptions()->GetModes();
    ui->checkBox_mode_config->setVisible(!modeConfigs.empty());
    ui->widget_mode_config_option->setVisible(!modeConfigs.empty());
    if(modeConfigs.empty()) return;

    ui->checkBox_mode_config->blockSignals(true);
    ui->checkBox_mode_config->setChecked(bIsModeConfig);
    ui->checkBox_mode_config->blockSignals(false);
    ui->widget_mode_config_option->setVisible(bIsModeConfig);

    if(bIsModeConfig){
        ui->widget_mode_config_option->setEnabled(!modeConfigs.empty() && ui->widget_mode_config_option->isEnabled());
        ui->comboBox_mode_index->blockSignals(true);
        ui->comboBox_mode_index->clear();
        for(ModeConfig::MODE_CONFIG config : modeConfigs){
            QString sMode;
            sMode.sprintf("Mode %d", config.iMode);
            ui->comboBox_mode_index->addItem(sMode);
        }
        ui->comboBox_mode_index->setCurrentIndex(m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentIndex());
        ui->comboBox_mode_index->blockSignals(false);
        ui->label_cotent_mode_description->setText(m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentModeInfo().csModeDesc);
    }

}

void CVideoDevicePreviewWidget::UpdateRectify()
{
    bool bIsRectify = m_pVideoDeviceController->GetVideoDeviceModel()->IsRectifyData();
    ui->checkBox_rectify_data->setChecked(bIsRectify);
    ui->checkBox_rectify_data->setEnabled(!m_pVideoDeviceController->GetPreviewOptions()->IsModeConfig());
}

void CVideoDevicePreviewWidget::UpdateHWPP()
{
    ui->checkBox_hw_pp->setVisible(m_pVideoDeviceController->GetVideoDeviceModel()->HWPPSupprot());
    ui->checkBox_hw_pp->blockSignals(true);
    ui->checkBox_hw_pp->setChecked(m_pVideoDeviceController->GetVideoDeviceModel()->IsHWPP());
    ui->checkBox_hw_pp->blockSignals(false);
}

void CVideoDevicePreviewWidget::UpdateModuleSync()
{
    bool bIsModuleSyncSupport = m_pVideoDeviceController->GetVideoDeviceModel()->ModuleSyncSupport();
    ui->checkBox_master->setVisible(bIsModuleSyncSupport);
    ui->checkBox_module_sync->setVisible(bIsModuleSyncSupport);

    ui->checkBox_module_sync->blockSignals(true);
    ui->checkBox_module_sync->setChecked(m_pVideoDeviceController->GetPreviewOptions()->IsModuleSync());
    ui->checkBox_module_sync->blockSignals(false);

    ui->checkBox_master->blockSignals(true);
    ui->checkBox_master->setChecked(m_pVideoDeviceController->GetPreviewOptions()->IsModuleSyncMaster());
    ui->checkBox_master->blockSignals(false);

}

void CVideoDevicePreviewWidget::UpdateInterleaveMode()
{
    bool bIsSupport = m_pVideoDeviceController->GetVideoDeviceModel()->InterleaveModeSupport();
    ui->checkBox_interleave_mode->setVisible(bIsSupport);
    ui->checkBox_interleave_mode->setChecked(m_pVideoDeviceController->GetVideoDeviceModel()->IsInterleaveMode());
}

void CVideoDevicePreviewWidget::UpdatePlyFilter()
{
    bool bIsSupport = m_pVideoDeviceController->GetVideoDeviceModel()->PlyFilterSupprot();
    ui->checkBox_ply_filter->setVisible(bIsSupport);

    bool bEnable = m_pVideoDeviceController->GetPreviewOptions()->IsPlyFilter();
    ui->checkBox_ply_filter->setChecked(bEnable);
}

void CVideoDevicePreviewWidget::UpdatePointCloudViewer()
{
    bool bPointColudAvaliable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_DEPTH);
    ui->checkBox_point_cloud_viewer->setEnabled(bPointColudAvaliable && ui->checkBox_point_cloud_viewer->isEnabled());
    ui->checkBox_point_cloud_viewer->setChecked(bPointColudAvaliable && m_pVideoDeviceController->GetPreviewOptions()->IsPointCloudViewer());
    ui->comboBox_point_cloud_viewer_format->blockSignals(true);
    ui->comboBox_point_cloud_viewer_format->clear();

    bool bColorStreamAvaliable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_COLOR) ||
                                 m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_KOLOR) ;
    if (bColorStreamAvaliable)
    {
        ui->comboBox_point_cloud_viewer_format->addItem("Color");
    }

    ui->comboBox_point_cloud_viewer_format->addItem("Depth Output");
    ui->comboBox_point_cloud_viewer_format->addItem("Single Color");

    PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_FORMAT format = m_pVideoDeviceController->GetPreviewOptions()->GetPointCloudViewOutputFormat();
    switch (format){
        case PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_COLOR:

            if (!bColorStreamAvaliable)
            {
                m_pVideoDeviceController->GetPreviewOptions()->SetPointCloudViewOutputFormat(PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_DEPTH);
                ui->comboBox_point_cloud_viewer_format->setCurrentText("Depth Output");
                break;
            }

            ui->comboBox_point_cloud_viewer_format->setCurrentText("Color");
            break;
        case PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_DEPTH:
            ui->comboBox_point_cloud_viewer_format->setCurrentText("Depth Output"); break;
        case PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_SINGLE_COLOR:
            ui->comboBox_point_cloud_viewer_format->setCurrentText("Single Color"); break;
        default: break;
    }
    ui->comboBox_point_cloud_viewer_format->blockSignals(false);

    ui->comboBox_point_cloud_viewer_format->setEnabled(ui->checkBox_point_cloud_viewer->isChecked());
    ui->horizontalSlider_point_size->setEnabled(ui->checkBox_point_cloud_viewer->isChecked());
    ui->horizontalSlider_point_size->blockSignals(true);
    ui->horizontalSlider_point_size->setValue(m_pVideoDeviceController->GetPreviewOptions()->GetPointCloudSize());
    ui->horizontalSlider_point_size->blockSignals(false);

    QString sPointSize = QString::number(ui->horizontalSlider_point_size->value()) + " / " +
                         QString::number(ui->horizontalSlider_point_size->maximum());
    ui->label_content_point_size->setText(sPointSize);
}

void CVideoDevicePreviewWidget::UpdateDepthOutput()
{    
    bool bDepthEnable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_DEPTH);
    ui->comboBox_depth_output->setEnabled(bDepthEnable);

    DEPTH_TRANSFER_CTRL depthTransferControl = m_pVideoDeviceController->GetPreviewOptions()->GetDepthDataTransferControl();
    switch (depthTransferControl){
        case DEPTH_IMG_COLORFUL_TRANSFER:
            ui->comboBox_depth_output->setCurrentText("Color");
            break;
        case DEPTH_IMG_GRAY_TRANSFER:
            ui->comboBox_depth_output->setCurrentText("Gray");
            break;
        default:
            return;
    }
}

void CVideoDevicePreviewWidget::UpdateDepthmapBits()
{    
    bool bDepthEnable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_DEPTH);
    ui->comboBox_depthmap_bits->setEnabled(bDepthEnable);

    ui->comboBox_depthmap_bits->blockSignals(true);
    ui->comboBox_depthmap_bits->clear();
    if (m_pVideoDeviceController->GetPreviewOptions()->IsModeConfig()){
        for (int nBits : m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentModeInfo().vecDepthType){
            QString sBits;
            switch (nBits){
                case 8: sBits = "8 Bits"; break;
                case 11: sBits = "11 Bits"; break;
                case 14: sBits = "14 Bits"; break;
                default: continue;
            }
            ui->comboBox_depthmap_bits->addItem(sBits);
        }
    }else{
        if (m_pVideoDeviceController->GetVideoDeviceModel()->IsDepthDataTypeSupport(CVideoDeviceModel::DEPTH_DATA_8BIT)){
            ui->comboBox_depthmap_bits->addItem("8 Bits");
        }

        if (m_pVideoDeviceController->GetVideoDeviceModel()->IsDepthDataTypeSupport(CVideoDeviceModel::DEPTH_DATA_11BIT)){
            ui->comboBox_depthmap_bits->addItem("11 Bits");
        }

        if (m_pVideoDeviceController->GetVideoDeviceModel()->IsDepthDataTypeSupport(CVideoDeviceModel::DEPTH_DATA_14BIT)){
            ui->comboBox_depthmap_bits->addItem("14 Bits");
        }
    }
    ui->comboBox_depthmap_bits->blockSignals(false);

    QString currentDepthDataType;
    switch (m_pVideoDeviceController->GetVideoDeviceModel()->GetDepthImageType()){
        case APCImageType::DEPTH_8BITS:
            currentDepthDataType = "8 Bits";
            break;
        case APCImageType::DEPTH_11BITS:
            currentDepthDataType = "11 Bits";
            break;
        case APCImageType::DEPTH_14BITS:
            currentDepthDataType = "14 Bits";
            break;
        default:
            return;
    }

    int index = ui->comboBox_depthmap_bits->findText(currentDepthDataType);
    if (-1 == index) return;

    ui->comboBox_depthmap_bits->setCurrentIndex(index);
}

void CVideoDevicePreviewWidget::UpdateZValue()
{
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_DEPTH);
    ui->groupBox_z_value->setEnabled(ui->groupBox_z_value->isEnabled() && bDepthStream);

    int nZNear, nZFar;
    m_pVideoDeviceController->GetPreviewOptions()->GetZRange(nZNear, nZFar);

    ui->spinBox_z_value_near->setValue(nZNear);
    ui->spinBox_z_value_far->setValue(nZFar);
}

void CVideoDevicePreviewWidget::UpdateIRLevel()
{
    unsigned short nMin, nMax;

    int ret;
    ret = m_pVideoDeviceController->GetVideoDeviceModel()->GetIRRange(nMin, nMax);
    if (APC_OK != ret)   return;

    ui->horizontalSlider_ir_level_control->setMinimum(nMin);
    ui->horizontalSlider_ir_level_control->setMaximum(nMax);

    int nValue = m_pVideoDeviceController->GetVideoDeviceModel()->GetIRValue();
    if (EOF == nValue) return;

    QString sValue;
    sValue.sprintf("%d / %d", nValue, nMax);
    ui->label_ir_level_value->setText(sValue);
    ui->horizontalSlider_ir_level_control->blockSignals(true);
    ui->horizontalSlider_ir_level_control->setValue(nValue);
    ui->horizontalSlider_ir_level_control->blockSignals(false);

    ui->checkBox_extend_maximum_ir->setVisible(m_pVideoDeviceController->GetVideoDeviceModel()->IRExtendSupport());
    if(ui->checkBox_extend_maximum_ir->isVisible()){
        ui->checkBox_extend_maximum_ir->blockSignals(true);
        ui->checkBox_extend_maximum_ir->setChecked(m_pVideoDeviceController->IsIRExtend());
        ui->checkBox_extend_maximum_ir->blockSignals(false);
    }
}

void CVideoDevicePreviewWidget::UpdateDepthROI()
{
    bool bDepthEnable = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_DEPTH);
    ui->horizontalSlider_depth_roi->setEnabled(bDepthEnable);

    int nDepthROIValue = m_pVideoDeviceController->GetPreviewOptions()->GetDepthROI();
    ui->horizontalSlider_depth_roi->setValue(nDepthROIValue / 10);
    QString depthROIText;
    depthROIText.sprintf("%d / %d", nDepthROIValue, ui->horizontalSlider_depth_roi->maximum() * 10);
    ui->label_depth_roi_value->setText(depthROIText);
}

void CVideoDevicePreviewWidget::on_checkBox_mode_config_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetPreviewOptions()->EnableModeConfig(bIsChecked);
    if(m_pVideoDeviceController->GetPreviewOptions()->IsModeConfig()){
        m_pVideoDeviceController->SelectModeConfigIndex(m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentIndex());
    }
    UpdateUI();
}

void CVideoDevicePreviewWidget::on_checkBox_rectify_data_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->EnableRectifyData(bIsChecked);
}

void CVideoDevicePreviewWidget::on_comboBox_depthmap_bits_currentTextChanged(const QString &text)
{
    int nDepthBits = 0;
    if (text.contains("8 Bits")){
        nDepthBits = 8;
    }else if(text.contains("11 Bits")){
        nDepthBits = 11;
    }else if(text.contains("14 Bits")){
        nDepthBits = 14;
    }
    m_pVideoDeviceController->SetDepthDataBits(nDepthBits);
}

void CVideoDevicePreviewWidget::on_checkBox_color_stream_stateChanged(int state)
{
     bool bIsChecked = Qt::Checked == state ? true : false;
     m_pVideoDeviceController->GetPreviewOptions()->EnableStream(CVideoDeviceModel::STREAM_COLOR, bIsChecked);
     UpdateStream();
     UpdatePointCloudViewer();
}

void CVideoDevicePreviewWidget::on_checkBox_kolor_stream_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetPreviewOptions()->EnableStream(CVideoDeviceModel::STREAM_KOLOR, bIsChecked);
    UpdateStream();
    UpdatePointCloudViewer();
}

void CVideoDevicePreviewWidget::on_checkBox_track_stream_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetPreviewOptions()->EnableStream(CVideoDeviceModel::STREAM_TRACK, bIsChecked);
    UpdateStream();
}

void CVideoDevicePreviewWidget::on_checkBox_depth_stream_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetPreviewOptions()->EnableStream(CVideoDeviceModel::STREAM_DEPTH, bIsChecked);
    UpdateStream();
    UpdateDepthmapBits();
    UpdateDepthOutput();
    UpdateDepthROI();
    UpdatePointCloudViewer();
}

void CVideoDevicePreviewWidget::on_horizontalSlider_depth_roi_valueChanged(int value)
{
    value *= 10;
    value = std::max(value, 1);
    m_pVideoDeviceController->GetPreviewOptions()->SetDepthROI(value);
    UpdateDepthROI();
}

void CVideoDevicePreviewWidget::on_horizontalSlider_ir_level_control_valueChanged(int value)
{
    m_pVideoDeviceController->SetIRLevel(value);
    UpdateIRLevel();
}

void CVideoDevicePreviewWidget::on_checkBox_extend_maximum_ir_stateChanged(int state)
{
     bool bIsChecked = Qt::Checked == state ? true : false;
     m_pVideoDeviceController->EnableIRExtend(bIsChecked);
     UpdateIRLevel();
}

void CVideoDevicePreviewWidget::on_spinBox_color_fps_valueChanged(int value)
{
    m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(CVideoDeviceModel::STREAM_COLOR, value);
    UpdateStream();
    UpdateInterleaveMode();
}

void CVideoDevicePreviewWidget::on_comboBox_color_fps_currentTextChanged(const QString &text)
{
    m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(CVideoDeviceModel::STREAM_COLOR, text.toInt());
    UpdateStream();
    UpdateInterleaveMode();
}

void CVideoDevicePreviewWidget::on_spinBox_kolor_fps_valueChanged(int value)
{
    m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(CVideoDeviceModel::STREAM_KOLOR, value);
    UpdateStream();
}

void CVideoDevicePreviewWidget::on_comboBox_kolor_fps_currentTextChanged(const QString &text)
{
    m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(CVideoDeviceModel::STREAM_KOLOR, text.toInt());
    UpdateStream();
}

void CVideoDevicePreviewWidget::on_spinBox_track_fps_valueChanged(int value)
{
    m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(CVideoDeviceModel::STREAM_TRACK, value);
    UpdateStream();
}

void CVideoDevicePreviewWidget::on_comboBox_track_fps_currentTextChanged(const QString &text)
{
    m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(CVideoDeviceModel::STREAM_TRACK, text.toInt());
    UpdateStream();
}

void CVideoDevicePreviewWidget::on_spinBox_depth_fps_valueChanged(int value)
{
    m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(CVideoDeviceModel::STREAM_DEPTH, value);
    UpdateStream();
}

void CVideoDevicePreviewWidget::on_comboBox_depth_fps_currentTextChanged(const QString &text)
{
    m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(CVideoDeviceModel::STREAM_DEPTH, text.toInt());
    UpdateStream();
}

void CVideoDevicePreviewWidget::on_pushButton_preview_clicked()
{
    m_pVideoDeviceController->StartStreaming();
//    CPointCloudViewerDialog *pDialog = new CPointCloudViewerDialog(this);
//    pDialog->resize(800, 480);
//    pDialog->show();
}

void CVideoDevicePreviewWidget::on_comboBox_color_stream_currentIndexChanged(int index)
{
    m_pVideoDeviceController->GetPreviewOptions()->SelectStreamIndex(CVideoDeviceModel::STREAM_COLOR, index);
}

void CVideoDevicePreviewWidget::on_comboBox_kolor_stream_currentIndexChanged(int index)
{
    m_pVideoDeviceController->GetPreviewOptions()->SelectStreamIndex(CVideoDeviceModel::STREAM_KOLOR, index);
}

void CVideoDevicePreviewWidget::on_comboBox_track_stream_currentIndexChanged(int index)
{
    m_pVideoDeviceController->GetPreviewOptions()->SelectStreamIndex(CVideoDeviceModel::STREAM_TRACK, index);
}

void CVideoDevicePreviewWidget::on_comboBox_depth_stream_currentIndexChanged(int index)
{
    m_pVideoDeviceController->GetPreviewOptions()->SelectStreamIndex(CVideoDeviceModel::STREAM_DEPTH, index);
}

void CVideoDevicePreviewWidget::on_comboBox_depth_output_currentTextChanged(const QString &text)
{
    if (!text.compare("Color")){
        m_pVideoDeviceController->GetPreviewOptions()->SetDepthDataTransferControl(DEPTH_IMG_COLORFUL_TRANSFER);
    }else if (!text.compare("Gray")){
        m_pVideoDeviceController->GetPreviewOptions()->SetDepthDataTransferControl(DEPTH_IMG_GRAY_TRANSFER);
    }else{
        m_pVideoDeviceController->GetPreviewOptions()->SetDepthDataTransferControl(DEPTH_IMG_NON_TRANSFER);
    }
}

void CVideoDevicePreviewWidget::on_pushButton_z_value_set_clicked()
{
    int nZNear = ui->spinBox_z_value_near->value();
    int nZFar = ui->spinBox_z_value_far->value();

    m_pVideoDeviceController->GetPreviewOptions()->SetZRange(nZNear, nZFar);
    m_pVideoDeviceController->AdjustZRange();
    UpdateZValue();
}

void CVideoDevicePreviewWidget::on_pushButton_snapshot_clicked()
{
    m_pVideoDeviceController->DoSnapShot();
}

void CVideoDevicePreviewWidget::on_checkBox_ply_filter_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetPreviewOptions()->EnablePlyFilter(bIsChecked);
    UpdatePlyFilter();
}

void CVideoDevicePreviewWidget::on_comboBox_mode_index_currentIndexChanged(int nIndex)
{
    m_pVideoDeviceController->SelectModeConfigIndex(nIndex);
    UpdateUI();
}

void CVideoDevicePreviewWidget::on_checkBox_interleave_mode_stateChanged(int state)
{
    UpdateModuleSync();
}

void CVideoDevicePreviewWidget::on_checkBox_module_sync_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->SetModuleSync(bIsChecked);
}

void CVideoDevicePreviewWidget::on_checkBox_master_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->SetModuleSyncMaster(bIsChecked);
}

void CVideoDevicePreviewWidget::on_comboBox_color_and_depth_stream_currentIndexChanged(int nIndex)
{
    m_pVideoDeviceController->UpdateStreamOptionForCombineMode(nIndex);
    UpdateStream();
}

void CVideoDevicePreviewWidget::on_checkBox_color_and_depth_stream_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;

    m_pVideoDeviceController->UpdateStreamOptionForCombineMode(bIsChecked ?
                                                               ui->comboBox_color_and_depth_stream->currentIndex() :
                                                               EOF);
    UpdateStream();

}

void CVideoDevicePreviewWidget::on_spinBox_color_and_depth_fps_valueChanged(int nValue)
{
    if (m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_COLOR)){
        m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(CVideoDeviceModel::STREAM_COLOR, nValue);
    }

    if (m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(CVideoDeviceModel::STREAM_DEPTH)){
        m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(CVideoDeviceModel::STREAM_DEPTH, nValue);
    }

    UpdateStream();
}

void CVideoDevicePreviewWidget::on_checkBox_hw_pp_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->EnableHWPP(bIsChecked);
}

void CVideoDevicePreviewWidget::on_pushButton_z_value_reset_clicked()
{
    int nDefaultZNear, nDefaultZFar;
    m_pVideoDeviceController->GetPreviewOptions()->GetDefaultZRange(nDefaultZNear, nDefaultZFar);
    ui->spinBox_z_value_near->setValue(nDefaultZNear);
    ui->spinBox_z_value_far->setValue(nDefaultZFar);
    ui->pushButton_z_value_set->click();
}

void CVideoDevicePreviewWidget::on_checkBox_point_cloud_viewer_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetPreviewOptions()->EnablePointCloudViewer(bIsChecked);
    UpdatePointCloudViewer();
}

void CVideoDevicePreviewWidget::on_horizontalSlider_point_size_valueChanged(int nValue)
{
    m_pVideoDeviceController->GetPreviewOptions()->SetPointCloudViewerPointSize(nValue);
    UpdatePointCloudViewer();
}

void CVideoDevicePreviewWidget::on_comboBox_point_cloud_viewer_format_currentTextChanged(const QString &text)
{
    if (!text.compare("Color")){
        m_pVideoDeviceController->GetPreviewOptions()->SetPointCloudViewOutputFormat(PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_COLOR);
    }else if (!text.compare("Depth Output")){
        m_pVideoDeviceController->GetPreviewOptions()->SetPointCloudViewOutputFormat(PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_DEPTH);
    }else if (!text.compare("Single Color")){
        m_pVideoDeviceController->GetPreviewOptions()->SetPointCloudViewOutputFormat(PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_SINGLE_COLOR);
    }

    UpdatePointCloudViewer();
}

void CVideoDevicePreviewWidget::on_spinBox_z_value_far_valueChanged(int nValue)
{
    if (nValue > 16383){
        ui->spinBox_z_value_far->setValue(16383);
    }
}
