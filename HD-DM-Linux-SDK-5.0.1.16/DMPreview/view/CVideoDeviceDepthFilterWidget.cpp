#include "CVideoDeviceDepthFilterWidget.h"
#include "ui_CVideoDeviceDepthFilterWidget.h"
#include "CVideoDeviceController.h"

CVideoDeviceDepthFilterWidget::CVideoDeviceDepthFilterWidget(CVideoDeviceController *pVideoDeviceController,
                                                             QWidget *parent):
QWidget(parent),
ui(new Ui::CVideoDeviceDepthFilterWidget),
m_pVideoDeviceController(pVideoDeviceController)
{
    ui->setupUi(this);
}

CVideoDeviceDepthFilterWidget::~CVideoDeviceDepthFilterWidget()
{
    delete ui;
}

void CVideoDeviceDepthFilterWidget::showEvent(QShowEvent *event)
{
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::UpdateSelf()
{
    UpdateFilterState();
    UpdateSubSample();
    UpdateEdgePreServing();
    UpdateRemoveCurve();
    UpdateHoleFill();
    UpdateTemporalFilter();
}

void CVideoDeviceDepthFilterWidget::UpdateFilterState()
{
    bool bEnableFilter = m_pVideoDeviceController->GetDepthFilterOptions()->IsDepthFilter();        
    ui->checkBox_depth_filter_enable->setEnabled(!m_pVideoDeviceController->GetDepthFilterOptions()->IsDepthEnableLock());
    ui->checkBox_depth_filter_enable->blockSignals(true);
    ui->checkBox_depth_filter_enable->setChecked(bEnableFilter);
    ui->checkBox_depth_filter_enable->blockSignals(false);

    ui->widget_depth_filter_option_full_min->setEnabled(bEnableFilter);

    DepthFilterOptions::STATE state = m_pVideoDeviceController->GetDepthFilterOptions()->GetState();

    ui->checkBox_depth_filter_full->blockSignals(true);
    ui->checkBox_depth_filter_full->setChecked(DepthFilterOptions::FULL == state);
    ui->checkBox_depth_filter_full->blockSignals(false);

    ui->checkBox_depth_filter_min->blockSignals(true);
    ui->checkBox_depth_filter_min->setChecked(DepthFilterOptions::MIN == state);
    ui->checkBox_depth_filter_min->blockSignals(false);

    bool bCustomOptionEnable = bEnableFilter && DepthFilterOptions::CUSTOM == state;
    ui->groupBox_depth_filter_sub_sample->setEnabled(bCustomOptionEnable);
    ui->groupBox_depth_filter_edge_preserving->setEnabled(bCustomOptionEnable);
    ui->groupBox_depth_filter_remove_curve->setEnabled(bCustomOptionEnable);
    ui->groupBox_depth_filter_hoel_fill->setEnabled(bCustomOptionEnable);
    ui->groupBox_depth_filter_temporal_filter->setEnabled(bCustomOptionEnable);
}

void CVideoDeviceDepthFilterWidget::UpdateSubSample()
{
    DepthFilterOptions *pDepthFilterOptions = m_pVideoDeviceController->GetDepthFilterOptions();

    ui->checkBox_depth_filter_sub_sample_enable->blockSignals(true);
    ui->checkBox_depth_filter_sub_sample_enable->setChecked(pDepthFilterOptions->IsSubSample());
    ui->checkBox_depth_filter_sub_sample_enable->blockSignals(false);

    ui->comboBox_depth_filter_sub_sample_mode->blockSignals(true);
    ui->comboBox_depth_filter_sub_sample_mode->setCurrentText(QString::number(pDepthFilterOptions->GetSubSampleMode()));
    ui->comboBox_depth_filter_sub_sample_mode->blockSignals(false);

    ui->comboBox_depth_filter_sub_sample_factor->blockSignals(true);
    ui->comboBox_depth_filter_sub_sample_factor->clear();
    switch(pDepthFilterOptions->GetSubSampleMode()){
        case 0:
            ui->comboBox_depth_filter_sub_sample_factor->addItem("3");
            ui->comboBox_depth_filter_sub_sample_factor->addItem("2");
            break;
        case 1:
            ui->comboBox_depth_filter_sub_sample_factor->addItem("4");
            ui->comboBox_depth_filter_sub_sample_factor->addItem("5");
            break;
        default: break;
    }
    ui->comboBox_depth_filter_sub_sample_factor->setCurrentText(QString::number(pDepthFilterOptions->GetSubSampleFactor()));
    ui->comboBox_depth_filter_sub_sample_factor->blockSignals(false);
}

void CVideoDeviceDepthFilterWidget::UpdateEdgePreServing()
{
    DepthFilterOptions *pDepthFilterOptions = m_pVideoDeviceController->GetDepthFilterOptions();
    ui->checkBox_depth_filter_edge_preserving_enable->blockSignals(true);
    ui->checkBox_depth_filter_edge_preserving_enable->setChecked(pDepthFilterOptions->IsEdgePreServingFilter());
    ui->checkBox_depth_filter_edge_preserving_enable->blockSignals(false);

    ui->doubleSpinBoxdepth_filter_edge_preserving_lumda_value->blockSignals(true);
    ui->doubleSpinBoxdepth_filter_edge_preserving_lumda_value->setValue(pDepthFilterOptions->GetLumda());
    ui->doubleSpinBoxdepth_filter_edge_preserving_lumda_value->blockSignals(false);

    ui->doubleSpinBox_depth_filter_edge_preserving_sigma_value->blockSignals(true);
    ui->doubleSpinBox_depth_filter_edge_preserving_sigma_value->setValue(pDepthFilterOptions->GetSigma());
    ui->doubleSpinBox_depth_filter_edge_preserving_sigma_value->blockSignals(false);

    ui->label_content_depth_filter_edge_preserving_level_value->setText(QString::number(pDepthFilterOptions->GetEdgeLevel()));

    ui->horizontalSlider_depth_filter_edge_preserving_level_value->blockSignals(true);
    ui->horizontalSlider_depth_filter_edge_preserving_level_value->setValue(pDepthFilterOptions->GetEdgeLevel());
    ui->horizontalSlider_depth_filter_edge_preserving_level_value->blockSignals(false);
}

void CVideoDeviceDepthFilterWidget::UpdateRemoveCurve()
{
    DepthFilterOptions *pDepthFilterOptions = m_pVideoDeviceController->GetDepthFilterOptions();
    ui->checkBox_depth_filter_remove_curve_enable->setEnabled(!pDepthFilterOptions->IsFlyingDepthCancellationLock() /*&&
                                                              ui->checkBox_depth_filter_remove_curve_enable->isEnabled()*/);

    ui->checkBox_depth_filter_remove_curve_enable->blockSignals(true);
    ui->checkBox_depth_filter_remove_curve_enable->setChecked(pDepthFilterOptions->IsFlyingDepthCancellation());
    ui->checkBox_depth_filter_remove_curve_enable->blockSignals(false);
}

void CVideoDeviceDepthFilterWidget::UpdateHoleFill()
{
    DepthFilterOptions *pDepthFilterOptions = m_pVideoDeviceController->GetDepthFilterOptions();
    ui->checkBox_depth_filter_hole_fill_enable->blockSignals(true);
    ui->checkBox_depth_filter_hole_fill_enable->setChecked(pDepthFilterOptions->IsHoleFill());
    ui->checkBox_depth_filter_hole_fill_enable->blockSignals(false);

    ui->checkBox_depth_filter_hole_fill_enable_horizontal->blockSignals(true);
    ui->checkBox_depth_filter_hole_fill_enable_horizontal->setChecked(pDepthFilterOptions->IsHorizontal());
    ui->checkBox_depth_filter_hole_fill_enable_horizontal->blockSignals(false);

    ui->label_content_depth_filter_hole_fill_level_value->setText(QString::number(pDepthFilterOptions->GetLevel()));

    ui->horizontalSlider_depth_filter_hole_fill_level_value_range_value->blockSignals(true);
    ui->horizontalSlider_depth_filter_hole_fill_level_value_range_value->setValue(pDepthFilterOptions->GetLevel());
    ui->horizontalSlider_depth_filter_hole_fill_level_value_range_value->blockSignals(false);
}

void CVideoDeviceDepthFilterWidget::UpdateTemporalFilter()
{
    DepthFilterOptions *pDepthFilterOptions = m_pVideoDeviceController->GetDepthFilterOptions();
    ui->checkBox_depth_filter_temporal_filter_enable->blockSignals(true);
    ui->checkBox_depth_filter_temporal_filter_enable->setChecked(pDepthFilterOptions->IsTempleFilter());
    ui->checkBox_depth_filter_temporal_filter_enable->blockSignals(false);

    ui->spinBoxdepth_filter_temporal_filter_history_value->blockSignals(true);
    ui->spinBoxdepth_filter_temporal_filter_history_value->setValue(pDepthFilterOptions->GetHistory());
    ui->spinBoxdepth_filter_temporal_filter_history_value->blockSignals(false);

    ui->label_content_depth_filter_temporal_filter_alpha_value->setText(QString::number(pDepthFilterOptions->GetAlpha()));

    ui->horizontalSlider_depth_filter_temporal_filter_alpha_value->blockSignals(true);
    ui->horizontalSlider_depth_filter_temporal_filter_alpha_value->setValue(pDepthFilterOptions->GetAlpha() * 10);
    ui->horizontalSlider_depth_filter_temporal_filter_alpha_value->blockSignals(false);
}

void CVideoDeviceDepthFilterWidget::on_checkBox_depth_filter_enable_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetDepthFilterOptions()->EnableDepthFilter(bIsChecked);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_checkBox_depth_filter_full_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetDepthFilterOptions()->SetState(bIsChecked ? DepthFilterOptions::FULL : DepthFilterOptions::CUSTOM);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_checkBox_depth_filter_min_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetDepthFilterOptions()->SetState(bIsChecked ? DepthFilterOptions::MIN : DepthFilterOptions::CUSTOM);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_checkBox_depth_filter_sub_sample_enable_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetDepthFilterOptions()->EnableSubSample(bIsChecked);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_checkBox_depth_filter_edge_preserving_enable_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetDepthFilterOptions()->EnableEdgePreServingFilter(bIsChecked);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_checkBox_depth_filter_remove_curve_enable_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetDepthFilterOptions()->EnableFlyingDepthCancellation(bIsChecked);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_checkBox_depth_filter_hole_fill_enable_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetDepthFilterOptions()->EnableHoleFill(bIsChecked);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_checkBox_depth_filter_temporal_filter_enable_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetDepthFilterOptions()->EnableTempleFilter(bIsChecked);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_comboBox_depth_filter_sub_sample_mode_currentTextChanged(const QString &text)
{
    int nValue = text.toInt();
    m_pVideoDeviceController->GetDepthFilterOptions()->SetSubSampleMode(nValue);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_comboBox_depth_filter_sub_sample_factor_currentTextChanged(const QString &text)
{
    int nValue = text.toInt();
    m_pVideoDeviceController->GetDepthFilterOptions()->SetSubSampleFactor(nValue);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_horizontalSlider_depth_filter_edge_preserving_level_value_valueChanged(int nValue)
{
    m_pVideoDeviceController->GetDepthFilterOptions()->SetEdgeLevel(nValue);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_checkBox_depth_filter_hole_fill_enable_horizontal_stateChanged(int state)
{
    bool bIsChecked = Qt::Checked == state ? true : false;
    m_pVideoDeviceController->GetDepthFilterOptions()->SetHorizontal(bIsChecked);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_horizontalSlider_depth_filter_hole_fill_level_value_range_value_valueChanged(int nValue)
{
    m_pVideoDeviceController->GetDepthFilterOptions()->SetLevel(nValue);
    UpdateUI();
}

void CVideoDeviceDepthFilterWidget::on_horizontalSlider_depth_filter_temporal_filter_alpha_value_valueChanged(int nValue)
{
    m_pVideoDeviceController->GetDepthFilterOptions()->SetAlpha(nValue / 10.0f);
    UpdateUI();
}
