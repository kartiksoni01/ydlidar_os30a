#ifndef CVIDEODEVICEDEPTHFILTERWIDGET_H
#define CVIDEODEVICEDEPTHFILTERWIDGET_H

#include <QWidget>
#include "CEYSDUIView.h"

namespace Ui {
class CVideoDeviceDepthFilterWidget;
}

class CVideoDeviceDepthFilterWidget : public QWidget,
                                      public CEYSDUIView
{
    Q_OBJECT
public:
    explicit CVideoDeviceDepthFilterWidget(CVideoDeviceController *pVideoDeviceController,
                                           QWidget *parent = nullptr);
    ~CVideoDeviceDepthFilterWidget();

    virtual void showEvent(QShowEvent *event);
    virtual void UpdateSelf();

private slots:
    void on_checkBox_depth_filter_enable_stateChanged(int state);

    void on_checkBox_depth_filter_full_stateChanged(int state);

    void on_checkBox_depth_filter_min_stateChanged(int state);

    void on_checkBox_depth_filter_sub_sample_enable_stateChanged(int state);

    void on_checkBox_depth_filter_edge_preserving_enable_stateChanged(int state);

    void on_checkBox_depth_filter_remove_curve_enable_stateChanged(int state);

    void on_checkBox_depth_filter_hole_fill_enable_stateChanged(int state);

    void on_checkBox_depth_filter_temporal_filter_enable_stateChanged(int state);

    void on_comboBox_depth_filter_sub_sample_mode_currentTextChanged(const QString &text);

    void on_comboBox_depth_filter_sub_sample_factor_currentTextChanged(const QString &text);

    void on_horizontalSlider_depth_filter_edge_preserving_level_value_valueChanged(int nValue);

    void on_checkBox_depth_filter_hole_fill_enable_horizontal_stateChanged(int state);

    void on_horizontalSlider_depth_filter_hole_fill_level_value_range_value_valueChanged(int nValue);

    void on_horizontalSlider_depth_filter_temporal_filter_alpha_value_valueChanged(int nValue);

private:
    void UpdateFilterState();
    void UpdateSubSample();
    void UpdateEdgePreServing();
    void UpdateRemoveCurve();
    void UpdateHoleFill();
    void UpdateTemporalFilter();

private:
    CVideoDeviceController *m_pVideoDeviceController;
    Ui::CVideoDeviceDepthFilterWidget *ui;
};

#endif // CVIDEODEVICEDEPTHFILTERWIDGET_H
