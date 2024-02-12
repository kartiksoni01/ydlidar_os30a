#ifndef CVIDEODEVICEPREVIEWWIDGET_H
#define CVIDEODEVICEPREVIEWWIDGET_H

#include <QWidget>
#include "CEYSDUIView.h"
#include "CVideoDeviceModel.h"

namespace Ui {
class CVideoDevicePreviewWidget;
}

class CVideoDeviceController;
class CVideoDevicePreviewWidget : public QWidget,
                                  public CEYSDUIView
{
    Q_OBJECT

public:
    explicit CVideoDevicePreviewWidget(CVideoDeviceController *pVideoDeviceController,
                                       QWidget *parent = nullptr);
    ~CVideoDevicePreviewWidget();

    virtual void UpdateSelf();
    virtual void showEvent(QShowEvent *event);

    void Preview();

private slots:
    void on_checkBox_mode_config_stateChanged(int state);
    void on_checkBox_rectify_data_stateChanged(int state);
    void on_comboBox_depthmap_bits_currentTextChanged(const QString &text);
    void on_checkBox_color_stream_stateChanged(int state);
    void on_checkBox_kolor_stream_stateChanged(int state);
    void on_checkBox_track_stream_stateChanged(int state);
    void on_checkBox_depth_stream_stateChanged(int state);
    void on_horizontalSlider_depth_roi_valueChanged(int value);
    void on_horizontalSlider_ir_level_control_valueChanged(int value);
    void on_checkBox_extend_maximum_ir_stateChanged(int state);
    void on_spinBox_color_fps_valueChanged(int value);
    void on_comboBox_color_fps_currentTextChanged(const QString &text);
    void on_spinBox_kolor_fps_valueChanged(int value);
    void on_comboBox_kolor_fps_currentTextChanged(const QString &text);
    void on_spinBox_track_fps_valueChanged(int value);
    void on_comboBox_track_fps_currentTextChanged(const QString &text);
    void on_spinBox_depth_fps_valueChanged(int value);
    void on_comboBox_depth_fps_currentTextChanged(const QString &text);

    void on_pushButton_preview_clicked();

    void on_comboBox_color_stream_currentIndexChanged(int index);
    void on_comboBox_kolor_stream_currentIndexChanged(int index);
    void on_comboBox_track_stream_currentIndexChanged(int index);
    void on_comboBox_depth_stream_currentIndexChanged(int index);

    void on_comboBox_depth_output_currentTextChanged(const QString &text);

    void on_pushButton_z_value_set_clicked();
    void on_pushButton_snapshot_clicked();
    void on_checkBox_ply_filter_stateChanged(int state);

    void on_comboBox_mode_index_currentIndexChanged(int nIndex);

    void on_checkBox_interleave_mode_stateChanged(int state);

    void on_checkBox_module_sync_stateChanged(int state);

    void on_checkBox_master_stateChanged(int state);

    void on_comboBox_color_and_depth_stream_currentIndexChanged(int nIndex);

    void on_checkBox_color_and_depth_stream_stateChanged(int state);

    void on_spinBox_color_and_depth_fps_valueChanged(int nValue);

    void on_checkBox_hw_pp_stateChanged(int state);

    void on_pushButton_z_value_reset_clicked();

    void on_checkBox_point_cloud_viewer_stateChanged(int state);

    void on_horizontalSlider_point_size_valueChanged(int nValue);

    void on_comboBox_point_cloud_viewer_format_currentTextChanged(const QString &text);

    void on_spinBox_z_value_far_valueChanged(int nValue);

private:
    virtual void paintEvent(QPaintEvent *event);
    void UpdateUSBPort();
    void UpdateStreamimgComponetState(bool bIsStreaming);
    void UpdateStream();
    void UpdateColorWithDepthStream();
    void UpdateStream(CVideoDeviceModel::STREAM_TYPE type);
    void UpdateFPS(CVideoDeviceModel::STREAM_TYPE type);
    void UpdateModeConfig();
    void UpdateDepthOutput();
    void UpdateDepthmapBits();
    void UpdateZValue();
    void UpdateIRLevel();
    void UpdateDepthROI();
    void UpdateRectify();
    void UpdateHWPP();
    void UpdateModuleSync();
    void UpdateInterleaveMode();
    void UpdatePlyFilter();
    void UpdatePointCloudViewer();

private:
    CVideoDeviceController *m_pVideoDeviceController;
    Ui::CVideoDevicePreviewWidget *ui;
};

#endif // CVIDEODEVICEPREVIEWWIDGET_H
