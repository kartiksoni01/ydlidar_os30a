#ifndef CVIDEODEVICEDEPTHACCURACYWIDGET_H
#define CVIDEODEVICEDEPTHACCURACYWIDGET_H

#include <QWidget>
#include "CEYSDUIView.h"

namespace Ui {
class CVideoDeviceDepthAccuracyWidget;
}

class CDepthAccuracyController;
class CVideoDeviceDepthAccuracyWidget : public QWidget,
                                        public CEYSDUIView
{
    Q_OBJECT
public:
    explicit CVideoDeviceDepthAccuracyWidget(CDepthAccuracyController *pCameraPropertyController,
                                             QWidget *parent = nullptr);
    ~CVideoDeviceDepthAccuracyWidget();

    virtual void paintEvent(QPaintEvent *event);
    virtual void showEvent(QShowEvent *event);
    virtual void hideEvent(QHideEvent *event);
    virtual void UpdateSelf();

private slots:
    void on_comboBox_depth_accuracy_stream_currentTextChanged(const QString &text);

    void on_comboBox_depth_accuracy_roi_currentTextChanged(const QString &text);

    void on_checkBox_depth_accuracy_ground_truth_stateChanged(int state);

    void on_doubleSpinBox_depth_accuracy_ground_truth_vaule_valueChanged(double dblValue);

private:
    void UpdateAccuracyList();
    void UpdateAccuracyRegion();
    void UpdateGroundTruth();
    void UpdateValue();

private:
    CDepthAccuracyController *m_pDepthAccuracyController;
    Ui::CVideoDeviceDepthAccuracyWidget *ui;
};

#endif // CVIDEODEVICEDEPTHACCURACYWIDGET_H
