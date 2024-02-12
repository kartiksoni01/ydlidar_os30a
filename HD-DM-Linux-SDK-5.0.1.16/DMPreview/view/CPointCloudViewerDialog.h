#ifndef CPOINTCLOUDVIEWERDIALOG_H
#define CPOINTCLOUDVIEWERDIALOG_H
#include <QDialog>
#include "CEYSDUIView.h"
#include "CPointCloudViewerWidget.h"

class CPointCloudViewerDialog : public QDialog,
                                public CEYSDUIView
{
Q_OBJECT

public:
    CPointCloudViewerDialog(CVideoDeviceController *pVideoDeviceController,
                            QWidget *parent = 0);
    ~CPointCloudViewerDialog();

    virtual void closeEvent(QCloseEvent *event);
    virtual void paintEvent(QPaintEvent *event);

    CPointCloudViewerWidget &GetPointCloudViewer(){ return m_pointCloudViewerWidget; }

private:
    CPointCloudViewerWidget m_pointCloudViewerWidget;
    CVideoDeviceController *m_pVideoDeviceController;
};

#endif // CPOINTCLOUDVIEWERDIALOG_H
