#include "CPointCloudViewerDialog.h"
#include "CVideoDeviceController.h"
#include <QVBoxLayout>

CPointCloudViewerDialog::CPointCloudViewerDialog(CVideoDeviceController *pVideoDeviceController,
                                                 QWidget *parent):
QDialog(parent),
m_pVideoDeviceController(pVideoDeviceController),
m_pointCloudViewerWidget(pVideoDeviceController, this)
{
    setLayout(new QVBoxLayout());
    layout()->setMargin(0);
    layout()->addWidget(&m_pointCloudViewerWidget);
    layout()->setSizeConstraint(QLayout::SetNoConstraint);

    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    setWindowFlags(Qt::Window | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
}

CPointCloudViewerDialog::~CPointCloudViewerDialog()
{
    delete layout();
}

void CPointCloudViewerDialog::closeEvent(QCloseEvent *event)
{
    if (CVideoDeviceModel::STREAMING == m_pVideoDeviceController->GetVideoDeviceModel()->GetState()){
        m_pVideoDeviceController->StopStreaming();
    }

    deleteLater();

    QDialog::closeEvent(event);
}

void CPointCloudViewerDialog::paintEvent(QPaintEvent *event)
{
    QString sTitle;
    sTitle.sprintf("Point Cloud Viewer FPS[%.2f]", m_pointCloudViewerWidget.GetFPS());
    setWindowTitle(sTitle);

    QDialog::paintEvent(event);
}
