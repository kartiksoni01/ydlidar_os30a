#include "CPreviewDialog.h"
#include <QLayout>
#include <QPainter>
#include <QCloseEvent>
#include "CImageDataModel.h"
#include "CVideoDeviceController.h"

CPreviewDialog::CPreviewDialog(CVideoDeviceController *pVideoDeviceController,
                               CImageDataModel *pImageDataModel,
                               QWidget *parent):
QDialog(parent),
m_pVideoDeviceController(pVideoDeviceController),
m_pImageDataModel(pImageDataModel),
m_fImageScale(1.0f),
m_nPreferWidth(0), m_nPreferHeight(0)
{
    setLayout(new QVBoxLayout());
    layout()->setMargin(0);
    layout()->addWidget(&m_imageLabel);
    layout()->setSizeConstraint(QLayout::SetNoConstraint);

    setMouseTracking(true);
    m_imageLabel.setMouseTracking(true);
    m_imageLabel.setScaledContents(true);

    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    setAttribute(Qt::WA_DeleteOnClose);

    setWindowFlags(Qt::Window | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
}

CPreviewDialog::~CPreviewDialog()
{
    delete layout();
    if (m_pImageDataModel) {
        delete m_pImageDataModel;
    }
}

void CPreviewDialog::UpdateSelf()
{
    CEYSDUIView::UpdateSelf();
}

void CPreviewDialog::closeEvent(QCloseEvent *event)
{
    if (m_pImageDataModel) {
        QMutexLocker locker(&m_mutex);
        delete m_pImageDataModel;
        m_pImageDataModel = nullptr;
    }

    if (CVideoDeviceModel::STREAMING == m_pVideoDeviceController->GetVideoDeviceModel()->GetState()){
        m_pVideoDeviceController->StopStreaming();
    }

    QDialog::closeEvent(event);
}

void CPreviewDialog::paintEvent(QPaintEvent *event)
{
    QMutexLocker locker(&m_mutex);

    if (!m_pImageDataModel) return;

    int nWidth = m_pImageDataModel->GetWidth();
    int nHeight = m_pImageDataModel->GetHeight();
    if (0 == nWidth || 0 == nHeight) return;

    auto data = m_pImageDataModel->GetRGBData();
    QImage img = QImage( &data[0], nWidth, nHeight, QImage::Format_RGB888);

    if( CImageDataModel::DEPTH == m_pImageDataModel->GetModelType()){
        CImageDataModel_Depth *pDepthImageDataModel = static_cast<CImageDataModel_Depth *>(m_pImageDataModel);
        if(pDepthImageDataModel->IsDepthAccuracyEnable() &&
           pDepthImageDataModel->GetDepthAccuracyRegionRatio() > 0.0)
        {
            QPainter painter(&img);
            painter.setPen(QPen(Qt::yellow, 2));

            float fHRatio = (float)img.width() / pDepthImageDataModel->GetWidth();
            float fVRatio = (float)img.height() / pDepthImageDataModel->GetHeight();

            QRect depthAccuracyRegion = pDepthImageDataModel->GetDepthAccuracyRegion();
            int nDepthAccuracyLeft = depthAccuracyRegion.left() * fHRatio;
            int nDepthAccuracyTop = depthAccuracyRegion.top() * fVRatio;
            int nDepthAccuracyRight = depthAccuracyRegion.right() * fHRatio;
            int nDepthAccuracyBottom = depthAccuracyRegion.bottom() * fVRatio;

            painter.drawRect(nDepthAccuracyLeft, nDepthAccuracyTop,
                             nDepthAccuracyRight - nDepthAccuracyLeft, nDepthAccuracyBottom - nDepthAccuracyTop);
            QFont font;
            const int nFontSize = 15 * ((float)img.height() / m_imageLabel.height());
            font.setPixelSize(nFontSize);
            painter.setFont(font);
            painter.drawText(QRect(nDepthAccuracyLeft, nDepthAccuracyTop - nFontSize - 10,
                                   nDepthAccuracyRight - nDepthAccuracyLeft, nFontSize + 10),
                             "Depth Accuray Region",
                             QTextOption(Qt::AlignCenter));
            painter.end();
        }
    }
    
    //+[Thermal device]
    if(APCImageType::COLOR_RGB24 == m_pImageDataModel->GetImageType()) {
            QPainter painter(&img);
            painter.setPen(QPen(Qt::green, 2));
            QFont font;
            const int nFontSize = 15 * ((float)img.height() / m_imageLabel.height());
            font.setPixelSize(nFontSize);
            painter.setFont(font);
            painter.drawText(QPoint(nWidth/2,nHeight/2-nFontSize/2-10),QString::number(m_pImageDataModel->GetSerialNumber(), 'f', 1)+"C");

            const int nLabelSize = 20 * ((float)img.height() / m_imageLabel.height());
            font.setPixelSize(nLabelSize);
            painter.setFont(font);
            painter.drawText(QPoint(nWidth/2,nHeight/2),"+");
            painter.end();
    }
    //-[Thermal device]

    QPixmap qPixMap = QPixmap::fromImage(*(&img), Qt::AutoColor);
    m_imageLabel.setPixmap(qPixMap);

    setWindowTitle(m_pImageDataModel->GetImgaeDataInfo());

    QDialog::paintEvent(event);
}

void CPreviewDialog::ResizePreviewDialog()
{
    int nWidth = m_pImageDataModel->GetWidth();
    int nHeight = m_pImageDataModel->GetHeight();

    if (0 == nWidth || 0 == nHeight) return;

    if (size().width() != m_nPreferWidth && size().height() != m_nPreferHeight){

        if (m_nPreferWidth > 0 && m_nPreferHeight > 0){
            float fWScale = (float)m_nPreferWidth / nWidth;
            float fHScale = (float)m_nPreferHeight / nHeight;

            m_fImageScale = std::min(fWScale, fHScale);

            if (m_fImageScale == fWScale){
                nWidth = m_nPreferWidth;
                nHeight *= m_fImageScale;
            }else{
                nWidth *= m_fImageScale;
                nHeight = m_nPreferHeight;
            }

        }        
        RUN_ON_UI_THREAD(
        resize(nWidth, nHeight);
        );
    }
}

void CPreviewDialog::mouseMoveEvent(QMouseEvent* event)
{
    if (!m_pImageDataModel) return;
    if (CImageDataModel::DEPTH != m_pImageDataModel->GetModelType()) return;

    int nWidth = m_pImageDataModel->GetWidth();
    int nHeight = m_pImageDataModel->GetHeight();

    float fWScale = (float)size().width() / nWidth;
    float fHScale = (float)size().height() / nHeight;

    int x = event->x() / fWScale;
    int y = event->y() / fHScale;

    m_pVideoDeviceController->UpdateSpecificDepthPosition(x, y);
}
