#ifndef CPREVIEWDIALOG_H
#define CPREVIEWDIALOG_H
#include <QDialog>
#include <QLabel>
#include "CEYSDUIView.h"

class CImageDataModel;
class CPreviewDialog : public QDialog,
                       public CEYSDUIView
{
Q_OBJECT
public:
    CPreviewDialog(CVideoDeviceController *pVideoDeviceController,
                   CImageDataModel *pImageDataModel,
                   QWidget *parent);
    ~CPreviewDialog();
    virtual void UpdateSelf();
    virtual void paintEvent(QPaintEvent *event);
    virtual void closeEvent(QCloseEvent *event);

    CImageDataModel *GetImageDataModel(){ return m_pImageDataModel; }

    void SetPreferSize(int nPreferWidth, int nPreferHeight)
    {
        m_nPreferWidth = nPreferWidth;
        m_nPreferHeight = nPreferHeight;
    }

    virtual void ResizePreviewDialog();

    virtual void mouseMoveEvent(QMouseEvent* event);

protected:
    QLabel m_imageLabel;
    CVideoDeviceController *m_pVideoDeviceController;
    CImageDataModel *m_pImageDataModel;

    QMutex m_mutex;

    float m_fImageScale;
    int m_nPreferWidth, m_nPreferHeight;
};

#endif // CPREVIEWDIALOG_H
