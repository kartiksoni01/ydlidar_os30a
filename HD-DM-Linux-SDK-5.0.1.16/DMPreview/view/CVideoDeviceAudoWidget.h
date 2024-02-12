#ifndef CVIDEODEVICEAUDOWIDGET_H
#define CVIDEODEVICEAUDOWIDGET_H

#include <QWidget>
#include "CEYSDUIView.h"

namespace Ui {
class CVideoDeviceAudioWidget;
}


class CVideoDeviceAudoWidget : public QWidget,
                               public CEYSDUIView
{
    Q_OBJECT
public:
    explicit CVideoDeviceAudoWidget(QWidget *parent = nullptr);
    ~CVideoDeviceAudoWidget();

    virtual void showEvent(QShowEvent *event);
    virtual void UpdateSelf();

    void DoAudioRecord();

private:
    void UpdateButton();
    void UpdateMessage();

private slots:
    void on_pushButton_audio_record_clicked();

private:
    Ui::CVideoDeviceAudioWidget *ui;
    bool m_bIsRecording;
    QString m_sMessage;
    QString m_sFilePath;
};

#endif // CVIDEODEVICEAUDOWIDGET_H
