#include "CVideoDeviceAudoWidget.h"
#include "ui_CVideoDeviceAudioWidget.h"
#include "CTaskInfoManager.h"
#include "CThreadWorkerManage.h"

CVideoDeviceAudoWidget::CVideoDeviceAudoWidget(QWidget *parent):
QWidget(parent),
ui(new Ui::CVideoDeviceAudioWidget),
m_bIsRecording(false)
{
    ui->setupUi(this);
}

CVideoDeviceAudoWidget::~CVideoDeviceAudoWidget()
{
    delete ui;
}

void CVideoDeviceAudoWidget::showEvent(QShowEvent *event)
{
    UpdateUI();
}

void CVideoDeviceAudoWidget::UpdateSelf()
{
    UpdateButton();
    UpdateMessage();
}

void CVideoDeviceAudoWidget::UpdateButton()
{
    ui->pushButton_audio_record->setEnabled(!m_bIsRecording);
}

void CVideoDeviceAudoWidget::UpdateMessage()
{
    ui->label_audio_record_message->setText(m_sMessage);
    ui->label_audio_record_path->setText(m_sFilePath);
}

void CVideoDeviceAudoWidget::on_pushButton_audio_record_clicked()
{
    CTaskInfo *pInfo = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::AUDIO_RECORD, this);
    CThreadWorkerManage::GetInstance()->AddTask(pInfo);
}

void CVideoDeviceAudoWidget::DoAudioRecord()
{
    char buffer[256];
#if defined(UAC_X86_SUPPORTED)
    char path[] = "../eSPDI/alsa-release/x86/64-bits/bin/arecord";
#endif

#if defined(UAC_ARM_64_SUPPORTED)
    char *path = "../eSPDI/alsa-release/arm/64-bits/bin/arecord";
#endif
    char deviceName[16];
    char outputName[] = "../out/Audio/audio_record.wav";
    int rec;
    int record_len;

    m_bIsRecording = true;
    m_sFilePath = "";
    char input[] = "FrogEye2-Track";
    rec = APC_getUACNAME(input, deviceName);
    if (rec != 0) {
        m_sMessage = "Device Not Ready";
        UpdateUI();
        return;
    } else {
        m_sMessage = "Recording....";
        UpdateUI();
    }

    record_len = ui->spinBox_audio_record_time->value() - 3;
    if (record_len <= 0)
        record_len = 1;
    sprintf(buffer, "%s -f cd -D %s -d %d -t wav -N %s", path, deviceName,
            record_len, outputName);
    system(buffer);
    m_sMessage = "Record Completed";
    m_bIsRecording = false;
    m_sFilePath.sprintf("File are saved to \"%s\"", outputName);
    UpdateUI();
}
