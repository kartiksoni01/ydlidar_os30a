#include "CMessageDialog.h"
#include <QVBoxLayout>
#include <QLabel>

CMessageDialog::CMessageDialog(QString sMessage, QWidget *parent):
QDialog(parent)
{
    setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    setAttribute(Qt::WA_DeleteOnClose);
    setWindowModality(Qt::ApplicationModal);

    setLayout(new QVBoxLayout());
    m_messageLable.setAlignment(Qt::AlignCenter);
    m_messageLable.setText(sMessage);
    QFont font = m_messageLable.font();
    font.setPointSize(10);
    font.setBold(true);
    m_messageLable.setFont(font);
    layout()->addWidget(&m_messageLable);

    setFixedSize(300, 100);

    QString sStyleSheet = " \
    QDialog \
    { \
        border: 4px solid #aaaaaa; \
        border-radius: 5px; \
        background: #ffffff; \
    } \
    ";
    setStyleSheet(sStyleSheet);

    AddUpdateTimer();
}

CMessageDialog::~CMessageDialog()
{
    delete layout();
}
