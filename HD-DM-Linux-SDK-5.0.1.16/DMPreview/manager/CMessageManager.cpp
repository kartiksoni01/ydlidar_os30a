#include "CMessageManager.h"
#include <QDialog>
#include "CMessageDialog.h"
#include "CEYSDUIView.h"
#include <QMessageBox>

CMessageDialog *CMessageManager::m_pMessageDialog = nullptr;

void CMessageManager::ShowMessage(QString sMessage, QWidget *pWidget)
{
    if(m_pMessageDialog && !sMessage.compare(m_pMessageDialog->GetMessage())) return;

    RUN_ON_UI_THREAD(
    CloseMessage();
    m_pMessageDialog = GeneratorMessageDialog(sMessage, pWidget);
    m_pMessageDialog->show();
    );
}

void CMessageManager::Error(QString sMessage)
{
    RUN_ON_UI_THREAD(
    QMessageBox::critical(nullptr, "Error", sMessage);
    );
}

void CMessageManager::CloseMessage()
{
    RUN_ON_UI_THREAD(
    if (m_pMessageDialog) {
        m_pMessageDialog->close();
        m_pMessageDialog = nullptr;
    }
    );
}

CMessageDialog *CMessageManager::GeneratorMessageDialog(QString sMessage, QWidget *pWidget)
{
    return new CMessageDialog(sMessage, pWidget);
}
