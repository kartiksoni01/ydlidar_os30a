#ifndef CMESSAGEMANAGER_H
#define CMESSAGEMANAGER_H
#include <QString>

class CMessageDialog;
class QWidget;
class CMessageManager
{
public:
    static void ShowMessage(QString sMessage, QWidget *pWidget = nullptr);
    static void CloseMessage();
    static void Error(QString sMessage);
private:

    static CMessageDialog *GeneratorMessageDialog(QString sMessage, QWidget *pWidget = nullptr);

    CMessageManager() = default;


private:
    static CMessageDialog *m_pMessageDialog;
};

#endif // CMESSAGEMANAGER_H
